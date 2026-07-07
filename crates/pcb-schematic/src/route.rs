//! Sheet routing for the placed model — labels, hierarchical anchors,
//! power doglegs and the real-wire passes of the validated `layout.ts`
//! proof of concept.
//!
//! Wiring order (each family avoids everything already placed):
//! 1. instance Reference/Value texts ([`crate::texts`]) — their boxes are
//!    keepouts for every wire placed next;
//! 2. real-wire TREES of the signal groups (Zener `anchor=` info or the
//!    non-hub heuristic — tree candidates are the most constrained);
//! 3. stub + **horizontal** net labels for the remaining signal endpoints
//!    (vertical pins get an elbow — labels are never rotated 90°), the
//!    wire underlining the full text; design-wide single-endpoint nets
//!    get a global label; hierarchical ports anchor **directly at the end
//!    of an internal stub** chosen by flow;
//! 4. facing pairs as direct/Z wires (top to bottom, staggered middle
//!    branch, zero-crossing candidates preferred);
//! 5. power/ground pins: short stub + generated power symbol on every
//!    pin, same-net symbols aligned in rows (common Y), dogleg retries
//!    avoiding foreign contacts; one PWR_FLAG per undriven rail chained
//!    next to the first symbol of its assigned sheet;
//! 6. no-connect markers, child sheet blocks, edge-column port fallback.
//!
//! The anti-contact registry guarantees that no stub lands on a foreign
//! wire or pin: only frank perpendicular crossings (which do not connect
//! in KiCad) are tolerated, and candidates are scored to avoid even those.

use std::collections::{BTreeMap, BTreeSet};

use crate::config::SchConfig;
use crate::geometry::BBox;
use crate::model::{DesignModel, NetClass};
use crate::place::{
    PlacedComp, SheetModel, SheetNet, inflate, label_stub_len, label_text_width, overlaps, raw_box,
};
use crate::round4;
use crate::sheets::SheetPlan;
use crate::texts::{Corridor, CorridorZone};
use crate::wiring::{
    EPS, Seg, dist, group_wire_subsets, is_hub_comp, is_pair_net_static, path_length_mm,
    point_on_seg, quant, seg_intersects_box, segs_cross_frank, segs_touch_forbidden,
};
use crate::writer::PortDirection;

type Point = (f64, f64);

/// Everything routed on one sheet, ready for emission.
#[derive(Default)]
pub struct RoutedSheet {
    pub wires: Vec<Vec<Point>>,
    pub junctions: Vec<Point>,
    pub net_labels: Vec<(String, Point, i32)>,
    pub global_labels: Vec<(String, Point, i32, PortDirection)>,
    pub hier_labels: Vec<(String, PortDirection, Point, i32)>,
    pub power_symbols: Vec<(String, Point, bool)>,
    pub pwr_flags: Vec<(Point, i32)>,
    pub no_connects: Vec<Point>,
    pub blocks: Vec<BlockModel>,
    /// Graphic zone outlines (functional / decoupling / ERC), purely visual.
    pub zones: Vec<Zone>,
    /// Content extents including routed elements (paper selection).
    pub extents: BBox,
}

/// A graphic zone outline with an optional corner title. Emitted as a thin
/// dashed rectangle on the notes layer — no electrical meaning.
pub struct Zone {
    pub bbox: BBox,
    pub title: Option<String>,
}

/// A child sheet block placed on this sheet.
pub struct BlockModel {
    /// Index of the child sheet in the plan.
    pub sheet: usize,
    pub at: Point,
    pub size: (f64, f64),
    pub pins: Vec<(String, PortDirection, Point)>,
}

/// Box of an already placed text, tagged with the net(s) it lets through.
pub(crate) struct LabelBox {
    pub bbox: BBox,
    pub net: String,
    /// Additional own nets of an instance text kept as a last resort on
    /// the path of its own stubs (the wire may run under the text).
    pub also: Vec<String>,
}

impl LabelBox {
    pub(crate) fn allows(&self, net: &str) -> bool {
        self.net == net || self.also.iter().any(|n| n == net)
    }
}

pub(crate) struct Reg {
    pub segs: Vec<Seg>,
    pub pins: Vec<(f64, f64, String)>,
    pub power_boxes: Vec<(BBox, String)>,
    pub label_boxes: Vec<LabelBox>,
}

#[derive(Clone)]
pub(crate) struct Attachment {
    pub path: Vec<Point>,
    pub symbol_at: Point,
    pub down: bool,
}

/// Power connection path: outgoing stub (+ vertical return for horizontal
/// pins). `h_extra`/`v_extra` stretch the branches for avoidance.
pub(crate) fn power_attachment(
    cfg: &SchConfig,
    pin: Point,
    dir: (f64, f64),
    down: bool,
    h_extra: f64,
    v_extra: f64,
) -> Attachment {
    // The base straight leg out of a pin is never shorter than the minimum
    // pin exit (rule #1): a right angle is only ever struck at least
    // `min_pin_exit_steps` grid steps away from the pin.
    let min_exit = cfg.min_pin_exit_mm();
    let g = cfg.power_stub_mm.max(min_exit);
    if dir.0.abs() < 0.5 {
        let end = (pin.0, round4(pin.1 + dir.1 * (g + v_extra).max(min_exit)));
        return Attachment {
            path: vec![pin, end],
            symbol_at: end,
            down,
        };
    }
    let elbow = (round4(pin.0 + dir.0 * (g + h_extra).max(min_exit)), pin.1);
    let end = (
        elbow.0,
        round4(elbow.1 + if down { g + v_extra } else { -(g + v_extra) }),
    );
    Attachment {
        path: vec![pin, elbow, end],
        symbol_at: end,
        down,
    }
}

/// Graphic footprint of a power symbol (arrow/bars + value text) around its
/// connection point.
pub(crate) fn power_symbol_graphic_box(net_name: &str, at: Point, down: bool) -> BBox {
    let half_text = (net_name.chars().count() as f64 * 0.762 + 1.27).max(2.54);
    if down {
        BBox {
            x1: at.0 - half_text,
            y1: at.1,
            x2: at.0 + half_text,
            y2: at.1 + 6.35,
        }
    } else {
        BBox {
            x1: at.0 - half_text,
            y1: at.1 - 7.62,
            x2: at.0 + half_text,
            y2: at.1,
        }
    }
}

/// Complete drawn box of a placed component: its canonical solid box unioned
/// with the Reference and Value texts at their FINAL placed anchors. The text
/// pass can slide the Value to the side (past the canonical box), so a zone
/// outline that relied on `p.bbox` alone would clip it — this box does not.
pub(crate) fn placed_full_box(design: &DesignModel, p: &PlacedComp) -> BBox {
    let comp = &design.comps[p.comp];
    let mut b = p.bbox;
    b.union(&crate::texts::ref_text_box(
        &comp.refdes,
        p.ref_at,
        p.ref_justify_right,
    ));
    b.union(&crate::texts::value_text_box(
        &comp.value,
        p.value_at,
        p.value_justify_right,
    ));
    b
}

/// Does a power net name read as a negative rail (VEE, V-, -12V, ...)? Used to
/// order power symbols by potential: a negative rail rides below ground.
pub(crate) fn is_negative_rail_name(name: &str) -> bool {
    let u = name.trim().to_ascii_uppercase();
    u.starts_with('-') || u.starts_with("V-") || u.contains("VEE") || u.contains("VNEG")
}

/// Vertical potential rank of a power net: the higher the rank the higher the
/// symbol rides on the sheet. Positive rails (VDD/VCC/+…) outrank ground,
/// ground outranks negative rails (VEE/-…). Drives the by-potential ordering
/// and alignment of a cluster's power symbols (rule #3).
pub(crate) fn power_potential_rank(name: &str, class: NetClass) -> i32 {
    match class {
        NetClass::Power => {
            if is_negative_rail_name(name) {
                -1
            } else {
                1
            }
        }
        NetClass::Ground => 0,
        NetClass::Signal => 0,
    }
}

/// A power symbol hangs its leg DOWNWARD (ground-style) at or below ground
/// potential — grounds and negative rails — and points up for positive rails.
pub(crate) fn power_points_down(name: &str, class: NetClass) -> bool {
    power_potential_rank(name, class) <= 0
}

/// Graphic footprint of a `PWR_FLAG` glyph anchored at its connection point
/// (the flag and its wide value text hang above the point).
pub(crate) fn pwr_flag_box(at: Point, rot: i32) -> BBox {
    // The pennant + wide value text hang on the side the glyph points to:
    // above the connection point at rot 0, below when flipped (rot 180, used
    // for ground flags so the pennant faces away from a wire arriving from
    // above and the net no longer folds back over the symbol).
    if rot == 180 {
        BBox {
            x1: at.0 - 2.54,
            y1: at.1 - 0.5,
            x2: at.0 + 2.54,
            y2: at.1 + 5.08,
        }
    } else {
        BBox {
            x1: at.0 - 2.54,
            y1: at.1 - 5.08,
            x2: at.0 + 2.54,
            y2: at.1 + 0.5,
        }
    }
}

impl Reg {
    fn new(label_boxes: Vec<LabelBox>) -> Reg {
        Reg {
            segs: Vec::new(),
            pins: Vec::new(),
            power_boxes: Vec::new(),
            label_boxes,
        }
    }

    pub(crate) fn register_path(&mut self, points: &[Point], net: &str) {
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            self.segs.push(Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: net.to_string(),
            });
        }
    }

    pub(crate) fn count_crossings(&self, points: &[Point]) -> usize {
        let mut n = 0;
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg = Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: String::new(),
            };
            for other in &self.segs {
                if segs_cross_frank(&seg, other) {
                    n += 1;
                }
            }
        }
        n
    }

    /// Frank crossings of `points` against segments of a DIFFERENT net.
    /// These are electrically harmless (no junction, no connection) and are
    /// the crossings the router tolerates under a cost penalty.
    pub(crate) fn foreign_crossings(&self, points: &[Point], net: &str) -> usize {
        let mut n = 0;
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg = Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: String::new(),
            };
            for other in &self.segs {
                if other.net != net && segs_cross_frank(&seg, other) {
                    n += 1;
                }
            }
        }
        n
    }

    /// True when `points` frank-crosses an already placed segment of the SAME
    /// net: two wires of one net that cross without a junction read as an
    /// accidental (and confusing) split, so this is always forbidden — even
    /// when foreign crossings are otherwise tolerated.
    pub(crate) fn same_net_frank_crossing(&self, points: &[Point], net: &str) -> bool {
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg = Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: String::new(),
            };
            for other in &self.segs {
                if other.net == net && segs_cross_frank(&seg, other) {
                    return true;
                }
            }
        }
        false
    }
}

/// Orientation of a net label relative to the outward (port) reading of its
/// stub. A **global/hierarchical** label is a directional port: its text reads
/// outward, away from the circuit, off the wire's terminal end (`outward`
/// kept as is). A **local** net label instead names the conductor it sits on:
/// its text must overhang the wire (the wire runs under the text and continues
/// past it) rather than hang off the end, so it reads *back over* the wire —
/// the outward rotation flipped by 180°. Only the text orientation changes; the
/// connection point is untouched, so connectivity/netlist/ERC are unaffected.
pub(crate) fn net_label_rotation(global: bool, outward: i32) -> i32 {
    if global {
        outward
    } else {
        (outward + 180).rem_euclid(360)
    }
}

pub(crate) fn label_text_box(name: &str, at: Point, rotation: i32) -> BBox {
    let w = label_text_width(name);
    if rotation == 0 {
        BBox {
            x1: at.0,
            y1: at.1 - 2.2,
            x2: at.0 + w,
            y2: at.1 + 0.4,
        }
    } else {
        BBox {
            x1: at.0 - w,
            y1: at.1 - 2.2,
            x2: at.0,
            y2: at.1 + 0.4,
        }
    }
}

/// Body box of a hierarchical label (glyph + text) anchored at `at`.
pub(crate) fn hier_text_box(name: &str, at: Point, rotation: i32) -> BBox {
    let w = label_text_width(name) + 1.27;
    if rotation == 0 {
        BBox {
            x1: at.0,
            y1: at.1 - 1.27,
            x2: at.0 + w,
            y2: at.1 + 1.27,
        }
    } else {
        BBox {
            x1: at.0 - w,
            y1: at.1 - 1.27,
            x2: at.0,
            y2: at.1 + 1.27,
        }
    }
}

/// Two axis-aligned text boxes collide only when they overlap by more than
/// `tol` on BOTH axes. Sibling labels aligned onto one column sit two grid
/// steps apart (2.54 mm) while the padded text box is a hair taller (2.6 mm),
/// so their keepouts graze by ~0.06 mm — the reference layout stacks them just
/// so. `tol` swallows that sub-grid graze while still catching a real overlap
/// (two labels a single step apart, or one on top of another).
pub(crate) fn boxes_collide_tol(a: &BBox, b: &BBox, tol: f64) -> bool {
    let xo = a.x2.min(b.x2) - a.x1.max(b.x1);
    let yo = a.y2.min(b.y2) - a.y1.max(b.y1);
    xo > tol && yo > tol
}

fn near(a: Point, b: Point) -> bool {
    (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS
}

/// Which routed-label vector an [`AlignItem`] lives in, and its index there.
#[derive(Clone, Copy)]
enum LabelSlot {
    Global(usize),
    Hier(usize),
    Net(usize),
}

/// How a horizontal label anchors to a wire, and thus how it may move onto a
/// shared X column without touching the netlist.
#[derive(Clone, Copy)]
enum AlignKind {
    /// Terminal vertex of a horizontal stub off a component pin. `fixed` is the
    /// inner (pin-side) end that stays put; extending the stub only grows the
    /// segment outward. `wire`/`last` locate the vertex in `out.wires`.
    Stub {
        wire: usize,
        last: bool,
        fixed: Point,
        comp: usize,
    },
    /// Interior point of a horizontal wire run spanning `[lo, hi]`; the label
    /// slides along the conductor it already names (the wire never moves).
    Interior { lo: f64, hi: f64 },
}

/// A label that is a candidate for soft column alignment.
struct AlignItem {
    slot: LabelSlot,
    net: String,
    at: Point,
    rot: i32,
    kind: AlignKind,
}

/// One endpoint of a net on the sheet: position + outward direction.
#[derive(Clone)]
pub(crate) struct Endpoint {
    pub placed: usize,
    pub pad: String,
    pub pos: Point,
    pub dir: (f64, f64),
}

pub(crate) struct Router<'a> {
    pub(crate) cfg: &'a SchConfig,
    pub(crate) design: &'a DesignModel,
    pub(crate) model: &'a SheetModel,
    pub(crate) plan: &'a SheetPlan,
    pub(crate) reg: Reg,
    pub(crate) out: RoutedSheet,
    pub(crate) warnings: &'a mut Vec<String>,
    /// Ports already anchored (hier label placed).
    pub(crate) port_anchored: BTreeSet<String>,
    /// Predicted power keepouts (from the text pass).
    pub(crate) corridors: Vec<Corridor>,
    /// Endpoints (quantized positions) already wired by a tree, per sheet
    /// net index: no labels for them.
    pub(crate) group_wired: BTreeMap<usize, BTreeSet<(i64, i64)>>,
    /// Per-net collapsed endpoints (stacked-pin buses emitted once).
    collapsed: BTreeMap<usize, Vec<Endpoint>>,
    /// Set while wiring an ANALOG net: its continuous wire may cross a
    /// *predicted* power corridor (the actual power stub, routed later,
    /// re-plans around the committed wire) — the uninterrupted analog wire
    /// takes precedence over the conservative keepout.
    pub(crate) analog_wiring: bool,
    /// Best-effort annotation labels for full-coverage analog trees, deferred
    /// until every analog tree is wired: `(net name, first tree segment index)`.
    /// Placing them last keeps a net's naming label from walling off a sibling
    /// leg's continuous wire (a differential shunt drop must be free to cross
    /// under where the label would otherwise sit).
    pub(crate) pending_annotations: Vec<(String, usize)>,
    /// Bounding box of the relegated `PWR_FLAG` band (set by `relegate_flags`),
    /// consumed by `compute_zones` to outline the ERC/utility area.
    flag_region: Option<BBox>,
}

/// Grow an optional accumulator box by another box (union, seeding on first).
fn union_opt(acc: &mut Option<BBox>, b: &BBox) {
    match acc {
        Some(c) => c.union(b),
        None => *acc = Some(*b),
    }
}

/// Route one placed sheet. `flag_nets` = undriven rails whose PWR_FLAG this
/// sheet carries. Also places the instance texts (the model records the
/// final anchors).
pub fn route_sheet(
    design: &DesignModel,
    plan: &SheetPlan,
    model: &mut SheetModel,
    cfg: &SchConfig,
    flag_nets: &BTreeSet<String>,
    warnings: &mut Vec<String>,
) -> RoutedSheet {
    // Texts first: their boxes become keepouts for everything wired next.
    let artifacts = crate::texts::place_instance_texts(cfg, design, model, warnings);
    let mut router = Router {
        cfg,
        design,
        model,
        plan,
        reg: Reg::new(artifacts.label_boxes),
        out: RoutedSheet::default(),
        warnings,
        port_anchored: BTreeSet::new(),
        corridors: artifacts.corridors,
        group_wired: BTreeMap::new(),
        collapsed: BTreeMap::new(),
        analog_wiring: false,
        pending_annotations: Vec::new(),
        flag_region: None,
    };
    router.init_registry();
    router.route_signals();
    router.route_power(flag_nets);
    router.mark_no_connects();
    router.place_blocks();
    router.place_port_fallbacks();
    // Soft pass: pull sibling ports / net labels onto a shared X column. Runs
    // after every label is placed and before the zones are outlined so the
    // functional cell encloses the final (aligned) label positions.
    router.align_sibling_ports();
    // Outline the functional / decoupling / ERC zones once every element
    // (including the relegated flags placed by `route_power`) is positioned.
    router.compute_zones();
    router.compute_extents();
    router.out
}

impl<'a> Router<'a> {
    fn init_registry(&mut self) {
        for (pi, p) in self.model.placed.iter().enumerate() {
            let geom = &self.design.comps[p.comp].geom;
            let mut seen: BTreeSet<(i64, i64)> = BTreeSet::new();
            for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
                let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                    continue;
                };
                if !seen.insert(quant(pos)) {
                    continue;
                }
                let net = self
                    .model
                    .pin_net
                    .get(&(pi, pin.number.clone()))
                    .map(|&sn| self.model.nets[sn].name.clone())
                    .unwrap_or_else(|| format!("~nc~{pi}.{}", pin.number));
                self.reg.pins.push((pos.0, pos.1, net));
            }
        }
    }

    /// Endpoints of a sheet net, deduplicated by position.
    fn net_endpoints(&self, net: &SheetNet) -> Vec<Endpoint> {
        let mut out: Vec<Endpoint> = Vec::new();
        let mut seen: BTreeSet<(i64, i64)> = BTreeSet::new();
        for (pi, pad) in &net.endpoints {
            let p = &self.model.placed[*pi];
            let geom = &self.design.comps[p.comp].geom;
            let Some(pin) = geom.pin(pad) else { continue };
            if pin.hidden {
                continue;
            }
            let Some(pos) = geom.pin_position(pad, p.at, p.rotation, p.mirror) else {
                continue;
            };
            if !seen.insert(quant(pos)) {
                continue;
            }
            let Some(dir) = geom.pin_outward(pad, p.rotation, p.mirror) else {
                continue;
            };
            out.push(Endpoint {
                placed: *pi,
                pad: pad.clone(),
                pos,
                dir,
            });
        }
        out
    }

    /// Cached, collapsed endpoints of a signal net (the stacked-pin bus
    /// wires are emitted exactly once, on first use).
    fn eps_for(&mut self, sn: usize) -> Vec<Endpoint> {
        if let Some(cached) = self.collapsed.get(&sn) {
            return cached.clone();
        }
        let net = &self.model.nets[sn];
        let name = net.name.clone();
        let eps = self.net_endpoints(net);
        let eps = self.collapse_stacked_pins(eps, &name, true, false);
        self.collapsed.insert(sn, eps.clone());
        eps
    }

    /// Collapse stacks of adjacent same-net pins of one component (multi
    /// drain/source packages) into a single endpoint: a straight bus wire
    /// runs along the pin column/row (every pin end lies on the wire, which
    /// connects in KiCad without junctions) and only the extreme pin keeps
    /// a label/power attachment. `extreme_up` picks the top/left pin of the
    /// stack (power rails), otherwise the bottom/right one (grounds).
    ///
    /// For `power` nets a horizontal stack prefers an *offset* bus — every pin
    /// taps out straight for at least `min_pin_exit_steps` grid steps before
    /// meeting the shared bus column (rule #1), and the returned representative
    /// caps that column with its power symbol — whenever the component edge is
    /// clear enough for it; otherwise it falls back to the through-tip bus (an
    /// edge that interleaves a foreign rail keeps the tight in-column bus).
    fn collapse_stacked_pins(
        &mut self,
        eps: Vec<Endpoint>,
        net: &str,
        extreme_up: bool,
        power: bool,
    ) -> Vec<Endpoint> {
        // Group by (component, outward direction, aligned coordinate).
        let mut groups: BTreeMap<(usize, i8, i8, i64), Vec<Endpoint>> = BTreeMap::new();
        for ep in eps {
            let horiz = ep.dir.0.abs() > 0.5;
            let fixed = if horiz { ep.pos.0 } else { ep.pos.1 };
            groups
                .entry((
                    ep.placed,
                    ep.dir.0.signum() as i8,
                    ep.dir.1.signum() as i8,
                    (fixed * 10000.0).round() as i64,
                ))
                .or_default()
                .push(ep);
        }
        let mut out: Vec<Endpoint> = Vec::new();
        for ((_, dx, _, _), mut members) in groups {
            let horiz = dx != 0;
            let key = |e: &Endpoint| if horiz { e.pos.1 } else { e.pos.0 };
            members.sort_by(|a, b| {
                key(a)
                    .partial_cmp(&key(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            // Split on gaps larger than two grid steps (not a stack).
            let mut runs: Vec<Vec<Endpoint>> = Vec::new();
            for ep in members {
                match runs.last_mut() {
                    Some(run) if key(&ep) - key(run.last().expect("non-empty")) <= 5.081 => {
                        run.push(ep)
                    }
                    _ => runs.push(vec![ep]),
                }
            }
            for run in runs {
                if run.len() < 2 {
                    out.extend(run);
                    continue;
                }
                // Power/ground stack on a side edge: try the offset bus first so
                // every pin leaves straight for >= min_pin_exit steps (rule #1).
                if power
                    && horiz
                    && let Some(rep) = self.try_offset_power_stack(&run, net, dx, extreme_up)
                {
                    out.push(rep);
                    continue;
                }
                // One segment per pin pair: a KiCad pin only connects to a
                // wire END, never to a wire interior.
                let points: Vec<Point> = run.iter().map(|e| e.pos).collect();
                self.out.wires.push(points.clone());
                self.reg.register_path(&points, net);
                let rep = if extreme_up {
                    run.into_iter().next().expect("non-empty")
                } else {
                    run.into_iter().next_back().expect("non-empty")
                };
                out.push(rep);
            }
        }
        out
    }

    /// Route a horizontal power/ground stack onto an offset bus: every pin taps
    /// straight out for at least `min_pin_exit_steps` grid steps to a shared
    /// column, and the returned representative caps that column with a vertical
    /// power-symbol leg. Returns `None` (the caller keeps the through-tip bus)
    /// when a foreign rail is interleaved on the same edge inside the band the
    /// bus and symbol grow into — that later rail could only clear the offset
    /// bus by shorting onto it — or when no clean offset column exists. On
    /// success the tap + bus wires and their junctions are committed. `run` is
    /// sorted by ascending pin ordinate; `dx` is the pins' outward sign.
    fn try_offset_power_stack(
        &mut self,
        run: &[Endpoint],
        net: &str,
        dx: i8,
        extreme_up: bool,
    ) -> Option<Endpoint> {
        let down = !extreme_up;
        let g = self.cfg.grid_mm;
        let stub = self.cfg.power_stub_mm;
        let min_exit = self.cfg.min_pin_exit_mm();
        let px = run[0].pos.0;
        let ymin = run[0].pos.1;
        let ymax = run[run.len() - 1].pos.1;
        // A foreign pin on this same edge column, inside the band the bus and
        // its symbol grow into, forbids the offset (it would be walled off).
        let (band_lo, band_hi) = if down {
            (ymin - EPS, ymax + stub + EPS)
        } else {
            (ymin - stub - EPS, ymax + EPS)
        };
        for (pxx, pyy, pnet) in &self.reg.pins {
            if pnet != net && (pxx - px).abs() < EPS && *pyy > band_lo && *pyy < band_hi {
                return None;
            }
        }
        let placed_excl: Vec<usize> = run.iter().map(|e| e.placed).collect();
        let extreme_y = if down { ymax } else { ymin };
        for step in 0..=8 {
            let bus_x = round4(px + dx as f64 * (min_exit + step as f64 * g));
            let taps: Vec<Vec<Point>> = run.iter().map(|e| vec![e.pos, (bus_x, e.pos.1)]).collect();
            let bus = vec![(bus_x, ymin), (bus_x, ymax)];
            let sym_y = if down {
                round4(ymax + stub)
            } else {
                round4(ymin - stub)
            };
            let gbox = power_symbol_graphic_box(net, (bus_x, sym_y), down);
            let clean = taps.iter().all(|t| self.path_ok(t, net, &placed_excl))
                && self.path_ok(&bus, net, &placed_excl)
                && self.graphic_box_clear(&gbox, net);
            if !clean {
                continue;
            }
            for t in &taps {
                self.out.wires.push(t.clone());
                self.reg.register_path(t, net);
            }
            self.out.wires.push(bus.clone());
            self.reg.register_path(&bus, net);
            for e in run {
                let j = (bus_x, e.pos.1);
                if self.junction_needed_at(j, net) {
                    self.push_junction(j);
                }
            }
            let rep0 = if extreme_up {
                &run[0]
            } else {
                &run[run.len() - 1]
            };
            return Some(Endpoint {
                placed: rep0.placed,
                pad: rep0.pad.clone(),
                pos: (bus_x, extreme_y),
                dir: (0.0, if down { 1.0 } else { -1.0 }),
            });
        }
        None
    }

    pub(crate) fn path_ok(&self, points: &[Point], net: &str, exclude: &[usize]) -> bool {
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg = Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: net.to_string(),
            };
            for (px, py, pnet) in &self.reg.pins {
                if pnet != net && point_on_seg(*px, *py, &seg) {
                    return false;
                }
            }
            for other in &self.reg.segs {
                if other.net != net && segs_touch_forbidden(&seg, other) {
                    return false;
                }
            }
            for (gb, gnet) in &self.reg.power_boxes {
                if gnet != net && seg_intersects_box(&seg, gb) {
                    return false;
                }
            }
            for lb in &self.reg.label_boxes {
                if !lb.allows(net) && seg_intersects_box(&seg, &lb.bbox) {
                    return false;
                }
            }
            // Segment through a third-party body?
            let sweep = BBox {
                x1: seg.x1.min(seg.x2) + EPS,
                y1: seg.y1.min(seg.y2) + EPS,
                x2: seg.x1.max(seg.x2) - EPS,
                y2: seg.y1.max(seg.y2) - EPS,
            };
            for (pi, p) in self.model.placed.iter().enumerate() {
                if exclude.contains(&pi) {
                    continue;
                }
                let rb = raw_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
                if overlaps(&rb, &sweep) {
                    return false;
                }
            }
        }
        true
    }

    /// Does this candidate path keep at least `min_net_spacing_steps` grid
    /// steps clear of every PARALLEL foreign-net wire already committed? Two
    /// same-orientation segments of different nets that overlap along their
    /// shared axis must sit at least that far apart on the perpendicular axis
    /// — the engineer's "two grid steps between nets by default, like out of
    /// power" rule, so parallel columns/rows read on one harmonized grid
    /// instead of hugging each other at a single step. Perpendicular (frank)
    /// crossings are untouched: they are electrically harmless and handled by
    /// the crossing-penalty pass. A zero perpendicular offset (collinear,
    /// same-axis overlap) is a forbidden contact handled by `path_ok`, not
    /// here. Disabled when the knob is zero.
    pub(crate) fn parallel_clear(&self, points: &[Point], net: &str) -> bool {
        let min_spacing = self.cfg.min_net_spacing_mm();
        if min_spacing <= EPS {
            return true;
        }
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let s_h = (w[0].1 - w[1].1).abs() < EPS;
            for other in &self.reg.segs {
                if other.net == net {
                    continue;
                }
                let o_h = (other.y1 - other.y2).abs() < EPS;
                if s_h != o_h {
                    continue; // perpendicular: not a parallel run
                }
                if s_h {
                    let dy = (w[0].1 - other.y1).abs();
                    if dy < EPS || dy >= min_spacing - EPS {
                        continue;
                    }
                    let lo = w[0].0.min(w[1].0).max(other.x1.min(other.x2));
                    let hi = w[0].0.max(w[1].0).min(other.x1.max(other.x2));
                    if lo < hi - EPS {
                        return false;
                    }
                } else {
                    let dx = (w[0].0 - other.x1).abs();
                    if dx < EPS || dx >= min_spacing - EPS {
                        continue;
                    }
                    let lo = w[0].1.min(w[1].1).max(other.y1.min(other.y2));
                    let hi = w[0].1.max(w[1].1).min(other.y1.max(other.y2));
                    if lo < hi - EPS {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// May a label text live here without covering anything foreign?
    /// `strict` also rejects overlap with same-net texts (two homonym
    /// stacked labels read as one and hide the second wire).
    pub(crate) fn label_box_clear(&self, bbox: &BBox, net: &str, strict: bool) -> bool {
        for p in self.model.placed.iter() {
            let rb = raw_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
            if overlaps(&rb, bbox) {
                return false;
            }
        }
        for (px, py, pnet) in &self.reg.pins {
            if pnet == net {
                continue;
            }
            if *px > bbox.x1 + EPS
                && *px < bbox.x2 - EPS
                && *py > bbox.y1 + EPS
                && *py < bbox.y2 - EPS
            {
                return false;
            }
        }
        for seg in &self.reg.segs {
            if seg.net != net && seg_intersects_box(seg, bbox) {
                return false;
            }
        }
        for (gb, gnet) in &self.reg.power_boxes {
            if gnet != net && overlaps(gb, bbox) {
                return false;
            }
        }
        for lb in &self.reg.label_boxes {
            if (strict || !lb.allows(net)) && overlaps(&lb.bbox, bbox) {
                return false;
            }
        }
        // Predicted power corridors: a label on the pin-to-symbol path of a
        // rail would doom every dogleg candidate. Same net tolerated; the
        // `Graphic` zones only apply to instance texts (the symbol knows
        // how to stretch away from a label).
        for c in &self.corridors {
            if c.zone == CorridorZone::Corridor && c.net != net && overlaps(&c.bbox, bbox) {
                return false;
            }
        }
        true
    }

    // --------------------------------------------------------------
    // Signals
    // --------------------------------------------------------------

    /// Reserve the label lane of a design-wide single-endpoint signal net (a
    /// boundary net that must carry a global label right off its one pin) as a
    /// keepout, BEFORE any net is wired — but only when a neighbour pin on the
    /// SAME component edge carries a wire-bearing (multi-endpoint) signal net
    /// that could box this port in. A multi-endpoint analog net routed first
    /// would otherwise run a long continuous wire straight across the boxed-in
    /// port's only escape, forcing the port label onto that wire (a short) or
    /// across it (a frank crossing). Reserving the lane keeps foreign wires off
    /// it, so the neighbour net routes clear or stays on labels — the engineer's
    /// "leave the boundary port its lane" rule, and the principled replacement
    /// for the old length cutoff (length never gates a wire; boxing in a
    /// stacked port's escape does). The trigger is deliberately narrow so a
    /// port with only quiet neighbours keeps the sheet untouched.
    fn reserve_boundary_lanes(&mut self) {
        let half = self.cfg.grid_mm / 2.0;
        for sn in 0..self.model.nets.len() {
            if self.model.nets[sn].class != NetClass::Signal
                || self.model.nets[sn].design_endpoints > 1
            {
                continue;
            }
            let name = self.model.nets[sn].name.clone();
            let eps = self.eps_for(sn);
            let Some(ep) = eps.first() else { continue };
            if !self.pin_boxed_by_wiring_neighbor(ep) {
                continue;
            }
            let len = label_stub_len(self.cfg, &name);
            let (px, py) = ep.pos;
            let bbox = if ep.dir.0.abs() > 0.5 {
                let xe = px + ep.dir.0 * len;
                BBox {
                    x1: px.min(xe),
                    y1: py - half,
                    x2: px.max(xe),
                    y2: py + half,
                }
            } else {
                let ye = py + ep.dir.1 * len;
                BBox {
                    x1: px - half,
                    y1: py.min(ye),
                    x2: px + half,
                    y2: py.max(ye),
                }
            };
            self.reg.label_boxes.push(LabelBox {
                bbox,
                net: name,
                also: Vec::new(),
            });
        }
    }

    /// Does a pin adjacent to `ep` on the same component (within roughly one
    /// pin pitch) carry a DIFFERENT wire-bearing (multi-endpoint) signal net?
    /// Such a neighbour is what boxes a single-endpoint boundary port in when it
    /// wires as a continuous drop past the port's stacked exit.
    fn pin_boxed_by_wiring_neighbor(&self, ep: &Endpoint) -> bool {
        let window = 2.0 * self.cfg.grid_mm + EPS;
        let p = &self.model.placed[ep.placed];
        let geom = &self.design.comps[p.comp].geom;
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                continue;
            };
            if (pos.0 - ep.pos.0).abs() < EPS && (pos.1 - ep.pos.1).abs() < EPS {
                continue; // the port pin itself
            }
            if (pos.0 - ep.pos.0).abs() + (pos.1 - ep.pos.1).abs() > window {
                continue; // not an immediate neighbour on the edge
            }
            if let Some(&osn) = self.model.pin_net.get(&(ep.placed, pin.number.clone()))
                && self.model.nets[osn].class == NetClass::Signal
                && self.model.nets[osn].design_endpoints > 1
            {
                return true;
            }
        }
        false
    }

    fn route_signals(&mut self) {
        self.reserve_boundary_lanes();
        // Port nets last: their hier-label stubs know how to stretch,
        // plain nets' elbows only have a dozen candidates near the body.
        let mut order: Vec<usize> = (0..self.model.nets.len())
            .filter(|&sn| self.model.nets[sn].class == NetClass::Signal)
            .collect();
        order.sort_by_key(|&sn| (self.model.nets[sn].port.is_some(), sn));

        // Facing pairs are wired LAST (top to bottom): a direct wire has
        // one channel while labels and doglegs have dozens of candidates.
        let mut pair_nets: Vec<usize> = Vec::new();
        let mut label_nets: Vec<usize> = Vec::new();
        for sn in order {
            if is_pair_net_static(
                self.cfg,
                self.design,
                &self.model.placed,
                &self.model.nets,
                sn,
            ) {
                pair_nets.push(sn);
                continue;
            }
            self.plan_and_wire_group(sn);
            label_nets.push(sn);
        }
        // Every analog tree is now committed: place the deferred full-coverage
        // annotation labels in the space the interleaved wires left free (a
        // label placed earlier could have blocked a sibling leg's crossing).
        let pending = std::mem::take(&mut self.pending_annotations);
        for (name, tree_seg_start) in pending {
            self.place_tree_net_label(&name, tree_seg_start, false);
        }
        for sn in label_nets {
            self.wire_signal_net(sn);
        }
        let mut top_y: BTreeMap<usize, i64> = BTreeMap::new();
        for &sn in &pair_nets {
            let eps = self.eps_for(sn);
            let min = eps
                .iter()
                .map(|e| (e.pos.1 * 10000.0).round() as i64)
                .min()
                .unwrap_or(i64::MAX);
            top_y.insert(sn, min);
        }
        pair_nets.sort_by_key(|&sn| (top_y[&sn], self.model.nets[sn].name.clone()));
        for sn in pair_nets {
            self.wire_signal_net(sn);
        }
    }

    /// Label pass of one signal net: direct wire for facing pairs, then
    /// stub + label for every endpoint not already wired by a tree; the
    /// hierarchical label of a port net anchors the stub of a preferred
    /// endpoint, the others keep homonym net labels.
    fn wire_signal_net(&mut self, sn: usize) {
        let eps = self.eps_for(sn);
        if eps.is_empty() {
            return;
        }
        if is_pair_net_static(
            self.cfg,
            self.design,
            &self.model.placed,
            &self.model.nets,
            sn,
        ) && self.try_direct_wire(sn, &eps)
        {
            return;
        }
        let name = self.model.nets[sn].name.clone();
        let single = self.model.nets[sn].design_endpoints <= 1;
        let port = self.model.nets[sn].port;
        let is_root = self.plan.sheets[self.model.sheet].parent.is_none();
        let mut rest: Vec<Endpoint> = match self.group_wired.get(&sn) {
            Some(wired) => eps
                .into_iter()
                .filter(|e| !wired.contains(&quant(e.pos)))
                .collect(),
            None => eps,
        };
        if rest.is_empty() {
            return;
        }
        // Promote a small root-sheet net to GLOBAL labels when a local label on
        // one of its endpoints would collide — no clean placement exists and
        // the short off-the-end fallback lands on a foreign net. A global label
        // is a self-contained hexagon centred on the pin row, so it fits a
        // crowded IC edge where floating local text cannot (the engineer's
        // AD7171 SPI_MISO on DOUT/RDY, boxed between AIN- and its own filter
        // column at the 2-step pin pitch — the reference layout draws it global
        // too). The scope is deliberately tight: only a point-to-point net (two
        // endpoints), where every endpoint reads cleanly as the same global net
        // — a widely fanned bus keeps its distributed local labels, one cramped
        // tap not being worth flipping the whole net. Net names are unique per
        // design, so a name only connects to its own net: the promotion is a
        // pure label-style change, never a netlist change. Confined to the root
        // sheet (the design boundary); sub-sheet port nets keep the
        // hierarchical-label anchor + local-homonym mechanism.
        let cramped = is_root
            && port.is_none()
            && self.model.nets[sn].design_endpoints == 2
            && rest.iter().any(|ep| self.local_label_collides(&name, ep));
        let use_global = single || cramped;
        if let Some(direction) = port
            && !is_root
            && !self.port_anchored.contains(&name)
            && let Some(anchored) = self.anchor_port(&name, direction, &rest)
        {
            rest.retain(|e| quant(e.pos) != quant(anchored.pos));
        }
        // Rule #1: a point-to-point signal net must carry a single boundary
        // label. When its two endpoints scattered into a pair of homonym labels
        // (no crossing-free tree joined them), keep one label and wire the other
        // onto it — the pull-up drops straight onto its pin across the analog
        // inputs, exactly as the reference layout draws it — instead of
        // duplicating the name. The scope is deliberately tight, matching the
        // cramped-promotion rule it supersedes: only a two-endpoint net (a wider
        // fan-out is a bus, best read as distributed labels), and never one that
        // lands on a hub (a many-pin part fans its bus out to labels by design).
        let two_endpoint = self.model.nets[sn].design_endpoints == 2;
        let touches_hub = rest
            .iter()
            .any(|ep| is_hub_comp(self.cfg, self.design, self.model.placed[ep.placed].comp));
        if self.cfg.dedup_signal_labels
            && port.is_none()
            && two_endpoint
            && rest.len() >= 2
            && !touches_hub
        {
            rest = self.dedup_signal_net(&name, rest);
            if rest.is_empty() {
                return;
            }
        }
        for ep in rest {
            self.emit_label_stub(&name, &ep, use_global);
        }
    }

    /// De-duplicate a scattered signal net onto a single label (Rule #1). Every
    /// endpoint would otherwise carry its own homonym label; instead one
    /// endpoint keeps the label (the module-boundary marker) and each OTHER
    /// endpoint is wired to it as a continuous wire. Foreign frank crossings are
    /// tolerated on those joins: they are electrically harmless and are the only
    /// way a pull-up standing above the analog inputs can drop onto its pin
    /// (the engineer's AD7171 R3 → DOUT/RDY across AIN+/AIN-). The wiring never
    /// moves a netlist node, so the exported `(ref.pin)` partition is unchanged.
    /// The surviving label is a GLOBAL port (a self-contained boundary hexagon,
    /// the module's I/O marker) — the reference AD7171 draws SPI_MISO exactly so.
    /// Returns the endpoints that still need their own label (empty when every
    /// other endpoint was wired); on zero progress the attempt is rolled back
    /// and the original `rest` is returned so per-endpoint labelling is
    /// byte-for-byte unchanged.
    fn dedup_signal_net(&mut self, name: &str, rest: Vec<Endpoint>) -> Vec<Endpoint> {
        // Label carrier: the clearest port endpoint — a horizontal pin (straight
        // stub label) on the busiest component (the IC output, not a two-pin
        // pull), deterministic by position.
        let rep = (0..rest.len())
            .min_by(|&a, &b| {
                let key = |e: &Endpoint| {
                    let horiz = i32::from(e.dir.1.abs() >= 0.5);
                    let pins = self.design.comps[self.model.placed[e.placed].comp].visible_pins;
                    (
                        horiz,
                        -(pins as i64),
                        (e.pos.0 * 1000.0) as i64,
                        (e.pos.1 * 1000.0) as i64,
                    )
                };
                key(&rest[a]).cmp(&key(&rest[b]))
            })
            .expect("rest is non-empty");

        let undo = self.snapshot_wiring();
        let start = self.reg.segs.len();
        // The one surviving label is the module-boundary port: a global hexagon.
        self.emit_label_stub(name, &rest[rep], true);

        // Tolerate the harmless frank crossings the drops need to reach the pin.
        let prev_analog = self.analog_wiring;
        self.analog_wiring = true;
        let mut wired: Vec<Endpoint> = vec![rest[rep].clone()];
        let mut unwired: Vec<Endpoint> = Vec::new();
        let mut joined = 0usize;
        for (i, ep) in rest.iter().enumerate() {
            if i == rep {
                continue;
            }
            let segs: Vec<(f64, f64, f64, f64)> = self.reg.segs[start..]
                .iter()
                .filter(|s| s.net == name)
                .map(|s| (s.x1, s.y1, s.x2, s.y2))
                .collect();
            let mut best: Option<(f64, Vec<Point>, Point)> = None;
            for (path, join) in self.tree_join_candidates(ep, &segs, &wired) {
                if !self.tree_candidate_feasible(&path, name) {
                    continue;
                }
                let cost = self.wire_cost(&path, name);
                if best.as_ref().is_none_or(|(bc, _, _)| cost < *bc - EPS) {
                    best = Some((cost, path, join));
                }
            }
            if let Some((_, path, join)) = best {
                let need_junction = self.junction_needed_at(join, name);
                self.reg.register_path(&path, name);
                self.out.wires.push(path);
                if need_junction {
                    self.push_junction(join);
                }
                wired.push(ep.clone());
                joined += 1;
            } else {
                unwired.push(ep.clone());
            }
        }
        self.analog_wiring = prev_analog;

        if joined == 0 {
            // No endpoint could join the labelled stub — nothing gained. Restore
            // the pre-dedup state so the caller labels every endpoint as before.
            self.rollback_wiring(&undo);
            return rest;
        }
        unwired
    }

    /// Would a LOCAL label on this endpoint be forced onto the short
    /// off-the-end fallback AND land on top of a foreign net there? Read-only
    /// mirror of the placement search in `emit_label_stub`. It returns true
    /// only for a genuinely *bad* cramp: no clean placement exists (no
    /// crossing-free straight stub for a horizontal pin, no routable elbow for
    /// a vertical one) AND the fallback the emitter would then commit overlaps
    /// a foreign net's wire/label/symbol. A hub's signal label that simply
    /// reads outward into open air is NOT flagged — its short stub is by
    /// design. This is the AD7171 SPI_MISO on DOUT/RDY, whose only fallback
    /// jams its text across the AIN- filter column: such a net is better as a
    /// self-contained global-label hexagon.
    fn local_label_collides(&self, name: &str, ep: &Endpoint) -> bool {
        let cfg = self.cfg;
        if ep.dir.1.abs() > 0.5 {
            let stub0 = cfg.stub_mm + 2.54;
            let elbow0 = cfg.label_elbow_mm.max(label_text_width(name));
            for stub in [stub0, stub0 + 2.54, cfg.stub_mm] {
                for side in [1.0, -1.0] {
                    for elbow in [elbow0, elbow0 + 2.54] {
                        let knee = (ep.pos.0, round4(ep.pos.1 + ep.dir.1 * stub));
                        let end = (round4(knee.0 + side * cfg.snap_up(elbow)), knee.1);
                        if self.path_ok(&[ep.pos, knee, end], name, &[ep.placed]) {
                            return false; // a clean elbow placement exists
                        }
                    }
                }
            }
            // Vertical fallback: short straight stub, horizontal label (rot 0).
            let end = (ep.pos.0, round4(ep.pos.1 + ep.dir.1 * 2.54));
            return !self.label_box_clear(&label_text_box(name, end, 0), name, false);
        }
        let base = label_stub_len(cfg, name);
        for extra in [0.0, 2.54, 5.08, 7.62, 10.16, 12.7] {
            let end = (round4(ep.pos.0 + ep.dir.0 * (base + extra)), ep.pos.1);
            let path = vec![ep.pos, end];
            if self.path_ok(&path, name, &[ep.placed]) && self.reg.count_crossings(&path) == 0 {
                return false; // a clean straight-stub placement exists
            }
        }
        // Horizontal fallback: short off-the-end stub, label reading outward.
        let end = (round4(ep.pos.0 + ep.dir.0 * 2.54), ep.pos.1);
        let rot = if ep.dir.0 >= 0.0 { 0 } else { 180 };
        !self.label_box_clear(&label_text_box(name, end, rot), name, false)
    }

    /// Stub + horizontal net label on a pin (name equality connects).
    /// Design-wide single-endpoint nets get a global label instead (a lone
    /// local label raises `isolated_pin_label`).
    fn emit_label_stub(&mut self, name: &str, ep: &Endpoint, global: bool) {
        let cfg = self.cfg;
        if ep.dir.1.abs() > 0.5 {
            // Vertical pin: extended stub + short horizontal elbow.
            let stub0 = cfg.stub_mm + 2.54;
            let elbow0 = cfg.label_elbow_mm.max(label_text_width(name));
            let mut candidates: Vec<(Vec<Point>, Point, i32)> = Vec::new();
            for stub in [stub0, stub0 + 2.54, cfg.stub_mm] {
                for side in [1.0, -1.0] {
                    for elbow in [elbow0, elbow0 + 2.54] {
                        let knee = (ep.pos.0, round4(ep.pos.1 + ep.dir.1 * stub));
                        let end = (round4(knee.0 + side * cfg.snap_up(elbow)), knee.1);
                        // The wire reaches the label from the knee side. A
                        // global label's flag faces that way (text reads outward,
                        // away from the pin); a local net label instead reads
                        // back over its elbow wire (`net_label_rotation`).
                        let outward = if side > 0.0 { 0 } else { 180 };
                        let rotation = net_label_rotation(global, outward);
                        candidates.push((vec![ep.pos, knee, end], end, rotation));
                    }
                }
            }
            // 4-level selection: STRICTLY free box (no text overlap, same
            // net included) > free in the labelBoxClear sense > first
            // placeable zero-crossing path > first placeable path.
            let mut fallback_any: Option<usize> = None;
            let mut fallback: Option<usize> = None;
            let mut relaxed: Option<usize> = None;
            let mut strict: Option<usize> = None;
            for (i, (path, at, rotation)) in candidates.iter().enumerate() {
                if !self.path_ok(path, name, &[ep.placed]) {
                    continue;
                }
                fallback_any.get_or_insert(i);
                if self.reg.count_crossings(path) != 0 {
                    continue;
                }
                fallback.get_or_insert(i);
                let bbox = label_text_box(name, *at, *rotation);
                if relaxed.is_none() && self.label_box_clear(&bbox, name, false) {
                    relaxed = Some(i);
                }
                if self.label_box_clear(&bbox, name, true) {
                    strict = Some(i);
                    break;
                }
            }
            if let Some(i) = strict.or(relaxed).or(fallback).or(fallback_any) {
                let (path, at, rotation) = candidates.swap_remove(i);
                self.commit_label(name, path, at, rotation, global);
                return;
            }
            // Last resort: short stub without elbow, horizontal label.
            let short = (ep.pos.0, round4(ep.pos.1 + ep.dir.1 * 2.54));
            self.commit_label(name, vec![ep.pos, short], short, 0, global);
            return;
        }

        // Horizontal pin: straight stub from the pin end to the label. The
        // label's connection point faces the pin. A global label's flag reads
        // outward off the far end; a local net label reads back over the stub
        // (the wire underlines the full text — `label_stub_len` guarantees the
        // stub is at least as long as the text — and runs on to the pin).
        let base = label_stub_len(cfg, name);
        let outward = if ep.dir.0 >= 0.0 { 0 } else { 180 };
        let rotation = net_label_rotation(global, outward);
        let mut chosen: Option<(Vec<Point>, Point)> = None;
        for extra in [0.0, 2.54, 5.08, 7.62, 10.16, 12.7] {
            let end = (round4(ep.pos.0 + ep.dir.0 * (base + extra)), ep.pos.1);
            let path = vec![ep.pos, end];
            if !self.path_ok(&path, name, &[ep.placed]) {
                continue;
            }
            if self.reg.count_crossings(&path) != 0 {
                continue;
            }
            if chosen.is_none() {
                chosen = Some((path.clone(), end));
            }
            if self.label_box_clear(&label_text_box(name, end, rotation), name, true) {
                chosen = Some((path, end));
                break;
            }
        }
        match chosen {
            Some((path, end)) => self.commit_label(name, path, end, rotation, global),
            None => {
                // Cramped fallback: the stub is too short to underline the
                // text, so the label keeps its outward (off-the-end) reading —
                // a local net label cannot overhang a wire that short.
                let end = (round4(ep.pos.0 + ep.dir.0 * 2.54), ep.pos.1);
                let rot = if ep.dir.0 >= 0.0 { 0 } else { 180 };
                self.commit_label(name, vec![ep.pos, end], end, rot, global);
            }
        }
    }

    fn commit_label(
        &mut self,
        name: &str,
        path: Vec<Point>,
        at: Point,
        rotation: i32,
        global: bool,
    ) {
        self.reg.register_path(&path, name);
        if path.len() >= 2 && dist(path[0], *path.last().unwrap()) > EPS {
            self.out.wires.push(path);
        }
        self.reg.label_boxes.push(LabelBox {
            bbox: label_text_box(name, at, rotation),
            net: name.to_string(),
            also: Vec::new(),
        });
        if global {
            self.out.global_labels.push((
                name.to_string(),
                at,
                rotation,
                PortDirection::Bidirectional,
            ));
        } else {
            self.out.net_labels.push((name.to_string(), at, rotation));
        }
    }

    // --------------------------------------------------------------
    // Hierarchical port anchors
    // --------------------------------------------------------------

    /// Anchor a port net's hierarchical label at the end of the stub of a
    /// preferred internal endpoint. Returns the anchored endpoint.
    fn anchor_port(
        &mut self,
        name: &str,
        direction: PortDirection,
        eps: &[Endpoint],
    ) -> Option<Endpoint> {
        if eps.is_empty() {
            return None;
        }
        let order = self.port_anchor_order(direction, eps);

        let mut relaxed: Option<(usize, Vec<Point>, Point, i32)> = None;
        let mut fallback: Option<(usize, Vec<Point>, Point, i32)> = None;
        let mut fallback_any: Option<(usize, Vec<Point>, Point, i32)> = None;
        for &ei in &order {
            let ep = &eps[ei];
            for (path, at, rotation) in self.hier_stub_candidates(name, ep) {
                if !self.path_ok(&path, name, &[ep.placed]) {
                    continue;
                }
                if fallback_any.is_none() {
                    fallback_any = Some((ei, path.clone(), at, rotation));
                }
                if self.reg.count_crossings(&path) != 0 {
                    continue;
                }
                if fallback.is_none() {
                    fallback = Some((ei, path.clone(), at, rotation));
                }
                let bbox = hier_text_box(name, at, rotation);
                if self.label_box_clear(&bbox, name, true) {
                    return Some(self.commit_hier(name, direction, ei, path, at, rotation, eps));
                }
                if relaxed.is_none() && self.label_box_clear(&bbox, name, false) {
                    relaxed = Some((ei, path.clone(), at, rotation));
                }
            }
        }
        let choice = relaxed.or(fallback).or(fallback_any).or_else(|| {
            // The connection prevails: shortest candidate of the preferred
            // endpoint, placed as is.
            let ei = order[0];
            self.hier_stub_candidates(name, &eps[ei])
                .into_iter()
                .next()
                .map(|(path, at, rotation)| (ei, path, at, rotation))
        });
        choice.map(|(ei, path, at, rotation)| {
            self.commit_hier(name, direction, ei, path, at, rotation, eps)
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn commit_hier(
        &mut self,
        name: &str,
        direction: PortDirection,
        ei: usize,
        path: Vec<Point>,
        at: Point,
        rotation: i32,
        eps: &[Endpoint],
    ) -> Endpoint {
        self.reg.register_path(&path, name);
        self.out.wires.push(path);
        self.out
            .hier_labels
            .push((name.to_string(), direction, at, rotation));
        self.reg.label_boxes.push(LabelBox {
            bbox: hier_text_box(name, at, rotation),
            net: name.to_string(),
            also: Vec::new(),
        });
        self.port_anchored.insert(name.to_string());
        eps[ei].clone()
    }

    /// Candidate stubs carrying a hierarchical label, shortest first.
    pub(crate) fn hier_stub_candidates(
        &self,
        name: &str,
        ep: &Endpoint,
    ) -> Vec<(Vec<Point>, Point, i32)> {
        let cfg = self.cfg;
        let mut out = Vec::new();
        if ep.dir.1.abs() > 0.5 {
            let stub0 = cfg.stub_mm + 2.54;
            for stub in [stub0, stub0 + 2.54, cfg.stub_mm] {
                for side in [1.0, -1.0] {
                    for elbow in [cfg.label_elbow_mm + 2.54, cfg.label_elbow_mm + 5.08] {
                        let knee = (ep.pos.0, round4(ep.pos.1 + ep.dir.1 * stub));
                        let end = (round4(knee.0 + side * elbow), knee.1);
                        let rotation = if side > 0.0 { 0 } else { 180 };
                        out.push((vec![ep.pos, knee, end], end, rotation));
                    }
                }
            }
            return out;
        }
        let base = label_stub_len(cfg, name);
        let rotation = if ep.dir.0 >= 0.0 { 0 } else { 180 };
        for extra in [0.0, 2.54, 5.08, 7.62, 10.16, 12.7] {
            let end = (round4(ep.pos.0 + ep.dir.0 * (base + extra)), ep.pos.1);
            out.push((vec![ep.pos, end], end, rotation));
        }
        out
    }

    /// Preference order of the endpoints for a port anchor: flow-matching
    /// side first (input -> leftmost left-pointing stub, output -> rightmost
    /// right-pointing), then against-flow horizontals, then vertical pins.
    pub(crate) fn port_anchor_order(
        &self,
        direction: PortDirection,
        eps: &[Endpoint],
    ) -> Vec<usize> {
        let mut horiz: Vec<usize> = (0..eps.len())
            .filter(|&i| eps[i].dir.1.abs() < 0.5)
            .collect();
        let mut vert: Vec<usize> = (0..eps.len())
            .filter(|&i| eps[i].dir.1.abs() >= 0.5)
            .collect();
        let by_x = |sign: f64| {
            move |a: &usize, b: &usize| {
                let d = sign * (eps[*a].pos.0 - eps[*b].pos.0);
                d.partial_cmp(&0.0).unwrap_or(std::cmp::Ordering::Equal)
            }
        };
        match direction {
            PortDirection::Input => {
                horiz.sort_by(by_x(1.0));
                vert.sort_by(by_x(1.0));
                let (mut with_flow, against): (Vec<usize>, Vec<usize>) =
                    horiz.into_iter().partition(|&i| eps[i].dir.0 < 0.0);
                with_flow.extend(against);
                with_flow.extend(vert);
                with_flow
            }
            PortDirection::Output => {
                horiz.sort_by(by_x(-1.0));
                vert.sort_by(by_x(-1.0));
                let (mut with_flow, against): (Vec<usize>, Vec<usize>) =
                    horiz.into_iter().partition(|&i| eps[i].dir.0 > 0.0);
                with_flow.extend(against);
                with_flow.extend(vert);
                with_flow
            }
            PortDirection::Bidirectional => {
                // The most "outward" endpoint (stub pointing away from the
                // content center) is the clearest.
                let (min_x, max_x) = self
                    .model
                    .placed
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                        (lo.min(p.bbox.x1), hi.max(p.bbox.x2))
                    });
                let mid = (min_x + max_x) / 2.0;
                horiz.sort_by(|a, b| {
                    let oa = eps[*a].dir.0 * (eps[*a].pos.0 - mid);
                    let ob = eps[*b].dir.0 * (eps[*b].pos.0 - mid);
                    ob.partial_cmp(&oa).unwrap_or(std::cmp::Ordering::Equal)
                });
                vert.sort_by(by_x(1.0));
                horiz.extend(vert);
                horiz
            }
        }
    }

    // --------------------------------------------------------------
    // Power
    // --------------------------------------------------------------

    /// Serve a bank of same-net power pins on one IC side edge with a SINGLE
    /// shared symbol. Each pin stubs out to a common bus column, a short
    /// vertical bus joins them and one symbol caps the extreme end (top for a
    /// rail, bottom for a ground). This is the engineer's "group the two GND
    /// pins / VDD+REFIN+ under one symbol" rule for pins that are NOT contiguous
    /// (a foreign pin sits between them, so `collapse_stacked_pins` cannot run a
    /// bus along the pin column). The bus is only committed when every segment
    /// is electrically clean (foreign contacts forbidden, frank crossings of
    /// intervening stubs tolerated); otherwise the pins fall back to one symbol
    /// each. Returns the endpoint indices served.
    fn merge_power_banks(
        &mut self,
        name: &str,
        eps: &[Endpoint],
        down: bool,
        flag_nets: &BTreeSet<String>,
        flagged: &mut BTreeSet<String>,
    ) -> BTreeSet<usize> {
        let mut served: BTreeSet<usize> = BTreeSet::new();
        // Bank = same component, same horizontal outward direction, same pin
        // column (one IC side edge). Vertical-pin banks are left to the
        // per-pin pass (top/bottom edges rarely stack a split rail).
        let mut groups: BTreeMap<(usize, i64, i64), Vec<usize>> = BTreeMap::new();
        for (i, ep) in eps.iter().enumerate() {
            if ep.dir.0.abs() < 0.5 {
                continue;
            }
            let key = (
                ep.placed,
                ep.dir.0.signum() as i64,
                (ep.pos.0 * 100.0).round() as i64,
            );
            groups.entry(key).or_default().push(i);
        }
        let g = self.cfg.grid_mm;
        let stub = self.cfg.power_stub_mm;
        // The bus sits at least a full pin exit out from the pin column so
        // every tap leaves its pin straight for >= `min_pin_exit_steps` grid
        // steps (rule #1); it then steps out one grid at a time, hugging the
        // component as closely as a clean column allows (rule #2).
        let base_off = self.cfg.min_pin_exit_mm().max(stub);
        for ((_, dirx, _), mut members) in groups {
            if members.len() < 2 {
                continue;
            }
            members.sort_by(|&a, &b| {
                eps[a]
                    .pos
                    .1
                    .partial_cmp(&eps[b].pos.1)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
            let px = eps[members[0]].pos.0;
            let ymin = eps[members[0]].pos.1;
            let ymax = eps[*members.last().unwrap()].pos.1;
            let placed_excl: Vec<usize> = members.iter().map(|&i| eps[i].placed).collect();
            // A merged rail bank lifts its symbol one grid step higher than the
            // bare power stub: the arrow crowns a whole pin column (its bus taps
            // several pins), so it wants a full harmonized channel — two steps
            // of exit plus a step of air — above the topmost tap instead of
            // hugging it at the plain two-step stub. Grounds keep the plain stub
            // (their glyph hangs below, into open space). The engineer's "raise
            // the VDD that climbs off REFIN+/VDD by one step" on AD7171.
            let rail_lift = self.cfg.grid_mm;
            for step in 0..=8 {
                let bus_x = round4(px + dirx as f64 * (base_off + step as f64 * g));
                let sym_y = if down {
                    round4(ymax + stub)
                } else {
                    round4(ymin - stub - rail_lift)
                };
                let (bus_top, bus_bot) = if down { (ymin, sym_y) } else { (sym_y, ymax) };
                let bus = vec![(bus_x, bus_top), (bus_x, bus_bot)];
                let mut segs: Vec<Vec<Point>> = members
                    .iter()
                    .map(|&i| vec![eps[i].pos, (bus_x, eps[i].pos.1)])
                    .collect();
                segs.push(bus.clone());
                let gbox = power_symbol_graphic_box(name, (bus_x, sym_y), down);
                let clean = segs.iter().all(|s| self.path_ok(s, name, &placed_excl))
                    && self.graphic_box_clear(&gbox, name)
                    // Rule #1: the bus column stands off every neighbouring net
                    // by the harmonized inter-net channel (two grid steps),
                    // never hugging a foreign wire at a single step.
                    && segs.iter().all(|s| self.parallel_clear(s, name));
                if !clean {
                    continue;
                }
                for s in &segs {
                    self.out.wires.push(s.clone());
                    self.reg.register_path(s, name);
                }
                for &i in &members {
                    let j = (bus_x, eps[i].pos.1);
                    if self.junction_needed_at(j, name) {
                        self.push_junction(j);
                    }
                }
                self.out
                    .power_symbols
                    .push((name.to_string(), (bus_x, sym_y), down));
                self.reg.power_boxes.push((gbox, name.to_string()));
                // On a relegated sheet the flag is deferred to the utility
                // band (`relegate_flags`); otherwise it chains here inline.
                if flag_nets.contains(name)
                    && flagged.insert(name.to_string())
                    && !self.model.relegate
                {
                    self.attach_pwr_flag(name, (bus_x, sym_y), (dirx as f64, 0.0));
                }
                served.extend(members.iter().copied());
                break;
            }
        }
        served
    }

    fn route_power(&mut self, flag_nets: &BTreeSet<String>) {
        let mut flagged: BTreeSet<String> = BTreeSet::new();
        for sn in 0..self.model.nets.len() {
            let net = &self.model.nets[sn];
            if net.class == NetClass::Signal {
                continue;
            }
            let name = net.name.clone();
            // Grounds and negative rails hang their symbols downward; positive
            // rails point up (rule #3 potential ordering).
            let down = power_points_down(&name, net.class);
            let eps = self.net_endpoints(net);
            let eps = self.collapse_stacked_pins(eps, &name, !down, true);
            let aligned = self.power_align_targets(&eps, down);
            // Group non-adjacent same-net pins of one IC edge under a single
            // shared symbol (a short bus joins them). Pins the merge served are
            // skipped by the per-pin pass below.
            let served = self.merge_power_banks(&name, &eps, down, flag_nets, &mut flagged);

            for (index, ep) in eps.iter().enumerate() {
                if served.contains(&index) {
                    continue;
                }
                let mut att = power_attachment(self.cfg, ep.pos, ep.dir, down, 0.0, 0.0);
                let mut found = false;

                // Aligned candidates first (row target), then free retries at
                // one-grid-step increments outward (never a shortened elbow —
                // the exit stays >= `min_pin_exit_steps`), then jogs for
                // vertical pins.
                let hs = [
                    0.0, 1.27, 2.54, 3.81, 5.08, 7.62, 10.16, 12.7, 15.24, 17.78, 20.32,
                ];
                let mut attempts: Vec<(f64, f64)> = Vec::new();
                if let Some(&ty) = aligned.get(&index) {
                    let v = (ty - att.symbol_at.1).abs();
                    for h in hs {
                        attempts.push((h, v));
                    }
                }
                for h in hs {
                    for v in [0.0, 2.54, 5.08, 7.62, 10.16] {
                        attempts.push((h, v));
                    }
                }
                let mut cands: Vec<Attachment> = attempts
                    .iter()
                    .map(|&(h, v)| power_attachment(self.cfg, ep.pos, ep.dir, down, h, v))
                    .collect();
                if ep.dir.1.abs() > 0.5 {
                    for stub1 in [5.08, 2.54, 7.62] {
                        for step in [2.54, 5.08, 7.62, 10.16, 12.7, 15.24] {
                            for side in [1.0, -1.0] {
                                for v in [0.0, 1.27, 2.54, 5.08] {
                                    let y1 = round4(ep.pos.1 + ep.dir.1 * stub1);
                                    let xm = round4(ep.pos.0 + side * step);
                                    let end = (xm, round4(y1 + ep.dir.1 * (2.54 + v)));
                                    cands.push(Attachment {
                                        path: vec![ep.pos, (ep.pos.0, y1), (xm, y1), end],
                                        symbol_at: end,
                                        down,
                                    });
                                }
                            }
                        }
                    }
                }
                // Selection favours HUGGING the component (rule #2): among the
                // clean candidates the branch column closest to the pin wins, so
                // a VDD/GND tap peels off right beside the part instead of being
                // pushed out past the global labels. A same-net alignment row
                // (rule #3) is honoured first — the symbol lands on the shared
                // potential ordinate — then the closest column, then (only as a
                // tie-break) the fewest frank crossings and the shortest path.
                // Frank crossings are electrically harmless, so hugging is
                // allowed to accept one rather than flee to a distant column.
                let target = aligned.get(&index).copied();
                let mut best_key: Option<(f64, f64, usize, f64)> = None;
                for cand in &cands {
                    let gbox = power_symbol_graphic_box(&name, cand.symbol_at, cand.down);
                    if !self.path_ok(&cand.path, &name, &[ep.placed])
                        || !self.graphic_box_clear(&gbox, &name)
                    {
                        continue;
                    }
                    let miss = target.map_or(0.0, |ty| (cand.symbol_at.1 - ty).abs());
                    let hug = (cand.symbol_at.0 - ep.pos.0).abs();
                    let key = (
                        miss,
                        hug,
                        self.reg.count_crossings(&cand.path),
                        path_length_mm(&cand.path),
                    );
                    let better = best_key.as_ref().is_none_or(|b| key < *b);
                    if better {
                        best_key = Some(key);
                        att = cand.clone();
                        found = true;
                    }
                }
                if !found {
                    // Visual-relaxed rescue: allow the wire to run through
                    // label/graphic boxes but NEVER create an electrical
                    // contact (foreign pin on the path, wire touching a
                    // foreign wire). A committed fallback that shorts two
                    // rails is a netlist corruption, not a cosmetic issue.
                    let mut extended: Vec<Attachment> = Vec::new();
                    for h in [
                        0.0, 1.27, 2.54, 3.81, 5.08, 7.62, 10.16, 12.7, 15.24, 17.78, 20.32, 25.4,
                        30.48,
                    ] {
                        for v in [0.0, 2.54, 5.08, 7.62, 10.16, 12.7, 15.24, 20.32] {
                            extended.push(power_attachment(self.cfg, ep.pos, ep.dir, down, h, v));
                        }
                    }
                    extended.extend(cands);
                    let mut best_crossings = usize::MAX;
                    for cand in &extended {
                        if self.path_electrically_clean(&cand.path, &name) {
                            let crossings = self.reg.count_crossings(&cand.path);
                            if crossings < best_crossings {
                                att = cand.clone();
                                best_crossings = crossings;
                                found = true;
                            }
                            if best_crossings == 0 {
                                break;
                            }
                        }
                    }
                    if found {
                        self.warnings.push(format!(
                            "power stub {name} of {} placed with visual overlap (electrically clean)",
                            self.design.comps[self.model.placed[ep.placed].comp].refdes
                        ));
                    } else {
                        self.warnings.push(format!(
                            "power stub {name} of {} placed with residual contact — REVIEW REQUIRED",
                            self.design.comps[self.model.placed[ep.placed].comp].refdes
                        ));
                    }
                }
                if att.path.len() >= 2 && dist(att.path[0], *att.path.last().unwrap()) > EPS {
                    self.out.wires.push(att.path.clone());
                    self.reg.register_path(&att.path, &name);
                }
                self.out
                    .power_symbols
                    .push((name.clone(), att.symbol_at, att.down));
                self.reg.power_boxes.push((
                    power_symbol_graphic_box(&name, att.symbol_at, att.down),
                    name.clone(),
                ));

                // One PWR_FLAG per undriven rail. On a relegated sheet it is
                // deferred to the utility band; otherwise it chains next to
                // the first power symbol of the assigned sheet.
                if flag_nets.contains(&name) && flagged.insert(name.clone()) && !self.model.relegate
                {
                    self.attach_pwr_flag(&name, att.symbol_at, ep.dir);
                }
            }
        }
        // Relegated sheets: gather the deferred flags into the utility band.
        if self.model.relegate {
            self.relegate_flags(&flagged);
        }
    }

    /// Aligned power rows: endpoints of the same net whose symbols land in
    /// the same Y band (holes <= max dx) share a common Y — the highest for
    /// a rail (arrows up), the lowest for a ground. Endpoints stacked on
    /// the same X (adjacent pins of one edge) are not a row: aligning them
    /// would pile identical symbols on one point.
    fn power_align_targets(&self, eps: &[Endpoint], down: bool) -> BTreeMap<usize, f64> {
        let cfg = self.cfg;
        let mut base: Vec<(usize, f64, f64)> = eps
            .iter()
            .enumerate()
            .filter(|(_, ep)| {
                // Relegated caps sit in their own band — never share a power
                // row with the flow (a long stretched stub would result).
                !self.model.placed[ep.placed].relegated
                    && (ep.dir.1.abs() < 0.5
                        || (down && ep.dir.1 > 0.5)
                        || (!down && ep.dir.1 < -0.5))
            })
            .map(|(index, ep)| {
                let att = power_attachment(cfg, ep.pos, ep.dir, down, 0.0, 0.0);
                (index, ep.pos.0, att.symbol_at.1)
            })
            .collect();
        base.sort_by(|a, b| {
            a.2.partial_cmp(&b.2)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        });
        let mut bands: Vec<Vec<(usize, f64, f64)>> = Vec::new();
        for item in base {
            match bands.last_mut() {
                Some(cur) if item.2 - cur.last().unwrap().2 <= cfg.power_align_max_dy_mm => {
                    cur.push(item)
                }
                _ => bands.push(vec![item]),
            }
        }
        let mut targets = BTreeMap::new();
        for mut band in bands {
            band.sort_by(|a, b| {
                a.1.partial_cmp(&b.1)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            });
            // By-potential alignment binds same-potential symbols on a common
            // ordinate across the whole cluster width, so a functional band of
            // grounds (or of rails) reads as one horizontal line regardless of
            // how far apart their pins sit; without it symbols only align when
            // their columns fall within `power_align_max_dx_mm`. The dy band
            // still caps how far a stub may stretch to reach the shared row.
            let max_dx = if cfg.align_power_by_potential {
                f64::INFINITY
            } else {
                cfg.power_align_max_dx_mm
            };
            let mut rows: Vec<Vec<(usize, f64, f64)>> = Vec::new();
            for item in band {
                match rows.last_mut() {
                    Some(cur) if item.1 - cur.last().unwrap().1 <= max_dx => cur.push(item),
                    _ => rows.push(vec![item]),
                }
            }
            for mut row in rows {
                // Drop same-column members (keep the first): stacked pins.
                let mut kept: Vec<(usize, f64, f64)> = Vec::new();
                row.retain(|item| {
                    let dup = kept.iter().any(|k| (k.1 - item.1).abs() < 1e-6);
                    if !dup {
                        kept.push(*item);
                    }
                    !dup
                });
                if row.len() < 2 {
                    continue;
                }
                let ys: Vec<f64> = row.iter().map(|r| r.2).collect();
                let min = ys.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = ys.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                if max - min > cfg.power_align_max_dy_mm {
                    continue;
                }
                let target = if down { max } else { min };
                for r in row {
                    targets.insert(r.0, target);
                }
            }
        }
        targets
    }

    /// Electrical-only path check: no foreign pin on the path and no
    /// touch/overlap with a foreign wire (frank crossings tolerated).
    /// Visual overlaps (bodies, label texts, power glyphs) are ignored.
    fn path_electrically_clean(&self, points: &[Point], net: &str) -> bool {
        for w in points.windows(2) {
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg = Seg {
                x1: w[0].0,
                y1: w[0].1,
                x2: w[1].0,
                y2: w[1].1,
                net: net.to_string(),
            };
            for (px, py, pnet) in &self.reg.pins {
                if pnet != net && point_on_seg(*px, *py, &seg) {
                    return false;
                }
            }
            for other in &self.reg.segs {
                if other.net != net && segs_touch_forbidden(&seg, other) {
                    return false;
                }
            }
        }
        true
    }

    fn graphic_box_clear(&self, bbox: &BBox, net: &str) -> bool {
        for (px, py, pnet) in &self.reg.pins {
            if pnet == net {
                continue;
            }
            if *px > bbox.x1 + EPS
                && *px < bbox.x2 - EPS
                && *py > bbox.y1 + EPS
                && *py < bbox.y2 - EPS
            {
                return false;
            }
        }
        for seg in &self.reg.segs {
            if seg.net != net && seg_intersects_box(seg, bbox) {
                return false;
            }
        }
        for (gb, gnet) in &self.reg.power_boxes {
            if gnet != net && overlaps(gb, bbox) {
                return false;
            }
        }
        for lb in &self.reg.label_boxes {
            if !lb.allows(net) && overlaps(&lb.bbox, bbox) {
                return false;
            }
        }
        true
    }

    /// PWR_FLAG chained next to a power symbol (end-to-end wires connect
    /// without a junction). The base offset clears the rail's value text
    /// (long net names widen the glyph footprint).
    fn attach_pwr_flag(&mut self, net: &str, symbol_at: Point, pin_dir: (f64, f64)) {
        let half_text = (net.chars().count() as f64 * 0.762 + 1.27).max(2.54);
        let base = self.cfg.snap_up(half_text + 1.27);
        let mut candidates: Vec<Point> = Vec::new();
        let first_side = if pin_dir.0 < -0.5 { -1.0 } else { 1.0 };
        for k in [0.0, 2.54, 5.08, 7.62] {
            candidates.push((round4(symbol_at.0 + first_side * (base + k)), symbol_at.1));
            candidates.push((round4(symbol_at.0 - first_side * (base + k)), symbol_at.1));
        }
        let flag_box = |at: Point| BBox {
            x1: at.0 - 2.54,
            y1: at.1 - 5.08,
            x2: at.0 + 2.54,
            y2: at.1 + 0.5,
        };
        let chosen = candidates
            .iter()
            .find(|&&at| {
                self.path_ok(&[symbol_at, at], net, &[])
                    && self.graphic_box_clear(&flag_box(at), net)
            })
            .copied()
            .unwrap_or((round4(symbol_at.0 + first_side * base), symbol_at.1));
        self.out.wires.push(vec![symbol_at, chosen]);
        self.reg.register_path(&[symbol_at, chosen], net);
        self.out.pwr_flags.push((chosen, 0));
        self.reg
            .power_boxes
            .push((flag_box(chosen), net.to_string()));
    }

    /// Bounding box of the relegated decoupling band: the caps' solid boxes
    /// plus the rail/ground symbols stacked on them. `None` when nothing was
    /// relegated on this sheet.
    fn utility_band_box(&self) -> Option<BBox> {
        let mut out: Option<BBox> = None;
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &self.model.placed {
            if !p.relegated {
                continue;
            }
            let full = placed_full_box(self.design, p);
            lo = lo.min(full.x1);
            hi = hi.max(full.x2);
            union_opt(&mut out, &full);
        }
        let mut out = out?;
        // The upright caps' rail/ground symbols sit on the caps' X column. The
        // relegated `PWR_FLAG` column is stacked in that same X band but belongs
        // to the ERC band (`flag_region`), not here: skip any power symbol whose
        // connection point falls inside it, otherwise the decoupling band would
        // swallow the flag stack and the two zones would overlap.
        for (name, at, down) in &self.out.power_symbols {
            if let Some(fr) = self.flag_region
                && fr.x1 - EPS <= at.0
                && at.0 <= fr.x2 + EPS
                && fr.y1 - EPS <= at.1
                && at.1 <= fr.y2 + EPS
            {
                continue;
            }
            if at.0 >= lo - EPS && at.0 <= hi + EPS {
                out.union(&power_symbol_graphic_box(name, *at, *down));
            }
        }
        Some(out)
    }

    /// Gather the deferred undriven-rail `PWR_FLAG`s into the utility band, in a
    /// column below the relegated decoupling row. Each flag gets its own
    /// homonym power symbol so it sits on the correct global rail through a
    /// short local wire — no long return trace into the flow. Power symbols and
    /// flags are not netlist nodes (the exported netlist lists only real
    /// component pins), so this is a pure relocation: the netlist is unchanged.
    fn relegate_flags(&mut self, flagged: &BTreeSet<String>) {
        if flagged.is_empty() {
            return;
        }
        let cfg = self.cfg;
        // Anchor the flag column under the relegated decoupling band; fall back
        // to the right of the functional flow when nothing was relegated there.
        let (col_x, mut y) = match self.utility_band_box() {
            Some(r) => (cfg.snap(r.x1), cfg.snap(r.y2 + cfg.utility_pitch_mm)),
            None => (
                cfg.snap(self.model.content_box.x2 + cfg.utility_gap_mm),
                cfg.snap(self.model.content_box.y1 + 12.7),
            ),
        };
        // Rails first (up glyph, texts above), grounds last (down glyph, texts
        // below): stacked this way each row's texts point away from its
        // neighbour instead of colliding in the gap — and it matches the
        // engineer's VDD-over-GND utility stack.
        let mut ordered: Vec<(&String, bool)> = flagged
            .iter()
            .map(|name| {
                let down = self
                    .model
                    .nets
                    .iter()
                    .find(|n| &n.name == name)
                    .map(|n| n.class == NetClass::Ground)
                    .unwrap_or(false);
                (name, down)
            })
            .collect();
        ordered.sort_by(|a, b| a.1.cmp(&b.1).then_with(|| a.0.cmp(b.0)));

        let flag_start = self.out.pwr_flags.len();
        let sym_start = self.out.power_symbols.len();
        let half = |s: &str| (s.chars().count() as f64) * 0.762 + 1.27;
        for (name, down) in ordered {
            let sym_at = (col_x, y);
            // A standalone power symbol (a global-rail node) with a PWR_FLAG
            // chained beside it: the flag declares the rail driven for ERC.
            self.out.power_symbols.push((name.clone(), sym_at, down));
            self.reg
                .power_boxes
                .push((power_symbol_graphic_box(name, sym_at, down), name.clone()));
            // Flag to the right, spaced so its wide "PWR_FLAG" value text clears
            // the rail's own value text (both sit above their glyph).
            let gap = cfg.snap_up(half(name) + half("PWR_FLAG") + 1.27);
            let flag_at = (round4(col_x + gap), y);
            // Rule #6: a power symbol / PWR_FLAG leaves along its pin axis
            // (vertical) for at least `min_pin_exit_steps` grid steps before any
            // bend — no wire struck at a right angle right on the glyph. Both
            // symbols exit into their free half-plane (down for an up-pointing
            // rail, up for a down-pointing ground) and the jog runs between the
            // two recessed ends: sym ↓ recul → across → ↑ flag (the reference
            // layout's ERC U-links). Purely a wire-shape change — the symbol and
            // flag connection points are unmoved, so ERC/netlist are unaffected.
            let recul = if down {
                -cfg.min_pin_exit_mm()
            } else {
                cfg.min_pin_exit_mm()
            };
            let sym_knee = (sym_at.0, round4(sym_at.1 + recul));
            let flag_knee = (flag_at.0, round4(flag_at.1 + recul));
            let flag_link = vec![sym_at, sym_knee, flag_knee, flag_at];
            self.reg.register_path(&flag_link, name);
            self.out.wires.push(flag_link);
            let flag_rot = if down { 180 } else { 0 };
            self.out.pwr_flags.push((flag_at, flag_rot));
            self.reg
                .power_boxes
                .push((pwr_flag_box(flag_at, flag_rot), name.clone()));
            y = cfg.snap(y + cfg.utility_pitch_mm);
        }
        // Record the ERC band for the zone outline.
        let flags: Vec<(Point, i32)> = self.out.pwr_flags[flag_start..].to_vec();
        let syms: Vec<(String, Point, bool)> = self.out.power_symbols[sym_start..].to_vec();
        let mut region: Option<BBox> = None;
        for (f, rot) in flags {
            union_opt(&mut region, &pwr_flag_box(f, rot));
        }
        for (name, at, down) in syms {
            union_opt(&mut region, &power_symbol_graphic_box(&name, at, down));
        }
        self.flag_region = region;
    }

    /// Outline the functional / decoupling / ERC areas with discreet graphic
    /// rectangles. Only relegated sheets get zones (a sheet with no utility
    /// band has nothing to separate). Purely visual — the rectangles carry no
    /// connectivity, so neither the netlist nor ERC is affected.
    fn compute_zones(&mut self) {
        if !self.model.relegate {
            return;
        }
        let m = self.cfg.zone_margin_mm;
        // The relegated bands define the right-hand column; nothing to do when
        // nothing was relegated.
        let decoupling = self.utility_band_box();
        let erc = self.flag_region;
        let (util_left, util_right) = match (decoupling, erc) {
            (Some(d), Some(e)) => (d.x1.min(e.x1), d.x2.max(e.x2)),
            (Some(d), None) => (d.x1, d.x2),
            (None, Some(e)) => (e.x1, e.x2),
            (None, None) => return,
        };

        // --- Functional content: the COMPLETE extent of every non-relegated
        // element (component solids already carry their Reference/Value text;
        // power symbols, every label kind, flags and wires are added here). The
        // utility seam segregates the flow from the relegated column. ---
        let seam_raw = util_left - EPS;
        let mut func: Option<BBox> = None;
        for p in &self.model.placed {
            if !p.relegated {
                union_opt(&mut func, &placed_full_box(self.design, p));
            }
        }
        for (name, at, down) in &self.out.power_symbols {
            if at.0 < seam_raw {
                union_opt(&mut func, &power_symbol_graphic_box(name, *at, *down));
            }
        }
        for &(at, rot) in &self.out.pwr_flags {
            if at.0 < seam_raw {
                union_opt(&mut func, &pwr_flag_box(at, rot));
            }
        }
        for (name, at, rot) in &self.out.net_labels {
            if at.0 < seam_raw {
                union_opt(&mut func, &label_text_box(name, *at, *rot));
            }
        }
        // A global label carries a flag glyph beyond its text on the wire side:
        // inflate so the whole port is enclosed.
        for (name, at, rot, _) in &self.out.global_labels {
            if at.0 < seam_raw {
                union_opt(&mut func, &inflate(&label_text_box(name, *at, *rot), 1.27));
            }
        }
        for (name, _, at, rot) in &self.out.hier_labels {
            if at.0 < seam_raw {
                union_opt(&mut func, &hier_text_box(name, *at, *rot));
            }
        }
        for w in &self.out.wires {
            for &pt in w {
                if pt.0 < seam_raw {
                    union_opt(
                        &mut func,
                        &BBox {
                            x1: pt.0,
                            y1: pt.1,
                            x2: pt.0,
                            y2: pt.1,
                        },
                    );
                }
            }
        }

        // --- Grid layout: one outer rectangle divided into cells that abut on
        // shared edges. Functional fills the left column at full height; the
        // right column stacks Decoupling over ERC. Every cell fully encloses
        // its content (margins added), guaranteeing nothing overflows. ---
        let func_right = func.map(|f| f.x2).unwrap_or(util_left);

        // Anchor extents for the vertical seam. A value text overhangs its
        // glyph, but the electrical anchor (a label's connection point, a
        // symbol's pin, a component body) must land in its own cell. Track the
        // rightmost functional anchor and the leftmost utility glyph so a
        // functional net label placed level with the utility column (its anchor
        // inside the column's X band) still falls in the functional cell. A
        // power symbol/flag is utility when its connection point sits inside the
        // decoupling or ERC content box; every local/global/hier label is a
        // functional annotation (the utility column carries only power symbols
        // and flags).
        let in_util = |at: Point| -> bool {
            [decoupling, erc].into_iter().flatten().any(|u| {
                at.0 >= u.x1 - EPS && at.0 <= u.x2 + EPS && at.1 >= u.y1 - EPS && at.1 <= u.y2 + EPS
            })
        };
        let mut func_ax2 = f64::NEG_INFINITY;
        let mut util_ax1 = f64::INFINITY;
        for p in &self.model.placed {
            let raw = raw_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
            if p.relegated {
                util_ax1 = util_ax1.min(raw.x1);
            } else {
                func_ax2 = func_ax2.max(raw.x2);
            }
        }
        for (_, at, _) in &self.out.power_symbols {
            if in_util(*at) {
                util_ax1 = util_ax1.min(at.0);
            } else {
                func_ax2 = func_ax2.max(at.0);
            }
        }
        for &(at, _rot) in &self.out.pwr_flags {
            if in_util(at) {
                util_ax1 = util_ax1.min(at.0);
            } else {
                func_ax2 = func_ax2.max(at.0);
            }
        }
        for (_, at, _) in &self.out.net_labels {
            func_ax2 = func_ax2.max(at.0);
        }
        for (_, at, _, _) in &self.out.global_labels {
            func_ax2 = func_ax2.max(at.0);
        }
        for (_, _, at, _) in &self.out.hier_labels {
            func_ax2 = func_ax2.max(at.0);
        }

        // Vertical seam between the functional column (left) and the utility
        // column (right). It defaults to the midpoint of the free space between
        // the two content boxes, but if a functional anchor sits right of that
        // midpoint (a signal net label placed level with the utility column) the
        // seam slides into the gap between that anchor and the leftmost utility
        // glyph, keeping the label in the functional cell (its wide text may
        // overhang, which is tolerated). The cells ABUT on this seam (optionally
        // parted by `zone_gap_mm`) and never grow past it, so the rectangles stay
        // mutually DISJOINT — two zone outlines never overlap.
        let x_seam_plain = (func_right + util_left) / 2.0;
        let x_seam = if func_ax2 > x_seam_plain && func_ax2 < util_ax1 {
            (func_ax2 + util_ax1) / 2.0
        } else {
            x_seam_plain
        };
        let gx = self.cfg.zone_gap_mm.min((util_left - func_right).max(0.0));

        // Outer rectangle, shared by every cell; edges snapped outward.
        let left = func.map(|f| f.x1).unwrap_or(util_left);
        let mut top = f64::INFINITY;
        let mut bot = f64::NEG_INFINITY;
        for b in [func, decoupling, erc].into_iter().flatten() {
            top = top.min(b.y1);
            bot = bot.max(b.y2);
        }
        let snap = self.cfg.zone_snap_mm;
        let out_lo = |v: f64| {
            round4(if snap > 0.0 {
                (v / snap).floor() * snap
            } else {
                v
            })
        };
        let out_hi = |v: f64| {
            round4(if snap > 0.0 {
                (v / snap).ceil() * snap
            } else {
                v
            })
        };
        let x_lo = out_lo(left - m);
        let x_hi = out_hi(util_right + m);
        let y_lo = out_lo(top - m);
        let y_hi = out_hi(bot + m);
        let f_x2 = round4(x_seam - gx / 2.0);
        let u_x1 = round4(x_seam + gx / 2.0);

        if func.is_some() {
            self.push_zone(
                BBox {
                    x1: x_lo,
                    y1: y_lo,
                    x2: f_x2,
                    y2: y_hi,
                },
                "Functional",
            );
        }
        match (decoupling, erc) {
            (Some(d), Some(e)) => {
                // Decoupling sits above ERC: split the column on a row seam
                // centered in the free space between the two bands. The two
                // cells abut on this seam (never expanding past it), so the
                // Decoupling and ERC rectangles stay disjoint.
                let y_seam = (d.y2 + e.y1) / 2.0;
                let gy = self.cfg.zone_gap_mm.min((e.y1 - d.y2).max(0.0));
                self.push_zone(
                    BBox {
                        x1: u_x1,
                        y1: y_lo,
                        x2: x_hi,
                        y2: round4(y_seam - gy / 2.0),
                    },
                    "Decoupling",
                );
                self.push_zone(
                    BBox {
                        x1: u_x1,
                        y1: round4(y_seam + gy / 2.0),
                        x2: x_hi,
                        y2: y_hi,
                    },
                    "ERC",
                );
            }
            (Some(_), None) => self.push_zone(
                BBox {
                    x1: u_x1,
                    y1: y_lo,
                    x2: x_hi,
                    y2: y_hi,
                },
                "Decoupling",
            ),
            (None, Some(_)) => self.push_zone(
                BBox {
                    x1: u_x1,
                    y1: y_lo,
                    x2: x_hi,
                    y2: y_hi,
                },
                "ERC",
            ),
            (None, None) => {}
        }
    }

    /// Push a zone outline, skipping degenerate or non-finite boxes.
    fn push_zone(&mut self, bbox: BBox, title: &str) {
        if !bbox.x1.is_finite() || bbox.width() < 2.54 || bbox.height() < 2.54 {
            return;
        }
        self.out.zones.push(Zone {
            bbox,
            title: Some(title.to_string()),
        });
    }

    // --------------------------------------------------------------
    // No-connects, blocks, port fallbacks, extents
    // --------------------------------------------------------------

    fn mark_no_connects(&mut self) {
        for (pi, p) in self.model.placed.iter().enumerate() {
            let geom = &self.design.comps[p.comp].geom;
            let mut seen: BTreeSet<(i64, i64)> = BTreeSet::new();
            for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
                let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                    continue;
                };
                if !seen.insert(quant(pos)) {
                    continue;
                }
                if !self.model.pin_net.contains_key(&(pi, pin.number.clone())) {
                    self.out.no_connects.push(pos);
                }
            }
        }
    }

    /// Child sheet blocks in a 2-column grid below the content; every sheet
    /// pin gets a stub + homonym net label (or the hierarchical label when
    /// the net is a port of this sheet exposed only by blocks).
    fn place_blocks(&mut self) {
        let sheet = &self.plan.sheets[self.model.sheet];
        if sheet.children.is_empty() {
            return;
        }
        let cfg = self.cfg;

        let mut bottom = self.model.content_box.y2;
        for w in &self.out.wires {
            for p in w {
                bottom = bottom.max(p.1);
            }
        }
        for (_, at, _) in &self.out.net_labels {
            bottom = bottom.max(at.1);
        }
        for (_, at, down) in &self.out.power_symbols {
            bottom = bottom.max(at.1 + if *down { 7.62 } else { 0.0 });
        }
        let origin = (
            cfg.snap(self.model.content_box.x1.max(cfg.margin_left_mm)),
            cfg.snap(bottom + cfg.sheet_region_top_gap_mm),
        );

        // Port nets of this sheet exposed only by child blocks anchor their
        // hierarchical label on the first homonym sheet pin.
        let mut block_anchor: BTreeMap<String, PortDirection> = BTreeMap::new();
        for net in &self.model.nets {
            if let Some(direction) = net.port
                && net.on_child_blocks
                && net.endpoints.is_empty()
                && !self.port_anchored.contains(&net.name)
            {
                block_anchor.insert(net.name.clone(), direction);
            }
        }

        let mut row_y = origin.1;
        let children = sheet.children.clone();
        for row in children.chunks(2) {
            let mut row_h = 0.0f64;
            for (col, &child) in row.iter().enumerate() {
                let ports = &self.plan.sheets[child].ports;
                let left: Vec<_> = ports
                    .iter()
                    .filter(|p| matches!(p.direction, PortDirection::Input))
                    .collect();
                let right: Vec<_> = ports
                    .iter()
                    .filter(|p| !matches!(p.direction, PortDirection::Input))
                    .collect();
                let n_side = left.len().max(right.len());
                let title = &self.plan.sheets[child].title;
                let w = cfg.snap_up(
                    cfg.sheet_block_min_width_mm
                        .max((title.chars().count() as f64 + 2.0) * 1.27),
                );
                let h = cfg.snap_up(12.7f64.max((n_side as f64 + 1.0) * 2.54));
                let bx = cfg.snap(origin.0 + col as f64 * (w + cfg.sheet_block_gap_x_mm));
                let by = cfg.snap(row_y);

                let mut pins: Vec<(String, PortDirection, Point)> = Vec::new();
                for (k, port) in left.iter().enumerate() {
                    let name = self.design.nets[port.net].name.clone();
                    let at = (bx, round4(by + 2.54 * (k as f64 + 1.0)));
                    pins.push((name.clone(), port.direction, at));
                    let end = (round4(bx - label_stub_len(cfg, &name)), at.1);
                    self.out.wires.push(vec![at, end]);
                    self.reg.register_path(&[at, end], &name);
                    if let Some(direction) = block_anchor.remove(&name) {
                        self.out
                            .hier_labels
                            .push((name.clone(), direction, end, 180));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: hier_text_box(&name, end, 180),
                            net: name.clone(),
                            also: Vec::new(),
                        });
                        self.port_anchored.insert(name);
                    } else {
                        // Local net label left of the block: the stub reaches it
                        // from the right, so the label reads back over the stub
                        // (text above the wire, wire running on to the block).
                        let rot = net_label_rotation(false, 180);
                        self.out.net_labels.push((name.clone(), end, rot));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: label_text_box(&name, end, rot),
                            net: name,
                            also: Vec::new(),
                        });
                    }
                }
                for (k, port) in right.iter().enumerate() {
                    let name = self.design.nets[port.net].name.clone();
                    let at = (round4(bx + w), round4(by + 2.54 * (k as f64 + 1.0)));
                    pins.push((name.clone(), port.direction, at));
                    let end = (round4(bx + w + label_stub_len(cfg, &name)), at.1);
                    self.out.wires.push(vec![at, end]);
                    self.reg.register_path(&[at, end], &name);
                    if let Some(direction) = block_anchor.remove(&name) {
                        self.out.hier_labels.push((name.clone(), direction, end, 0));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: hier_text_box(&name, end, 0),
                            net: name.clone(),
                            also: Vec::new(),
                        });
                        self.port_anchored.insert(name);
                    } else {
                        // Local net label right of the block: the stub reaches
                        // it from the left, so the label reads back over the stub
                        // (text above the wire, wire running on to the block).
                        let rot = net_label_rotation(false, 0);
                        self.out.net_labels.push((name.clone(), end, rot));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: label_text_box(&name, end, rot),
                            net: name,
                            also: Vec::new(),
                        });
                    }
                }
                self.out.blocks.push(BlockModel {
                    sheet: child,
                    at: (bx, by),
                    size: (w, h),
                    pins,
                });
                row_h = row_h.max(h);
            }
            row_y += row_h + cfg.sheet_block_gap_y_mm;
        }
    }

    /// Edge-column fallback for port nets without any internal anchor
    /// (neither a component pin nor a child block pin).
    fn place_port_fallbacks(&mut self) {
        let sheet = &self.plan.sheets[self.model.sheet];
        if sheet.parent.is_none() {
            if !sheet.ports.is_empty() {
                self.warnings
                    .push("ports declared on the root sheet — ignored".to_string());
            }
            return;
        }
        let cfg = self.cfg;
        let pending: Vec<(String, PortDirection)> = self
            .model
            .nets
            .iter()
            .filter(|n| n.port.is_some() && !self.port_anchored.contains(&n.name))
            .map(|n| (n.name.clone(), n.port.expect("filtered")))
            .collect();
        if pending.is_empty() {
            return;
        }
        let content = &self.model.content_box;
        let start_y = cfg.snap(content.y1 + 2.54);
        let mut left_i = 0usize;
        let mut right_i = 0usize;
        for (name, direction) in pending {
            self.warnings.push(format!(
                "port \"{name}\" has no internal anchor — edge-column fallback"
            ));
            if matches!(direction, PortDirection::Output) {
                let x = cfg.snap(content.x2 + cfg.port_column_mm);
                let y = cfg.snap(start_y + right_i as f64 * cfg.port_pitch_mm);
                right_i += 1;
                let end = (round4(x - label_stub_len(cfg, &name)), y);
                self.out
                    .hier_labels
                    .push((name.clone(), direction, (x, y), 0));
                self.out.wires.push(vec![(x, y), end]);
                self.reg.register_path(&[(x, y), end], &name);
                // The homonym is a local net label: it reads back over its stub
                // (text above the wire) while the sibling hierarchical label
                // above keeps the outward port reading.
                self.out
                    .net_labels
                    .push((name.clone(), end, net_label_rotation(false, 180)));
            } else {
                let x = cfg.snap(content.x1 - cfg.port_column_mm);
                let y = cfg.snap(start_y + left_i as f64 * cfg.port_pitch_mm);
                left_i += 1;
                let end = (round4(x + label_stub_len(cfg, &name)), y);
                self.out
                    .hier_labels
                    .push((name.clone(), direction, (x, y), 180));
                self.out.wires.push(vec![(x, y), end]);
                self.reg.register_path(&[(x, y), end], &name);
                // The homonym is a local net label: it reads back over its stub
                // (text above the wire) while the sibling hierarchical label
                // above keeps the outward port reading.
                self.out
                    .net_labels
                    .push((name.clone(), end, net_label_rotation(false, 0)));
            }
        }
    }

    // --------------------------------------------------------------
    // Soft sibling alignment (`align_sibling_ports`)
    // --------------------------------------------------------------

    /// Pull sibling ports / net labels onto a shared X column (see the config
    /// doc). Best-effort: a group is aligned only when every member reaches the
    /// common column without a frank crossing, a text overlap or a foreign
    /// contact; any group that would regress is left untouched. Only stub
    /// extension (pin fixed) and in-wire label sliding are used, so the netlist
    /// is unchanged.
    fn align_sibling_ports(&mut self) {
        if !self.cfg.align_sibling_ports {
            return;
        }
        let items = self.collect_align_items();
        // Scope 1: ports leaving the same side of the same component.
        for cluster in self.port_clusters(&items) {
            self.try_align_cluster(&items, &cluster);
        }
        // Scope 2: sibling net-label annotations on parallel nets.
        for cluster in self.net_label_clusters(&items) {
            self.try_align_cluster(&items, &cluster);
        }
    }

    /// Snapshot every horizontal label that can be column-aligned: ports
    /// (global/hierarchical) that terminate a stub off a component pin, and
    /// local net labels that sit inside a horizontal wire run.
    fn collect_align_items(&self) -> Vec<AlignItem> {
        let mut items = Vec::new();
        for (i, (name, at, rot, _)) in self.out.global_labels.iter().enumerate() {
            if let Some(kind) = self.classify_stub(*at, *rot) {
                items.push(AlignItem {
                    slot: LabelSlot::Global(i),
                    net: name.clone(),
                    at: *at,
                    rot: *rot,
                    kind,
                });
            }
        }
        for (i, (name, _dir, at, rot)) in self.out.hier_labels.iter().enumerate() {
            if let Some(kind) = self.classify_stub(*at, *rot) {
                items.push(AlignItem {
                    slot: LabelSlot::Hier(i),
                    net: name.clone(),
                    at: *at,
                    rot: *rot,
                    kind,
                });
            }
        }
        for (i, (name, at, rot)) in self.out.net_labels.iter().enumerate() {
            if let Some(kind) = self.classify_interior(name, *at, *rot) {
                items.push(AlignItem {
                    slot: LabelSlot::Net(i),
                    net: name.clone(),
                    at: *at,
                    rot: *rot,
                    kind,
                });
            }
        }
        items
    }

    /// Classify a label anchor as a horizontal stub end off a component pin.
    /// Returns `None` unless `at` is the terminal vertex of exactly one wire,
    /// its terminal segment is horizontal, the text reads outward off that end,
    /// and the inner end is a real component pin.
    fn classify_stub(&self, at: Point, rot: i32) -> Option<AlignKind> {
        if rot != 0 && rot != 180 {
            return None;
        }
        let mut found: Option<(usize, bool, Point)> = None;
        let mut count = 0usize;
        for (wi, w) in self.out.wires.iter().enumerate() {
            if w.len() < 2 {
                continue;
            }
            if near(w[0], at) {
                let nb = w[1];
                if (nb.1 - at.1).abs() < EPS && (nb.0 - at.0).abs() > EPS {
                    found = Some((wi, false, nb));
                    count += 1;
                }
            }
            let last = *w.last().expect("len >= 2");
            if near(last, at) {
                let nb = w[w.len() - 2];
                if (nb.1 - at.1).abs() < EPS && (nb.0 - at.0).abs() > EPS {
                    found = Some((wi, true, nb));
                    count += 1;
                }
            }
        }
        if count != 1 {
            return None;
        }
        let (wire, last, fixed) = found?;
        let outward = (at.0 - fixed.0).signum();
        let read = if rot == 0 { 1.0 } else { -1.0 };
        if (outward - read).abs() > EPS {
            return None;
        }
        let comp = self.comp_at_pin(fixed)?;
        Some(AlignKind::Stub {
            wire,
            last,
            fixed,
            comp,
        })
    }

    /// Classify a net label as an interior annotation of a horizontal wire run
    /// of its own net. Returns the run's `[lo, hi]` X-extent when the run is at
    /// least as wide as the text (room to slide the label and stay underlined).
    fn classify_interior(&self, name: &str, at: Point, rot: i32) -> Option<AlignKind> {
        if rot != 0 && rot != 180 {
            return None;
        }
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        let mut hit = false;
        for s in &self.reg.segs {
            if s.net != name || (s.y1 - s.y2).abs() >= EPS || (s.y1 - at.1).abs() >= EPS {
                continue;
            }
            let (xlo, xhi) = (s.x1.min(s.x2), s.x1.max(s.x2));
            if at.0 >= xlo - EPS && at.0 <= xhi + EPS {
                lo = lo.min(xlo);
                hi = hi.max(xhi);
                hit = true;
            }
        }
        if !hit || hi - lo < label_text_width(name) - EPS {
            return None;
        }
        Some(AlignKind::Interior { lo, hi })
    }

    /// Placed component owning a visible pin at `p`, if any.
    fn comp_at_pin(&self, p: Point) -> Option<usize> {
        for (pi, pc) in self.model.placed.iter().enumerate() {
            let geom = &self.design.comps[pc.comp].geom;
            for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
                if let Some(pos) = geom.pin_position(&pin.number, pc.at, pc.rotation, pc.mirror)
                    && near(pos, p)
                {
                    return Some(pi);
                }
            }
        }
        None
    }

    /// Sibling port clusters: stub labels grouped by (component, side, reading).
    /// Only groups of two or more with a non-trivial but bounded X spread are
    /// returned (a wider spread is an intentional detour, left alone).
    fn port_clusters(&self, items: &[AlignItem]) -> Vec<Vec<usize>> {
        let dx_window = 20.0 * self.cfg.grid_mm;
        let mut groups: BTreeMap<(usize, i64, i32), Vec<usize>> = BTreeMap::new();
        for (idx, it) in items.iter().enumerate() {
            if let AlignKind::Stub { comp, fixed, .. } = it.kind {
                let outward = (it.at.0 - fixed.0).signum() as i64;
                groups.entry((comp, outward, it.rot)).or_default().push(idx);
            }
        }
        let mut clusters = Vec::new();
        for members in groups.into_values() {
            if members.len() < 2 {
                continue;
            }
            let xs: Vec<f64> = members.iter().map(|&i| items[i].at.0).collect();
            let mn = xs.iter().cloned().fold(f64::INFINITY, f64::min);
            let mx = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            if mx - mn <= EPS || mx - mn > dx_window + EPS {
                continue;
            }
            clusters.push(members);
        }
        clusters
    }

    /// Sibling net-label clusters: interior annotations of parallel nets that
    /// share a reading direction, sit on neighbouring rows and columns, and run
    /// over a common X band. Clustered by proximity (union-find).
    fn net_label_clusters(&self, items: &[AlignItem]) -> Vec<Vec<usize>> {
        let interior: Vec<usize> = items
            .iter()
            .enumerate()
            .filter(|(_, it)| matches!(it.kind, AlignKind::Interior { .. }))
            .map(|(i, _)| i)
            .collect();
        let dx_window = 20.0 * self.cfg.grid_mm;
        let dy_window = 6.0 * self.cfg.grid_mm;
        let n = interior.len();
        let mut parent: Vec<usize> = (0..n).collect();
        fn find(parent: &mut [usize], a: usize) -> usize {
            let mut r = a;
            while parent[r] != r {
                r = parent[r];
            }
            let mut c = a;
            while parent[c] != c {
                let next = parent[c];
                parent[c] = r;
                c = next;
            }
            r
        }
        let siblings = |a: usize, b: usize| -> bool {
            let (ia, ib) = (&items[interior[a]], &items[interior[b]]);
            let AlignKind::Interior { lo: la, hi: ha } = ia.kind else {
                return false;
            };
            let AlignKind::Interior { lo: lb, hi: hb } = ib.kind else {
                return false;
            };
            ia.rot == ib.rot
                && ia.net != ib.net
                && (ia.at.1 - ib.at.1).abs() > EPS
                && (ia.at.1 - ib.at.1).abs() <= dy_window + EPS
                && (ia.at.0 - ib.at.0).abs() <= dx_window + EPS
                // the two runs overlap over a common X band (genuine parallels)
                && la.max(lb) < ha.min(hb) - EPS
        };
        // Triangular pass over index pairs (both indices drive the union-find).
        #[allow(clippy::needless_range_loop)]
        for a in 0..n {
            for b in (a + 1)..n {
                if siblings(a, b) {
                    let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                    if ra != rb {
                        parent[ra] = rb;
                    }
                }
            }
        }
        let mut by_root: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (a, &orig) in interior.iter().enumerate() {
            let r = find(&mut parent, a);
            by_root.entry(r).or_default().push(orig);
        }
        let mut clusters = Vec::new();
        for members in by_root.into_values() {
            if members.len() < 2 {
                continue;
            }
            let xs: Vec<f64> = members.iter().map(|&i| items[i].at.0).collect();
            let mn = xs.iter().cloned().fold(f64::INFINITY, f64::min);
            let mx = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            if mx - mn <= EPS {
                continue;
            }
            clusters.push(members);
        }
        clusters
    }

    /// Text keepout of a routed label of a given slot at a candidate anchor.
    fn slot_box(&self, slot: LabelSlot, name: &str, at: Point, rot: i32) -> BBox {
        match slot {
            LabelSlot::Hier(_) => hier_text_box(name, at, rot),
            _ => label_text_box(name, at, rot),
        }
    }

    /// Try to align one cluster onto the extreme member's X. Validates every
    /// member first and only commits when the whole group is clean; otherwise
    /// nothing moves.
    fn try_align_cluster(&mut self, items: &[AlignItem], cluster: &[usize]) {
        let rot = items[cluster[0]].rot;
        let xs = cluster.iter().map(|&i| items[i].at.0);
        let target = if rot == 0 {
            xs.fold(f64::NEG_INFINITY, f64::max)
        } else {
            xs.fold(f64::INFINITY, f64::min)
        };
        // Per-member feasibility + trial boxes.
        struct Trial {
            idx: usize,
            new_at: Point,
            new_box: BBox,
            stub: Option<(usize, bool, Point, Point)>, // wire, last, fixed, old_at
        }
        let mut trials: Vec<Trial> = Vec::new();
        for &i in cluster {
            let it = &items[i];
            let new_at = (target, it.at.1);
            match it.kind {
                AlignKind::Stub {
                    wire, last, fixed, ..
                } => {
                    // Only ever an outward extension (target is the extreme).
                    let old_reach = (it.at.0 - fixed.0).abs();
                    let new_reach = (new_at.0 - fixed.0).abs();
                    if new_reach + EPS < old_reach {
                        return;
                    }
                    let new_seg = vec![fixed, new_at];
                    let old_seg = vec![fixed, it.at];
                    if !self.path_ok(&new_seg, &it.net, &[])
                        || !self.parallel_clear(&new_seg, &it.net)
                        || self.reg.foreign_crossings(&new_seg, &it.net)
                            > self.reg.foreign_crossings(&old_seg, &it.net)
                    {
                        return;
                    }
                    trials.push(Trial {
                        idx: i,
                        new_at,
                        new_box: self.slot_box(it.slot, &it.net, new_at, rot),
                        stub: Some((wire, last, fixed, it.at)),
                    });
                }
                AlignKind::Interior { lo, hi } => {
                    if target < lo - EPS || target > hi + EPS {
                        return;
                    }
                    trials.push(Trial {
                        idx: i,
                        new_at,
                        new_box: self.slot_box(it.slot, &it.net, new_at, rot),
                        stub: None,
                    });
                }
            }
        }
        // Group-level clearance (tolerant among siblings, strict elsewhere).
        let member_nets: BTreeSet<&str> = cluster.iter().map(|&i| items[i].net.as_str()).collect();
        let boxes: Vec<(&str, &BBox)> = trials
            .iter()
            .map(|t| (items[t.idx].net.as_str(), &t.new_box))
            .collect();
        if !self.trial_boxes_clear(&boxes, &member_nets) {
            return;
        }
        // Commit: move labels, grow the stubs, keep the registry consistent.
        for t in &trials {
            let it = &items[t.idx];
            match it.slot {
                LabelSlot::Global(i) => self.out.global_labels[i].1 = t.new_at,
                LabelSlot::Hier(i) => self.out.hier_labels[i].2 = t.new_at,
                LabelSlot::Net(i) => self.out.net_labels[i].1 = t.new_at,
            }
            if let Some((wire, last, _fixed, old_at)) = t.stub {
                let w = &mut self.out.wires[wire];
                let vi = if last { w.len() - 1 } else { 0 };
                w[vi] = t.new_at;
                // Grow the matching registry segment so later clusters see it.
                for s in self.reg.segs.iter_mut() {
                    if s.net != it.net || (s.y1 - s.y2).abs() >= EPS {
                        continue;
                    }
                    if (s.x1 - old_at.0).abs() < EPS && (s.y1 - old_at.1).abs() < EPS {
                        s.x1 = t.new_at.0;
                    } else if (s.x2 - old_at.0).abs() < EPS && (s.y2 - old_at.1).abs() < EPS {
                        s.x2 = t.new_at.0;
                    }
                }
            }
            // Refresh the label keepout in the registry (find the old box).
            let old_box = self.slot_box(it.slot, &it.net, it.at, rot);
            for lb in self.reg.label_boxes.iter_mut() {
                if lb.net == it.net && boxes_collide_tol(&lb.bbox, &old_box, EPS) {
                    lb.bbox = t.new_box;
                    break;
                }
            }
        }
    }

    /// Are the candidate label boxes clear? Bodies, foreign pins and foreign
    /// wires are strict keepouts; label-vs-label (both foreign labels and the
    /// cluster siblings themselves) uses the sub-grid tolerance so a tidy
    /// two-step stack passes while a real overlap is rejected.
    fn trial_boxes_clear(&self, boxes: &[(&str, &BBox)], member_nets: &BTreeSet<&str>) -> bool {
        let tol = 0.5;
        for (net, b) in boxes {
            for p in self.model.placed.iter() {
                let rb = raw_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
                if overlaps(&rb, b) {
                    return false;
                }
            }
            for (px, py, pnet) in &self.reg.pins {
                if pnet != net
                    && *px > b.x1 + EPS
                    && *px < b.x2 - EPS
                    && *py > b.y1 + EPS
                    && *py < b.y2 - EPS
                {
                    return false;
                }
            }
            for seg in &self.reg.segs {
                if &seg.net != net && seg_intersects_box(seg, b) {
                    return false;
                }
            }
            for (gb, gnet) in &self.reg.power_boxes {
                if gnet != net && overlaps(gb, b) {
                    return false;
                }
            }
            for c in &self.corridors {
                if c.zone == CorridorZone::Corridor && &c.net != net && overlaps(&c.bbox, b) {
                    return false;
                }
            }
            for lb in &self.reg.label_boxes {
                if member_nets.contains(lb.net.as_str()) {
                    continue; // the moving members' own (stale) boxes
                }
                if boxes_collide_tol(&lb.bbox, b, tol) {
                    return false;
                }
            }
        }
        // Siblings against each other.
        for i in 0..boxes.len() {
            for j in (i + 1)..boxes.len() {
                if boxes[i].0 != boxes[j].0 && boxes_collide_tol(boxes[i].1, boxes[j].1, tol) {
                    return false;
                }
            }
        }
        true
    }

    fn compute_extents(&mut self) {
        let mut extents = self.model.content_box;
        let mut grow = |b: BBox| {
            extents.union(&b);
        };
        for w in &self.out.wires {
            for p in w {
                grow(BBox {
                    x1: p.0,
                    y1: p.1,
                    x2: p.0,
                    y2: p.1,
                });
            }
        }
        for (name, at, rotation) in &self.out.net_labels {
            grow(label_text_box(name, *at, *rotation));
        }
        for (name, at, rotation, _) in &self.out.global_labels {
            grow(inflate(&label_text_box(name, *at, *rotation), 1.27));
        }
        for (name, _, at, rotation) in &self.out.hier_labels {
            grow(hier_text_box(name, *at, *rotation));
        }
        for (name, at, down) in &self.out.power_symbols {
            grow(power_symbol_graphic_box(name, *at, *down));
        }
        for &(f, rot) in &self.out.pwr_flags {
            grow(pwr_flag_box(f, rot));
        }
        for b in &self.out.blocks {
            grow(BBox {
                x1: b.at.0 - 5.08,
                y1: b.at.1 - 2.54,
                x2: b.at.0 + b.size.0 + 5.08,
                y2: b.at.1 + b.size.1 + 2.54,
            });
        }
        self.out.extents = extents;
    }

    /// Used by `plan_and_wire_group` (defined in [`crate::wiring`]) — kept
    /// here so both modules share one collapsed-endpoint cache.
    pub(crate) fn eps_for_net(&mut self, sn: usize) -> Vec<Endpoint> {
        self.eps_for(sn)
    }

    /// Group prediction for one net (shared with the orientation engine).
    pub(crate) fn group_subsets(&self, eps: &[Endpoint]) -> Vec<Vec<usize>> {
        let eps_placed: Vec<usize> = eps.iter().map(|e| e.placed).collect();
        let cfg = self.cfg;
        let design = self.design;
        let placed = &self.model.placed;
        group_wire_subsets(
            &eps_placed,
            &|pi| is_hub_comp(cfg, design, placed[pi].comp),
            &|pi| placed[pi].group_root,
            &|pi| design.comps[placed[pi].comp].refdes.clone(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seg(x1: f64, y1: f64, x2: f64, y2: f64, net: &str) -> Seg {
        Seg {
            x1,
            y1,
            x2,
            y2,
            net: net.to_string(),
        }
    }

    #[test]
    fn net_label_rotation_flips_locals_keeps_ports() {
        // #4: a global/hierarchical label keeps its outward (port) reading; a
        // local net label reads back over its wire — the outward rotation
        // flipped by 180°.
        for outward in [0, 180] {
            assert_eq!(net_label_rotation(true, outward), outward);
            assert_eq!(
                net_label_rotation(false, outward),
                (outward + 180).rem_euclid(360)
            );
        }
        // A local label at `at` on a rightward stub (wire runs +x from the
        // anchor) gets rotation 180, whose text box lies to the LEFT of the
        // anchor — over the wire — not off its far end.
        let at = (50.0, 20.0);
        let over = label_text_box("BUS", at, net_label_rotation(false, 0));
        assert!(
            over.x2 <= at.0 + 1e-6 && over.x1 < at.0,
            "text overhangs the wire"
        );
    }

    #[test]
    fn sibling_label_boxes_tolerate_a_two_step_stack_but_not_a_real_overlap() {
        // The soft-alignment guard clears sibling labels aligned onto one X
        // column when they sit two grid steps apart (their padded keepouts graze
        // by ~0.06 mm), but rejects a real overlap: a single grid step of stacking
        // or one label on top of another.
        let a = label_text_box("AIN_P", (50.0, 20.0), 180);
        let two_step = label_text_box("AIN_N", (50.0, 22.54), 180); // +2 grid steps
        let one_step = label_text_box("AIN_N", (50.0, 21.27), 180); // +1 grid step
        let on_top = label_text_box("AIN_N", (50.0, 20.0), 180); // same anchor
        assert!(
            !boxes_collide_tol(&a, &two_step, 0.5),
            "a tidy two-step stack must pass the tolerance"
        );
        assert!(
            boxes_collide_tol(&a, &one_step, 0.5),
            "a one-step stack is a real overlap"
        );
        assert!(
            boxes_collide_tol(&a, &on_top, 0.5),
            "coincident labels collide"
        );
    }

    #[test]
    fn frank_crossing_is_tolerated_but_touch_is_forbidden() {
        let h = seg(0.0, 0.0, 10.0, 0.0, "A");
        let v_cross = seg(5.0, -5.0, 5.0, 5.0, "B");
        assert!(segs_cross_frank(&h, &v_cross));
        assert!(!segs_touch_forbidden(&h, &v_cross));

        // T contact: the vertical ends ON the horizontal wire.
        let v_touch = seg(5.0, 0.0, 5.0, 5.0, "B");
        assert!(!segs_cross_frank(&h, &v_touch));
        assert!(segs_touch_forbidden(&h, &v_touch));

        // Collinear overlap on the same axis.
        let h2 = seg(8.0, 0.0, 15.0, 0.0, "B");
        assert!(segs_touch_forbidden(&h, &h2));
        let h3 = seg(11.0, 0.0, 15.0, 0.0, "B");
        assert!(!segs_touch_forbidden(&h, &h3));
    }

    #[test]
    fn power_attachment_shapes() {
        let cfg = SchConfig::default();
        // Vertical pin: straight stub.
        let a = power_attachment(&cfg, (10.0, 10.0), (0.0, 1.0), true, 0.0, 0.0);
        assert_eq!(a.path.len(), 2);
        assert_eq!(a.symbol_at, (10.0, 12.54));
        // Horizontal pin: elbow + vertical return (up for a rail).
        let b = power_attachment(&cfg, (10.0, 10.0), (1.0, 0.0), false, 0.0, 0.0);
        assert_eq!(b.path.len(), 3);
        assert_eq!(b.symbol_at, (12.54, 7.46));
    }

    #[test]
    fn power_potential_orders_rails_ground_negatives() {
        // Positive rail outranks ground outranks negative rail.
        assert_eq!(power_potential_rank("VDD", NetClass::Power), 1);
        assert_eq!(power_potential_rank("VCC", NetClass::Power), 1);
        assert_eq!(power_potential_rank("+5V", NetClass::Power), 1);
        assert_eq!(power_potential_rank("GND", NetClass::Ground), 0);
        assert_eq!(power_potential_rank("VSS", NetClass::Ground), 0);
        assert_eq!(power_potential_rank("VEE", NetClass::Power), -1);
        assert_eq!(power_potential_rank("-12V", NetClass::Power), -1);
        assert_eq!(power_potential_rank("V-", NetClass::Power), -1);
        assert!(
            power_potential_rank("VDD", NetClass::Power)
                > power_potential_rank("GND", NetClass::Ground)
        );
        assert!(
            power_potential_rank("GND", NetClass::Ground)
                > power_potential_rank("VEE", NetClass::Power)
        );
        // Grounds and negative rails hang down; positive rails point up.
        assert!(power_points_down("GND", NetClass::Ground));
        assert!(power_points_down("VEE", NetClass::Power));
        assert!(!power_points_down("VDD", NetClass::Power));
        assert!(!power_points_down("+3V3", NetClass::Power));
    }

    #[test]
    fn net_aware_crossings_split_foreign_from_same_net() {
        let mut reg = Reg::new(Vec::new());
        reg.register_path(&[(0.0, 0.0), (10.0, 0.0)], "A"); // horizontal wire of net A
        let vcross = [(5.0, -5.0), (5.0, 5.0)]; // vertical through the interior of A
        // Foreign net B: a frank crossing (tolerated under penalty), not same-net.
        assert_eq!(reg.foreign_crossings(&vcross, "B"), 1);
        assert!(!reg.same_net_frank_crossing(&vcross, "B"));
        // Same net A: a same-net frank crossing (a missing junction, forbidden).
        assert_eq!(reg.foreign_crossings(&vcross, "A"), 0);
        assert!(reg.same_net_frank_crossing(&vcross, "A"));
        // A T-contact that only touches the wire END is not a frank crossing.
        let vtouch = [(5.0, 0.0), (5.0, 5.0)];
        assert_eq!(reg.foreign_crossings(&vtouch, "B"), 0);
        assert!(!reg.same_net_frank_crossing(&vtouch, "A"));
        // count_crossings stays net-agnostic (sums both kinds).
        assert_eq!(reg.count_crossings(&vcross), 1);
    }

    #[test]
    fn label_box_allows_extra_nets() {
        let lb = LabelBox {
            bbox: BBox {
                x1: 0.0,
                y1: 0.0,
                x2: 1.0,
                y2: 1.0,
            },
            net: "A".to_string(),
            also: vec!["B".to_string()],
        };
        assert!(lb.allows("A"));
        assert!(lb.allows("B"));
        assert!(!lb.allows("C"));
    }
}
