//! Per-sheet placement engine (port of the placement half of the validated
//! `layout.ts` proof of concept).
//!
//! Strategy:
//! * majors are laid out left-to-right in **columns by signal flow**
//!   (input connectors, protection, regulation, MCU, sensors, others,
//!   output connectors), vertical order by connectivity BFS rank;
//! * satellites (anchored parts) stick to their anchor: decoupling rows
//!   below the IC, pulls above, bulk on the flank — regular rows centered
//!   on the anchor, spread by `satellite_pitch_mm`;
//! * two-pin drawn parts get a placement pre-orientation (power rail up,
//!   ground down, paired pin facing its mate);
//! * collision resolution works on **solid boxes** (body + pins + text
//!   allowances + predicted power/label stub keepouts) — ring search around
//!   the preferred direction for satellites, downward push for majors;
//! * components with a manual `# pcb:sch` position are **pinned**: they are
//!   never moved (auto-placed content flows around them, the whole sheet is
//!   only translated uniformly into the page).

use std::collections::{BTreeMap, HashMap};

use pcb_sch::position::MirrorAxis;

use crate::config::SchConfig;
use crate::geometry::BBox;
use crate::model::{DesignModel, NetClass, Role, Side};
use crate::round4;
use crate::sheets::{SheetDef, SheetPlan};
use crate::writer::PortDirection;

/// One component placed on a sheet.
pub struct PlacedComp {
    /// Index into `DesignModel::comps`.
    pub comp: usize,
    pub at: (f64, f64),
    pub rotation: i32,
    pub mirror: Option<MirrorAxis>,
    /// Manual `# pcb:sch` position: never moved by the engine.
    pub pinned: bool,
    /// Resolved anchor (index into the sheet's `placed` vector).
    pub anchor: Option<usize>,
    /// Root of the anchor chain (the component itself for majors). Two
    /// endpoints with the same root belong to one placement group and wire
    /// together with real wires ("everything goes together" rule).
    pub group_root: usize,
    pub side: Side,
    pub role: Role,
    /// Moved out of the functional flow into the right-hand utility band
    /// (decoupling/bulk relegation). Auto-placed parts only.
    pub relegated: bool,
    /// Solid box (sheet coordinates), kept in sync with `at`.
    pub bbox: BBox,
    /// Canonical text anchors (filled after the final translation).
    pub ref_at: (f64, f64),
    pub value_at: (f64, f64),
    pub value_justify_right: bool,
    /// Reference text is right-justified (anchor is its RIGHT edge). Set when
    /// the lateral text fallback drops the block to the LEFT of the body: the
    /// texts then glue their right edge to the component (symmetric with the
    /// right-side block, which reads left-justified). The canonical Reference
    /// is always left-justified, so this defaults to false.
    pub ref_justify_right: bool,
}

/// One net as seen from a sheet.
pub struct SheetNet {
    /// Index into `DesignModel::nets`.
    pub net: usize,
    pub name: String,
    pub class: NetClass,
    /// Local endpoints: (placed index, pad number).
    pub endpoints: Vec<(usize, String)>,
    /// Direction when this net is a hierarchical port of the sheet.
    pub port: Option<PortDirection>,
    /// The net appears on at least one child sheet block of this sheet.
    pub on_child_blocks: bool,
    /// Total endpoint count across the whole design.
    pub design_endpoints: usize,
}

/// A placed sheet, ready for routing and emission.
pub struct SheetModel {
    /// Index into the plan.
    pub sheet: usize,
    pub placed: Vec<PlacedComp>,
    pub nets: Vec<SheetNet>,
    /// (placed index, pad) -> index into `nets`.
    pub pin_net: HashMap<(usize, String), usize>,
    /// Content bounding box after placement (solids included).
    pub content_box: BBox,
    /// Utility relegation is active on this sheet (it carries a real IC and
    /// the feature is enabled): decoupling caps were pulled into the right
    /// band, undriven-rail flags are relegated below them by the router, and
    /// the functional/decoupling/ERC zones are outlined.
    pub relegate: bool,
}

/// Signal-flow categories, left to right.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Category {
    ConnLeft,
    Protection,
    Regulator,
    Mcu,
    Sensor,
    Other,
    ConnRight,
}

const CATEGORY_ORDER: [Category; 7] = [
    Category::ConnLeft,
    Category::Protection,
    Category::Regulator,
    Category::Mcu,
    Category::Sensor,
    Category::Other,
    Category::ConnRight,
];

fn categorize(role: Role) -> Category {
    match role {
        Role::Connector => Category::ConnLeft,
        Role::Protection => Category::Protection,
        Role::Regulator => Category::Regulator,
        Role::Mcu => Category::Mcu,
        Role::Sensor => Category::Sensor,
        _ => Category::Other,
    }
}

/// Roles allowed to rotate (drawn two-pin parts). Boxes (ICs, connectors)
/// only mirror — and mirroring is decided by the orientation engine.
pub(crate) fn is_rotatable(role: Role) -> bool {
    matches!(
        role,
        Role::Passive | Role::Decoupling | Role::Bulk | Role::Pull | Role::Led | Role::Crystal
    )
}

// ----------------------------------------------------------------------
// Geometry helpers
// ----------------------------------------------------------------------

pub(crate) fn apply_mirror(x: f64, y: f64, mirror: Option<MirrorAxis>) -> (f64, f64) {
    match mirror {
        Some(MirrorAxis::X) => (x, -y),
        Some(MirrorAxis::Y) => (-x, y),
        None => (x, y),
    }
}

pub(crate) fn rotate_ccw(x: f64, y: f64, rotation: i32) -> (f64, f64) {
    match rotation.rem_euclid(360) {
        90 => (-y, x),
        180 => (-x, -y),
        270 => (y, -x),
        _ => (x, y),
    }
}

/// Sheet-coordinate bounding box of a symbol instance (raw: body + pin
/// connection points, no text allowances).
pub(crate) fn raw_box(
    geom: &crate::geometry::SymbolGeom,
    at: (f64, f64),
    rotation: i32,
    mirror: Option<MirrorAxis>,
) -> BBox {
    let lb = &geom.full_bbox;
    let mut out = BBox::EMPTY;
    for (x, y) in [
        (lb.x1, lb.y1),
        (lb.x1, lb.y2),
        (lb.x2, lb.y1),
        (lb.x2, lb.y2),
    ] {
        let (mx, my) = apply_mirror(x, y, mirror);
        let (rx, ry) = rotate_ccw(mx, my, rotation);
        out.include(round4(at.0 + rx), round4(at.1 - ry));
    }
    out
}

/// Body-only box (graphics without pins) in sheet coordinates.
pub(crate) fn body_box(
    geom: &crate::geometry::SymbolGeom,
    at: (f64, f64),
    rotation: i32,
    mirror: Option<MirrorAxis>,
) -> BBox {
    let lb = &geom.body_bbox;
    let mut out = BBox::EMPTY;
    for (x, y) in [
        (lb.x1, lb.y1),
        (lb.x1, lb.y2),
        (lb.x2, lb.y1),
        (lb.x2, lb.y2),
    ] {
        let (mx, my) = apply_mirror(x, y, mirror);
        let (rx, ry) = rotate_ccw(mx, my, rotation);
        out.include(round4(at.0 + rx), round4(at.1 - ry));
    }
    out
}

pub(crate) fn inflate(b: &BBox, m: f64) -> BBox {
    BBox {
        x1: b.x1 - m,
        y1: b.y1 - m,
        x2: b.x2 + m,
        y2: b.y2 + m,
    }
}

pub(crate) fn overlaps(a: &BBox, b: &BBox) -> bool {
    a.x1 < b.x2 && b.x1 < a.x2 && a.y1 < b.y2 && b.y1 < a.y2
}

fn union_box(a: &BBox, b: &BBox) -> BBox {
    let mut out = *a;
    out.union(b);
    out
}

fn point_box(p: (f64, f64), m: f64) -> BBox {
    BBox {
        x1: p.0 - m,
        y1: p.1 - m,
        x2: p.0 + m,
        y2: p.1 + m,
    }
}

/// Upper bound of a label text width (~1.27 mm per character + margin).
pub(crate) fn label_text_width(name: &str) -> f64 {
    (name.chars().count() as f64 + 1.0) * 1.27
}

/// Solid box of a component for an explicit transform: raw box (body +
/// pins) plus the Reference above the top-left corner and the Value below
/// (text widths included). Shared by the placement engine and the
/// orientation engine (which probes candidate transforms).
pub(crate) fn solid_box_for(
    cfg: &SchConfig,
    design: &DesignModel,
    p: &PlacedComp,
    rotation: i32,
    mirror: Option<MirrorAxis>,
) -> BBox {
    let geom = &design.comps[p.comp].geom;
    let raw = raw_box(geom, p.at, rotation, mirror);
    let comp = &design.comps[p.comp];
    let ref_w = label_text_width(&comp.refdes);
    let val_w = label_text_width(&comp.value);
    let mut b = raw;
    b.y1 -= cfg.ref_gap_grid_steps as f64 * cfg.grid_mm + 2.54;
    b.x2 = b.x2.max(raw.x1 + ref_w);
    b.y2 += cfg.value_gap_grid_steps as f64 * cfg.grid_mm + 2.54;
    let has_bottom = geom.pins.iter().filter(|pin| !pin.hidden).any(|pin| {
        geom.pin_outward(&pin.number, rotation, mirror)
            .map(|d| d.1 > 0.5)
            .unwrap_or(false)
    });
    if has_bottom {
        b.x1 = b.x1.min(raw.x2 - val_w);
    } else {
        b.x2 = b.x2.max(raw.x1 + val_w);
    }
    b
}

/// Length of the horizontal wire carrying a net label: the wire must
/// underline the full text and respect the configured minimum.
pub(crate) fn label_stub_len(cfg: &SchConfig, name: &str) -> f64 {
    let min_mm = cfg.label_stub_min_grid_steps as f64 * cfg.grid_mm;
    cfg.snap_up(min_mm.max(cfg.stub_mm).max(label_text_width(name)))
}

// ----------------------------------------------------------------------
// The engine
// ----------------------------------------------------------------------

struct Engine<'a> {
    cfg: &'a SchConfig,
    design: &'a DesignModel,
    placed: Vec<PlacedComp>,
    nets: Vec<SheetNet>,
    pin_net: HashMap<(usize, String), usize>,
    warnings: &'a mut Vec<String>,
    /// Set by `run`: this sheet is eligible for utility relegation.
    relegate: bool,
}

/// Build and place one sheet.
pub fn place_sheet(
    design: &DesignModel,
    plan: &SheetPlan,
    sheet_idx: usize,
    cfg: &SchConfig,
    warnings: &mut Vec<String>,
) -> SheetModel {
    let sheet = &plan.sheets[sheet_idx];

    // ------------------------------------------------------------------
    // Components of the sheet.
    // ------------------------------------------------------------------
    let mut placed: Vec<PlacedComp> = sheet
        .comps
        .iter()
        .map(|&ci| {
            let comp = &design.comps[ci];
            let (at, rotation, mirror, pinned) = match &comp.manual {
                Some(m) => (m.at, m.rotation, m.mirror, true),
                None => ((0.0, 0.0), 0, None, false),
            };
            PlacedComp {
                comp: ci,
                at,
                rotation,
                mirror,
                pinned,
                anchor: None,
                group_root: 0, // resolved in `run`
                side: Side::default_for(comp.role),
                role: comp.role,
                relegated: false,
                bbox: BBox::EMPTY,
                ref_at: (0.0, 0.0),
                value_at: (0.0, 0.0),
                value_justify_right: false,
                ref_justify_right: false,
            }
        })
        .collect();

    // Snap pinned origins onto the grid: the viewer snaps the *anchor*
    // (symbol bbox corner) to its grid, so the derived origin is off-grid
    // by a sub-grid residue inherent to the mapping. Corrections below half
    // a grid step are expected and silent; anything larger would reveal a
    // conversion bug and is worth a warning.
    for p in placed.iter_mut().filter(|p| p.pinned) {
        let snapped = (cfg.snap(p.at.0), cfg.snap(p.at.1));
        let (dx, dy) = (snapped.0 - p.at.0, snapped.1 - p.at.1);
        if dx.abs() > cfg.grid_mm / 2.0 + 1e-6 || dy.abs() > cfg.grid_mm / 2.0 + 1e-6 {
            warnings.push(format!(
                "manual position of {} snapped onto the grid ({dx:.3} mm, {dy:.3} mm)",
                design.comps[p.comp].refdes,
            ));
        }
        p.at = snapped;
    }

    // ------------------------------------------------------------------
    // Sheet nets.
    // ------------------------------------------------------------------
    let comp_to_placed: HashMap<usize, usize> = placed
        .iter()
        .enumerate()
        .map(|(pi, p)| (p.comp, pi))
        .collect();
    let port_by_net: BTreeMap<usize, PortDirection> =
        sheet.ports.iter().map(|p| (p.net, p.direction)).collect();
    let child_port_nets: std::collections::BTreeSet<usize> = sheet
        .children
        .iter()
        .flat_map(|&c| plan.sheets[c].ports.iter().map(|p| p.net))
        .collect();

    let mut nets: Vec<SheetNet> = Vec::new();
    let mut pin_net: HashMap<(usize, String), usize> = HashMap::new();
    for (ni, net) in design.nets.iter().enumerate() {
        let endpoints: Vec<(usize, String)> = net
            .endpoints
            .iter()
            .filter_map(|(ci, pad)| comp_to_placed.get(ci).map(|&pi| (pi, pad.clone())))
            .collect();
        let is_port = port_by_net.contains_key(&ni);
        let on_blocks = child_port_nets.contains(&ni);
        if endpoints.is_empty() && !is_port && !on_blocks {
            continue;
        }
        let idx = nets.len();
        for (pi, pad) in &endpoints {
            pin_net.insert((*pi, pad.clone()), idx);
        }
        nets.push(SheetNet {
            net: ni,
            name: net.name.clone(),
            class: net.class,
            endpoints,
            port: port_by_net.get(&ni).copied(),
            on_child_blocks: on_blocks,
            design_endpoints: net.endpoints.len(),
        });
    }

    let mut engine = Engine {
        cfg,
        design,
        placed,
        nets,
        pin_net,
        warnings,
        relegate: false,
    };
    engine.resolve_anchors();
    engine.run(sheet);

    let content_box = engine.content_box();
    let relegate = engine.relegate;
    SheetModel {
        sheet: sheet_idx,
        placed: engine.placed,
        nets: engine.nets,
        pin_net: engine.pin_net,
        content_box,
        relegate,
    }
}

impl<'a> Engine<'a> {
    // --------------------------------------------------------------
    // Anchors
    // --------------------------------------------------------------

    /// Resolve satellite anchors: explicit `anchor` attribute first, then
    /// connectivity heuristics (decoupling/bulk -> the hub sharing the rail,
    /// pull/led/filter -> the major sharing the signal net, crystal -> the
    /// major sharing both crystal nets).
    fn resolve_anchors(&mut self) {
        let majors: Vec<usize> = (0..self.placed.len())
            .filter(|&pi| {
                !self.placed[pi].role.is_satellite() && self.placed[pi].role != Role::Crystal
            })
            .collect();

        for pi in 0..self.placed.len() {
            let comp = &self.design.comps[self.placed[pi].comp];

            // Explicit anchor attribute: sibling instance name or refdes.
            if let Some(attr) = &comp.anchor_attr {
                let target = self.placed.iter().position(|p| {
                    let c = &self.design.comps[p.comp];
                    c.refdes == *attr
                        || c.path.last().map(String::as_str) == Some(attr.as_str())
                        || c.path_key.ends_with(&format!(".{attr}"))
                });
                match target {
                    Some(ti) if ti != pi => {
                        self.placed[pi].anchor = Some(ti);
                        if let Some(side) = comp.side_attr {
                            self.placed[pi].side = side;
                        }
                        continue;
                    }
                    _ => self.warnings.push(format!(
                        "anchor \"{attr}\" of {} not found on its sheet — placed by flow",
                        comp.refdes
                    )),
                }
            }

            // Heuristic anchors for satellite roles.
            let role = self.placed[pi].role;
            let wanted_class = match role {
                Role::Decoupling | Role::Bulk => Some(true), // power/ground net
                Role::Pull | Role::Led | Role::Passive => Some(false), // signal net
                Role::Crystal => Some(false),
                _ => None,
            };
            let Some(power_side) = wanted_class else {
                continue;
            };
            if self.design.comps[self.placed[pi].comp].visible_pins != 2 {
                continue;
            }
            // Passives only become satellites when they touch a rail on one
            // side (filter/pull pattern); a series part on signal-only nets
            // stays in the flow UNLESS it is the terminal element of an
            // input/output chain feeding an IC (connector/external label ->
            // passives -> IC pin) — then it rides next to that IC's pin.
            let my_nets: Vec<usize> = self.placed_net_indices(pi);
            let classes: Vec<NetClass> = my_nets.iter().map(|&sn| self.nets[sn].class).collect();
            if role == Role::Passive
                && !(classes.contains(&NetClass::Ground) || classes.contains(&NetClass::Power))
            {
                if let Some((ti, side)) = self.input_chain_anchor(pi, &my_nets, &majors) {
                    self.placed[pi].anchor = Some(ti);
                    self.placed[pi].side = side;
                }
                continue;
            }

            let candidate_nets: Vec<usize> = my_nets
                .iter()
                .copied()
                .filter(|&sn| {
                    let is_power = self.nets[sn].class != NetClass::Signal;
                    is_power == power_side
                })
                .collect();
            let mut best: Option<usize> = None;
            for &sn in &candidate_nets {
                for (opi, _) in &self.nets[sn].endpoints {
                    if *opi == pi || !majors.contains(opi) {
                        continue;
                    }
                    let better = match best {
                        None => true,
                        Some(cur) => {
                            let a = self.design.comps[self.placed[*opi].comp].visible_pins;
                            let b = self.design.comps[self.placed[cur].comp].visible_pins;
                            a > b
                                || (a == b
                                    && natord::compare(
                                        &self.design.comps[self.placed[*opi].comp].refdes,
                                        &self.design.comps[self.placed[cur].comp].refdes,
                                    )
                                    .is_lt())
                        }
                    };
                    if better {
                        best = Some(*opi);
                    }
                }
            }
            if role == Role::Crystal {
                // Crystal: both nets must land on the same major.
                let sig: Vec<usize> = my_nets
                    .iter()
                    .copied()
                    .filter(|&sn| self.nets[sn].class == NetClass::Signal)
                    .collect();
                if sig.len() == 2 {
                    let on_both = |m: usize| {
                        sig.iter()
                            .all(|&sn| self.nets[sn].endpoints.iter().any(|(opi, _)| *opi == m))
                    };
                    best = best.filter(|&m| on_both(m));
                } else {
                    best = None;
                }
            }
            if let Some(ti) = best {
                self.placed[pi].anchor = Some(ti);
            }
        }

        self.chain_shunt_caps();
    }

    /// Re-parent a shunt/filter capacitor onto the series resistor it filters
    /// with: a two-pin cap that touches ground and shares its signal node with
    /// an input-chain resistor rides just below that resistor (IC -> R -> C),
    /// so the filtered node stays a tight local cluster instead of stretching
    /// from the IC body down to a flank cap. Decoupling caps across two rails
    /// (no signal net) are untouched.
    fn chain_shunt_caps(&mut self) {
        let n = self.placed.len();
        for ci in 0..n {
            if !matches!(self.placed[ci].role, Role::Passive | Role::Decoupling) {
                continue;
            }
            if self.design.comps[self.placed[ci].comp].visible_pins != 2 {
                continue;
            }
            let my = self.placed_net_indices(ci);
            if !my.iter().any(|&sn| self.nets[sn].class == NetClass::Ground) {
                continue;
            }
            let mut target: Option<usize> = None;
            for &sn in my
                .iter()
                .filter(|&&sn| self.nets[sn].class == NetClass::Signal)
            {
                for (opi, _) in &self.nets[sn].endpoints {
                    if *opi == ci || !self.is_input_chain_resistor(*opi) {
                        continue;
                    }
                    target = Some(match target {
                        None => *opi,
                        Some(t) => {
                            if natord::compare(
                                &self.design.comps[self.placed[*opi].comp].refdes,
                                &self.design.comps[self.placed[t].comp].refdes,
                            )
                            .is_lt()
                            {
                                *opi
                            } else {
                                t
                            }
                        }
                    });
                }
            }
            if let Some(r) = target {
                self.placed[ci].anchor = Some(r);
                self.placed[ci].side = Side::Below; // shunt hangs down to ground
            }
        }
    }

    /// An input-chain series resistor: a two-pin passive anchored to a part
    /// while touching signal nets only (the terminal element seated at an IC
    /// input by [`input_chain_anchor`]).
    fn is_input_chain_resistor(&self, pi: usize) -> bool {
        self.placed[pi].role == Role::Passive
            && self.placed[pi].anchor.is_some()
            && self.design.comps[self.placed[pi].comp].visible_pins == 2
            && self
                .placed_net_indices(pi)
                .iter()
                .all(|&sn| self.nets[sn].class == NetClass::Signal)
    }

    /// Sheet-net indices touched by a placed component (dedup, pad order).
    fn placed_net_indices(&self, pi: usize) -> Vec<usize> {
        let mut out = Vec::new();
        let comp = &self.design.comps[self.placed[pi].comp];
        let mut pads: Vec<&String> = comp
            .geom
            .pins
            .iter()
            .filter(|p| !p.hidden)
            .map(|p| &p.number)
            .collect();
        pads.sort_by(|a, b| natord::compare(a, b));
        for pad in pads {
            if let Some(&sn) = self.pin_net.get(&(pi, pad.clone()))
                && !out.contains(&sn)
            {
                out.push(sn);
            }
        }
        out
    }

    /// Is this placed component a real multi-pin part (an IC/major worth
    /// anchoring a satellite to)? A major with at least the configured pin
    /// count — never a two-pin drawn passive.
    fn is_ic_major(&self, pi: usize) -> bool {
        !self.placed[pi].role.is_satellite()
            && self.placed[pi].role != Role::Crystal
            && self.design.comps[self.placed[pi].comp].visible_pins >= self.cfg.input_chain_min_pins
    }

    /// Terminal-chain anchor for a series two-pin passive that only touches
    /// signal nets. It rides next to the IC it feeds when: one of its nets is
    /// analog and lands on an IC major (`is_ic_major`), and its OTHER net is
    /// a genuine chain terminal — it reaches no *other* IC major (a connector
    /// port, an external label or a shunt to more passives, never an
    /// IC-to-IC interconnect). Returns the IC and the side its shared pin
    /// exits on. Deterministic: the IC with the most pins wins, refdes breaks
    /// ties.
    fn input_chain_anchor(
        &self,
        pi: usize,
        my_nets: &[usize],
        majors: &[usize],
    ) -> Option<(usize, Side)> {
        let mut best: Option<(usize, usize)> = None; // (major, shared net)
        for &sn in my_nets {
            if self.nets[sn].class != NetClass::Signal {
                continue;
            }
            if self.design.nets[self.nets[sn].net].digital {
                continue; // digital nets ride labels, never a wired satellite
            }
            // The passive's OTHER net(s) must not reach a second IC major:
            // an IC-to-IC series part belongs to the flow, not to a flank.
            let other_reaches_ic = my_nets.iter().any(|&on| {
                on != sn
                    && self.nets[on]
                        .endpoints
                        .iter()
                        .any(|(opi, _)| *opi != pi && self.is_ic_major(*opi))
            });
            if other_reaches_ic {
                continue;
            }
            for (opi, _) in &self.nets[sn].endpoints {
                if *opi == pi || !majors.contains(opi) || !self.is_ic_major(*opi) {
                    continue;
                }
                let better = match best {
                    None => true,
                    Some((cur, _)) => {
                        let a = self.design.comps[self.placed[*opi].comp].visible_pins;
                        let b = self.design.comps[self.placed[cur].comp].visible_pins;
                        a > b
                            || (a == b
                                && natord::compare(
                                    &self.design.comps[self.placed[*opi].comp].refdes,
                                    &self.design.comps[self.placed[cur].comp].refdes,
                                )
                                .is_lt())
                    }
                };
                if better {
                    best = Some((*opi, sn));
                }
            }
        }
        let (major, sn) = best?;
        let side = self.major_pin_side(major, sn).unwrap_or(Side::Left);
        Some((major, side))
    }

    /// Side of a major on which its pin carrying net `sn` exits, in the
    /// major's current orientation (used to seat an input-chain satellite on
    /// the pin it feeds rather than under a default flank).
    fn major_pin_side(&self, major: usize, sn: usize) -> Option<Side> {
        let p = &self.placed[major];
        let geom = &self.design.comps[p.comp].geom;
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            if self.pin_net.get(&(major, pin.number.clone())) != Some(&sn) {
                continue;
            }
            let dir = geom.pin_outward(&pin.number, p.rotation, p.mirror)?;
            return Some(if dir.0 < -0.5 {
                Side::Left
            } else if dir.0 > 0.5 {
                Side::Right
            } else if dir.1 < -0.5 {
                Side::Above
            } else {
                Side::Below
            });
        }
        None
    }

    /// Sheet Y of the IC pin that carries one of `r`'s nets (the pin the
    /// input-chain resistor `r` feeds). Used to order a filter column so the
    /// short wires never cross.
    fn ic_pin_y(&self, ic: usize, r: usize) -> Option<f64> {
        let r_nets = self.placed_net_indices(r);
        let p = &self.placed[ic];
        let geom = &self.design.comps[p.comp].geom;
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(&sn) = self.pin_net.get(&(ic, pin.number.clone())) else {
                continue;
            };
            if !r_nets.contains(&sn) {
                continue;
            }
            if let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) {
                return Some(pos.1);
            }
        }
        None
    }

    /// Sheet position and outward unit direction of the first IC pin carrying
    /// net `sn` — the pin a satellite on that net feeds or pulls. Used to seat
    /// a pull directly over the pin it pulls.
    fn ic_pin_pos_dir(&self, ic: usize, sn: usize) -> Option<((f64, f64), (f64, f64))> {
        let p = &self.placed[ic];
        let geom = &self.design.comps[p.comp].geom;
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            if self.pin_net.get(&(ic, pin.number.clone())) != Some(&sn) {
                continue;
            }
            let pos = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror)?;
            let dir = geom.pin_outward(&pin.number, p.rotation, p.mirror)?;
            return Some((pos, dir));
        }
        None
    }

    /// Shunt capacitors re-parented under a resistor by [`chain_shunt_caps`].
    fn shunt_caps_of(&self, r: usize) -> Vec<usize> {
        let mut caps: Vec<usize> = (0..self.placed.len())
            .filter(|&c| {
                !self.placed[c].pinned
                    && self.placed[c].anchor == Some(r)
                    && matches!(self.placed[c].role, Role::Passive | Role::Decoupling)
            })
            .collect();
        caps.sort_by(|&a, &b| {
            natord::compare(
                &self.design.comps[self.placed[a].comp].refdes,
                &self.design.comps[self.placed[b].comp].refdes,
            )
        });
        caps
    }

    /// Re-seat input filters into tidy aligned clusters (proximity +
    /// alignment). Each IC's input-chain resistors stack in one column just
    /// off the IC's input edge: horizontal resistors on a shared X, ordered by
    /// the IC pin they feed, each shunt capacitor hanging directly below its
    /// resistor. The cluster is compact and collision-free by construction.
    fn tidy_input_filters(&mut self) {
        let mut by_ic: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for pi in 0..self.placed.len() {
            if self.placed[pi].pinned || !self.is_input_chain_resistor(pi) {
                continue;
            }
            let Some(a) = self.placed[pi].anchor else {
                continue;
            };
            if self.is_ic_major(a) {
                by_ic.entry(a).or_default().push(pi);
            }
        }
        for (ic, rs) in by_ic {
            // Keep only the resistors seated on the dominant horizontal side.
            let side = self.placed[rs[0]].side;
            if !matches!(side, Side::Left | Side::Right) {
                continue;
            }
            let mut column: Vec<usize> = rs
                .into_iter()
                .filter(|&r| self.placed[r].side == side)
                .collect();
            if column.is_empty() {
                continue;
            }
            self.stack_filter_column(ic, &mut column, side);
        }
    }

    /// Stack one IC's input filter as a tidy cluster off `side` of the IC: the
    /// series resistors align on a shared X (horizontal, ordered by the IC pin
    /// they feed) and every shunt capacitor drops into one aligned row (shared
    /// Y) below the column, spread across the widened filter zone between the
    /// column and the IC — the shape the engineer draws for a differential
    /// input filter. The caps stay on the filtered-node side of the column, so
    /// their drops never reach the resistors' external stubs (no short) and the
    /// row reads as one clean bank.
    fn stack_filter_column(&mut self, ic: usize, rs: &mut [usize], side: Side) {
        let cfg = self.cfg;
        // Resistors read horizontal in a vertical column; caps stand upright.
        for &r in rs.iter() {
            self.placed[r].rotation = 90;
            self.placed[r].mirror = None;
            self.placed[r].bbox = self.solid_box(r);
            for c in self.shunt_caps_of(r) {
                self.placed[c].rotation = 0;
                self.placed[c].mirror = None;
                self.placed[c].bbox = self.solid_box(c);
            }
        }
        // Order top-to-bottom by the IC pin each resistor feeds.
        rs.sort_by(|&a, &b| {
            let ya = self.ic_pin_y(ic, a).unwrap_or(f64::MAX);
            let yb = self.ic_pin_y(ic, b).unwrap_or(f64::MAX);
            ya.partial_cmp(&yb)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    natord::compare(
                        &self.design.comps[self.placed[a].comp].refdes,
                        &self.design.comps[self.placed[b].comp].refdes,
                    )
                })
        });
        let caps: Vec<usize> = rs.iter().flat_map(|&r| self.shunt_caps_of(r)).collect();

        // Widen the gap to the IC so the cap row fits on the node side.
        let ic_raw = self.raw_box_of(ic);
        let r_half = rs
            .iter()
            .map(|&r| {
                let b = raw_box(
                    &self.design.comps[self.placed[r].comp].geom,
                    (0.0, 0.0),
                    self.placed[r].rotation,
                    None,
                );
                (b.x2 - b.x1) / 2.0
            })
            .fold(0.0f64, f64::max);
        let c_pitch = caps
            .iter()
            .map(|&c| cfg.snap_up((self.placed[c].bbox.x2 - self.placed[c].bbox.x1) + cfg.grid_mm))
            .fold(0.0f64, f64::max)
            .max(cfg.satellite_pitch_mm);
        let cap_span = if caps.len() > 1 {
            (caps.len() as f64 - 1.0) * c_pitch
        } else {
            0.0
        };
        // Spread the filter off the IC edge with a generous band of air (the
        // sheet is mostly empty): the filtered nets then run as long, but still
        // continuous, wires and a pin-seated pull-up at the IC edge stays clear
        // of the filter caps. The gap is at least the room the shunt-cap row
        // needs on the node side. It is NO LONGER bounded by any wire-length
        // limit: the filter→IC wire (and its jogged sibling leg) stays a
        // continuous wire whatever its length, so the filter spreads to match
        // the reference layout (the AD7171 pushes its filter far to the left,
        // its IC well to the right) — bounded only by the sensible default.
        let gap = if caps.is_empty() {
            cfg.satellite_gap_mm
        } else {
            let min_for_caps = cfg.snap_up(2.0 * cfg.satellite_gap_mm + cap_span);
            cfg.snap_up(cfg.input_filter_gap_mm.max(min_for_caps))
        };
        let colx = match side {
            Side::Left => cfg.snap(ic_raw.x1 - gap - r_half),
            _ => cfg.snap(ic_raw.x2 + gap + r_half),
        };

        // Resistor column: tight uniform pitch, centered on the fed IC pins.
        let r_pitch = rs
            .iter()
            .map(|&r| cfg.snap_up((self.placed[r].bbox.y2 - self.placed[r].bbox.y1) + cfg.grid_mm))
            .fold(0.0f64, f64::max);
        // Align the top resistor exactly on the IC pin it feeds so its leg is
        // a straight horizontal wire into the pin (0 bends), then stack the
        // rest of the column below at `r_pitch`. The pins of a tight
        // differential input sit closer than one resistor cell, so only one leg
        // can be perfectly horizontal — the engineer aligns the top one (AIN+)
        // and lets the others jog. Falls back to centering on the average pin Y
        // when the fed pin's ordinate is unknown.
        let top = match self.ic_pin_y(ic, rs[0]) {
            Some(y) => cfg.snap(y),
            None => {
                let avg_pin_y = {
                    let ys: Vec<f64> = rs.iter().filter_map(|&r| self.ic_pin_y(ic, r)).collect();
                    if ys.is_empty() {
                        self.placed[ic].at.1
                    } else {
                        ys.iter().sum::<f64>() / ys.len() as f64
                    }
                };
                cfg.snap(avg_pin_y - r_pitch * (rs.len() as f64 - 1.0) / 2.0)
            }
        };
        let mut col_bottom = f64::NEG_INFINITY;
        for (i, &r) in rs.iter().enumerate() {
            let ry = round4(top + i as f64 * r_pitch);
            let rb = self.placed[r].bbox;
            let dy = cfg.snap(ry - (rb.y1 + rb.y2) / 2.0);
            self.placed[r].at = (colx, round4(self.placed[r].at.1 + dy));
            self.placed[r].bbox = self.solid_box(r);
            col_bottom = col_bottom.max(self.placed[r].bbox.y2);
        }
        if caps.is_empty() {
            return;
        }

        // Cap row: aligned Y below the column and below the IC body, spread
        // across the filter zone on the node side of the resistors.
        let c_half = caps
            .iter()
            .map(|&c| (self.placed[c].bbox.y2 - self.placed[c].bbox.y1) / 2.0)
            .fold(0.0f64, f64::max);
        let row_y = cfg.snap(col_bottom.max(ic_raw.y2) + cfg.satellite_gap_mm + c_half);
        let start_x = match side {
            Side::Left => cfg.snap(colx + r_half + cfg.satellite_gap_mm),
            _ => cfg.snap(colx - r_half - cfg.satellite_gap_mm - cap_span),
        };
        for (i, &c) in caps.iter().enumerate() {
            let cx = round4(start_x + i as f64 * c_pitch);
            let cb = self.placed[c].bbox;
            let dx = cfg.snap(cx - (cb.x1 + cb.x2) / 2.0);
            let dy = cfg.snap(row_y - (cb.y1 + cb.y2) / 2.0);
            self.placed[c].at = (
                round4(self.placed[c].at.0 + dx),
                round4(self.placed[c].at.1 + dy),
            );
            self.placed[c].bbox = self.solid_box(c);
        }
    }

    /// Re-seat each pull resistor directly over the IC pin it pulls so its free
    /// leg drops straight onto that pin's exit stub as one continuous wire. The
    /// engineer seats the DOUT pull-up just off the pin it pulls, not floating
    /// over the IC body where the net would be forced to break into a label.
    /// Only pulls anchored to an IC major, standing upright, whose pulled pin
    /// exits horizontally are re-seated: the vertical drop then lands on the
    /// pin's horizontal stub just outside the body. Purely a move along X — the
    /// vertical position (above/below the IC) and the netlist are untouched.
    fn reseat_pullups(&mut self) {
        let cfg = self.cfg;
        for pi in 0..self.placed.len() {
            if self.placed[pi].pinned || self.placed[pi].role != Role::Pull {
                continue;
            }
            // Upright two-pin part only (rail leg vertical, drop on its axis).
            if !matches!(self.placed[pi].rotation.rem_euclid(360), 0 | 180) {
                continue;
            }
            let Some(ic) = self.placed[pi].anchor else {
                continue;
            };
            if !self.is_ic_major(ic) {
                continue;
            }
            // The pulled signal pin on the IC (the pull's non-power net).
            let my_nets = self.placed_net_indices(pi);
            let Some(&sig) = my_nets
                .iter()
                .find(|&&sn| self.nets[sn].class == NetClass::Signal)
            else {
                continue;
            };
            let Some((pin_pos, dir)) = self.ic_pin_pos_dir(ic, sig) else {
                continue;
            };
            // Only horizontal-exit pins: the upright pull's drop then lands on
            // the pin's horizontal stub. A vertical-exit pin already faces the
            // pull; leave it to the generic placement.
            if dir.0.abs() < 0.5 {
                continue;
            }
            // Align the pull's vertical axis (its drop) onto the pin's exit
            // stub, `pullup_pin_gap_mm` outside the body; keep its Y.
            let colx = cfg.snap(pin_pos.0 + dir.0 * cfg.pullup_pin_gap_mm);
            self.placed[pi].at = (colx, self.placed[pi].at.1);
            self.placed[pi].bbox = self.solid_box(pi);
        }
    }

    /// Does the sheet carry a real IC (a part with at least
    /// `input_chain_min_pins` visible pins)? Only such sheets get their
    /// decoupling relegated and their zones outlined — a passive-only sheet
    /// has no functional flow to keep clean.
    fn has_ic_major(&self) -> bool {
        (0..self.placed.len()).any(|pi| self.is_ic_major(pi))
    }

    /// Relegate the auto-placed rail-to-rail capacitors (decoupling and bulk)
    /// into a tidy row in the right-hand utility band, clear of the
    /// functional flow. Each cap is stood upright (rail pin up, ground pin
    /// down) so the router caps it with a power symbol on top and a ground
    /// symbol below; the caps align on a shared row centered just under the
    /// functional top and spread by a uniform pitch. Manually positioned
    /// parts are exempt. Purely a move — the nets are untouched.
    fn relegate_decoupling(&mut self) {
        let cfg = self.cfg;
        let mut targets: Vec<usize> = (0..self.placed.len())
            .filter(|&pi| {
                !self.placed[pi].pinned
                    && matches!(self.placed[pi].role, Role::Decoupling | Role::Bulk)
            })
            .collect();
        if targets.is_empty() {
            return;
        }
        targets.sort_by(|&a, &b| {
            natord::compare(
                &self.design.comps[self.placed[a].comp].refdes,
                &self.design.comps[self.placed[b].comp].refdes,
            )
        });
        let target_set: std::collections::BTreeSet<usize> = targets.iter().copied().collect();

        // Right edge and top of everything that stays in the flow. The right
        // edge uses the predicted routed keepouts (`solids`: power-symbol
        // doglegs and label stubs), not just the body/text box — an IC spreads
        // its right-edge power symbols well past its body, and the relegated
        // band must clear that whole extent, not overlap it.
        let mut func_right = f64::NEG_INFINITY;
        let mut func_top = f64::INFINITY;
        for pi in 0..self.placed.len() {
            if target_set.contains(&pi) {
                continue;
            }
            func_top = func_top.min(self.placed[pi].bbox.y1);
            for tb in self.solids(pi) {
                func_right = func_right.max(tb.bbox.x2);
            }
        }
        if !func_right.is_finite() {
            return; // the sheet is only caps — nothing to separate from
        }

        // Stand every cap upright (rail up, ground down) and size the row.
        for &t in &targets {
            self.orient_two_pin(t);
            self.placed[t].bbox = self.solid_box(t);
        }
        let cell_w = targets
            .iter()
            .map(|&t| self.placed[t].bbox.x2 - self.placed[t].bbox.x1)
            .fold(0.0f64, f64::max);
        let pitch = cfg.snap_up(cell_w + cfg.grid_mm).max(cfg.utility_pitch_mm);
        let x0 = cfg.snap(func_right + cfg.utility_gap_mm + cell_w / 2.0);
        // The row sits just below the functional top so the upward rail
        // symbols stay inside the sheet's vertical span.
        let row_cy = cfg.snap(func_top + 12.7);
        for (i, &t) in targets.iter().enumerate() {
            let cx = round4(x0 + i as f64 * pitch);
            let b = self.placed[t].bbox;
            let dx = cfg.snap(cx - (b.x1 + b.x2) / 2.0);
            let dy = cfg.snap(row_cy - (b.y1 + b.y2) / 2.0);
            self.placed[t].at = (
                round4(self.placed[t].at.0 + dx),
                round4(self.placed[t].at.1 + dy),
            );
            self.placed[t].relegated = true;
            self.placed[t].bbox = self.solid_box(t);
        }
    }

    // --------------------------------------------------------------
    // Master flow
    // --------------------------------------------------------------

    fn run(&mut self, sheet: &SheetDef) {
        // Resolve anchor chains to a root; unresolved chains become majors.
        let roots = self.resolve_groups();
        for (pi, &root) in roots.iter().enumerate() {
            self.placed[pi].group_root = root;
        }

        // Solid boxes for pinned comps (fixed).
        for pi in 0..self.placed.len() {
            self.placed[pi].bbox = self.solid_box(pi);
        }

        // Compose free groups in relative coordinates.
        let mut group_of: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (pi, &root) in roots.iter().enumerate() {
            group_of.entry(root).or_default().push(pi);
        }

        let mut free_groups: Vec<(usize, Vec<usize>, BBox)> = Vec::new();
        let mut pinned_anchored: Vec<usize> = Vec::new();
        for (&root, members) in &group_of {
            if self.placed[root].pinned {
                // Satellites of a pinned anchor place absolutely around it.
                pinned_anchored.extend(members.iter().copied().filter(|&m| {
                    m != root && !self.placed[m].pinned && self.placed[m].anchor.is_some()
                }));
                continue;
            }
            let free_members: Vec<usize> = members
                .iter()
                .copied()
                .filter(|&m| !self.placed[m].pinned)
                .collect();
            if free_members.is_empty() {
                continue;
            }
            let bbox = self.compose_group(root, &free_members);
            free_groups.push((root, free_members, bbox));
        }

        // Columns for free groups, offset to the right of pinned content.
        let pinned_box = self.pinned_content_box();
        self.place_columns(&mut free_groups, &pinned_box);

        // Satellites anchored to pinned components (absolute coordinates).
        self.place_satellites(&pinned_anchored, true);
        for &pi in &pinned_anchored {
            self.orient_two_pin(pi);
            self.placed[pi].bbox = self.solid_box(pi);
        }

        // Global safety net: collision resolution over everything.
        let order: Vec<usize> = (0..self.placed.len()).collect();
        self.resolve_collisions(&order);

        // Final, group-aware orientation (positions frozen — only
        // rotation/mirror change, before texts and wiring).
        crate::orientation::orient_for_wiring(
            self.cfg,
            self.design,
            &mut self.placed,
            &self.nets,
            &self.pin_net,
        );

        // Re-seat input filters into tidy, aligned, collision-free clusters
        // (proximity + alignment). Runs AFTER orientation so the cluster's own
        // orientations are final (resistors horizontal, caps upright) and
        // nothing re-rotates or scatters it before wiring.
        self.tidy_input_filters();

        // Seat each pull resistor over the pin it pulls so its leg drops onto
        // that pin's exit stub as one continuous wire (runs after orientation
        // so the pull is upright and the IC pins are in their final places).
        self.reseat_pullups();

        // Relegate the rail-to-rail capacitors (decoupling/bulk) into a tidy
        // row in the right-hand utility band, out of the functional flow.
        // Runs LAST (positions + orientations of the flow are final) and only
        // when the sheet carries a real IC worth keeping uncluttered. The
        // router later relegates the undriven-rail flags below this row and
        // outlines the zones.
        self.relegate = self.cfg.relegate_utility && self.has_ic_major();
        if self.relegate {
            self.relegate_decoupling();
        }

        // Uniform translation into the page (margins), preserving the
        // relative geometry of pinned components.
        let content = self.content_box();
        let has_fallback_left_ports = sheet.ports.iter().any(|p| {
            let n = self
                .nets
                .iter()
                .find(|n| n.net == p.net)
                .expect("port net present");
            n.endpoints.is_empty()
                && !n.on_child_blocks
                && !matches!(p.direction, PortDirection::Output)
        });
        let reserve = if has_fallback_left_ports {
            self.cfg.port_column_mm + 5.08
        } else {
            0.0
        };
        let shift_x = self
            .cfg
            .snap(self.cfg.margin_left_mm + reserve - content.x1);
        let shift_y = self.cfg.snap(self.cfg.margin_top_mm - content.y1);
        if shift_x != 0.0 || shift_y != 0.0 {
            let any_pinned = self.placed.iter().any(|p| p.pinned);
            if any_pinned {
                self.warnings.push(format!(
                    "manual positions translated uniformly by ({shift_x:.2} mm, {shift_y:.2} mm) to fit the page (relative geometry preserved)"
                ));
            }
            for pi in 0..self.placed.len() {
                self.placed[pi].at = (
                    round4(self.placed[pi].at.0 + shift_x),
                    round4(self.placed[pi].at.1 + shift_y),
                );
                self.placed[pi].bbox = self.solid_box(pi);
            }
        }

        // Canonical Reference/Value text anchors.
        for pi in 0..self.placed.len() {
            self.canonical_texts(pi);
        }

        let _ = sheet;
    }

    /// Resolve anchor chains: every placed comp maps to its group root.
    fn resolve_groups(&mut self) -> Vec<usize> {
        let n = self.placed.len();
        let mut root: Vec<Option<usize>> = vec![None; n];
        for (pi, slot) in root.iter_mut().enumerate() {
            if self.placed[pi].anchor.is_none() {
                *slot = Some(pi);
            }
        }
        let mut pending: Vec<usize> = (0..n).filter(|&pi| root[pi].is_none()).collect();
        let mut guard = pending.len() * 3 + 10;
        while !pending.is_empty() && guard > 0 {
            guard -= 1;
            let pi = pending.remove(0);
            let anchor = self.placed[pi].anchor.expect("pending implies anchor");
            match root[anchor] {
                Some(r) => root[pi] = Some(r),
                None => pending.push(pi),
            }
        }
        for pi in pending {
            self.warnings.push(format!(
                "unresolved anchor chain for {} — treated as a major",
                self.design.comps[self.placed[pi].comp].refdes
            ));
            self.placed[pi].anchor = None;
            root[pi] = Some(pi);
        }
        root.into_iter().map(|r| r.expect("all resolved")).collect()
    }

    /// Compose a free group in relative coordinates: major at the origin,
    /// satellites around it, pre-orientation, intra-group collisions.
    fn compose_group(&mut self, root: usize, members: &[usize]) -> BBox {
        self.placed[root].at = (0.0, 0.0);
        self.placed[root].bbox = self.solid_box(root);
        let satellites: Vec<usize> = members.iter().copied().filter(|&m| m != root).collect();
        self.place_satellites(&satellites, false);
        for &m in members {
            self.orient_two_pin(m);
        }
        for &m in members {
            self.placed[m].bbox = self.solid_box(m);
        }
        self.resolve_collisions(members);
        let mut bbox = inflate(&self.placed[root].bbox, self.cfg.component_pad_mm);
        for &m in members {
            for b in self.solids(m) {
                bbox = union_box(&bbox, &b.bbox);
            }
        }
        bbox
    }

    /// Satellite placement: anchor edge + role default side, regular rows
    /// centered on the anchor (`satellite_pitch_mm` spread).
    fn place_satellites(&mut self, satellites: &[usize], absolute: bool) {
        let cfg = self.cfg;
        let mut pending: Vec<usize> = satellites.to_vec();
        pending.sort_by(|&a, &b| {
            let ra = self.placed[a].anchor.unwrap_or(usize::MAX);
            let rb = self.placed[b].anchor.unwrap_or(usize::MAX);
            ra.cmp(&rb).then_with(|| {
                natord::compare(
                    &self.design.comps[self.placed[a].comp].refdes,
                    &self.design.comps[self.placed[b].comp].refdes,
                )
            })
        });

        // Row sizes per (anchor, side, role group).
        let group_key = |this: &Engine<'a>, pi: usize| -> (usize, u8, u8) {
            let side = this.placed[pi].side;
            let role_group = match this.placed[pi].role {
                Role::Decoupling => 0u8,
                Role::Bulk => 1,
                Role::Pull => 2,
                _ => 3,
            };
            (
                this.placed[pi].anchor.unwrap_or(usize::MAX),
                side as u8,
                role_group,
            )
        };
        let mut group_size: BTreeMap<(usize, u8, u8), usize> = BTreeMap::new();
        for &pi in &pending {
            *group_size.entry(group_key(self, pi)).or_insert(0) += 1;
        }
        let mut row_index: BTreeMap<(usize, u8, u8), usize> = BTreeMap::new();

        let mut guard = pending.len() * 3 + 10;
        let mut placed_set: std::collections::BTreeSet<usize> = if absolute {
            // Pinned anchors are already placed.
            self.placed
                .iter()
                .enumerate()
                .filter(|(_, p)| p.pinned)
                .map(|(i, _)| i)
                .collect()
        } else {
            std::collections::BTreeSet::new()
        };

        while !pending.is_empty() && guard > 0 {
            guard -= 1;
            let pi = pending.remove(0);
            let Some(anchor) = self.placed[pi].anchor else {
                self.placed[pi].at = (0.0, 0.0);
                self.placed[pi].bbox = self.solid_box(pi);
                continue;
            };
            // Wait until the anchor itself is placed (satellite chains).
            let anchor_pending = pending.contains(&anchor);
            if anchor_pending {
                pending.push(pi);
                continue;
            }
            if absolute && !placed_set.contains(&anchor) && !self.placed[anchor].pinned {
                pending.push(pi);
                continue;
            }

            let side = self.placed[pi].side;
            let key = group_key(self, pi);
            let idx = *row_index.get(&key).unwrap_or(&0);
            row_index.insert(key, idx + 1);
            let n = *group_size.get(&key).unwrap_or(&1);
            let spread = (idx as f64 - (n as f64 - 1.0) / 2.0) * cfg.satellite_pitch_mm;
            let gap = match self.placed[pi].role {
                Role::Decoupling | Role::Pull => cfg.role_row_gap_mm,
                _ => cfg.satellite_gap_mm,
            };

            let a = self.placed[anchor].bbox;
            let (ax, ay) = self.placed[anchor].at;
            // Local footprint of the satellite relative to its origin, in
            // its current orientation.
            let local = raw_box(
                &self.design.comps[self.placed[pi].comp].geom,
                (0.0, 0.0),
                self.placed[pi].rotation,
                self.placed[pi].mirror,
            );
            let cx_off = (local.x1 + local.x2) / 2.0;
            let cy_off = (local.y1 + local.y2) / 2.0;
            let (cx, cy) = match side {
                Side::Below => (ax + spread - cx_off, a.y2 + gap - local.y1),
                Side::Above => (ax + spread - cx_off, a.y1 - gap - local.y2),
                Side::Left => (a.x1 - gap - local.x2, ay + spread - cy_off),
                Side::Right => (a.x2 + gap - local.x1, ay + spread - cy_off),
            };
            self.placed[pi].at = (cfg.snap(cx), cfg.snap(cy));
            self.placed[pi].bbox = self.solid_box(pi);
            placed_set.insert(pi);
        }
        if !pending.is_empty() {
            let names: Vec<&str> = pending
                .iter()
                .map(|&pi| self.design.comps[self.placed[pi].comp].refdes.as_str())
                .collect();
            self.warnings
                .push(format!("unresolved anchor chain for: {}", names.join(", ")));
            for pi in pending {
                self.placed[pi].at = (0.0, 0.0);
                self.placed[pi].bbox = self.solid_box(pi);
            }
        }
    }

    /// Columns by signal flow for the free groups.
    fn place_columns(
        &mut self,
        groups: &mut [(usize, Vec<usize>, BBox)],
        pinned_box: &Option<BBox>,
    ) {
        let cfg = self.cfg;
        let ranks = self.bfs_ranks();

        let mut by_cat: BTreeMap<Category, Vec<usize>> = BTreeMap::new();
        for (gi, (root, _, _)) in groups.iter().enumerate() {
            by_cat
                .entry(categorize(self.placed[*root].role))
                .or_default()
                .push(gi);
        }

        // Free content starts to the right of pinned content, if any.
        let (x0, y0) = match pinned_box {
            Some(b) => (cfg.snap_up(b.x2 + cfg.col_gap_mm), b.y1),
            None => (0.0, 0.0),
        };

        // A category taller than the tallest usable page wraps into
        // several columns (a single endless stack overflows every paper).
        let wrap_h = {
            let paper = match cfg.paper {
                crate::config::Paper::Auto => crate::config::Paper::A3,
                fixed => fixed,
            };
            let (_, h) = crate::config::Paper::SIZES
                .iter()
                .find(|(p, _, _)| *p == paper)
                .map(|(_, w, h)| (*w, *h))
                .unwrap_or((297.0, 210.0));
            (h - 2.0 * cfg.margin_top_mm).max(80.0)
        };

        let mut x_cursor = x0;
        for cat in CATEGORY_ORDER {
            let Some(list) = by_cat.get(&cat) else {
                continue;
            };
            let mut list = list.clone();
            list.sort_by(|&a, &b| {
                let ka = ranks.get(&groups[a].0).copied().unwrap_or(usize::MAX);
                let kb = ranks.get(&groups[b].0).copied().unwrap_or(usize::MAX);
                ka.cmp(&kb).then_with(|| {
                    natord::compare(
                        &self.design.comps[self.placed[groups[a].0].comp].refdes,
                        &self.design.comps[self.placed[groups[b].0].comp].refdes,
                    )
                })
            });

            let mut column: Vec<usize> = Vec::new();
            let mut columns: Vec<Vec<usize>> = Vec::new();
            let mut col_h = 0.0f64;
            for &gi in &list {
                let h = groups[gi].2.y2 - groups[gi].2.y1;
                if !column.is_empty() && col_h + h > wrap_h {
                    columns.push(std::mem::take(&mut column));
                    col_h = 0.0;
                }
                col_h += h + cfg.row_gap_mm;
                column.push(gi);
            }
            if !column.is_empty() {
                columns.push(column);
            }

            for list in columns {
                let mut half_w = 0.0f64;
                for &gi in &list {
                    half_w = half_w.max((groups[gi].2.x2 - groups[gi].2.x1) / 2.0);
                }
                let col_x = x_cursor + half_w;
                let mut y_cursor = y0;
                for &gi in &list {
                    let bbox = groups[gi].2;
                    let cx = (bbox.x1 + bbox.x2) / 2.0;
                    let dx = cfg.snap(col_x - cx);
                    let dy = cfg.snap(y_cursor - bbox.y1);
                    for &m in &groups[gi].1 {
                        self.placed[m].at = (
                            round4(self.placed[m].at.0 + dx),
                            round4(self.placed[m].at.1 + dy),
                        );
                        self.placed[m].bbox = self.solid_box(m);
                    }
                    groups[gi].2 = BBox {
                        x1: bbox.x1 + dx,
                        y1: bbox.y1 + dy,
                        x2: bbox.x2 + dx,
                        y2: bbox.y2 + dy,
                    };
                    y_cursor = groups[gi].2.y2 + cfg.row_gap_mm;
                }
                x_cursor = col_x + half_w + cfg.col_gap_mm;
            }
        }
    }

    /// BFS rank by signal connectivity, seeded on the input connectors.
    fn bfs_ranks(&self) -> HashMap<usize, usize> {
        let mut adj: BTreeMap<usize, std::collections::BTreeSet<usize>> = BTreeMap::new();
        for net in &self.nets {
            if net.class != NetClass::Signal {
                continue;
            }
            let mut refs: Vec<usize> = net.endpoints.iter().map(|(pi, _)| *pi).collect();
            refs.sort_unstable();
            refs.dedup();
            for i in 0..refs.len() {
                for j in i + 1..refs.len() {
                    adj.entry(refs[i]).or_default().insert(refs[j]);
                    adj.entry(refs[j]).or_default().insert(refs[i]);
                }
            }
        }
        let mut seeds: Vec<usize> = (0..self.placed.len())
            .filter(|&pi| categorize(self.placed[pi].role) == Category::ConnLeft)
            .collect();
        seeds.sort_by(|&a, &b| {
            natord::compare(
                &self.design.comps[self.placed[a].comp].refdes,
                &self.design.comps[self.placed[b].comp].refdes,
            )
        });
        let mut rank: HashMap<usize, usize> = HashMap::new();
        let mut queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        for s in seeds {
            rank.insert(s, 0);
            queue.push_back(s);
        }
        while let Some(cur) = queue.pop_front() {
            let r = rank[&cur];
            if let Some(nexts) = adj.get(&cur) {
                for &next in nexts {
                    if let std::collections::hash_map::Entry::Vacant(e) = rank.entry(next) {
                        e.insert(r + 1);
                        queue.push_back(next);
                    }
                }
            }
        }
        rank
    }

    /// Placement pre-orientation of drawn two-pin parts: power rail up,
    /// ground down, paired pin facing its mate.
    fn orient_two_pin(&mut self, pi: usize) {
        if self.placed[pi].pinned || !is_rotatable(self.placed[pi].role) {
            return;
        }
        let comp = &self.design.comps[self.placed[pi].comp];
        let visible: Vec<usize> = comp
            .geom
            .pins
            .iter()
            .enumerate()
            .filter(|(_, p)| !p.hidden)
            .map(|(i, _)| i)
            .collect();
        if visible.len() != 2 {
            return;
        }

        let mut best = self.placed[pi].rotation;
        let mut best_score = f64::NEG_INFINITY;
        for rot in [0, 90, 180, 270] {
            let mut score = 0.0;
            for &vi in &visible {
                let pin = &comp.geom.pins[vi];
                let Some(dir) = comp.geom.pin_outward(&pin.number, rot, None) else {
                    continue;
                };
                let Some(&sn) = self.pin_net.get(&(pi, pin.number.clone())) else {
                    continue;
                };
                match self.nets[sn].class {
                    NetClass::Ground => {
                        score += if dir.1 > 0.5 {
                            2.0
                        } else if dir.1 < -0.5 {
                            -2.0
                        } else {
                            0.0
                        }
                    }
                    NetClass::Power => {
                        score += if dir.1 < -0.5 {
                            2.0
                        } else if dir.1 > 0.5 {
                            -2.0
                        } else {
                            0.0
                        }
                    }
                    NetClass::Signal => {
                        let net = &self.nets[sn];
                        if net.endpoints.len() == 2 && net.port.is_none() {
                            let mate = net
                                .endpoints
                                .iter()
                                .find(|(opi, pad)| *opi != pi || *pad != pin.number);
                            if let Some((opi, _)) = mate
                                && *opi != pi
                            {
                                let (mx, my) = self.placed[*opi].at;
                                let vx = mx - self.placed[pi].at.0;
                                let vy = my - self.placed[pi].at.1;
                                let dominant = if vx.abs() >= vy.abs() {
                                    (vx.signum(), 0.0)
                                } else {
                                    (0.0, vy.signum())
                                };
                                let dot = dir.0 * dominant.0 + dir.1 * dominant.1;
                                score += if dot > 0.0 {
                                    3.0
                                } else if dot < 0.0 {
                                    -1.0
                                } else {
                                    0.0
                                };
                            }
                        }
                    }
                }
            }
            if score > best_score {
                best_score = score;
                best = rot;
            }
        }
        self.placed[pi].rotation = best;
        self.placed[pi].bbox = self.solid_box(pi);
    }

    // --------------------------------------------------------------
    // Solid boxes & collisions
    // --------------------------------------------------------------

    /// Raw sheet box of a placed comp.
    fn raw_box_of(&self, pi: usize) -> BBox {
        let p = &self.placed[pi];
        raw_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror)
    }

    /// Solid box: raw box + Reference above the top-left corner + Value
    /// below (text widths included).
    fn solid_box(&self, pi: usize) -> BBox {
        let p = &self.placed[pi];
        solid_box_for(self.cfg, self.design, p, p.rotation, p.mirror)
    }

    fn has_bottom_pins(&self, pi: usize) -> bool {
        let p = &self.placed[pi];
        let geom = &self.design.comps[p.comp].geom;
        geom.pins.iter().filter(|pin| !pin.hidden).any(|pin| {
            geom.pin_outward(&pin.number, p.rotation, p.mirror)
                .map(|d| d.1 > 0.5)
                .unwrap_or(false)
        })
    }

    /// Solid boxes of a comp: inflated solid box + predicted power stub
    /// keepouts + label stub keepouts (tagged by net so same-net reserves
    /// do not repel each other).
    fn solids(&self, pi: usize) -> Vec<TaggedBox> {
        let mut out = vec![TaggedBox {
            bbox: inflate(&self.placed[pi].bbox, self.cfg.component_pad_mm),
            net: None,
        }];
        out.extend(self.power_keepouts(pi));
        out.extend(self.label_keepouts(pi));
        out
    }

    fn power_keepouts(&self, pi: usize) -> Vec<TaggedBox> {
        let mut out = Vec::new();
        let p = &self.placed[pi];
        let geom = &self.design.comps[p.comp].geom;
        let mut seen: std::collections::BTreeSet<(i64, i64)> = std::collections::BTreeSet::new();
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(&sn) = self.pin_net.get(&(pi, pin.number.clone())) else {
                continue;
            };
            if self.nets[sn].class == NetClass::Signal {
                continue;
            }
            let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                continue;
            };
            let key = ((pos.0 * 10000.0) as i64, (pos.1 * 10000.0) as i64);
            if !seen.insert(key) {
                continue;
            }
            let Some(dir) = geom.pin_outward(&pin.number, p.rotation, p.mirror) else {
                continue;
            };
            let down = self.nets[sn].class == NetClass::Ground;
            let att = crate::route::power_attachment(self.cfg, pos, dir, down, 0.0, 0.0);
            let mut corridor = point_box(att.path[0], 1.27);
            for pt in &att.path {
                corridor = union_box(&corridor, &point_box(*pt, 1.27));
            }
            out.push(TaggedBox {
                bbox: corridor,
                net: Some(self.nets[sn].name.clone()),
            });
            out.push(TaggedBox {
                bbox: crate::route::power_symbol_graphic_box(
                    &self.nets[sn].name,
                    att.symbol_at,
                    att.down,
                ),
                net: Some(self.nets[sn].name.clone()),
            });
        }
        out
    }

    fn label_keepouts(&self, pi: usize) -> Vec<TaggedBox> {
        let mut out = Vec::new();
        let p = &self.placed[pi];
        let geom = &self.design.comps[p.comp].geom;
        let mut seen: std::collections::BTreeSet<(i64, i64)> = std::collections::BTreeSet::new();
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(&sn) = self.pin_net.get(&(pi, pin.number.clone())) else {
                continue;
            };
            if self.nets[sn].class != NetClass::Signal {
                continue;
            }
            // Paired nets get a direct/Z wire, not a stub + label: no
            // label keepout (the wire candidates stagger by themselves).
            if crate::wiring::is_pair_net_static(
                self.cfg,
                self.design,
                &self.placed,
                &self.nets,
                sn,
            ) {
                continue;
            }
            let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                continue;
            };
            let key = ((pos.0 * 10000.0) as i64, (pos.1 * 10000.0) as i64);
            if !seen.insert(key) {
                continue;
            }
            let Some(dir) = geom.pin_outward(&pin.number, p.rotation, p.mirror) else {
                continue;
            };
            let name = self.nets[sn].name.clone();
            let bbox = if dir.1.abs() > 0.5 {
                // Vertical pin: reserve only the vertical stub corridor.
                let stub = self.cfg.stub_mm + 2.54;
                let elbow = (pos.0, pos.1 + dir.1 * stub);
                union_box(&point_box(pos, 1.524), &point_box(elbow, 1.524))
            } else {
                let len = label_stub_len(self.cfg, &name) + 1.27;
                let end = (pos.0 + dir.0 * len, pos.1 + dir.1 * len);
                union_box(&point_box(pos, 1.524), &point_box(end, 1.524))
            };
            out.push(TaggedBox {
                bbox,
                net: Some(name),
            });
        }
        out
    }

    /// Collision resolution (pinned first and immobile; majors pushed down;
    /// satellites ring-searched around their preferred direction).
    fn resolve_collisions(&mut self, ordered: &[usize]) {
        let mut accepted: Vec<usize> = Vec::new();
        let queue: Vec<usize> = ordered
            .iter()
            .copied()
            .filter(|&pi| self.placed[pi].pinned)
            .chain(
                ordered
                    .iter()
                    .copied()
                    .filter(|&pi| !self.placed[pi].pinned),
            )
            .collect();

        for pi in queue {
            if self.placed[pi].pinned {
                // Overlaps between pinned components reflect the user's own
                // (viewer) layout — kept verbatim, no warning.
                accepted.push(pi);
                continue;
            }
            if !self.conflicts(pi, &accepted) {
                accepted.push(pi);
                continue;
            }
            let origin = self.placed[pi].at;
            let mut placed_ok = false;
            if self.placed[pi].anchor.is_some() {
                let dirs = self.push_directions(pi);
                let axis: Vec<(f64, f64)> = dirs.iter().copied().filter(|d| d.1 == 0.0).collect();
                'axis: for k in 1..=8 {
                    for dir in &axis {
                        self.placed[pi].at =
                            (self.cfg.snap(origin.0 + dir.0 * k as f64 * 2.54), origin.1);
                        self.placed[pi].bbox = self.solid_box(pi);
                        if !self.conflicts(pi, &accepted) {
                            placed_ok = true;
                            break 'axis;
                        }
                    }
                }
                if !placed_ok {
                    'ring: for k in 1..=40 {
                        for dir in &dirs {
                            self.placed[pi].at = (
                                self.cfg.snap(origin.0 + dir.0 * k as f64 * 2.54),
                                self.cfg.snap(origin.1 + dir.1 * k as f64 * 2.54),
                            );
                            self.placed[pi].bbox = self.solid_box(pi);
                            if !self.conflicts(pi, &accepted) {
                                placed_ok = true;
                                break 'ring;
                            }
                        }
                    }
                }
            } else {
                for k in 1..=200 {
                    self.placed[pi].at = (origin.0, self.cfg.snap(origin.1 + k as f64 * 2.54));
                    self.placed[pi].bbox = self.solid_box(pi);
                    if !self.conflicts(pi, &accepted) {
                        placed_ok = true;
                        break;
                    }
                }
            }
            if !placed_ok {
                self.warnings.push(format!(
                    "unresolved overlap for {} (kept in place)",
                    self.design.comps[self.placed[pi].comp].refdes
                ));
                self.placed[pi].at = origin;
                self.placed[pi].bbox = self.solid_box(pi);
            }
            accepted.push(pi);
        }
    }

    fn conflicts(&self, pi: usize, accepted: &[usize]) -> bool {
        let mine = self.solids(pi);
        accepted.iter().any(|&other| {
            self.solids(other).iter().any(|ob| {
                mine.iter().any(|mb| {
                    if let (Some(a), Some(b)) = (&mb.net, &ob.net)
                        && a == b
                    {
                        return false;
                    }
                    overlaps(&mb.bbox, &ob.bbox)
                })
            })
        })
    }

    /// Avoidance directions of a satellite, preference order.
    fn push_directions(&self, pi: usize) -> Vec<(f64, f64)> {
        let Some(anchor) = self.placed[pi].anchor else {
            return vec![(0.0, 1.0)];
        };
        match self.placed[pi].side {
            Side::Left => vec![(-1.0, 0.0), (0.0, 1.0), (0.0, -1.0), (1.0, 0.0)],
            Side::Right => vec![(1.0, 0.0), (0.0, 1.0), (0.0, -1.0), (-1.0, 0.0)],
            side => {
                let dx = if self.placed[pi].at.0 < self.placed[anchor].at.0 {
                    -1.0
                } else {
                    1.0
                };
                let dy = if side == Side::Above { -1.0 } else { 1.0 };
                vec![(dx, 0.0), (-dx, 0.0), (0.0, dy), (0.0, -dy)]
            }
        }
    }

    fn pinned_content_box(&self) -> Option<BBox> {
        let mut out: Option<BBox> = None;
        for pi in 0..self.placed.len() {
            if !self.placed[pi].pinned {
                continue;
            }
            for b in self.solids(pi) {
                out = Some(match out {
                    Some(cur) => union_box(&cur, &b.bbox),
                    None => b.bbox,
                });
            }
        }
        out
    }

    fn content_box(&self) -> BBox {
        let mut out: Option<BBox> = None;
        for pi in 0..self.placed.len() {
            for b in self.solids(pi) {
                out = Some(match out {
                    Some(cur) => union_box(&cur, &b.bbox),
                    None => b.bbox,
                });
            }
        }
        out.unwrap_or(BBox {
            x1: 0.0,
            y1: 0.0,
            x2: 0.0,
            y2: 0.0,
        })
    }

    /// Canonical Reference/Value anchors: Reference above the top-left
    /// corner of the body, Value below the body (right-justified above the
    /// lowest pin when the symbol has bottom pins, left-justified
    /// otherwise).
    fn canonical_texts(&mut self, pi: usize) {
        let p = &self.placed[pi];
        let body = body_box(&self.design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
        let gap = self.cfg.value_gap_grid_steps as f64 * self.cfg.grid_mm;
        let ref_gap = self.cfg.ref_gap_grid_steps as f64 * self.cfg.grid_mm;
        let raw = self.raw_box_of(pi);
        let ref_at = (round4(body.x1), round4(body.y1 - ref_gap));
        let (value_at, right) = if self.has_bottom_pins(pi) {
            ((round4(body.x2), round4(raw.y2 + gap + 1.27)), true)
        } else {
            ((round4(body.x1), round4(body.y2 + gap + 1.27)), false)
        };
        let placed = &mut self.placed[pi];
        placed.ref_at = ref_at;
        placed.value_at = value_at;
        placed.value_justify_right = right;
        placed.ref_justify_right = false;
    }
}

/// A solid box tagged with an optional net (same-net boxes never conflict).
pub(crate) struct TaggedBox {
    pub bbox: BBox,
    pub net: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::DesignModel;
    use crate::sheets::plan_sheets;

    /// Place the root sheet of a fixture and return (model, placed sheet).
    fn placed(sch: &pcb_sch::Schematic) -> (DesignModel, SheetModel) {
        let cfg = SchConfig::default();
        let design = DesignModel::build(sch, &cfg).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(sch, &design, &cfg, "t", &mut warnings);
        let model = place_sheet(&design, &plan, 0, &cfg, &mut warnings);
        (design, model)
    }

    fn find(design: &DesignModel, m: &SheetModel, path_key: &str) -> usize {
        m.placed
            .iter()
            .position(|p| design.comps[p.comp].path_key == path_key)
            .unwrap_or_else(|| panic!("{path_key} not placed"))
    }

    /// Proximity: a series input resistor feeding an IC input pin is anchored
    /// to that IC (grouped next to it), not ejected into the column flow.
    #[test]
    fn input_chain_resistor_anchors_to_its_ic() {
        let sch = crate::testkit::diff_filter();
        let (design, m) = placed(&sch);
        let u1 = find(&design, &m, "U1");
        for r in ["RP", "RN"] {
            let ri = find(&design, &m, r);
            assert_eq!(
                m.placed[ri].group_root, u1,
                "{r} must group with U1 (input-chain proximity)"
            );
        }
    }

    /// Alignment: similar input resistors share a column (common X); their
    /// shunt capacitors share a row (common Y).
    #[test]
    fn input_filter_resistors_column_caps_row() {
        let sch = crate::testkit::diff_filter();
        let (design, m) = placed(&sch);
        let rp = &m.placed[find(&design, &m, "RP")];
        let rn = &m.placed[find(&design, &m, "RN")];
        let cp = &m.placed[find(&design, &m, "CP")];
        let cn = &m.placed[find(&design, &m, "CN")];
        assert!(
            (rp.at.0 - rn.at.0).abs() < 1e-6,
            "resistors must share a column: RP.x={} RN.x={}",
            rp.at.0,
            rn.at.0
        );
        assert!(
            (cp.at.1 - cn.at.1).abs() < 1e-6,
            "shunt caps must share a row: CP.y={} CN.y={}",
            cp.at.1,
            cn.at.1
        );
    }

    /// A plain interconnect resistor between two ICs is NOT pulled onto a
    /// flank — the input-chain rule only fires for a terminal input branch.
    #[test]
    fn interconnect_resistor_stays_in_flow() {
        use crate::testkit::{add_box, add_r, port_ref};
        let module = pcb_sch::ModuleRef::from_path(std::path::Path::new("/test.zen"), "<root>");
        let mut sch = pcb_sch::Schematic::new();
        let root = pcb_sch::InstanceRef::new(module.clone(), vec![]);
        let mut root_inst = pcb_sch::Instance::module(module.clone());
        // Two IC boxes joined through a series resistor RS (A -> RS -> B).
        let u1 = add_box(&mut sch, &["U1"], &[("A", "1"), ("P", "2"), ("G", "3")]);
        let u2 = add_box(&mut sch, &["U2"], &[("B", "1"), ("P", "2"), ("G", "3")]);
        let rs = add_r(&mut sch, &["RS"], "1k");
        for (n, r) in [("U1", u1), ("U2", u2), ("RS", rs)] {
            root_inst.add_child(n.to_string(), r);
        }
        sch.add_instance(root.clone(), root_inst);
        sch.set_root_ref(root);
        sch.add_net(
            pcb_sch::Net::new("Net".to_string(), "A", 1)
                .with_port(port_ref(&["U1"], "A"))
                .with_port(port_ref(&["RS"], "1")),
        );
        sch.add_net(
            pcb_sch::Net::new("Net".to_string(), "B", 2)
                .with_port(port_ref(&["U2"], "B"))
                .with_port(port_ref(&["RS"], "2")),
        );
        sch.add_net(
            pcb_sch::Net::new("Power".to_string(), "P", 3)
                .with_port(port_ref(&["U1"], "P"))
                .with_port(port_ref(&["U2"], "P")),
        );
        sch.add_net(
            pcb_sch::Net::new("Ground".to_string(), "G", 4)
                .with_port(port_ref(&["U1"], "G"))
                .with_port(port_ref(&["U2"], "G")),
        );
        sch.assign_reference_designators();
        let (design, m) = placed(&sch);
        let rs = find(&design, &m, "RS");
        assert!(
            m.placed[rs].anchor.is_none(),
            "an IC-to-IC series resistor must stay a free major, not a flank satellite"
        );
    }

    /// Alignment (mission #5): the top input resistor of a filter is placed at
    /// the EXACT ordinate of the IC pin it feeds, so its leg is a straight
    /// horizontal wire into the pin (0 bends). The pins of a tight pair sit
    /// closer than one resistor cell, so only the top leg can be perfectly
    /// horizontal — the engineer aligns that one (AIN+).
    #[test]
    fn input_filter_top_resistor_aligns_to_its_ic_pin() {
        let sch = crate::testkit::diff_filter();
        let (design, m) = placed(&sch);
        let u1 = find(&design, &m, "U1");
        let rp = find(&design, &m, "RP"); // feeds AINP (pad "1"), the top pin
        let up = &m.placed[u1];
        let pin_y = design.comps[up.comp]
            .geom
            .pin_position("1", up.at, up.rotation, up.mirror)
            .expect("AINP pin position")
            .1;
        assert!(
            (m.placed[rp].at.1 - pin_y).abs() < 1e-6,
            "top input resistor RP.y={} must equal its IC pin Y={} (straight leg)",
            m.placed[rp].at.1,
            pin_y
        );
    }

    /// Placement (mission #3): a pull resistor is re-seated over the pin it
    /// pulls, `pullup_pin_gap_mm` outside the body along the pin's exit, so its
    /// free leg drops straight onto that pin's stub as one continuous wire
    /// instead of floating over the IC body.
    #[test]
    fn pullup_reseats_over_the_pin_it_pulls() {
        let cfg = SchConfig::default();
        let sch = crate::testkit::adc_dout_congested();
        let (design, m) = placed(&sch);
        let u1 = find(&design, &m, "U1");
        let rpu = find(&design, &m, "RPU"); // pull-up on DOUT (pad "3")
        let up = &m.placed[u1];
        let pin = design.comps[up.comp]
            .geom
            .pin_position("3", up.at, up.rotation, up.mirror)
            .expect("DOUT position");
        let dir = design.comps[up.comp]
            .geom
            .pin_outward("3", up.rotation, up.mirror)
            .expect("DOUT direction");
        assert!(
            dir.0.abs() > 0.5,
            "DOUT must exit horizontally for this test"
        );
        let expect_x = cfg.snap(pin.0 + dir.0 * cfg.pullup_pin_gap_mm);
        assert!(
            (m.placed[rpu].at.0 - expect_x).abs() < 1e-6,
            "pull-up RPU.x={} must sit on the DOUT exit stub at x={}",
            m.placed[rpu].at.0,
            expect_x
        );
    }

    #[test]
    fn raw_box_rotation() {
        let geom = crate::geometry::parse_lib_symbol(crate::testkit::R_SMALL, None).unwrap();
        let b0 = raw_box(&geom, (100.0, 50.0), 0, None);
        // Vertical resistor: taller than wide.
        assert!(b0.y2 - b0.y1 > b0.x2 - b0.x1);
        let b90 = raw_box(&geom, (100.0, 50.0), 90, None);
        assert!(b90.x2 - b90.x1 > b90.y2 - b90.y1);
    }

    #[test]
    fn label_stub_len_underlines_text() {
        let cfg = SchConfig::default();
        assert!(label_stub_len(&cfg, "A") >= cfg.label_stub_min_grid_steps as f64 * cfg.grid_mm);
        assert!(label_stub_len(&cfg, "A_LONG_NET_NAME") >= label_text_width("A_LONG_NET_NAME"));
        // Always a grid multiple.
        let l = label_stub_len(&cfg, "SOME_NET");
        assert!((l / cfg.grid_mm - (l / cfg.grid_mm).round()).abs() < 1e-9);
    }
}
