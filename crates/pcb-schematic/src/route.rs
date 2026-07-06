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
    SheetModel, SheetNet, inflate, label_stub_len, label_text_width, overlaps, raw_box,
};
use crate::round4;
use crate::sheets::SheetPlan;
use crate::texts::{Corridor, CorridorZone};
use crate::wiring::{
    EPS, Seg, dist, group_wire_subsets, is_hub_comp, is_pair_net_static, point_on_seg, quant,
    seg_intersects_box, segs_cross_frank, segs_touch_forbidden,
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
    pub pwr_flags: Vec<Point>,
    pub no_connects: Vec<Point>,
    pub blocks: Vec<BlockModel>,
    /// Content extents including routed elements (paper selection).
    pub extents: BBox,
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
    let g = cfg.power_stub_mm;
    if dir.0.abs() < 0.5 {
        let end = (pin.0, round4(pin.1 + dir.1 * (g + v_extra)));
        return Attachment {
            path: vec![pin, end],
            symbol_at: end,
            down,
        };
    }
    let elbow = (round4(pin.0 + dir.0 * (g + h_extra)), pin.1);
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
    };
    router.init_registry();
    router.route_signals();
    router.route_power(flag_nets);
    router.mark_no_connects();
    router.place_blocks();
    router.place_port_fallbacks();
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
        let eps = self.collapse_stacked_pins(eps, &name, true);
        self.collapsed.insert(sn, eps.clone());
        eps
    }

    /// Collapse stacks of adjacent same-net pins of one component (multi
    /// drain/source packages) into a single endpoint: a straight bus wire
    /// runs along the pin column/row (every pin end lies on the wire, which
    /// connects in KiCad without junctions) and only the extreme pin keeps
    /// a label/power attachment. `extreme_up` picks the top/left pin of the
    /// stack (power rails), otherwise the bottom/right one (grounds).
    fn collapse_stacked_pins(
        &mut self,
        eps: Vec<Endpoint>,
        net: &str,
        extreme_up: bool,
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

    fn route_signals(&mut self) {
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
        if let Some(direction) = port
            && !is_root
            && !self.port_anchored.contains(&name)
            && let Some(anchored) = self.anchor_port(&name, direction, &rest)
        {
            rest.retain(|e| quant(e.pos) != quant(anchored.pos));
        }
        for ep in rest {
            self.emit_label_stub(&name, &ep, single);
        }
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
                        let rotation = if side > 0.0 { 180 } else { 0 };
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

        // Horizontal pin: straight stub underlining the full text, label at
        // the distal end turned back toward the pin.
        let base = label_stub_len(cfg, name);
        let rotation = if ep.dir.0 >= 0.0 { 180 } else { 0 };
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
                // Cramped fallback: short stub, text pointing outward.
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

    fn route_power(&mut self, flag_nets: &BTreeSet<String>) {
        let mut flagged: BTreeSet<String> = BTreeSet::new();
        for sn in 0..self.model.nets.len() {
            let net = &self.model.nets[sn];
            if net.class == NetClass::Signal {
                continue;
            }
            let name = net.name.clone();
            let down = net.class == NetClass::Ground;
            let eps = self.net_endpoints(net);
            let eps = self.collapse_stacked_pins(eps, &name, !down);
            let aligned = self.power_align_targets(&eps, down);

            for (index, ep) in eps.iter().enumerate() {
                let mut att = power_attachment(self.cfg, ep.pos, ep.dir, down, 0.0, 0.0);
                let mut found = false;

                // Aligned candidates first (row target), then free retries
                // (including a shortened -1.27 mm elbow that escapes a
                // condemned canonical column), then jogs for vertical pins.
                let hs = [
                    0.0, 2.54, 5.08, 7.62, 10.16, 12.7, 15.24, 17.78, 20.32, -1.27,
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
                let mut best_crossings = usize::MAX;
                for cand in &cands {
                    let gbox = power_symbol_graphic_box(&name, cand.symbol_at, cand.down);
                    if self.path_ok(&cand.path, &name, &[ep.placed])
                        && self.graphic_box_clear(&gbox, &name)
                    {
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
                if !found {
                    // Visual-relaxed rescue: allow the wire to run through
                    // label/graphic boxes but NEVER create an electrical
                    // contact (foreign pin on the path, wire touching a
                    // foreign wire). A committed fallback that shorts two
                    // rails is a netlist corruption, not a cosmetic issue.
                    let mut extended: Vec<Attachment> = Vec::new();
                    for h in [
                        -1.27, 0.0, 2.54, 5.08, 7.62, 10.16, 12.7, 15.24, 17.78, 20.32, 25.4, 30.48,
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

                // One PWR_FLAG per undriven rail, chained next to the first
                // power symbol of the assigned sheet.
                if flag_nets.contains(&name) && flagged.insert(name.clone()) {
                    self.attach_pwr_flag(&name, att.symbol_at, ep.dir);
                }
            }
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
                ep.dir.1.abs() < 0.5 || (down && ep.dir.1 > 0.5) || (!down && ep.dir.1 < -0.5)
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
            let mut rows: Vec<Vec<(usize, f64, f64)>> = Vec::new();
            for item in band {
                match rows.last_mut() {
                    Some(cur) if item.1 - cur.last().unwrap().1 <= cfg.power_align_max_dx_mm => {
                        cur.push(item)
                    }
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
        self.out.pwr_flags.push(chosen);
        self.reg
            .power_boxes
            .push((flag_box(chosen), net.to_string()));
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
                        self.out.net_labels.push((name.clone(), end, 0));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: label_text_box(&name, end, 0),
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
                        self.out.net_labels.push((name.clone(), end, 180));
                        self.reg.label_boxes.push(LabelBox {
                            bbox: label_text_box(&name, end, 180),
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
                self.out.net_labels.push((name.clone(), end, 0));
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
                self.out.net_labels.push((name.clone(), end, 180));
            }
        }
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
        for f in &self.out.pwr_flags {
            grow(BBox {
                x1: f.0 - 2.54,
                y1: f.1 - 5.08,
                x2: f.0 + 2.54,
                y2: f.1 + 0.5,
            });
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
