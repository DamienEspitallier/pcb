//! Real-wire engine: anti-contact registry primitives, wire candidate
//! families, group prediction and the tree wiring pass (port of the wiring
//! half of the validated `layout.ts` proof of concept).
//!
//! Rules implemented here:
//! * **real wires by default** — endpoints of one Zener placement group
//!   (transitive `anchor=`) wire together as a tree with real junctions;
//!   without group info a net wires when it has <= 3 endpoints, none of
//!   them on a hub (`hub_pin_count_threshold`, applied per component);
//! * a candidate wire path must be clean: no touch/overlap with foreign
//!   wires or pins, no body traversal, no crossing of predicted power
//!   corridors, **zero frank crossings** and no same-net overlap — a net
//!   that only routes by crossing stays on labels;
//! * candidate families: straight / Z / U-detour for facing pins, L for
//!   perpendicular pins, hooks for pins leaving on the same side — every
//!   segment leaves and lands **in the axis of its pin**;
//! * mixed outcomes are allowed: the wired subset drops its labels, the
//!   hub side keeps them, and one net label on the tree names the net.

use std::collections::BTreeSet;

use crate::config::SchConfig;
use crate::geometry::BBox;
use crate::model::DesignModel;
use crate::place::{PlacedComp, SheetNet, label_text_width, overlaps, raw_box};
use crate::round4;
use crate::route::{Endpoint, Router};

pub(crate) const EPS: f64 = 1e-3;

pub(crate) type Point = (f64, f64);

/// One registered wire segment (axis-aligned) tagged with its net.
pub(crate) struct Seg {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub net: String,
}

pub(crate) fn mk_seg(a: Point, b: Point, net: &str) -> Seg {
    Seg {
        x1: a.0,
        y1: a.1,
        x2: b.0,
        y2: b.1,
        net: net.to_string(),
    }
}

pub(crate) fn dist(a: Point, b: Point) -> f64 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt()
}

pub(crate) fn within(v: f64, lo: f64, hi: f64) -> bool {
    v >= lo.min(hi) - EPS && v <= lo.max(hi) + EPS
}

pub(crate) fn point_on_seg(x: f64, y: f64, seg: &Seg) -> bool {
    if (seg.y1 - seg.y2).abs() < EPS {
        (y - seg.y1).abs() < EPS && within(x, seg.x1, seg.x2)
    } else if (seg.x1 - seg.x2).abs() < EPS {
        (x - seg.x1).abs() < EPS && within(y, seg.y1, seg.y2)
    } else {
        false
    }
}

pub(crate) fn strictly_inside(v: f64, lo: f64, hi: f64) -> bool {
    v > lo.min(hi) + EPS && v < lo.max(hi) - EPS
}

/// Forbidden contact between segments of different nets (axis aligned):
/// touching or overlapping is forbidden, frank crossings are tolerated.
pub(crate) fn segs_touch_forbidden(a: &Seg, b: &Seg) -> bool {
    let a_h = (a.y1 - a.y2).abs() < EPS;
    let b_h = (b.y1 - b.y2).abs() < EPS;
    if a_h != b_h {
        let ix = if a_h { b.x1 } else { a.x1 };
        let iy = if a_h { a.y1 } else { b.y1 };
        let in_a = point_on_seg(ix, iy, a);
        let in_b = point_on_seg(ix, iy, b);
        if !in_a || !in_b {
            return false;
        }
        let interior_a = if a_h {
            strictly_inside(ix, a.x1, a.x2)
        } else {
            strictly_inside(iy, a.y1, a.y2)
        };
        let interior_b = if b_h {
            strictly_inside(ix, b.x1, b.x2)
        } else {
            strictly_inside(iy, b.y1, b.y2)
        };
        return !(interior_a && interior_b);
    }
    if a_h {
        if (a.y1 - b.y1).abs() > EPS {
            return false;
        }
        within(a.x1, b.x1, b.x2) || within(a.x2, b.x1, b.x2) || within(b.x1, a.x1, a.x2)
    } else {
        if (a.x1 - b.x1).abs() > EPS {
            return false;
        }
        within(a.y1, b.y1, b.y2) || within(a.y2, b.y1, b.y2) || within(b.y1, a.y1, a.y2)
    }
}

/// Frank (perpendicular, interior x interior) crossing between 2 segments.
pub(crate) fn segs_cross_frank(a: &Seg, b: &Seg) -> bool {
    let a_h = (a.y1 - a.y2).abs() < EPS;
    let b_h = (b.y1 - b.y2).abs() < EPS;
    if a_h == b_h {
        return false;
    }
    let ix = if a_h { b.x1 } else { a.x1 };
    let iy = if a_h { a.y1 } else { b.y1 };
    let interior_a = if a_h {
        strictly_inside(ix, a.x1, a.x2)
    } else {
        strictly_inside(iy, a.y1, a.y2)
    };
    let interior_b = if b_h {
        strictly_inside(ix, b.x1, b.x2)
    } else {
        strictly_inside(iy, b.y1, b.y2)
    };
    interior_a && interior_b
}

/// Does the (axis-aligned) segment run through the interior of a box?
pub(crate) fn seg_intersects_box(seg: &Seg, b: &BBox) -> bool {
    let sx1 = seg.x1.min(seg.x2);
    let sy1 = seg.y1.min(seg.y2);
    let sx2 = seg.x1.max(seg.x2);
    let sy2 = seg.y1.max(seg.y2);
    sx1 < b.x2 - EPS && sx2 > b.x1 + EPS && sy1 < b.y2 - EPS && sy2 > b.y1 + EPS
}

pub(crate) fn path_length_mm(path: &[Point]) -> f64 {
    path.windows(2)
        .map(|w| (w[1].0 - w[0].0).abs() + (w[1].1 - w[0].1).abs())
        .sum()
}

/// Number of right-angle bends in an axis-aligned path: consecutive
/// non-degenerate segments whose orientation flips (horizontal to vertical
/// or vice versa). Collinear points and zero-length hops are not bends.
pub(crate) fn path_bends(path: &[Point]) -> usize {
    let mut bends = 0;
    for w in path.windows(3) {
        if dist(w[0], w[1]) < EPS || dist(w[1], w[2]) < EPS {
            continue;
        }
        let h1 = (w[1].1 - w[0].1).abs() < EPS;
        let h2 = (w[2].1 - w[1].1).abs() < EPS;
        if h1 != h2 {
            bends += 1;
        }
    }
    bends
}

pub(crate) fn quant(p: Point) -> (i64, i64) {
    (
        (p.0 * 10000.0).round() as i64,
        (p.1 * 10000.0).round() as i64,
    )
}

// ----------------------------------------------------------------------
// Candidate wire families
// ----------------------------------------------------------------------

/// Pin-to-pin tree wire candidates: same families as the direct pair wire
/// (straight / Z / U-detour for two facing pins) **plus** the shapes pairs
/// do not accept: an L for two perpendicular pins and a hook for two pins
/// leaving on the same side. Every segment leaves and lands in the axis of
/// its pin — a wire never hits a pin shaft sideways.
pub(crate) fn tree_pair_candidates(
    cfg: &SchConfig,
    a_in: (Point, Point),
    b_in: (Point, Point),
) -> Vec<Vec<Point>> {
    let g = cfg.grid_mm;
    let snap = |v: f64| round4((v / g).round() * g);
    let mut out: Vec<Vec<Point>> = Vec::new();
    let (mut a, mut b) = (a_in, b_in);
    let (h_a, h_b) = (a.1.0.abs() > 0.5, b.1.0.abs() > 0.5);

    // Facing horizontally: straight / U detours / Z.
    if h_a && h_b && a.1.0 != b.1.0 {
        if a.1.0 < 0.0 {
            std::mem::swap(&mut a, &mut b);
        }
        let dx = b.0.0 - a.0.0;
        if dx <= EPS {
            return out;
        }
        if (a.0.1 - b.0.1).abs() < EPS {
            out.push(vec![a.0, b.0]);
            for inset in [2.54, 5.08] {
                let x1 = snap(a.0.0 + inset);
                let x2 = snap(b.0.0 - inset);
                if x2 - x1 < EPS {
                    continue;
                }
                for dy in [-7.62, 7.62, -10.16, 10.16, -12.7, 12.7] {
                    let ym = snap(a.0.1 + dy);
                    out.push(vec![a.0, (x1, a.0.1), (x1, ym), (x2, ym), (x2, b.0.1), b.0]);
                }
            }
        } else {
            let mid = (a.0.0 + b.0.0) / 2.0;
            for k in [0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0] {
                let xm = snap(mid + k * 2.54);
                if xm < a.0.0 + 2.54 - EPS || xm > b.0.0 - 2.54 + EPS {
                    continue;
                }
                out.push(vec![a.0, (xm, a.0.1), (xm, b.0.1), b.0]);
            }
        }
        return out;
    }

    // Facing vertically.
    if !h_a && !h_b && a.1.1 != b.1.1 {
        if a.1.1 < 0.0 {
            std::mem::swap(&mut a, &mut b);
        }
        let dy = b.0.1 - a.0.1;
        if dy <= EPS {
            return out;
        }
        if (a.0.0 - b.0.0).abs() < EPS {
            out.push(vec![a.0, b.0]);
            for inset in [2.54, 5.08] {
                let y1 = snap(a.0.1 + inset);
                let y2 = snap(b.0.1 - inset);
                if y2 - y1 < EPS {
                    continue;
                }
                for dxu in [-7.62, 7.62, -10.16, 10.16, -12.7, 12.7] {
                    let xm = snap(a.0.0 + dxu);
                    out.push(vec![a.0, (a.0.0, y1), (xm, y1), (xm, y2), (b.0.0, y2), b.0]);
                }
            }
        } else {
            let mid = (a.0.1 + b.0.1) / 2.0;
            for k in [0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0] {
                let ym = snap(mid + k * 2.54);
                if ym < a.0.1 + 2.54 - EPS || ym > b.0.1 - 2.54 + EPS {
                    continue;
                }
                out.push(vec![a.0, (a.0.0, ym), (b.0.0, ym), b.0]);
            }
        }
        return out;
    }

    // Perpendicular: L through the corner [b.x, a.y] (a = horizontal pin).
    if !h_a && h_b {
        std::mem::swap(&mut a, &mut b);
    }
    if a.1.0.abs() > 0.5 && b.1.1.abs() > 0.5 {
        let corner = (b.0.0, a.0.1);
        if (corner.0 - a.0.0) * a.1.0 > EPS && (corner.1 - b.0.1) * b.1.1 > EPS {
            out.push(vec![a.0, corner, b.0]);
        }
        return out;
    }

    // Same direction: hook around the exit side.
    if a.1.0.abs() > 0.5 && a.1.0 == b.1.0 {
        if (a.0.1 - b.0.1).abs() < EPS {
            return out;
        }
        let base = if a.1.0 > 0.0 {
            a.0.0.max(b.0.0)
        } else {
            a.0.0.min(b.0.0)
        };
        for s in [2.54, 5.08, 7.62, 10.16, 12.7] {
            let xm = snap(base + a.1.0 * s);
            out.push(vec![a.0, (xm, a.0.1), (xm, b.0.1), b.0]);
        }
        return out;
    }
    if a.1.1.abs() > 0.5 && a.1.1 == b.1.1 {
        if (a.0.0 - b.0.0).abs() < EPS {
            return out;
        }
        let base = if a.1.1 > 0.0 {
            a.0.1.max(b.0.1)
        } else {
            a.0.1.min(b.0.1)
        };
        for s in [2.54, 5.08, 7.62, 10.16, 12.7] {
            let ym = snap(base + a.1.1 * s);
            out.push(vec![a.0, (a.0.0, ym), (b.0.0, ym), b.0]);
        }
        return out;
    }
    out
}

// ----------------------------------------------------------------------
// Group prediction
// ----------------------------------------------------------------------

/// Predict the endpoint subsets of a signal net that wire as real-wire
/// trees (shared between the orientation engine and the actual wiring):
/// * one Zener anchor family (single group root): everything, hub included;
/// * <= 3 endpoints: the non-hub endpoints (>= 2 of them);
/// * larger nets: every same-group subset with >= 2 members.
///
/// Returns index subsets into `eps_placed`.
pub(crate) fn group_wire_subsets(
    eps_placed: &[usize],
    is_hub: &dyn Fn(usize) -> bool,
    group_root: &dyn Fn(usize) -> usize,
    refdes: &dyn Fn(usize) -> String,
) -> Vec<Vec<usize>> {
    if eps_placed.len() <= 1 {
        return Vec::new();
    }
    let roots: BTreeSet<usize> = eps_placed.iter().map(|&pi| group_root(pi)).collect();
    if roots.len() == 1 {
        return vec![(0..eps_placed.len()).collect()];
    }
    if eps_placed.len() <= 3 {
        let non_hub: Vec<usize> = (0..eps_placed.len())
            .filter(|&i| !is_hub(eps_placed[i]))
            .collect();
        if non_hub.len() < 2 {
            return Vec::new();
        }
        return vec![non_hub];
    }
    // Large net: only same-group subsets wire.
    let mut roots: Vec<usize> = roots.into_iter().collect();
    roots.sort_by(|&a, &b| natord::compare(&refdes(a), &refdes(b)));
    let mut subsets: Vec<Vec<usize>> = Vec::new();
    for root in roots {
        let members: Vec<usize> = (0..eps_placed.len())
            .filter(|&i| group_root(eps_placed[i]) == root)
            .collect();
        if members.len() >= 2 {
            subsets.push(members);
        }
    }
    subsets
}

/// A hub breaks its signal nets into labels (rule applied per component).
pub(crate) fn is_hub_comp(cfg: &SchConfig, design: &DesignModel, ci: usize) -> bool {
    design.comps[ci].visible_pins > cfg.hub_pin_count_threshold
}

/// "Wirable pair" net: exactly 2 pins on different components, neither a
/// port nor exposed by a child block, with **facing** outward directions on
/// one axis — destined for a direct/Z wire, so no labels and no label
/// keepout. The hub threshold also applies here, except when both
/// components belong to the same placement group (Zener `anchor=` info).
pub(crate) fn is_pair_net_static(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &[PlacedComp],
    nets: &[SheetNet],
    sn: usize,
) -> bool {
    let net = &nets[sn];
    if net.endpoints.len() != 2 || net.port.is_some() || net.on_child_blocks {
        return false;
    }
    let (a, b) = (&net.endpoints[0], &net.endpoints[1]);
    if a.0 == b.0 {
        return false;
    }
    let (pa, pb) = (&placed[a.0], &placed[b.0]);
    let same_group = pa.group_root == pb.group_root;
    if !same_group && (is_hub_comp(cfg, design, pa.comp) || is_hub_comp(cfg, design, pb.comp)) {
        return false;
    }
    let Some(da) = design.comps[pa.comp]
        .geom
        .pin_outward(&a.1, pa.rotation, pa.mirror)
    else {
        return false;
    };
    let Some(db) = design.comps[pb.comp]
        .geom
        .pin_outward(&b.1, pb.rotation, pb.mirror)
    else {
        return false;
    };
    (da.0.abs() > 0.5 && db.0.abs() > 0.5 && da.0 != db.0)
        || (da.1.abs() > 0.5 && db.1.abs() > 0.5 && da.1 != db.1)
}

// ----------------------------------------------------------------------
// Tree wiring pass (Router extension)
// ----------------------------------------------------------------------

/// Snapshot of the wiring state for rollback (a tree that cannot get its
/// net label rolls back entirely; the net falls back to labels).
pub(crate) struct WiringSnapshot {
    wires: usize,
    segs: usize,
    junctions: usize,
    net_labels: usize,
    global_labels: usize,
    hier_labels: usize,
    label_boxes: usize,
}

impl Router<'_> {
    pub(crate) fn snapshot_wiring(&self) -> WiringSnapshot {
        WiringSnapshot {
            wires: self.out.wires.len(),
            segs: self.reg.segs.len(),
            junctions: self.out.junctions.len(),
            net_labels: self.out.net_labels.len(),
            global_labels: self.out.global_labels.len(),
            hier_labels: self.out.hier_labels.len(),
            label_boxes: self.reg.label_boxes.len(),
        }
    }

    pub(crate) fn rollback_wiring(&mut self, snap: &WiringSnapshot) {
        self.out.wires.truncate(snap.wires);
        self.reg.segs.truncate(snap.segs);
        self.out.junctions.truncate(snap.junctions);
        self.out.net_labels.truncate(snap.net_labels);
        self.out.global_labels.truncate(snap.global_labels);
        self.out.hier_labels.truncate(snap.hier_labels);
        self.reg.label_boxes.truncate(snap.label_boxes);
    }

    /// Collinear overlap with an already placed wire of the same net
    /// (doubled wire).
    pub(crate) fn overlaps_same_net(&self, path: &[Point], net: &str) -> bool {
        for w in path.windows(2) {
            let seg = mk_seg(w[0], w[1], net);
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let seg_h = (seg.y1 - seg.y2).abs() < EPS;
            for other in &self.reg.segs {
                if other.net != net {
                    continue;
                }
                let other_h = (other.y1 - other.y2).abs() < EPS;
                if seg_h != other_h {
                    continue;
                }
                if seg_h {
                    if (seg.y1 - other.y1).abs() > EPS {
                        continue;
                    }
                    let lo = seg.x1.min(seg.x2).max(other.x1.min(other.x2));
                    let hi = seg.x1.max(seg.x2).min(other.x1.max(other.x2));
                    if hi - lo > EPS {
                        return true;
                    }
                } else {
                    if (seg.x1 - other.x1).abs() > EPS {
                        continue;
                    }
                    let lo = seg.y1.min(seg.y2).max(other.y1.min(other.y2));
                    let hi = seg.y1.max(seg.y2).min(other.y1.max(other.y2));
                    if hi - lo > EPS {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// Hard feasibility of a real-wire tree candidate, independent of foreign
    /// crossings: clean route (`path_ok` with **no** exclusion — a wire never
    /// crosses a body, not even its own component's), no doubled wire, no
    /// SAME-net frank crossing (that would be a missing junction) and no
    /// traversal of a foreign predicted power corridor. Foreign crossings are
    /// electrically harmless and are scored separately (`wire_cost`) rather than
    /// rejected here: an analog backbone may accept one under penalty to stay
    /// continuous instead of breaking into labels.
    ///
    /// Length is deliberately NOT a feasibility gate — an analog net stays one
    /// continuous wire whatever its length. Only a physically absurd path
    /// (beyond `wire_length_guard_mm`, a routing-bug guard) is rejected.
    pub(crate) fn tree_candidate_feasible(&self, path: &[Point], net: &str) -> bool {
        if path_length_mm(path) > self.cfg.wire_length_guard_mm + EPS {
            return false;
        }
        if !self.path_ok(path, net, &[]) {
            return false;
        }
        if self.overlaps_same_net(path, net) {
            return false;
        }
        if self.reg.same_net_frank_crossing(path, net) {
            return false;
        }
        // A digital tree dodges the full predicted power corridor (the
        // conservative max-stretch keepout). An analog backbone is the
        // engineer's top priority — a single continuous wire from the passives
        // into the part — so it ignores the PREDICTED power keepouts entirely:
        // power stubs are routed last and re-plan around the committed wire
        // (min-crossing doglegs, then a visual-relaxed rescue that still
        // forbids any real contact). The wire is only kept off genuine
        // electrical contacts (foreign pin on the path, foreign wire touch),
        // which `path_ok` already guarantees.
        if !self.analog_wiring {
            for w in path.windows(2) {
                let seg = mk_seg(w[0], w[1], net);
                for c in &self.corridors {
                    if c.zone == crate::texts::CorridorZone::Corridor
                        && c.net != net
                        && seg_intersects_box(&seg, &c.bbox)
                    {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// A real-wire tree candidate placeable with NO frank crossing at all
    /// (`tree_candidate_feasible` plus zero crossings). Kept for the
    /// crossing-averse passes (branch stubs): those jut into a neighbour's
    /// band and must not also cross a foreign wire.
    pub(crate) fn tree_candidate_ok(&self, path: &[Point], net: &str) -> bool {
        self.tree_candidate_feasible(path, net) && self.reg.count_crossings(path) == 0
    }

    /// Cost of a candidate wire, in millimeters of equivalent length: its
    /// Manhattan length, plus `bend_penalty_mm` per right-angle bend (straight
    /// wires preferred, bends pushed onto branches), plus `crossing_penalty_mm`
    /// per foreign-net frank crossing (crossings avoided but accepted when they
    /// keep the net continuous). Same-net crossings are excluded upstream by
    /// `tree_candidate_feasible`.
    pub(crate) fn wire_cost(&self, path: &[Point], net: &str) -> f64 {
        path_length_mm(path)
            + self.cfg.bend_penalty_mm * path_bends(path) as f64
            + self.cfg.crossing_penalty_mm * self.reg.foreign_crossings(path, net) as f64
    }

    /// A junction is required when a NEW wire ends at `p` (kicad-cli
    /// probes): a same-net pin already connects coincident ends without a
    /// junction; a wire end on the **interior** of a segment (tee) or a
    /// third wire end at one point does need one.
    pub(crate) fn junction_needed_at(&self, p: Point, net: &str) -> bool {
        if self
            .reg
            .pins
            .iter()
            .any(|(px, py, pn)| pn == net && (px - p.0).abs() < EPS && (py - p.1).abs() < EPS)
        {
            return false;
        }
        let mut ends = 0;
        let mut interior = false;
        for seg in &self.reg.segs {
            if seg.net != net {
                continue;
            }
            let end_a = (seg.x1 - p.0).abs() < EPS && (seg.y1 - p.1).abs() < EPS;
            let end_b = (seg.x2 - p.0).abs() < EPS && (seg.y2 - p.1).abs() < EPS;
            if end_a || end_b {
                ends += 1;
            } else if point_on_seg(p.0, p.1, seg) {
                interior = true;
            }
        }
        interior || ends >= 2
    }

    pub(crate) fn push_junction(&mut self, at: Point) {
        if !self
            .out
            .junctions
            .iter()
            .any(|j| (j.0 - at.0).abs() < 1e-6 && (j.1 - at.1).abs() < 1e-6)
        {
            self.out.junctions.push(at);
        }
    }

    /// Candidates joining an endpoint to an already placed tree: straight
    /// tee in the pin axis onto a perpendicular segment (real junction),
    /// two-segment L toward a segment or its end, or a pin-to-pin wire
    /// toward an already wired endpoint. Sorted shortest first.
    pub(crate) fn tree_join_candidates(
        &self,
        ep: &Endpoint,
        segs: &[(f64, f64, f64, f64)],
        wired: &[Endpoint],
    ) -> Vec<(Vec<Point>, Point)> {
        let mut out: Vec<(Vec<Point>, Point)> = Vec::new();
        for &(x1, y1, x2, y2) in segs {
            let seg_h = (y1 - y2).abs() < EPS;
            if dist((x1, y1), (x2, y2)) < EPS {
                continue;
            }
            if ep.dir.1.abs() > 0.5 && seg_h {
                // Vertical pin toward a horizontal segment: straight tee or
                // L via the closest end.
                let y = y1;
                if (y - ep.pos.1) * ep.dir.1 > EPS {
                    if within(ep.pos.0, x1, x2) {
                        let join = (ep.pos.0, y);
                        out.push((vec![ep.pos, join], join));
                    } else {
                        let ex = if (x1 - ep.pos.0).abs() <= (x2 - ep.pos.0).abs() {
                            x1
                        } else {
                            x2
                        };
                        let join = (round4(ex), y);
                        out.push((vec![ep.pos, (ep.pos.0, y), join], join));
                    }
                }
            }
            if ep.dir.0.abs() > 0.5 && !seg_h {
                let x = x1;
                if (x - ep.pos.0) * ep.dir.0 > EPS {
                    if within(ep.pos.1, y1, y2) {
                        let join = (x, ep.pos.1);
                        out.push((vec![ep.pos, join], join));
                    } else {
                        let ey = if (y1 - ep.pos.1).abs() <= (y2 - ep.pos.1).abs() {
                            y1
                        } else {
                            y2
                        };
                        let join = (x, round4(ey));
                        out.push((vec![ep.pos, (x, ep.pos.1), join], join));
                    }
                }
            }
            if ep.dir.1.abs() > 0.5 && !seg_h {
                // Vertical pin toward a vertical segment: stub + horizontal
                // return (tee).
                for s in [2.54, 5.08, 7.62, 10.16] {
                    let yk = round4(ep.pos.1 + ep.dir.1 * s);
                    if !within(yk, y1, y2) {
                        continue;
                    }
                    let join = (x1, yk);
                    out.push((vec![ep.pos, (ep.pos.0, yk), join], join));
                }
            }
            if ep.dir.0.abs() > 0.5 && seg_h {
                for s in [2.54, 5.08, 7.62, 10.16] {
                    let xk = round4(ep.pos.0 + ep.dir.0 * s);
                    if !within(xk, x1, x2) {
                        continue;
                    }
                    let join = (xk, y1);
                    out.push((vec![ep.pos, (xk, ep.pos.1), join], join));
                }
            }
        }
        for w in wired {
            for cand in tree_pair_candidates(self.cfg, (ep.pos, ep.dir), (w.pos, w.dir)) {
                let join = *cand.last().expect("non-empty candidate");
                out.push((cand, join));
            }
        }
        out.sort_by(|p, q| {
            path_length_mm(&p.0)
                .partial_cmp(&path_length_mm(&q.0))
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(
                    p.1.0
                        .partial_cmp(&q.1.0)
                        .unwrap_or(std::cmp::Ordering::Equal),
                )
                .then(
                    p.1.1
                        .partial_cmp(&q.1.1)
                        .unwrap_or(std::cmp::Ordering::Equal),
                )
        });
        out
    }

    /// Wire an endpoint subset as a tree of real wires: seed = a routable
    /// pair, then greedy joins of the remaining endpoints (closest to the
    /// tree first) by tee/L/direct wire — real junction at every tee.
    /// Returns the endpoints actually wired (subset of the input); the others
    /// fall back to labels. **No rollback here** — the caller snapshots.
    ///
    /// With `best_coverage` the seed pair is chosen to maximize the number of
    /// endpoints reached (each candidate seed is grown in a sandbox and rolled
    /// back, then the best is replayed) — analog nets want the whole net on a
    /// single continuous wire, so a greedy seed that dead-ends after two pins
    /// is not good enough. Without it the first routable seed wins (the
    /// original, cheaper behavior kept for digital hub trees).
    pub(crate) fn wire_group_tree(
        &mut self,
        net_name: &str,
        subset: &[Endpoint],
        best_coverage: bool,
    ) -> Vec<Endpoint> {
        let mut list: Vec<Endpoint> = subset.to_vec();
        list.sort_by(|a, b| {
            natord::compare(
                &self.design.comps[self.model.placed[a.placed].comp].refdes,
                &self.design.comps[self.model.placed[b.placed].comp].refdes,
            )
            .then_with(|| a.pad.cmp(&b.pad))
        });

        // Seed candidates: pairs by increasing Manhattan distance.
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        for i in 0..list.len() {
            for j in i + 1..list.len() {
                pairs.push((i, j));
            }
        }
        pairs.sort_by(|p, q| {
            let dp = (list[p.1].pos.0 - list[p.0].pos.0).abs()
                + (list[p.1].pos.1 - list[p.0].pos.1).abs();
            let dq = (list[q.1].pos.0 - list[q.0].pos.0).abs()
                + (list[q.1].pos.1 - list[q.0].pos.1).abs();
            dp.partial_cmp(&dq).unwrap_or(std::cmp::Ordering::Equal)
        });

        if !best_coverage {
            let idx = self.grow_tree(net_name, &list, &pairs);
            return idx.into_iter().map(|i| list[i].clone()).collect();
        }

        // Best coverage: grow from each seed in a sandbox, keep the seed
        // reaching the most endpoints; among equal-coverage seeds keep the
        // cheapest tree (fewest crossings and bends, then shortest), earliest
        // seed breaking any remaining tie. Analog subsets are small, so trying
        // every seed is cheap and yields the cleanest continuous wire.
        let mut best_seed: Option<(usize, usize)> = None;
        let mut best_len = 0usize;
        let mut best_cost = f64::INFINITY;
        for &seed in &pairs {
            let undo = self.snapshot_wiring();
            let idx = self.grow_tree(net_name, &list, std::slice::from_ref(&seed));
            let cost = self.tree_cost_since(net_name, undo.wires);
            self.rollback_wiring(&undo);
            let better = idx.len() > best_len || (idx.len() == best_len && cost < best_cost - EPS);
            if better {
                best_len = idx.len();
                best_cost = cost;
                best_seed = Some(seed);
            }
        }
        let Some(seed) = best_seed else {
            return Vec::new();
        };
        let idx = self.grow_tree(net_name, &list, std::slice::from_ref(&seed));
        idx.into_iter().map(|i| list[i].clone()).collect()
    }

    /// Total `wire_cost` of the wires appended since index `wires_start`.
    /// Used to rank equal-coverage seed trees in `wire_group_tree`.
    fn tree_cost_since(&self, net: &str, wires_start: usize) -> f64 {
        self.out.wires[wires_start..]
            .iter()
            .map(|w| self.wire_cost(w, net))
            .sum()
    }

    /// Grow one real-wire tree from the first routable pair in `seed_pairs`,
    /// then greedily join the remaining endpoints. Returns the sorted indices
    /// (into `list`) actually wired; wires/junctions are committed to `self`.
    fn grow_tree(
        &mut self,
        net_name: &str,
        list: &[Endpoint],
        seed_pairs: &[(usize, usize)],
    ) -> Vec<usize> {
        let tree_seg_start = self.reg.segs.len();
        let mut wired_idx: Vec<usize> = Vec::new();
        // Foreign crossings are tolerated (under `wire_cost` penalty) only when
        // an analog backbone would otherwise break into labels; digital trees
        // stay strictly crossing-free.
        let allow_crossings = self.analog_wiring;
        'seed: for &(i, j) in seed_pairs {
            // Lowest-cost seed shape for this pair: straightest, fewest bends,
            // and crossing-free when a crossing-free shape exists.
            let mut best: Option<(f64, Vec<Point>)> = None;
            for cand in tree_pair_candidates(
                self.cfg,
                (list[i].pos, list[i].dir),
                (list[j].pos, list[j].dir),
            ) {
                if !self.tree_candidate_feasible(&cand, net_name) {
                    continue;
                }
                if !allow_crossings && self.reg.foreign_crossings(&cand, net_name) != 0 {
                    continue;
                }
                let cost = self.wire_cost(&cand, net_name);
                let better = match &best {
                    Some((bc, _)) => cost < *bc - EPS,
                    None => true,
                };
                if better {
                    best = Some((cost, cand));
                }
            }
            if let Some((_, cand)) = best {
                self.out.wires.push(cand.clone());
                self.reg.register_path(&cand, net_name);
                wired_idx.push(i);
                wired_idx.push(j);
                break 'seed;
            }
        }
        if wired_idx.is_empty() {
            return Vec::new();
        }

        let mut rest: Vec<usize> = (0..list.len()).filter(|i| !wired_idx.contains(i)).collect();
        let mut progress = true;
        while progress && !rest.is_empty() {
            progress = false;
            let dist_to_tree = |i: usize| -> f64 {
                wired_idx
                    .iter()
                    .map(|&w| {
                        (list[w].pos.0 - list[i].pos.0).abs()
                            + (list[w].pos.1 - list[i].pos.1).abs()
                    })
                    .fold(f64::INFINITY, f64::min)
            };
            rest.sort_by(|&a, &b| {
                dist_to_tree(a)
                    .partial_cmp(&dist_to_tree(b))
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then_with(|| {
                        natord::compare(
                            &self.design.comps[self.model.placed[list[a].placed].comp].refdes,
                            &self.design.comps[self.model.placed[list[b].placed].comp].refdes,
                        )
                    })
            });
            for (ri, &i) in rest.iter().enumerate() {
                let tree_segs: Vec<(f64, f64, f64, f64)> = self.reg.segs[tree_seg_start..]
                    .iter()
                    .filter(|s| s.net == net_name)
                    .map(|s| (s.x1, s.y1, s.x2, s.y2))
                    .collect();
                let wired_eps: Vec<Endpoint> = wired_idx.iter().map(|&w| list[w].clone()).collect();
                // Lowest-cost join for this endpoint: a straight tee onto the
                // backbone beats an L into a corner (bends land on the branch,
                // not the main run), and a crossing-free join beats a crossing.
                let mut best: Option<(f64, Vec<Point>, Point)> = None;
                for (path, join) in self.tree_join_candidates(&list[i], &tree_segs, &wired_eps) {
                    if !self.tree_candidate_feasible(&path, net_name) {
                        continue;
                    }
                    if !allow_crossings && self.reg.foreign_crossings(&path, net_name) != 0 {
                        continue;
                    }
                    let cost = self.wire_cost(&path, net_name);
                    let better = match &best {
                        Some((bc, _, _)) => cost < *bc - EPS,
                        None => true,
                    };
                    if better {
                        best = Some((cost, path, join));
                    }
                }
                let done = if let Some((_, path, join)) = best {
                    let need_junction = self.junction_needed_at(join, net_name);
                    self.out.wires.push(path.clone());
                    self.reg.register_path(&path, net_name);
                    if need_junction {
                        self.push_junction(join);
                    }
                    true
                } else {
                    false
                };
                if done {
                    wired_idx.push(i);
                    rest.remove(ri);
                    progress = true;
                    break;
                }
            }
        }
        wired_idx.sort_unstable();
        wired_idx
    }

    /// No segment (same net included) may cross the box of a text about to
    /// be placed ON the tree (`skip` = the carrying segment itself).
    fn box_clear_of_all_segs(&self, bbox: &BBox, skip: Option<(f64, f64, f64, f64)>) -> bool {
        for seg in &self.reg.segs {
            if let Some((x1, y1, x2, y2)) = skip
                && (seg.x1 - x1).abs() < EPS
                && (seg.y1 - y1).abs() < EPS
                && (seg.x2 - x2).abs() < EPS
                && (seg.y2 - y2).abs() < EPS
            {
                continue;
            }
            if seg_intersects_box(seg, bbox) {
                return false;
            }
        }
        true
    }

    /// Net label placed ON the tree: anchored on a grid point of a
    /// horizontal segment (a mid-wire label names the net), text underlined
    /// by the segment. Fallback: a hanging branch (stub + junction) carrying
    /// the label — never a vertical label. Returns false when no clean spot
    /// exists.
    pub(crate) fn place_tree_net_label(
        &mut self,
        net_name: &str,
        tree_seg_start: usize,
        allow_branch: bool,
    ) -> bool {
        let g = self.cfg.grid_mm;
        let w = label_text_width(net_name);
        let snap = |v: f64| round4((v / g).round() * g);
        // A cosmetic annotation (`!allow_branch`) may sit on a segment shorter
        // than the text and let the label overhang one end (clearance still
        // enforced) — the continuous analog backbone rarely offers a full
        // text-width straight run. A functional label must stay underlined.
        let min_seg = if allow_branch {
            w
        } else {
            self.cfg.label_elbow_mm
        };
        let overhang = if allow_branch { 1.27 } else { w };
        let mut horiz: Vec<(f64, f64, f64, f64)> = self.reg.segs[tree_seg_start..]
            .iter()
            .filter(|s| s.net == net_name)
            .filter(|s| (s.y1 - s.y2).abs() < EPS && (s.x2 - s.x1).abs() >= min_seg - EPS)
            .map(|s| (s.x1, s.y1, s.x2, s.y2))
            .collect();
        horiz.sort_by(|p, q| {
            ((q.2 - q.0).abs())
                .partial_cmp(&(p.2 - p.0).abs())
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(p.1.partial_cmp(&q.1).unwrap_or(std::cmp::Ordering::Equal))
                .then(p.0.partial_cmp(&q.0).unwrap_or(std::cmp::Ordering::Equal))
        });
        for seg in &horiz {
            let xlo = seg.0.min(seg.2);
            let xhi = seg.0.max(seg.2);
            let y = seg.1;
            let mut anchors: Vec<(Point, i32)> = Vec::new();
            let mut x = snap(xlo + w);
            while x <= xhi + EPS {
                anchors.push(((round4(x), y), 180));
                x = round4(x + g);
            }
            let mut x = snap(xhi - w);
            while x >= xlo - EPS {
                anchors.push(((round4(x), y), 0));
                x = round4(x - g);
            }
            // Overhang anchors at the segment ends (annotation mode only).
            if !allow_branch {
                anchors.push(((round4(xhi), y), 180));
                anchors.push(((round4(xlo), y), 0));
            }
            for (at, rotation) in anchors {
                let bbox = crate::route::label_text_box(net_name, at, rotation);
                if bbox.x1 < xlo - overhang - EPS || bbox.x2 > xhi + overhang + EPS {
                    continue;
                }
                if !self.label_box_clear(&bbox, net_name, true) {
                    continue;
                }
                if !self.box_clear_of_all_segs(&bbox, Some(*seg)) {
                    continue;
                }
                self.out
                    .net_labels
                    .push((net_name.to_string(), at, rotation));
                self.reg.label_boxes.push(crate::route::LabelBox {
                    bbox,
                    net: net_name.to_string(),
                    also: Vec::new(),
                });
                return true;
            }
        }
        // Fallback: hanging branch with a real junction at the branch point.
        // Skipped for a purely cosmetic annotation (`allow_branch == false`):
        // a branch jutting out of the tree into a neighbor's routing band is
        // worse than an unnamed continuous wire.
        if !allow_branch {
            return false;
        }
        if let Some((path, at, rotation)) = self.tree_branch_stub(net_name, tree_seg_start) {
            self.out.wires.push(path.clone());
            self.reg.register_path(&path, net_name);
            self.push_junction(path[0]);
            let bbox = crate::route::label_text_box(net_name, at, rotation);
            self.out
                .net_labels
                .push((net_name.to_string(), at, rotation));
            self.reg.label_boxes.push(crate::route::LabelBox {
                bbox,
                net: net_name.to_string(),
                also: Vec::new(),
            });
            return true;
        }
        false
    }

    /// Hanging branch from a tree segment: grid branch point, short
    /// perpendicular stub, horizontal elbow underlining the text. Serves
    /// labels AND hierarchical labels of trees whose pins offer no free
    /// direction. Returns path + text anchor + label rotation.
    fn tree_branch_stub(
        &self,
        net_name: &str,
        tree_seg_start: usize,
    ) -> Option<(Vec<Point>, Point, i32)> {
        let g = self.cfg.grid_mm;
        let w = label_text_width(net_name).max(self.cfg.label_elbow_mm);
        let snap = |v: f64| round4((v / g).round() * g);
        let mut ordered: Vec<(f64, f64, f64, f64)> = self.reg.segs[tree_seg_start..]
            .iter()
            .filter(|s| s.net == net_name)
            .map(|s| (s.x1, s.y1, s.x2, s.y2))
            .collect();
        ordered.sort_by(|p, q| {
            let lp = (p.2 - p.0).abs() + (p.3 - p.1).abs();
            let lq = (q.2 - q.0).abs() + (q.3 - q.1).abs();
            lq.partial_cmp(&lp).unwrap_or(std::cmp::Ordering::Equal)
        });
        for seg in &ordered {
            let seg_h = (seg.1 - seg.3).abs() < EPS;
            if seg_h {
                let xlo = seg.0.min(seg.2);
                let xhi = seg.0.max(seg.2);
                let mut x = snap(xlo + 2.54);
                while x <= xhi - 2.54 + EPS {
                    for dy in [1.0, -1.0] {
                        for stub in [5.08, 7.62] {
                            let knee = (x, round4(seg.1 + dy * stub));
                            for side in [1.0, -1.0] {
                                let end = (round4(x + side * w), knee.1);
                                let path = vec![(x, seg.1), knee, end];
                                let rotation = if side > 0.0 { 180 } else { 0 };
                                let bbox = crate::route::label_text_box(net_name, end, rotation);
                                if !self.tree_candidate_ok(&path, net_name) {
                                    continue;
                                }
                                if !self.label_box_clear(&bbox, net_name, true) {
                                    continue;
                                }
                                if !self.box_clear_of_all_segs(&bbox, None) {
                                    continue;
                                }
                                return Some((path, end, rotation));
                            }
                        }
                    }
                    x = round4(x + g);
                }
            } else {
                let ylo = seg.1.min(seg.3);
                let yhi = seg.1.max(seg.3);
                let mut y = snap(ylo + 2.54);
                while y <= yhi - 2.54 + EPS {
                    for side in [1.0, -1.0] {
                        let end = (round4(seg.0 + side * w), y);
                        let path = vec![(seg.0, y), end];
                        let rotation = if side > 0.0 { 180 } else { 0 };
                        let bbox = crate::route::label_text_box(net_name, end, rotation);
                        if !self.tree_candidate_ok(&path, net_name) {
                            continue;
                        }
                        if !self.label_box_clear(&bbox, net_name, true) {
                            continue;
                        }
                        if !self.box_clear_of_all_segs(&bbox, None) {
                            continue;
                        }
                        return Some((path, end, rotation));
                    }
                    y = round4(y + g);
                }
            }
        }
        None
    }

    /// Hierarchical label of a port net wired as a tree: first at the end
    /// of a clean stub of a tree endpoint (flow order — "direct hier label"
    /// rule), otherwise at the end of a hanging branch (junction).
    pub(crate) fn place_tree_hier_label(
        &mut self,
        net_name: &str,
        direction: crate::writer::PortDirection,
        tree_eps: &[Endpoint],
        tree_seg_start: usize,
    ) -> bool {
        for ei in self.port_anchor_order(direction, tree_eps) {
            let ep = &tree_eps[ei];
            for (path, at, rotation) in self.hier_stub_candidates(net_name, ep) {
                if !self.path_ok(&path, net_name, &[ep.placed]) {
                    continue;
                }
                if self.overlaps_same_net(&path, net_name) {
                    continue;
                }
                if self.reg.count_crossings(&path) != 0 {
                    continue;
                }
                let bbox = crate::route::hier_text_box(net_name, at, rotation);
                if !self.label_box_clear(&bbox, net_name, true) {
                    continue;
                }
                if !self.box_clear_of_all_segs(&bbox, None) {
                    continue;
                }
                self.out.wires.push(path.clone());
                self.reg.register_path(&path, net_name);
                self.out
                    .hier_labels
                    .push((net_name.to_string(), direction, at, rotation));
                self.reg.label_boxes.push(crate::route::LabelBox {
                    bbox,
                    net: net_name.to_string(),
                    also: Vec::new(),
                });
                self.port_anchored.insert(net_name.to_string());
                return true;
            }
        }
        if let Some((path, at, branch_rotation)) = self.tree_branch_stub(net_name, tree_seg_start) {
            self.out.wires.push(path.clone());
            self.reg.register_path(&path, net_name);
            self.push_junction(path[0]);
            // Hier label body extends OPPOSITE the stub elbow.
            let rotation = if branch_rotation == 180 { 0 } else { 180 };
            let bbox = crate::route::hier_text_box(net_name, at, rotation);
            self.out
                .hier_labels
                .push((net_name.to_string(), direction, at, rotation));
            self.reg.label_boxes.push(crate::route::LabelBox {
                bbox,
                net: net_name.to_string(),
                also: Vec::new(),
            });
            self.port_anchored.insert(net_name.to_string());
            return true;
        }
        false
    }

    /// A signal net is analog (wired continuously) unless the analog/digital
    /// heuristic marked it digital. Called only on signal nets.
    fn net_is_analog(&self, sn: usize) -> bool {
        !self.design.nets[self.model.nets[sn].net].digital
    }

    /// Plan and wire a signal net with real wires when the rule allows.
    ///
    /// **Analog** nets are wired as a single continuous tree over ALL their
    /// endpoints (best-coverage seeding, never dropping the IC/hub side): the
    /// engineer wants an uninterrupted wire from the passives to the part.
    /// The tree carries one *annotation* net label (best effort — a wire that
    /// covers the whole net keeps flowing even when no clean label spot
    /// exists), and any endpoint the router could not reach falls back to a
    /// label (logged).
    ///
    /// **Digital** nets keep the historical group behavior: Zener group info
    /// first (all endpoints of one placement group together, hub included),
    /// the `<= 3` non-hub heuristic otherwise, one net label when the tree
    /// does not cover the whole net; hubs break into labels.
    pub(crate) fn plan_and_wire_group(&mut self, sn: usize) {
        let eps = self.eps_for_net(sn);
        let name = self.model.nets[sn].name.clone();
        let port = self.model.nets[sn].port;
        let on_blocks = self.model.nets[sn].on_child_blocks;
        let is_root = self.plan.sheets[self.model.sheet].parent.is_none();
        let analog = self.net_is_analog(sn);
        self.analog_wiring = analog;

        // Analog: the whole net is one continuous tree. Digital: the group
        // heuristic (hub-aware) decides the wired subsets.
        let subsets = if analog {
            if eps.len() >= 2 {
                vec![(0..eps.len()).collect::<Vec<usize>>()]
            } else {
                Vec::new()
            }
        } else {
            self.group_subsets(&eps)
        };
        if subsets.is_empty() {
            return;
        }

        let mut all_wired: BTreeSet<(i64, i64)> = BTreeSet::new();
        let mut wired_count = 0usize;
        for subset_idx in &subsets {
            let subset: Vec<Endpoint> = subset_idx.iter().map(|&i| eps[i].clone()).collect();
            let undo = self.snapshot_wiring(); // segs index == tree start
            let tree_seg_start = undo.segs;
            let wired = self.wire_group_tree(&name, &subset, analog);
            if wired.len() < 2 {
                self.rollback_wiring(&undo);
                continue;
            }
            let mut hier_on_tree = false;
            if let Some(direction) = port
                && !is_root
                && !self.port_anchored.contains(&name)
            {
                hier_on_tree = self.place_tree_hier_label(&name, direction, &wired, tree_seg_start);
            }
            // A label is functionally required as soon as the tree does not
            // cover the whole net (homonym labels elsewhere), child block
            // sheet pins expose it, or a port has no hier label on the tree
            // yet — its absence would leave the wired part electrically
            // detached from the labeled remainder.
            let rest_count = eps.len() - wired_count - wired.len();
            let needs_label = rest_count > 0
                || on_blocks
                || (port.is_some() && !is_root && !hier_on_tree)
                || subsets.len() > 1;
            // Analog trees never sprout a jutting branch for their label: a
            // branch reaching out of the tree walls off a neighbor's routing
            // band. If the required label cannot sit on the wire itself the
            // whole tree rolls back to labels (best effort, no interference).
            if needs_label {
                if !self.place_tree_net_label(&name, tree_seg_start, !analog) {
                    self.rollback_wiring(&undo);
                    if hier_on_tree {
                        self.port_anchored.remove(&name);
                    }
                    continue;
                }
            } else if analog {
                // Full-coverage analog net: its single naming label is pure
                // annotation (the wire already connects the whole net), so it
                // is DEFERRED until every analog tree is wired. Placing it now
                // would register a text keepout that could wall off a sibling
                // differential leg's continuous wire (its shunt drop must be
                // free to cross under where this label will sit); placing it
                // last lets the crossing wires interleave first, then the label
                // fills a remaining gap (best effort — KiCad auto-names if none).
                self.pending_annotations
                    .push((name.clone(), tree_seg_start));
            }
            wired_count += wired.len();
            for ep in &wired {
                all_wired.insert(quant(ep.pos));
            }
        }
        // Analog best-effort: report the endpoints that could not join the
        // continuous wire and fell back to labels — a fallback the engineer
        // asked to be told about.
        if analog && wired_count < eps.len() {
            let broke_small = eps.iter().any(|e| {
                !all_wired.contains(&quant(e.pos))
                    && self.design.comps[self.model.placed[e.placed].comp].visible_pins
                        <= self.cfg.analog_break_pin_count
            });
            if broke_small || wired_count == 0 {
                self.warnings.push(format!(
                    "analog net {name}: {}/{} endpoint(s) wired continuously, the rest fell back to labels (no clean route)",
                    wired_count,
                    eps.len()
                ));
            }
        }
        if !all_wired.is_empty() {
            self.group_wired.insert(sn, all_wired);
        }
        self.analog_wiring = false;
    }

    /// Direct wire for a facing 2-pin net: straight segment when aligned,
    /// otherwise a Z (dogleg) whose middle branch is staggered to never
    /// touch a foreign wire/pin. U-detours skirt obstacles on aligned
    /// channels. Among placeable candidates the first with zero frank
    /// crossings wins. Falls back to stub+labels when no path exists.
    pub(crate) fn try_direct_wire(&mut self, sn: usize, eps: &[Endpoint]) -> bool {
        if eps.len() != 2 || self.model.nets[sn].port.is_some() {
            return false;
        }
        if self.model.nets[sn].on_child_blocks {
            return false;
        }
        let name = self.model.nets[sn].name.clone();
        let (mut a, mut b) = (eps[0].clone(), eps[1].clone());
        if a.placed == b.placed {
            return false;
        }
        let manhattan = (b.pos.0 - a.pos.0).abs() + (b.pos.1 - a.pos.1).abs();
        // No upper length gate: a 2-pin analog net is a wire at any length.
        // Only reject a degenerate (coincident) or physically absurd span.
        if manhattan > self.cfg.wire_length_guard_mm || manhattan < 1e-6 {
            return false;
        }
        let exclude = [a.placed, b.placed];
        let g = self.cfg.grid_mm;
        let snap = |v: f64| round4((v / g).round() * g);

        let emit_best = |router: &mut Router, candidates: &[Vec<Point>]| -> bool {
            let mut best: Option<&Vec<Point>> = None;
            let mut best_crossings = usize::MAX;
            for cand in candidates {
                if !router.path_ok(cand, &name, &exclude) {
                    continue;
                }
                let crossings = router.reg.count_crossings(cand);
                if crossings < best_crossings {
                    best = Some(cand);
                    best_crossings = crossings;
                    if crossings == 0 {
                        break;
                    }
                }
            }
            let Some(best) = best else { return false };
            if best_crossings > 0 {
                // A facing 2-pin net whose ONLY direct wire would frank-cross a
                // foreign net does not route cleanly: fall back to labels rather
                // than force a crossing wire. Length is never the reason — a
                // crossing-free wire is emitted at any length — only the
                // crossing is (the engineer's "a wire must not start crossing
                // other nets" rule).
                return false;
            }
            router.out.wires.push(best.clone());
            router.reg.register_path(best, &name);
            true
        };

        // Horizontal channel.
        if a.dir.0.abs() > 0.5 && b.dir.0.abs() > 0.5 && a.dir.0 != b.dir.0 {
            if a.dir.0 < 0.0 {
                std::mem::swap(&mut a, &mut b);
            }
            let dx = b.pos.0 - a.pos.0;
            if dx <= EPS {
                return false;
            }
            if (a.pos.1 - b.pos.1).abs() < EPS {
                if emit_best(self, &[vec![a.pos, b.pos]]) {
                    return true;
                }
                let mut detours: Vec<Vec<Point>> = Vec::new();
                for inset in [2.54, 5.08] {
                    let x1 = snap(a.pos.0 + inset);
                    let x2 = snap(b.pos.0 - inset);
                    if x2 - x1 < EPS {
                        continue;
                    }
                    for dy in [-7.62, 7.62, -10.16, 10.16, -12.7, 12.7] {
                        let ym = snap(a.pos.1 + dy);
                        detours.push(vec![
                            a.pos,
                            (x1, a.pos.1),
                            (x1, ym),
                            (x2, ym),
                            (x2, b.pos.1),
                            b.pos,
                        ]);
                    }
                }
                return emit_best(self, &detours);
            }
            let mid = (a.pos.0 + b.pos.0) / 2.0;
            let mut zs: Vec<Vec<Point>> = Vec::new();
            for k in [0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0] {
                let xm = snap(mid + k * 2.54);
                if xm < a.pos.0 + 2.54 - EPS || xm > b.pos.0 - 2.54 + EPS {
                    continue;
                }
                zs.push(vec![a.pos, (xm, a.pos.1), (xm, b.pos.1), b.pos]);
            }
            return emit_best(self, &zs);
        }

        // Vertical channel.
        if a.dir.1.abs() > 0.5 && b.dir.1.abs() > 0.5 && a.dir.1 != b.dir.1 {
            if a.dir.1 < 0.0 {
                std::mem::swap(&mut a, &mut b);
            }
            let dy = b.pos.1 - a.pos.1;
            if dy <= EPS {
                return false;
            }
            if (a.pos.0 - b.pos.0).abs() < EPS {
                if emit_best(self, &[vec![a.pos, b.pos]]) {
                    return true;
                }
                let mut detours: Vec<Vec<Point>> = Vec::new();
                for inset in [2.54, 5.08] {
                    let y1 = snap(a.pos.1 + inset);
                    let y2 = snap(b.pos.1 - inset);
                    if y2 - y1 < EPS {
                        continue;
                    }
                    for dxu in [-7.62, 7.62, -10.16, 10.16, -12.7, 12.7] {
                        let xm = snap(a.pos.0 + dxu);
                        detours.push(vec![
                            a.pos,
                            (a.pos.0, y1),
                            (xm, y1),
                            (xm, y2),
                            (b.pos.0, y2),
                            b.pos,
                        ]);
                    }
                }
                return emit_best(self, &detours);
            }
            let mid = (a.pos.1 + b.pos.1) / 2.0;
            let mut zs: Vec<Vec<Point>> = Vec::new();
            for k in [0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0] {
                let ym = snap(mid + k * 2.54);
                if ym < a.pos.1 + 2.54 - EPS || ym > b.pos.1 - 2.54 + EPS {
                    continue;
                }
                zs.push(vec![a.pos, (a.pos.0, ym), (b.pos.0, ym), b.pos]);
            }
            return emit_best(self, &zs);
        }
        false
    }
}

/// Static probe used by the orientation engine: is one of the tree wire
/// candidates between two pins placeable in the **static** sense — within the
/// local orientation reach `orient_probe_reach_mm` (a placement proximity
/// heuristic: a pin faces a group only when that group is within local reach,
/// NOT the wire-vs-label gate), no instance body traversed (no exclusion: a
/// hook that dodges a connector by crossing its body is NOT routable), no
/// foreign pin on the path, no foreign predicted power corridor cut?
/// Dynamic obstacles (wires/labels placed later) remain the tree's
/// business: the probe is deliberately optimistic — enough to tell "turned
/// toward the group" from "back to the group".
#[allow(clippy::too_many_arguments)]
pub(crate) fn probe_routable(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &[PlacedComp],
    a: (Point, Point),
    b: (Point, Point),
    a_net: Option<&str>,
    corridors: &[(BBox, String)],
) -> bool {
    let bodies: Vec<BBox> = placed
        .iter()
        .map(|p| raw_box(&design.comps[p.comp].geom, p.at, p.rotation, p.mirror))
        .collect();
    let mut pin_positions: Vec<Point> = Vec::new();
    for p in placed {
        let geom = &design.comps[p.comp].geom;
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            if let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) {
                pin_positions.push(pos);
            }
        }
    }
    'cand: for path in tree_pair_candidates(cfg, a, b) {
        if path_length_mm(&path) > cfg.orient_probe_reach_mm + EPS {
            continue;
        }
        for w in path.windows(2) {
            let seg = mk_seg(w[0], w[1], "~probe~");
            if dist(w[0], w[1]) < EPS {
                continue;
            }
            let sweep = BBox {
                x1: seg.x1.min(seg.x2) + EPS,
                y1: seg.y1.min(seg.y2) + EPS,
                x2: seg.x1.max(seg.x2) - EPS,
                y2: seg.y1.max(seg.y2) - EPS,
            };
            for body in &bodies {
                if overlaps(body, &sweep) {
                    continue 'cand;
                }
            }
            for (bbox, net) in corridors {
                if a_net != Some(net.as_str()) && seg_intersects_box(&seg, bbox) {
                    continue 'cand;
                }
            }
            for pp in &pin_positions {
                // The path ends ARE the two connected pins: allowed.
                if (pp.0 - a.0.0).abs() < 1e-6 && (pp.1 - a.0.1).abs() < 1e-6 {
                    continue;
                }
                if (pp.0 - b.0.0).abs() < 1e-6 && (pp.1 - b.0.1).abs() < 1e-6 {
                    continue;
                }
                if point_on_seg(pp.0, pp.1, &seg) {
                    continue 'cand;
                }
            }
        }
        return true;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> SchConfig {
        SchConfig::default()
    }

    #[test]
    fn facing_pins_get_straight_then_z_candidates() {
        // a at (0,0) exits +x, b at (10.16,0) exits -x: straight first.
        let cands = tree_pair_candidates(
            &cfg(),
            ((0.0, 0.0), (1.0, 0.0)),
            ((10.16, 0.0), (-1.0, 0.0)),
        );
        assert_eq!(cands[0], vec![(0.0, 0.0), (10.16, 0.0)]);
        assert!(cands.len() > 1, "U-detours follow the straight candidate");
        // Misaligned: Z shapes with a staggered middle column.
        let zs = tree_pair_candidates(
            &cfg(),
            ((0.0, 0.0), (1.0, 0.0)),
            ((10.16, 5.08), (-1.0, 0.0)),
        );
        assert!(!zs.is_empty());
        for z in &zs {
            assert_eq!(z.len(), 4);
            assert_eq!(z[0], (0.0, 0.0));
            assert_eq!(z[3], (10.16, 5.08));
            // Middle column strictly between the pins.
            assert!(z[1].0 > 0.0 && z[1].0 < 10.16);
        }
    }

    #[test]
    fn perpendicular_pins_get_an_l() {
        // a at (0,0) exits +x; b at (10.16, 10.16) exits up (-y).
        let cands = tree_pair_candidates(
            &cfg(),
            ((0.0, 0.0), (1.0, 0.0)),
            ((10.16, 10.16), (0.0, -1.0)),
        );
        assert_eq!(cands, vec![vec![(0.0, 0.0), (10.16, 0.0), (10.16, 10.16)]]);
        // Wrong quadrant: no candidate (the corner is behind a pin).
        let none = tree_pair_candidates(
            &cfg(),
            ((0.0, 0.0), (-1.0, 0.0)),
            ((10.16, 10.16), (0.0, -1.0)),
        );
        assert!(none.is_empty());
    }

    #[test]
    fn same_side_pins_get_hooks() {
        let cands =
            tree_pair_candidates(&cfg(), ((0.0, 0.0), (1.0, 0.0)), ((0.0, 7.62), (1.0, 0.0)));
        assert!(!cands.is_empty());
        for c in &cands {
            assert_eq!(c.len(), 4);
            // The hook swings past the rightmost exit.
            assert!(c[1].0 > 0.0);
            assert_eq!(c[1].0, c[2].0);
        }
    }

    #[test]
    fn back_to_back_pins_have_no_candidates() {
        // Facing directions but negative channel width (backs turned).
        let cands = tree_pair_candidates(
            &cfg(),
            ((10.16, 0.0), (1.0, 0.0)),
            ((0.0, 0.0), (-1.0, 0.0)),
        );
        assert!(cands.is_empty());
    }

    #[test]
    fn subsets_single_group_takes_everything() {
        // 3 endpoints, all in group rooted at 0 (hub included).
        let subs = group_wire_subsets(&[0, 1, 2], &|pi| pi == 0, &|_| 0, &|pi| format!("U{pi}"));
        assert_eq!(subs, vec![vec![0, 1, 2]]);
    }

    #[test]
    fn subsets_small_net_drops_the_hub_side() {
        // 3 endpoints in 3 groups; endpoint 1 is a hub.
        let subs = group_wire_subsets(&[0, 1, 2], &|pi| pi == 1, &|pi| pi, &|pi| format!("U{pi}"));
        assert_eq!(subs, vec![vec![0, 2]]);
        // Two of three on hubs: nothing to wire.
        let none = group_wire_subsets(&[0, 1, 2], &|pi| pi != 0, &|pi| pi, &|pi| format!("U{pi}"));
        assert!(none.is_empty());
    }

    #[test]
    fn subsets_large_net_wires_same_group_clusters() {
        // 5 endpoints: comps 0,1 in group 0; comps 2,3 in group 2; comp 4 alone.
        let root = |pi: usize| match pi {
            0 | 1 => 0,
            2 | 3 => 2,
            _ => pi,
        };
        let subs = group_wire_subsets(&[0, 1, 2, 3, 4], &|_| false, &root, &|pi| format!("U{pi}"));
        assert_eq!(subs, vec![vec![0, 1], vec![2, 3]]);
    }

    #[test]
    fn path_length_is_manhattan() {
        assert!((path_length_mm(&[(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) - 7.0).abs() < 1e-9);
    }

    #[test]
    fn path_bends_counts_orientation_flips() {
        // Straight: no bend.
        assert_eq!(path_bends(&[(0.0, 0.0), (10.0, 0.0)]), 0);
        // Collinear split: still no bend.
        assert_eq!(path_bends(&[(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]), 0);
        // L: one bend.
        assert_eq!(path_bends(&[(0.0, 0.0), (10.0, 0.0), (10.0, 5.0)]), 1);
        // Z: two bends.
        assert_eq!(
            path_bends(&[(0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (10.0, 5.0)]),
            2
        );
        // A zero-length hop between two collinear runs is not a bend.
        assert_eq!(
            path_bends(&[(0.0, 0.0), (5.0, 0.0), (5.0, 0.0), (10.0, 0.0)]),
            0
        );
    }
}
