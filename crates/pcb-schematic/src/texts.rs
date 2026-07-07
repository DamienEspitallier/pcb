//! Reference/Value text placement (port of `placeInstanceTexts` from the
//! validated `layout.ts` proof of concept).
//!
//! Rules:
//! * canonical anchors first — Reference above the top-left corner of the
//!   **body**, Value at the bottom (right-justified over the lowest pin
//!   when the symbol has downward pins) — with an anti-drift slide (<= 1
//!   grid step, plus what it takes to clear the component's own pins, the
//!   anchor never leaving 2 steps from the raw box);
//! * collision is checked against real predicted obstacles: other bodies,
//!   already placed texts, predicted power stub corridors (starting at the
//!   PIN END — the body-to-pin band stays free), the component's own label
//!   stub zones and pin shafts with an **asymmetric** number halo (eeschema
//!   renders the number on one side only);
//! * a **lateral fallback** (ref above value, left-justified, right of the
//!   body then left) triggers only on a real collision; the block is
//!   centered on the body axis (+-0.8 mm) so a middle pin wire passes
//!   between the two lines;
//! * last resort keeps the canonical spot, tags its box with the nets of
//!   its own stubs (the wire may run under the text) and warns only when
//!   the text covers a foreign element;
//! * identical situations produce identical outcomes (deterministic
//!   placement order), and canonical spots that are statically free are
//!   **reserved** so an earlier fallback cannot steal them.

use crate::config::SchConfig;
use crate::geometry::BBox;
use crate::model::{DesignModel, NetClass};
use crate::place::{
    PlacedComp, SheetModel, SheetNet, body_box, label_stub_len, label_text_width, overlaps, raw_box,
};
use crate::round4;
use crate::route::{LabelBox, power_attachment};
use crate::wiring::is_pair_net_static;

type Point = (f64, f64);

const EPS: f64 = 1e-3;

/// Zone kind of a predicted power keepout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CorridorZone {
    /// Pin-to-symbol wire corridor (maximum vertical stretch included).
    Corridor,
    /// Graphic footprint (arrow/bars + rail name) at the minimal attachment.
    Graphic,
}

/// One predicted power keepout box.
pub(crate) struct Corridor {
    pub bbox: BBox,
    pub net: String,
    /// Placed-component index owning the pin.
    pub owner: usize,
    pub zone: CorridorZone,
}

/// Everything the text pass hands over to the router.
pub(crate) struct TextArtifacts {
    pub label_boxes: Vec<LabelBox>,
    pub corridors: Vec<Corridor>,
}

fn point_box(p: Point, m: f64) -> BBox {
    BBox {
        x1: p.0 - m,
        y1: p.1 - m,
        x2: p.0 + m,
        y2: p.1 + m,
    }
}

fn union_box(a: &BBox, b: &BBox) -> BBox {
    let mut out = *a;
    out.union(b);
    out
}

/// Reference text box. Left-justified (default): bottom-left anchor, text
/// above and to the right. Right-justified: bottom-right anchor, text above
/// and to the LEFT (used by the left-side lateral fallback so the block's
/// right edge glues to the component).
pub(crate) fn ref_text_box(name: &str, at: Point, justify_right: bool) -> BBox {
    let w = label_text_width(name);
    if justify_right {
        BBox {
            x1: at.0 - w,
            y1: at.1 - 2.2,
            x2: at.0,
            y2: at.1 + 0.4,
        }
    } else {
        BBox {
            x1: at.0,
            y1: at.1 - 2.2,
            x2: at.0 + w,
            y2: at.1 + 0.4,
        }
    }
}

/// Value text box (top anchor: text below, by justification).
pub(crate) fn value_text_box(name: &str, at: Point, justify_right: bool) -> BBox {
    let w = label_text_width(name);
    if justify_right {
        BBox {
            x1: at.0 - w,
            y1: at.1 - 0.4,
            x2: at.0,
            y2: at.1 + 2.2,
        }
    } else {
        BBox {
            x1: at.0,
            y1: at.1 - 0.4,
            x2: at.0 + w,
            y2: at.1 + 2.2,
        }
    }
}

/// Graphic footprint of a power symbol split in TWO precise boxes: narrow
/// shaft (line + arrow/bars, +-1.6 mm) on the connection side, wide rail
/// text at the far end only. A single wide box ate the top-left corner of
/// the neighbour and triggered lateral fallbacks for nothing.
pub(crate) fn power_graphic_zone_boxes(net_name: &str, at: Point, down: bool) -> [BBox; 2] {
    let half_text = (net_name.chars().count() as f64 * 0.762 + 1.27).max(2.54);
    if down {
        [
            BBox {
                x1: at.0 - 1.6,
                y1: at.1,
                x2: at.0 + 1.6,
                y2: at.1 + 3.81,
            },
            BBox {
                x1: at.0 - half_text,
                y1: at.1 + 3.81,
                x2: at.0 + half_text,
                y2: at.1 + 6.35,
            },
        ]
    } else {
        [
            BBox {
                x1: at.0 - 1.6,
                y1: at.1 - 3.81,
                x2: at.0 + 1.6,
                y2: at.1,
            },
            BBox {
                x1: at.0 - half_text,
                y1: at.1 - 7.62,
                x2: at.0 + half_text,
                y2: at.1 - 3.81,
            },
        ]
    }
}

/// Predicted power stub corridors of ALL placed components: a text (of an
/// instance or a net label) sitting on one would doom every candidate of
/// the power dogleg at wiring time. The corridor covers the maximum
/// vertical stretch of the stub (v_extra <= 10.16 of the wiring retries),
/// +-1.27 mm around the path, one box PER SEGMENT, and starts at the PIN
/// END — the body-to-pin-end band stays free for canonical texts. The
/// graphic footprint at the minimal attachment is reserved for instance
/// texts only (`Graphic` zone) — and skipped when the glyph folds back
/// toward the pin (it fatally covers the stub anyway).
pub(crate) fn power_stub_corridors(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &[PlacedComp],
    nets: &[SheetNet],
    pin_net: &std::collections::HashMap<(usize, String), usize>,
) -> Vec<Corridor> {
    let mut out: Vec<Corridor> = Vec::new();
    for (pi, p) in placed.iter().enumerate() {
        let geom = &design.comps[p.comp].geom;
        let mut seen: std::collections::BTreeSet<(i64, i64)> = std::collections::BTreeSet::new();
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(&sn) = pin_net.get(&(pi, pin.number.clone())) else {
                continue;
            };
            if nets[sn].class == NetClass::Signal {
                continue;
            }
            let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                continue;
            };
            if !seen.insert(((pos.0 * 10000.0) as i64, (pos.1 * 10000.0) as i64)) {
                continue;
            }
            let Some(dir) = geom.pin_outward(&pin.number, p.rotation, p.mirror) else {
                continue;
            };
            let down = nets[sn].class == NetClass::Ground;
            let name = nets[sn].name.clone();
            let att = power_attachment(cfg, pos, dir, down, 0.0, 10.16);
            for (i, w) in att.path.windows(2).enumerate() {
                let mut seg = union_box(&point_box(w[0], 1.27), &point_box(w[1], 1.27));
                if i == 0 {
                    // Clip on the component side: the wire starts at the pin
                    // end, never before.
                    if dir.0 > 0.5 {
                        seg.x1 = pos.0;
                    } else if dir.0 < -0.5 {
                        seg.x2 = pos.0;
                    } else if dir.1 > 0.5 {
                        seg.y1 = pos.1;
                    } else {
                        seg.y2 = pos.1;
                    }
                }
                out.push(Corridor {
                    bbox: seg,
                    net: name.clone(),
                    owner: pi,
                    zone: CorridorZone::Corridor,
                });
            }
            let att0 = power_attachment(cfg, pos, dir, down, 0.0, 0.0);
            let away = if att0.down {
                att0.symbol_at.1 >= pos.1 - EPS
            } else {
                att0.symbol_at.1 <= pos.1 + EPS
            };
            if away || dir.1.abs() < 0.5 {
                for gb in power_graphic_zone_boxes(&name, att0.symbol_at, att0.down) {
                    out.push(Corridor {
                        bbox: gb,
                        net: name.clone(),
                        owner: pi,
                        zone: CorridorZone::Graphic,
                    });
                }
            }
        }
    }
    out
}

/// Zones reserved by the SIGNAL label stubs of one component: the vertical
/// stub corridor (shaft + upcoming elbow) and, for a horizontal pin, the
/// footprint of the label TEXT at every stub length the label pass tries.
/// Paired nets (upcoming direct/Z wire) reserve only the thin exit band of
/// the pin: the wire leaves along its axis, the Z elbow staggers around
/// the middle.
fn own_signal_stub_zones(
    cfg: &SchConfig,
    design: &DesignModel,
    model: &SheetModel,
    pi: usize,
) -> Vec<(BBox, String)> {
    let mut out: Vec<(BBox, String)> = Vec::new();
    let p = &model.placed[pi];
    let geom = &design.comps[p.comp].geom;
    let mut seen: std::collections::BTreeSet<(i64, i64)> = std::collections::BTreeSet::new();
    for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
        let Some(&sn) = model.pin_net.get(&(pi, pin.number.clone())) else {
            continue;
        };
        if model.nets[sn].class != NetClass::Signal {
            continue;
        }
        let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
            continue;
        };
        if !seen.insert(((pos.0 * 10000.0) as i64, (pos.1 * 10000.0) as i64)) {
            continue;
        }
        let Some(dir) = geom.pin_outward(&pin.number, p.rotation, p.mirror) else {
            continue;
        };
        let name = model.nets[sn].name.clone();
        if is_pair_net_static(cfg, design, &model.placed, &model.nets, sn) {
            let mate = model.nets[sn]
                .endpoints
                .iter()
                .find(|(opi, opad)| *opi != pi || *opad != pin.number);
            let mate_pos = mate.and_then(|(opi, opad)| {
                let op = &model.placed[*opi];
                design.comps[op.comp]
                    .geom
                    .pin_position(opad, op.at, op.rotation, op.mirror)
            });
            if let Some(mate_pos) = mate_pos {
                let axis_x = dir.1.abs() < 0.5;
                let p0 = if axis_x { pos.0 } else { pos.1 };
                let q0 = if axis_x { mate_pos.0 } else { mate_pos.1 };
                let aligned = if axis_x {
                    (pos.1 - mate_pos.1).abs() < EPS
                } else {
                    (pos.0 - mate_pos.0).abs() < EPS
                };
                let sgn = if axis_x { dir.0 } else { dir.1 };
                let mid = (p0 + q0) / 2.0;
                let far = if aligned {
                    q0
                } else if sgn > 0.0 {
                    (p0 + 2.54).max(mid - 7.62)
                } else {
                    (p0 - 2.54).min(mid + 7.62)
                };
                let bbox = if axis_x {
                    BBox {
                        x1: pos.0.min(far),
                        y1: pos.1 - 0.4,
                        x2: pos.0.max(far),
                        y2: pos.1 + 0.4,
                    }
                } else {
                    BBox {
                        x1: pos.0 - 0.4,
                        y1: pos.1.min(far),
                        x2: pos.0 + 0.4,
                        y2: pos.1.max(far),
                    }
                };
                out.push((bbox, name));
            }
            continue;
        }
        if dir.1.abs() > 0.5 {
            // Stub + longest retry of the label pass.
            let stub = cfg.stub_mm + 5.08;
            let elbow = (pos.0, pos.1 + dir.1 * stub);
            let mut zone = union_box(&point_box(pos, 1.524), &point_box(elbow, 1.524));
            // The wire starts at the pin END: the body-side band stays free.
            if dir.1 > 0.5 {
                zone.y1 = pos.1;
            } else {
                zone.y2 = pos.1;
            }
            out.push((zone, name));
            continue;
        }
        // Horizontal pin: wire band at pos.y + label text above the wire,
        // from the base length to the maximal retry (+12.7).
        let base = label_stub_len(cfg, &name);
        let w = label_text_width(&name);
        let end_min = pos.0 + dir.0 * base;
        let end_max = pos.0 + dir.0 * (base + 12.7);
        out.push((
            BBox {
                x1: pos.0.min(end_max),
                y1: pos.1 - 0.4,
                x2: pos.0.max(end_max),
                y2: pos.1 + 0.4,
            },
            name.clone(),
        ));
        out.push((
            BBox {
                x1: if dir.0 > 0.0 { end_min - w } else { end_max },
                y1: pos.1 - 2.2,
                x2: if dir.0 > 0.0 { end_max } else { end_min + w },
                y2: pos.1 + 0.4,
            },
            name,
        ));
    }
    out
}

/// Canonical text anchors of a placed instance: Reference at the TOP-LEFT
/// corner of the body (text above the outline, extending right), Value at
/// the bottom — right-justified when the symbol has downward pins (the
/// anti-collision slide pushes it below them), left-justified under the
/// body otherwise.
fn canonical_anchors(
    cfg: &SchConfig,
    design: &DesignModel,
    p: &PlacedComp,
) -> (Point, Point, bool) {
    let body = body_box(&design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
    let geom = &design.comps[p.comp].geom;
    let has_bottom = geom.pins.iter().filter(|pin| !pin.hidden).any(|pin| {
        geom.pin_outward(&pin.number, p.rotation, p.mirror)
            .map(|d| d.1 > 0.5)
            .unwrap_or(false)
    });
    let gap = cfg.value_gap_grid_steps as f64 * cfg.grid_mm;
    let ref_gap = cfg.ref_gap_grid_steps as f64 * cfg.grid_mm;
    let ref_at = (round4(body.x1), round4(body.y1 - ref_gap));
    if has_bottom {
        ((ref_at), (round4(body.x2), round4(body.y2 + gap)), true)
    } else {
        ((ref_at), (round4(body.x1), round4(body.y2 + gap)), false)
    }
}

/// Place the Reference/Value texts of every instance once the final
/// placement is known (called BEFORE wiring: the retained boxes become
/// keepouts for the wires/labels/doglegs placed next).
pub(crate) fn place_instance_texts(
    cfg: &SchConfig,
    design: &DesignModel,
    model: &mut SheetModel,
    warnings: &mut Vec<String>,
) -> TextArtifacts {
    let corridors = power_stub_corridors(cfg, design, &model.placed, &model.nets, &model.pin_net);
    // Signal stub zones of ALL instances: a text on a NEIGHBOUR's future
    // stub forces the wire through the text at wiring time.
    let signal_zones: Vec<(BBox, String, usize)> = (0..model.placed.len())
        .flat_map(|pi| {
            own_signal_stub_zones(cfg, design, model, pi)
                .into_iter()
                .map(move |(bbox, net)| (bbox, net, pi))
        })
        .collect();

    // Own zones of one instance: signal stubs + pin shafts/numbers — from
    // the body edge to the pin end (+0.4 mm). Symbols drawn with
    // `pin_numbers hide` reduce the corridor to the wire (+-0.4): a lateral
    // block can sit right above/below a middle pin. The number is rendered
    // on ONE side of the shaft (left of a vertical pin, above a horizontal
    // one — kicad-cli SVG probe): the halo is asymmetric.
    let zones_for = |pi: usize, body: &BBox| -> Vec<(BBox, Option<String>)> {
        let p = &model.placed[pi];
        let geom = &design.comps[p.comp].geom;
        let mut own: Vec<(BBox, Option<String>)> = signal_zones
            .iter()
            .filter(|(_, _, zpi)| *zpi == pi)
            .map(|(bbox, net, _)| (*bbox, Some(net.clone())))
            .collect();
        let across = if geom.pin_numbers_hidden { 0.4 } else { 1.6 };
        for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
            let Some(dir) = geom.pin_outward(&pin.number, p.rotation, p.mirror) else {
                continue;
            };
            let Some(pos) = geom.pin_position(&pin.number, p.at, p.rotation, p.mirror) else {
                continue;
            };
            let bbox = BBox {
                x1: if dir.0 > 0.5 {
                    body.x2
                } else if dir.0 < -0.5 {
                    pos.0 - 0.4
                } else {
                    pos.0 - across
                },
                y1: if dir.1 > 0.5 {
                    body.y2
                } else if dir.1 < -0.5 {
                    pos.1 - 0.4
                } else {
                    pos.1 - across
                },
                x2: if dir.0 < -0.5 { body.x1 } else { pos.0 + 0.4 },
                y2: if dir.1 < -0.5 { body.y1 } else { pos.1 + 0.4 },
            };
            let net = model
                .pin_net
                .get(&(pi, pin.number.clone()))
                .map(|&sn| model.nets[sn].name.clone());
            own.push((bbox, net));
        }
        own
    };

    let raw_boxes: Vec<BBox> = model
        .placed
        .iter()
        .map(|p| raw_box(&design.comps[p.comp].geom, p.at, p.rotation, p.mirror))
        .collect();

    // Reserve statically free canonical anchors: a fallback of an instance
    // placed EARLIER must not steal the free canonical spot of an instance
    // placed LATER. First come (placement order) wins; reservations never
    // overlap each other.
    let mut reserved: Vec<(BBox, usize)> = Vec::new();
    for pi in 0..model.placed.len() {
        let p = &model.placed[pi];
        let body = body_box(&design.comps[p.comp].geom, p.at, p.rotation, p.mirror);
        let own_zones = zones_for(pi, &body);
        let (ref_at, value_at, justify_right) = canonical_anchors(cfg, design, p);
        let refdes = &design.comps[p.comp].refdes;
        let value = &design.comps[p.comp].value;
        let ref_box = ref_text_box(refdes, ref_at, false);
        let val_box = value_text_box(value, value_at, justify_right);
        let static_hit = |bbox: &BBox| -> bool {
            for (oi, rb) in raw_boxes.iter().enumerate() {
                if oi != pi && overlaps(rb, bbox) {
                    return true;
                }
            }
            for c in &corridors {
                if overlaps(&c.bbox, bbox) {
                    return true;
                }
            }
            for (zb, _, zpi) in &signal_zones {
                if *zpi != pi && overlaps(zb, bbox) {
                    return true;
                }
            }
            for (zb, _) in &own_zones {
                if overlaps(zb, bbox) {
                    return true;
                }
            }
            for (rb, _) in &reserved {
                if overlaps(rb, bbox) {
                    return true;
                }
            }
            false
        };
        if !static_hit(&ref_box) && !static_hit(&val_box) {
            reserved.push((ref_box, pi));
            reserved.push((val_box, pi));
        }
    }

    let mut label_boxes: Vec<LabelBox> = Vec::new();
    // (ref anchor, ref justify-right, value anchor, value justify-right).
    let mut outcomes: Vec<(Point, bool, Point, bool)> = Vec::with_capacity(model.placed.len());
    for pi in 0..model.placed.len() {
        let p = &model.placed[pi];
        let comp = &design.comps[p.comp];
        let body = body_box(&comp.geom, p.at, p.rotation, p.mirror);
        let raw = raw_boxes[pi];
        let own_zones = zones_for(pi, &body);
        let refdes = comp.refdes.clone();
        let value = comp.value.clone();

        // Hard collision: foreign elements only.
        let hard_hit = |bbox: &BBox, label_boxes: &[LabelBox]| -> bool {
            for (oi, rb) in raw_boxes.iter().enumerate() {
                if oi != pi && overlaps(rb, bbox) {
                    return true;
                }
            }
            for lb in label_boxes {
                if overlaps(&lb.bbox, bbox) {
                    return true;
                }
            }
            for c in &corridors {
                if c.owner != pi && overlaps(&c.bbox, bbox) {
                    return true;
                }
            }
            for (zb, _, zpi) in &signal_zones {
                if *zpi != pi && overlaps(zb, bbox) {
                    return true;
                }
            }
            for (rb, rpi) in &reserved {
                if *rpi != pi && overlaps(rb, bbox) {
                    return true;
                }
            }
            false
        };
        let collides = |bbox: &BBox, label_boxes: &[LabelBox]| -> bool {
            if hard_hit(bbox, label_boxes) {
                return true;
            }
            for c in &corridors {
                if c.owner == pi && overlaps(&c.bbox, bbox) {
                    return true;
                }
            }
            for (zb, _) in &own_zones {
                if overlaps(zb, bbox) {
                    return true;
                }
            }
            false
        };
        // Soft collision of the last resort: own power corridors tolerated
        // (the box gets tagged, the stub runs under the text).
        let soft_collides = |bbox: &BBox, label_boxes: &[LabelBox]| -> bool {
            if hard_hit(bbox, label_boxes) {
                return true;
            }
            for (zb, _) in &own_zones {
                if overlaps(zb, bbox) {
                    return true;
                }
            }
            false
        };

        let slide = |pred: &dyn Fn(&BBox) -> bool,
                     base: Point,
                     dir_y: f64,
                     mk_box: &dyn Fn(Point) -> BBox,
                     k_max: i32|
         -> (Point, BBox, bool) {
            for k in 0..=k_max {
                let at = (base.0, round4(base.1 + dir_y * k as f64 * cfg.grid_mm));
                let bbox = mk_box(at);
                if !pred(&bbox) {
                    return (at, bbox, true);
                }
            }
            (base, mk_box(base), false)
        };

        let (plan_ref, plan_val, justify_right) = canonical_anchors(cfg, design, p);
        // Anti-drift: slide <= 1 step, PLUS what it takes to jump the
        // component's own pins — the anchor always stays within 2 steps of
        // the raw box (never a detached text).
        let g = cfg.grid_mm;
        let ref_k_max = 1.max(3.min(((plan_ref.1 - (raw.y1 - 2.0 * g)) / g + EPS).floor() as i32));
        let val_k_max = 1.max(3.min(((raw.y2 + 2.0 * g - plan_val.1) / g + EPS).floor() as i32));

        let refdes_box = |at: Point| ref_text_box(&refdes, at, false);
        let value_box = |at: Point| value_text_box(&value, at, justify_right);
        let coll = |bbox: &BBox| collides(bbox, &label_boxes);
        let (ref_at, ref_bbox, ref_ok) = slide(&coll, plan_ref, -1.0, &refdes_box, ref_k_max);
        let (val_at, val_bbox, val_ok) = slide(&coll, plan_val, 1.0, &value_box, val_k_max);

        let mut chosen = (
            ref_at,
            ref_bbox,
            false,
            val_at,
            val_bbox,
            justify_right,
            false,
        );
        if !ref_ok || !val_ok {
            // LATERAL FALLBACK: ref above value, centered on the body axis so a
            // middle-pin wire passes between the two lines. Try the RIGHT of the
            // body first (block LEFT-justified, its left edge stepped rightward
            // off the body), then the LEFT (block RIGHT-justified, its right
            // edge glued one grid step off the body then stepped leftward, never
            // into the left page margin), then a rescue spot glued to the right
            // edge. Left-justifying on the right and right-justifying on the
            // left keeps the block's INNER edge against the component on either
            // side, so a left-side block reads as symmetric with a right-side
            // one (the engineer's C2 tweak) instead of trailing a ragged right
            // edge far from the body.
            let cy = round4((body.y1 + body.y2) / 2.0);
            let max_w = label_text_width(&refdes).max(label_text_width(&value));
            // (anchor_x, justify_right). Right side: anchor is the block's LEFT
            // edge, stepped rightward, left-justified.
            let x0 = round4(((body.x2 + g) / g - 1e-6).ceil() * g);
            let mut cands: Vec<(f64, bool)> = (0..=8)
                .map(|k| (round4(x0 + k as f64 * g), false))
                .collect();
            // Left side: anchor is the block's RIGHT edge, one grid step off the
            // body then stepped leftward, right-justified. The widest line's
            // left edge must stay within the page margin.
            let x0r = round4(((body.x1 - g) / g + 1e-6).floor() * g);
            for k in 0..=8 {
                let xr = round4(x0r - k as f64 * g);
                if xr - max_w >= cfg.margin_left_mm {
                    cands.push((xr, true));
                }
            }
            // Rescue: glued to the right edge, left-justified.
            cands.push((round4((body.x2 / g - 1e-6).ceil() * g), false));
            let mut lateral = None;
            for (x, jr) in cands {
                let r_at = (x, round4(cy - 0.8));
                let v_at = (x, round4(cy + 0.8));
                let r_box = ref_text_box(&refdes, r_at, jr);
                let v_box = value_text_box(&value, v_at, jr);
                if !collides(&r_box, &label_boxes) && !collides(&v_box, &label_boxes) {
                    lateral = Some((r_at, r_box, jr, v_at, v_box, jr, false));
                    break;
                }
            }
            match lateral {
                Some(l) => chosen = l,
                None => {
                    // Last resort: least-bad canonical spot (soft slide),
                    // same anti-drift bound.
                    let soft = |bbox: &BBox| soft_collides(bbox, &label_boxes);
                    let (r_at, r_box, _) = slide(&soft, plan_ref, -1.0, &refdes_box, ref_k_max);
                    let (v_at, v_box, _) = slide(&soft, plan_val, 1.0, &value_box, val_k_max);
                    chosen = (r_at, r_box, false, v_at, v_box, justify_right, true);
                    // Warn on realistic overlap only: foreign bodies/texts/
                    // graphic footprints. The maximal foreign corridor is an
                    // upper bound — the wiring can shorten or jog around.
                    let warn_hit = |bbox: &BBox| -> bool {
                        for (oi, rb) in raw_boxes.iter().enumerate() {
                            if oi != pi && overlaps(rb, bbox) {
                                return true;
                            }
                        }
                        for lb in &label_boxes {
                            if overlaps(&lb.bbox, bbox) {
                                return true;
                            }
                        }
                        for c in &corridors {
                            if c.zone == CorridorZone::Graphic
                                && c.owner != pi
                                && overlaps(&c.bbox, bbox)
                            {
                                return true;
                            }
                        }
                        false
                    };
                    if warn_hit(&chosen.1) || warn_hit(&chosen.4) {
                        warnings.push(format!(
                            "texts of {refdes} kept at their canonical spot despite an overlap"
                        ));
                    }
                }
            }
        }

        let (ref_at, ref_bbox, ref_justify_right, val_at, val_bbox, justify_right, last_resort) =
            chosen;
        outcomes.push((ref_at, ref_justify_right, val_at, justify_right));

        // Last resort: the box lets its OWN stubs through (multi-net tag);
        // a cleanly placed text stays a hard obstacle for everyone.
        let tags_for = |bbox: &BBox| -> Vec<String> {
            if !last_resort {
                return Vec::new();
            }
            let mut nets: Vec<String> = Vec::new();
            for (zb, net) in &own_zones {
                if let Some(net) = net
                    && overlaps(zb, bbox)
                    && !nets.contains(net)
                {
                    nets.push(net.clone());
                }
            }
            for c in &corridors {
                if c.owner == pi && overlaps(&c.bbox, bbox) && !nets.contains(&c.net) {
                    nets.push(c.net.clone());
                }
            }
            nets
        };
        let mut ref_nets = tags_for(&ref_bbox);
        let mut val_nets = tags_for(&val_bbox);
        // A net tie's designator never obstructs routing (see SOFT_TEXT_PREFIX):
        // it is an inline bridge sitting on the wire it must not wall off.
        let untagged = if comp.is_net_tie {
            format!("{}{refdes}", crate::route::SOFT_TEXT_PREFIX)
        } else {
            format!("~text~{refdes}")
        };
        label_boxes.push(LabelBox {
            bbox: ref_bbox,
            net: if ref_nets.is_empty() {
                untagged.clone()
            } else {
                ref_nets.remove(0)
            },
            also: ref_nets,
        });
        label_boxes.push(LabelBox {
            bbox: val_bbox,
            net: if val_nets.is_empty() {
                untagged
            } else {
                val_nets.remove(0)
            },
            also: val_nets,
        });
    }

    for (pi, (ref_at, ref_justify_right, value_at, justify_right)) in
        outcomes.into_iter().enumerate()
    {
        let placed = &mut model.placed[pi];
        placed.ref_at = ref_at;
        placed.value_at = value_at;
        placed.value_justify_right = justify_right;
        placed.ref_justify_right = ref_justify_right;
    }

    TextArtifacts {
        label_boxes,
        corridors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_boxes_extend_from_their_anchor() {
        let r = ref_text_box("R1", (10.0, 20.0), false);
        assert!(r.x1 == 10.0 && r.x2 > 10.0);
        assert!(r.y1 < 20.0 && r.y2 > 20.0 - 1e-9);
        // Right-justified reference: anchor is the RIGHT edge, text extends left.
        let r_right = ref_text_box("R1", (10.0, 20.0), true);
        assert!(r_right.x2 == 10.0 && r_right.x1 < 10.0);
        assert!(r_right.y1 < 20.0 && r_right.y2 > 20.0 - 1e-9);
        let v_right = value_text_box("10k", (30.0, 40.0), true);
        assert!(v_right.x2 == 30.0 && v_right.x1 < 30.0);
        assert!(v_right.y2 > 40.0);
        let v_left = value_text_box("10k", (30.0, 40.0), false);
        assert!(v_left.x1 == 30.0 && v_left.x2 > 30.0);
    }

    #[test]
    fn left_side_lateral_block_is_right_justified() {
        // TWEAK 2: when the lateral text fallback drops a component's
        // Reference/Value block to the LEFT of its body, both texts must be
        // RIGHT-justified so their right edge glues to the component (symmetric
        // with a right-side block, which stays left-justified). We box a shunt
        // cap against the LEFT edge of a wide wall: a foreign body below the cap
        // makes its canonical text collide (lateral fires) and every right-side
        // lateral candidate lands inside the wall body, so the only free spot is
        // on the cap's left.
        use crate::place::place_sheet;
        use crate::sheets::plan_sheets;
        let sch = crate::testkit::cap_boxed_by_wall();
        let cfg = SchConfig::default();
        let design = DesignModel::build(&sch, &cfg).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(&sch, &design, &cfg, "t", &mut warnings);
        let mut model = place_sheet(&design, &plan, 0, &cfg, &mut warnings);

        let wall = model
            .placed
            .iter()
            .position(|p| design.comps[p.comp].refdes == "U1")
            .expect("wall placed");
        let cap = model
            .placed
            .iter()
            .position(|p| design.comps[p.comp].refdes == "C1")
            .expect("cap placed");
        let blocker = model
            .placed
            .iter()
            .position(|p| design.comps[p.comp].refdes == "R1")
            .expect("blocker placed");

        // Lay out the trio by hand (absolute positions), leaving ample room to
        // the cap's LEFT — well clear of the page margin so left candidates are
        // not filtered out. The cap sits at the wall's mid-height (between the
        // wall's two right-edge pins, and the wall projects no left-edge lane),
        // flush against the wall's left edge; a passive body sits just below the
        // cap to block its canonical text spot.
        let cx = 60.0;
        let cy = 100.0;
        model.placed[cap].at = (cx, cy);
        model.placed[cap].rotation = 0;
        model.placed[cap].mirror = None;
        // Wall to the right: its body left edge three grid units off the cap.
        model.placed[wall].at = (round4(cx + 3.0 + 10.16), cy);
        model.placed[wall].rotation = 0;
        model.placed[wall].mirror = None;
        // Foreign body just below the cap (a component's own corridor never
        // blocks its own text, so we need a neighbour).
        model.placed[blocker].at = (cx, round4(cy + 6.0));
        model.placed[blocker].rotation = 0;
        model.placed[blocker].mirror = None;

        place_instance_texts(&cfg, &design, &mut model, &mut warnings);

        let c = &model.placed[cap];
        // The block dropped to the LEFT of the body...
        assert!(
            c.ref_at.0 <= c.at.0 && c.value_at.0 <= c.at.0,
            "the lateral block must sit left of the cap body (ref_x={}, val_x={}, body_x={})",
            c.ref_at.0,
            c.value_at.0,
            c.at.0
        );
        // ...and therefore both texts are RIGHT-justified (right edge glued to
        // the body), the anchor being the block's right edge.
        assert!(
            c.ref_justify_right,
            "left-side lateral Reference must be right-justified"
        );
        assert!(
            c.value_justify_right,
            "left-side lateral Value must be right-justified"
        );
        // Right-justified boxes extend LEFT of their anchor, so the whole block
        // sits left of its right edge, which hugs the body.
        let refbox = ref_text_box(&design.comps[c.comp].refdes, c.ref_at, c.ref_justify_right);
        assert!(refbox.x2 <= c.at.0 + EPS && refbox.x1 < refbox.x2);
    }

    #[test]
    fn power_graphic_zone_is_narrow_at_the_shaft() {
        let [shaft, text] = power_graphic_zone_boxes("GND", (0.0, 0.0), true);
        assert!(shaft.x2 - shaft.x1 <= 3.3);
        assert!(text.x2 - text.x1 > shaft.x2 - shaft.x1);
        // The shaft starts at the connection point.
        assert_eq!(shaft.y1, 0.0);
        assert!(text.y1 >= shaft.y2 - 1e-9);
        // Rail (up): mirrored above the connection point.
        let [shaft_up, text_up] = power_graphic_zone_boxes("VCC", (0.0, 0.0), false);
        assert_eq!(shaft_up.y2, 0.0);
        assert!(text_up.y2 <= shaft_up.y1 + 1e-9);
    }
}
