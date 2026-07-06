//! Final, group-aware orientation — decided AFTER placement (positions
//! frozen: the placement pre-orientation worked in relative coordinates
//! and collision resolution may have moved the mates since) and BEFORE
//! texts/wiring (it conditions the routes of the real-wire trees).
//!
//! Permissions per component (user rule):
//! * horizontal flip (mirror `y`): EVERYONE — except connectors under a
//!   board-edge constraint (`edge=` attribute), whose pins must keep
//!   facing the sheet interior;
//! * rotation 90/180/270: drawn two-pin parts only — never the boxes.
//!
//! Score, decreasing priority (pins deduplicated by position):
//! * (a) a SIGNAL pin member of a predicted wired group turned TOWARD its
//!   tree target: +8 when a mate is ROUTABLE (static probe over the tree
//!   wire candidates) facing/perpendicular, +6 when only a hook routes
//!   (both pins exiting the same side); when NO mate routes the pin falls
//!   back to rule (b) — no directional bonus toward an unreachable mate;
//! * (b) +3/-1 — facing the mate of a 2-pin net (center of the mate);
//! * (c) +-2 — power: rail UP, ground DOWN.
//!
//! (a) routable dominates (b)+(c) combined; at equal score the identity
//! wins (d) — candidates are enumerated identity first and replaced only
//! on a strictly higher score (determinism and least surprise). An
//! anti-collision guard rejects a 90/270 rotation that would create a NEW
//! solid-box overlap. Two bounded passes: the targets (mate pin positions)
//! settle once the partners are reoriented.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use pcb_sch::position::MirrorAxis;

use crate::config::SchConfig;
use crate::model::{DesignModel, NetClass, Role};
use crate::place::{PlacedComp, SheetNet, is_rotatable, overlaps, solid_box_for};
use crate::texts::{CorridorZone, power_stub_corridors};
use crate::wiring::{group_wire_subsets, is_hub_comp, probe_routable};

#[cfg(test)]
type Point = (f64, f64);

/// May this component mirror horizontally? Everyone, except connectors
/// under a board-edge constraint (their pins point into the sheet by
/// contract — a mirror would flip the connectivity drawing).
fn is_mirrorable(design: &DesignModel, p: &PlacedComp) -> bool {
    !(p.role == Role::Connector && design.comps[p.comp].edge_attr.is_some())
}

/// Endpoint key of the tree-target map.
type PinKey = (usize, String);

/// Predicted tree partners per endpoint "placed.pad": the OTHER members of
/// the subset predicted to wire with real wires (same prediction as the
/// actual wiring pass).
fn build_tree_targets(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &[PlacedComp],
    nets: &[SheetNet],
) -> BTreeMap<PinKey, Vec<PinKey>> {
    let mut targets: BTreeMap<PinKey, Vec<PinKey>> = BTreeMap::new();
    for net in nets {
        if net.class != NetClass::Signal {
            continue;
        }
        // Endpoints deduplicated by position (stacked pins count once).
        let mut eps: Vec<(usize, String)> = Vec::new();
        let mut seen: BTreeSet<(i64, i64)> = BTreeSet::new();
        for (pi, pad) in &net.endpoints {
            let p = &placed[*pi];
            let geom = &design.comps[p.comp].geom;
            let Some(pin) = geom.pin(pad) else { continue };
            if pin.hidden {
                continue;
            }
            let Some(pos) = geom.pin_position(pad, p.at, p.rotation, p.mirror) else {
                continue;
            };
            if !seen.insert((
                (pos.0 * 10000.0).round() as i64,
                (pos.1 * 10000.0).round() as i64,
            )) {
                continue;
            }
            eps.push((*pi, pad.clone()));
        }
        let eps_placed: Vec<usize> = eps.iter().map(|(pi, _)| *pi).collect();
        let subsets = group_wire_subsets(
            &eps_placed,
            &|pi| is_hub_comp(cfg, design, placed[pi].comp),
            &|pi| placed[pi].group_root,
            &|pi| design.comps[placed[pi].comp].refdes.clone(),
        );
        for subset in subsets {
            for &i in &subset {
                let partners: Vec<PinKey> = subset
                    .iter()
                    .filter(|&&o| o != i && eps[o].0 != eps[i].0)
                    .map(|&o| eps[o].clone())
                    .collect();
                if !partners.is_empty() {
                    targets.insert(eps[i].clone(), partners);
                }
            }
        }
    }
    targets
}

/// Orientation score of one instance for a candidate transform.
#[allow(clippy::too_many_arguments)]
fn orientation_score(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &[PlacedComp],
    nets: &[SheetNet],
    pin_net: &HashMap<(usize, String), usize>,
    pi: usize,
    rotation: i32,
    mirror: Option<MirrorAxis>,
    targets: &BTreeMap<PinKey, Vec<PinKey>>,
    corridors: &[(crate::geometry::BBox, String)],
) -> f64 {
    let p = &placed[pi];
    let geom = &design.comps[p.comp].geom;
    let mut score = 0.0;
    let mut seen: BTreeSet<(i64, i64)> = BTreeSet::new();
    for pin in geom.pins.iter().filter(|pin| !pin.hidden) {
        let Some(pos) = geom.pin_position(&pin.number, p.at, rotation, mirror) else {
            continue;
        };
        if !seen.insert((
            (pos.0 * 10000.0).round() as i64,
            (pos.1 * 10000.0).round() as i64,
        )) {
            continue;
        }
        let Some(dir) = geom.pin_outward(&pin.number, rotation, mirror) else {
            continue;
        };
        let Some(&sn) = pin_net.get(&(pi, pin.number.clone())) else {
            continue;
        };
        let net = &nets[sn];
        match net.class {
            NetClass::Ground => {
                score += if dir.1 > 0.5 {
                    2.0
                } else if dir.1 < -0.5 {
                    -2.0
                } else {
                    0.0
                };
                continue;
            }
            NetClass::Power => {
                score += if dir.1 < -0.5 {
                    2.0
                } else if dir.1 > 0.5 {
                    -2.0
                } else {
                    0.0
                };
                continue;
            }
            NetClass::Signal => {}
        }
        if let Some(partners) = targets.get(&(pi, pin.number.clone())) {
            let a_net = Some(net.name.as_str());
            let mut best = 0.0f64;
            for (opi, opad) in partners {
                let op = &placed[*opi];
                let ogeom = &design.comps[op.comp].geom;
                let Some(o_pos) = ogeom.pin_position(opad, op.at, op.rotation, op.mirror) else {
                    continue;
                };
                let Some(o_dir) = ogeom.pin_outward(opad, op.rotation, op.mirror) else {
                    continue;
                };
                let same_dir = (o_dir.0 - dir.0).abs() < 1e-6 && (o_dir.1 - dir.1).abs() < 1e-6;
                let value = if same_dir { 6.0 } else { 8.0 };
                if value <= best {
                    continue;
                }
                if probe_routable(
                    cfg,
                    design,
                    placed,
                    (pos, dir),
                    (o_pos, o_dir),
                    a_net,
                    corridors,
                ) {
                    best = value;
                }
            }
            if best > 0.0 {
                score += best;
                continue;
            }
            // No routable mate: fall back to rule (b) below.
        }
        if net.endpoints.len() == 2 && net.port.is_none() {
            let mate = net
                .endpoints
                .iter()
                .find(|(opi, opad)| *opi != pi || *opad != pin.number);
            if let Some((opi, _)) = mate
                && *opi != pi
            {
                let (mx, my) = placed[*opi].at;
                let vx = mx - p.at.0;
                let vy = my - p.at.1;
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
    score
}

/// Run the orientation engine over one placed sheet (positions frozen,
/// orientation only — including anchored satellites with side/offset).
pub(crate) fn orient_for_wiring(
    cfg: &SchConfig,
    design: &DesignModel,
    placed: &mut [PlacedComp],
    nets: &[SheetNet],
    pin_net: &HashMap<(usize, String), usize>,
) {
    let targets = build_tree_targets(cfg, design, placed, nets);
    for _pass in 0..2 {
        let mut changed = false;
        for pi in 0..placed.len() {
            if placed[pi].pinned {
                continue;
            }
            let rotatable = is_rotatable(placed[pi].role);
            let mirrorable = is_mirrorable(design, &placed[pi]);
            let cur_rot = placed[pi].rotation;
            let cur_mirror = placed[pi].mirror;
            let rotations: Vec<i32> = if rotatable {
                let mut rots = vec![cur_rot];
                rots.extend([0, 90, 180, 270].iter().filter(|&&r| r != cur_rot));
                rots
            } else {
                vec![cur_rot]
            };
            // Current state first (identity, rule (d)), then the flip.
            let mirrors: Vec<Option<MirrorAxis>> = if mirrorable {
                match cur_mirror {
                    Some(MirrorAxis::Y) => vec![Some(MirrorAxis::Y), None],
                    other => vec![other, Some(MirrorAxis::Y)],
                }
            } else {
                vec![cur_mirror]
            };
            if rotations.len() == 1 && mirrors.len() == 1 {
                continue;
            }
            // Predicted power corridors of the OTHER instances, with their
            // CURRENT orientations (recomputed per instance: an applied
            // transform moves its owner's power stubs).
            let corridors: Vec<(crate::geometry::BBox, String)> =
                power_stub_corridors(cfg, design, placed, nets, pin_net)
                    .into_iter()
                    .filter(|c| c.zone == CorridorZone::Corridor && c.owner != pi)
                    .map(|c| (c.bbox, c.net))
                    .collect();
            let old_box = placed[pi].bbox;
            let mut best: (i32, Option<MirrorAxis>) = (cur_rot, cur_mirror);
            let mut best_score = f64::NEG_INFINITY;
            let mut first = true;
            for &mirror in &mirrors {
                for &rotation in &rotations {
                    let score = orientation_score(
                        cfg, design, placed, nets, pin_net, pi, rotation, mirror, &targets,
                        &corridors,
                    );
                    if first {
                        // Identity: baseline of rule (d).
                        best = (rotation, mirror);
                        best_score = score;
                        first = false;
                        continue;
                    }
                    if score <= best_score {
                        continue;
                    }
                    // Anti-collision guard: no NEW solid-box overlap.
                    let nb = solid_box_for(cfg, design, &placed[pi], rotation, mirror);
                    let clash = placed.iter().enumerate().any(|(oi, other)| {
                        oi != pi && overlaps(&nb, &other.bbox) && !overlaps(&old_box, &other.bbox)
                    });
                    if clash {
                        continue;
                    }
                    best = (rotation, mirror);
                    best_score = score;
                }
            }
            if best.0 != cur_rot || best.1 != cur_mirror {
                placed[pi].rotation = best.0;
                placed[pi].mirror = best.1;
                placed[pi].bbox = solid_box_for(cfg, design, &placed[pi], best.0, best.1);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

/// Position/direction pair used by tests.
#[cfg(test)]
pub(crate) fn pin_pos_dir(
    design: &DesignModel,
    placed: &[PlacedComp],
    pi: usize,
    pad: &str,
) -> Option<(Point, Point)> {
    let p = &placed[pi];
    let geom = &design.comps[p.comp].geom;
    let pos = geom.pin_position(pad, p.at, p.rotation, p.mirror)?;
    let dir = geom.pin_outward(pad, p.rotation, p.mirror)?;
    Some((pos, dir))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::place::{PlacedComp, place_sheet};
    use crate::sheets::plan_sheets;
    use crate::testkit::{divider, hierarchical_design};

    /// Build the root sheet's placed/oriented model of a design.
    fn root_model(sch: &pcb_sch::Schematic) -> (DesignModel, crate::place::SheetModel) {
        let cfg = SchConfig::default();
        let design = DesignModel::build(sch, &cfg).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(sch, &design, &cfg, "t", &mut warnings);
        let model = place_sheet(&design, &plan, 0, &cfg, &mut warnings);
        (design, model)
    }

    fn find(design: &DesignModel, placed: &[PlacedComp], refdes: &str) -> usize {
        placed
            .iter()
            .position(|p| design.comps[p.comp].refdes == refdes)
            .unwrap_or_else(|| panic!("{refdes} not placed"))
    }

    /// Rule (a): orientation turns the shared MID pins toward a ROUTABLE
    /// mate (facing or same-side hook) so the pair wires with a real wire —
    /// verified with the same static probe the engine scores with.
    #[test]
    fn divider_orients_mid_pins_routable() {
        use crate::wiring::probe_routable;
        let sch = divider();
        let (design, model) = root_model(&sch);
        let r1 = find(&design, &model.placed, "R1");
        let r2 = find(&design, &model.placed, "R2");
        let a = pin_pos_dir(&design, &model.placed, r1, "2").unwrap();
        let b = pin_pos_dir(&design, &model.placed, r2, "1").unwrap();
        let cfg = SchConfig::default();
        assert!(
            probe_routable(&cfg, &design, &model.placed, a, b, Some("MID"), &[]),
            "MID pins must be oriented into a routable configuration: {a:?} {b:?}"
        );
    }

    /// Rule (c): with no competing routable signal mate, a two-pin part
    /// across a rail and a ground orients its rail pin UP and its ground pin
    /// DOWN (decoupling cap standing upright).
    #[test]
    fn decoupling_cap_orients_rail_up_ground_down() {
        use crate::testkit::{R_SMALL, add_component, port_ref};
        use pcb_sch::{Instance, InstanceRef, ModuleRef, Net, Schematic};
        use std::path::Path;

        let module = ModuleRef::from_path(Path::new("/test.zen"), "<root>");
        let mut sch = Schematic::new();
        let root = InstanceRef::new(module.clone(), vec![]);
        let mut root_inst = Instance::module(module.clone());
        let c1 = add_component(
            &mut sch,
            &["C1"],
            R_SMALL,
            &[("1", "1"), ("2", "2")],
            "100nF",
            Some("capacitor"),
        );
        root_inst.add_child("C1".to_string(), c1);
        sch.add_instance(root.clone(), root_inst);
        sch.set_root_ref(root);
        sch.add_net(Net::new("Power".to_string(), "VCC", 1).with_port(port_ref(&["C1"], "1")));
        sch.add_net(Net::new("Ground".to_string(), "GND", 2).with_port(port_ref(&["C1"], "2")));
        sch.assign_reference_designators();

        let (design, model) = root_model(&sch);
        let c = find(&design, &model.placed, "C1");
        let (_, d_vcc) = pin_pos_dir(&design, &model.placed, c, "1").unwrap();
        let (_, d_gnd) = pin_pos_dir(&design, &model.placed, c, "2").unwrap();
        assert!(d_vcc.1 < -0.5, "rail pin should point up, got {d_vcc:?}");
        assert!(d_gnd.1 > 0.5, "ground pin should point down, got {d_gnd:?}");
    }

    /// Boxes (ICs) never rotate — the orientation engine may only mirror
    /// them. `U1` (a 6-pin box) keeps rotation 0.
    #[test]
    fn box_symbol_is_never_rotated() {
        let sch = hierarchical_design();
        let cfg = SchConfig::default();
        let design = DesignModel::build(&sch, &cfg).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(&sch, &design, &cfg, "t", &mut warnings);
        // `big` is its own sheet (sheet index 1 in pre-order).
        let big = plan
            .sheets
            .iter()
            .position(|s| s.title.contains("big"))
            .expect("big sheet");
        let model = place_sheet(&design, &plan, big, &cfg, &mut warnings);
        let u1 = find(&design, &model.placed, "U1");
        assert_eq!(model.placed[u1].rotation, 0, "a box must not rotate");
    }

    /// Orientation is deterministic: the same design orients identically
    /// twice (rule (d) — identity kept at equal score).
    #[test]
    fn orientation_is_deterministic() {
        let sch = divider();
        let (_, a) = root_model(&sch);
        let (_, b) = root_model(&sch);
        for (pa, pb) in a.placed.iter().zip(&b.placed) {
            assert_eq!((pa.rotation, pa.mirror), (pb.rotation, pb.mirror));
        }
    }
}
