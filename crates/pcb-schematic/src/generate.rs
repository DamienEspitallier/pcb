//! Top-level generator: evaluated design in, hierarchical `.kicad_sch`
//! files out.
//!
//! Pipeline: [`crate::model`] extracts components/nets/roles/manual
//! positions, [`crate::sheets`] plans the sheet hierarchy, [`crate::place`]
//! lays out every sheet, [`crate::route`] draws stubs/labels/power symbols,
//! and this module emits each sheet through [`crate::writer::SheetWriter`]
//! (parents first, so hierarchical instance paths flow down).

use std::collections::BTreeSet;

use anyhow::Result;
use pcb_sch::Schematic;

use crate::config::{Paper, SchConfig};
use crate::model::{DesignModel, NetClass};
use crate::place::{SheetModel, place_sheet};
use crate::route::{RoutedSheet, route_sheet};
use crate::sheets::{SheetPlan, plan_sheets};
use crate::writer::{PlaceSymbol, SheetRef, SheetRefPin, SheetWriter, SheetWriterOptions};

/// Options for one generation run.
#[derive(Debug, Clone)]
pub struct SchOptions {
    /// Base name of the generated project (root file is
    /// `<project_name>.kicad_sch`); also the KiCad project name recorded in
    /// symbol instance blocks.
    pub project_name: String,
    /// Generator id written in the file header.
    pub generator: String,
    pub generator_version: String,
    pub config: SchConfig,
}

impl SchOptions {
    pub fn new(project_name: impl Into<String>) -> Self {
        Self {
            project_name: project_name.into(),
            generator: "pcb".to_string(),
            generator_version: env!("CARGO_PKG_VERSION").to_string(),
            config: SchConfig::default(),
        }
    }
}

/// One generated file (root sheet or sub-sheet).
#[derive(Debug, Clone)]
pub struct SchFile {
    pub file_name: String,
    pub content: String,
}

/// Result of a generation run.
#[derive(Debug, Clone)]
pub struct GeneratedSchematic {
    /// All sheet files, root first (plan pre-order).
    pub files: Vec<SchFile>,
    pub root_file_name: String,
    pub warnings: Vec<String>,
}

/// Generate the hierarchical KiCad schematic of an evaluated design.
///
/// Components must already carry reference designators (call
/// [`Schematic::assign_reference_designators`] first when building the IR
/// by hand — the evaluator does it automatically).
pub fn generate_schematic(sch: &Schematic, opts: &SchOptions) -> Result<GeneratedSchematic> {
    let cfg = &opts.config;
    let design = DesignModel::build(sch, cfg)?;
    let mut warnings = design.warnings.clone();

    let plan = plan_sheets(sch, &design, cfg, &opts.project_name, &mut warnings);

    // Assign each undriven power rail's PWR_FLAG to the first sheet (plan
    // order) that actually wires the net; a rail with no endpoint anywhere
    // needs no flag (no power symbol is ever placed).
    let mut flag_nets: Vec<BTreeSet<String>> = vec![BTreeSet::new(); plan.sheets.len()];
    for net in &design.nets {
        if net.class == NetClass::Signal || net.driven || net.endpoints.is_empty() {
            continue;
        }
        let si = net
            .endpoints
            .iter()
            .map(|(ci, _)| plan.comp_sheet[*ci])
            .min()
            .expect("non-empty endpoints");
        flag_nets[si].insert(net.name.clone());
    }

    // Place and route every sheet (routing also finalizes the text
    // anchors recorded in the model).
    let mut models: Vec<SheetModel> = Vec::with_capacity(plan.sheets.len());
    let mut routed: Vec<RoutedSheet> = Vec::with_capacity(plan.sheets.len());
    for (si, sheet_flags) in flag_nets.iter().enumerate() {
        let mut model = place_sheet(&design, &plan, si, cfg, &mut warnings);
        let r = route_sheet(&design, &plan, &mut model, cfg, sheet_flags, &mut warnings);
        models.push(model);
        routed.push(r);
    }

    // Emit (plan order = pre-order: parents before children).
    let mut prefixes: Vec<Option<String>> = vec![None; plan.sheets.len()];
    let mut files: Vec<SchFile> = Vec::with_capacity(plan.sheets.len());
    for si in 0..plan.sheets.len() {
        let (content, child_prefixes) = emit_sheet(
            &design,
            &plan,
            &models[si],
            &routed[si],
            si,
            prefixes[si].clone(),
            opts,
        )?;
        for (child, prefix) in child_prefixes {
            prefixes[child] = Some(prefix);
        }
        files.push(SchFile {
            file_name: plan.sheets[si].file_name.clone(),
            content,
        });
    }

    Ok(GeneratedSchematic {
        root_file_name: plan.sheets[0].file_name.clone(),
        files,
        warnings,
    })
}

/// Pick the smallest paper that fits the routed extents.
fn pick_paper(cfg: &SchConfig, extents: &crate::geometry::BBox) -> Paper {
    match cfg.paper {
        Paper::Auto => {
            for (paper, w, h) in Paper::SIZES {
                if extents.x2 <= w - 12.7 && extents.y2 <= h - 19.05 {
                    return *paper;
                }
            }
            Paper::SIZES.last().map(|(p, _, _)| *p).unwrap_or(Paper::A4)
        }
        fixed => fixed,
    }
}

/// Emit one sheet document. Returns the serialized content and the
/// hierarchical instance path prefixes of the child sheets.
fn emit_sheet(
    design: &DesignModel,
    plan: &SheetPlan,
    model: &SheetModel,
    routed: &RoutedSheet,
    si: usize,
    prefix: Option<String>,
    opts: &SchOptions,
) -> Result<(String, Vec<(usize, String)>)> {
    let sheet = &plan.sheets[si];
    let is_root = sheet.parent.is_none();
    let paper = pick_paper(&opts.config, &routed.extents);

    let mut writer = SheetWriter::new(SheetWriterOptions {
        title: sheet.title.clone(),
        file_name: sheet.file_name.clone(),
        paper: paper.kicad_name().to_string(),
        generator: opts.generator.clone(),
        generator_version: opts.generator_version.clone(),
        project_name: opts.project_name.clone(),
        is_root,
        instance_path_prefix: prefix,
        power_ref_offset: (si as u32) * 1000,
    });

    // Symbols.
    for placed in &model.placed {
        let comp = &design.comps[placed.comp];
        writer.add_lib_symbol(&comp.geom);
        writer.place_symbol(PlaceSymbol {
            rotation: placed.rotation,
            mirror: placed.mirror,
            dnp: comp.dnp,
            footprint: comp.footprint.as_deref(),
            datasheet: comp.datasheet.as_deref(),
            description: comp.description.as_deref(),
            mpn: comp.mpn.as_deref(),
            uuid_key: &comp.path_key,
            ref_at: Some(placed.ref_at),
            value_at: Some(placed.value_at),
            // Reference: anchored bottom-left (text above the anchor);
            // Value: anchored top (text below), left or right justified.
            ref_justify: &["left", "bottom"],
            value_justify: if placed.value_justify_right {
                &["right", "top"]
            } else {
                &["left", "top"]
            },
            ..PlaceSymbol::new(&comp.geom.lib_id, &comp.refdes, &comp.value, placed.at)
        })?;
    }

    // Wires and markers.
    for wire in &routed.wires {
        writer.add_wire(wire);
    }
    for at in &routed.junctions {
        writer.add_junction(*at);
    }
    for (name, at, rotation) in &routed.net_labels {
        writer.add_net_label(name, *at, *rotation);
    }
    for (name, at, rotation, direction) in &routed.global_labels {
        writer.add_global_label(name, *at, *rotation, *direction);
    }
    for (name, direction, at, rotation) in &routed.hier_labels {
        writer.add_hier_label(name, *direction, *at, *rotation);
    }
    for (name, at, down) in &routed.power_symbols {
        writer.add_power_symbol(name, *at, *down);
    }
    for at in &routed.pwr_flags {
        writer.add_pwr_flag(*at);
    }
    for at in &routed.no_connects {
        writer.add_no_connect(*at);
    }

    // Graphic zone outlines (functional / decoupling / ERC): purely visual,
    // no connectivity, drawn as a backdrop behind the symbols.
    for zone in &routed.zones {
        writer.add_zone_rect((zone.bbox.x1, zone.bbox.y1), (zone.bbox.x2, zone.bbox.y2));
        if let Some(title) = &zone.title {
            writer.add_zone_title(title, (zone.bbox.x1 + 0.5, zone.bbox.y1 - 0.5));
        }
    }

    // Child sheet blocks; their uuids extend the instance path.
    let own_prefix = writer.path_prefix_public();
    let mut child_prefixes = Vec::new();
    for block in &routed.blocks {
        let child = &plan.sheets[block.sheet];
        let uuid = writer.add_sheet_ref(&SheetRef {
            file_name: child.file_name.clone(),
            title: child.title.clone(),
            at: block.at,
            size: block.size,
            pins: block
                .pins
                .iter()
                .map(|(name, direction, at)| SheetRefPin {
                    name: name.clone(),
                    direction: *direction,
                    at: *at,
                })
                .collect(),
            page: child.page.clone(),
        });
        child_prefixes.push((block.sheet, format!("{own_prefix}/{uuid}")));
    }

    Ok((writer.serialize(), child_prefixes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ATTR_SYMBOL_VALUE;
    use crate::testkit::{
        analog_filter, decoupled_adc, diff_filter, digital_bus, divider, hierarchical_design,
    };

    /// X coordinate of a placed component symbol, found by its refdes. Reads the
    /// symbol's own `(at X Y R)` (the first `(at` after its `(lib_id`).
    fn symbol_x(content: &str, reference: &str) -> Option<f64> {
        let refm = format!("(property \"Reference\" \"{reference}\"");
        for block in content.split("\t(symbol\n").skip(1) {
            if !block.contains(&refm) {
                continue;
            }
            let at = block.lines().find(|l| l.trim_start().starts_with("(at "))?;
            return at
                .trim()
                .trim_start_matches("(at ")
                .split_whitespace()
                .next()
                .and_then(|s| s.parse::<f64>().ok());
        }
        None
    }

    #[test]
    fn analog_net_takes_the_continuous_wiring_path() {
        // The RC-filter node FILT is analog and routed by the continuous-wire
        // engine. With RF grouped next to U1 by the input-chain proximity rule,
        // the whole node now routes as one continuous wire named by a single
        // label — proving the ANALOG path handled it, never the digital
        // group/label path, and with no reluctant fallback.
        let out = generate_schematic(&analog_filter(), &SchOptions::new("flt")).unwrap();
        let c = &out.files[0].content;
        assert!(c.contains("(wire"), "FILT must be routed with real wires");
        assert!(
            c.matches("(label \"FILT\"").count() <= 1,
            "analog FILT is a continuous wire named by at most one label, not the digital label path"
        );
        assert!(
            !out.warnings.iter().any(|w| w.contains("analog net FILT")),
            "FILT routes cleanly now; unexpected fallback in {:?}",
            out.warnings
        );
    }

    #[test]
    fn differential_filter_both_legs_stay_continuous_wires() {
        // Two differential filter legs (AINP_FILT, AINN_FILT), each a series R
        // and a shunt C into the IC. Their shunt drops interleave, so wiring one
        // leg continuously forces the other to frank-cross it. The router now
        // accepts that crossing (electrically harmless — no junction, no
        // connection — merely cost-penalised) so BOTH legs stay continuous wires
        // instead of one collapsing into labels.
        let out = generate_schematic(&diff_filter(), &SchOptions::new("diff")).unwrap();
        let c = &out.files[0].content;
        for leg in ["AINP_FILT", "AINN_FILT"] {
            assert!(
                !out.warnings
                    .iter()
                    .any(|w| w.contains(&format!("analog net {leg}"))),
                "{leg} must stay a continuous wire (no label fallback); warnings were {:?}",
                out.warnings
            );
        }
        // Each leg is a continuous wire named by at most one annotation label,
        // never the multi-stub label fallback a broken net produces.
        for leg in ["AINP_FILT", "AINN_FILT"] {
            let n = c.matches(&format!("(label \"{leg}\"")).count();
            assert!(
                n <= 1,
                "{leg} must be one continuous wire (<=1 label), found {n} labels"
            );
        }
        assert!(c.contains("(wire"), "the legs are routed with real wires");
    }

    #[test]
    fn digital_net_takes_the_label_path() {
        // BUS is digital (IC-IC + pull): it never enters the analog
        // continuous-wire engine, so no analog fallback is ever logged for it.
        let out = generate_schematic(&digital_bus(), &SchOptions::new("bus")).unwrap();
        assert!(
            !out.warnings.iter().any(|w| w.contains("analog net BUS")),
            "digital BUS must not be routed as analog; warnings were {:?}",
            out.warnings
        );
        assert!(out.files[0].content.contains("\"U1\"") && out.files[0].content.contains("\"U2\""));
    }

    #[test]
    fn analog_two_pin_net_is_a_real_wire() {
        // A 2-pin analog signal net (the divider MID, classified analog) is a
        // continuous wire, never a label.
        let out = generate_schematic(&divider(), &SchOptions::new("d")).unwrap();
        assert_eq!(out.files[0].content.matches("(label \"MID\"").count(), 0);
        assert!(out.files[0].content.contains("(wire"));
    }

    #[test]
    fn flat_generation_is_deterministic_and_complete() {
        let sch = divider();
        let opts = SchOptions::new("divider");
        let a = generate_schematic(&sch, &opts).unwrap();
        let b = generate_schematic(&sch, &opts).unwrap();
        assert_eq!(a.files[0].content, b.files[0].content);
        assert_eq!(a.root_file_name, "divider.kicad_sch");
        assert_eq!(a.files.len(), 1);

        let text = &a.files[0].content;
        assert!(text.contains("\"R1\""));
        assert!(text.contains("\"R2\""));
        // MID is a facing 2-pin net: wired with a REAL wire, no labels.
        assert_eq!(text.matches("(label \"MID\"").count(), 0);
        assert!(text.contains("(wire"));
        assert!(text.contains("\"pcb_power:VCC\""));
        assert!(text.contains("\"pcb_power:GND\""));
        // Undriven rails get exactly one PWR_FLAG each.
        assert_eq!(text.matches("\"#FLG").count() / 2, 2, "one flag per rail");
    }

    #[test]
    fn shared_power_symbol_for_same_net_pins_of_one_ic() {
        // U1's left edge carries two VDD pins split by a signal pin: they are
        // joined by a short bus under ONE shared VDD symbol, and its two
        // adjacent GND pins collapse under ONE shared GND symbol. Relegation is
        // disabled here so the count isolates the banking behaviour (a
        // relegated sheet also emits a homonym rail symbol beside each flag).
        let mut opts = SchOptions::new("bank");
        opts.config.relegate_utility = false;
        let out = generate_schematic(&crate::testkit::power_bank(), &opts).unwrap();
        let c = &out.files[0].content;
        assert_eq!(
            c.matches("(lib_id \"pcb_power:VDD\")").count(),
            1,
            "two VDD pins of one IC must share a single symbol"
        );
        assert_eq!(
            c.matches("(lib_id \"pcb_power:GND\")").count(),
            1,
            "two adjacent GND pins of one IC must share a single symbol"
        );
    }

    #[test]
    fn relegation_moves_decoupling_and_flags_out_of_the_flow() {
        // A relegated sheet (real IC + default config) pulls its rail-to-rail
        // decoupling cap and its undriven-rail PWR_FLAGs into the right-hand
        // utility band and outlines the functional / decoupling / ERC zones —
        // all without changing which power nets exist.
        let out = generate_schematic(&decoupled_adc(), &SchOptions::new("dec")).unwrap();
        let c = &out.files[0].content;
        // Three graphic zone rectangles (functional / decoupling / ERC).
        assert_eq!(
            c.matches("(type dash)").count(),
            3,
            "three dashed zone outlines must be drawn"
        );
        // Zone titles are emitted.
        for title in ["Functional", "Decoupling", "ERC"] {
            assert!(
                c.contains(&format!("(text \"{title}\"")),
                "missing zone title {title}"
            );
        }
        // The decoupling cap C1 is relegated to the right of the IC U1.
        let cx = symbol_x(c, "C1").expect("C1 placed");
        let ux = symbol_x(c, "U1").expect("U1 placed");
        assert!(
            cx > ux + 10.0,
            "decoupling C1 (x={cx}) must sit well right of U1 (x={ux})"
        );
    }

    #[test]
    fn relegation_preserves_the_netlist_and_can_be_disabled() {
        // Relegation is a pure relocation: with it on or off the same power
        // symbols exist for the same rails (VDD driven, GND driven), so the
        // exported connectivity is identical. Toggling the knob only moves the
        // flag/cap glyphs.
        let mut on = SchOptions::new("dec");
        on.config.relegate_utility = true;
        let mut off = SchOptions::new("dec");
        off.config.relegate_utility = false;
        let a = generate_schematic(&decoupled_adc(), &on).unwrap();
        let b = generate_schematic(&decoupled_adc(), &off).unwrap();
        // Zones appear only when relegating.
        assert_eq!(a.files[0].content.matches("(type dash)").count(), 3);
        assert_eq!(b.files[0].content.matches("(type dash)").count(), 0);
        // Both keep exactly one PWR_FLAG per undriven rail (VDD + GND).
        for out in [&a, &b] {
            assert_eq!(
                out.files[0].content.matches("\"#FLG").count() / 2,
                2,
                "one flag per undriven rail regardless of relegation"
            );
        }
    }

    /// Sheet-level dashed zone rectangles as (x1, y1, x2, y2), left to right.
    fn zone_rects(content: &str) -> Vec<(f64, f64, f64, f64)> {
        let lines: Vec<&str> = content.lines().collect();
        let pt = |w: &[&str], kw: &str| -> Option<(f64, f64)> {
            let l = w.iter().find(|l| l.trim_start().starts_with(kw))?;
            let mut it = l
                .trim()
                .trim_start_matches(kw)
                .trim_end_matches(')')
                .split_whitespace();
            Some((it.next()?.parse().ok()?, it.next()?.parse().ok()?))
        };
        let mut out = Vec::new();
        for (i, l) in lines.iter().enumerate() {
            if l.trim() != "(rectangle" {
                continue;
            }
            let w = &lines[i..(i + 7).min(lines.len())];
            if !w.iter().any(|l| l.contains("(type dash)")) {
                continue; // a symbol-body rectangle, not a zone
            }
            if let (Some(s), Some(e)) = (pt(w, "(start "), pt(w, "(end ")) {
                out.push((s.0.min(e.0), s.1.min(e.1), s.0.max(e.0), s.1.max(e.1)));
            }
        }
        out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        out
    }

    /// Consecutive wire endpoints as ((x1,y1),(x2,y2)).
    fn wire_segments(content: &str) -> Vec<((f64, f64), (f64, f64))> {
        let mut out = Vec::new();
        for block in content.split("\t(wire\n").skip(1) {
            let pts: Vec<(f64, f64)> = block
                .lines()
                .take_while(|l| !l.contains("(stroke"))
                .flat_map(|l| {
                    l.match_indices("(xy ").map(move |(i, _)| {
                        let mut it = l[i + 4..].split_whitespace();
                        let x: f64 = it.next().unwrap().parse().unwrap();
                        let y: f64 = it.next().unwrap().trim_end_matches(')').parse().unwrap();
                        (x, y)
                    })
                })
                .collect();
            for w in pts.windows(2) {
                out.push((w[0], w[1]));
            }
        }
        out
    }

    /// Labels of one kind (`global_label`, `label`, ...) as (name, x, y, rot).
    fn labels_of(content: &str, kind: &str) -> Vec<(String, f64, f64, i32)> {
        let mut out = Vec::new();
        for block in content.split(&format!("({kind} \"")).skip(1) {
            let name = block.split('"').next().unwrap_or("").to_string();
            let Some(at) = block.lines().find(|l| l.trim_start().starts_with("(at ")) else {
                continue;
            };
            let mut it = at.trim().trim_start_matches("(at ").split_whitespace();
            let x = it.next().and_then(|s| s.parse().ok()).unwrap_or(f64::NAN);
            let y = it.next().and_then(|s| s.parse().ok()).unwrap_or(f64::NAN);
            let rot = it
                .next()
                .and_then(|s| s.trim_end_matches(')').parse().ok())
                .unwrap_or(0);
            out.push((name, x, y, rot));
        }
        out
    }

    /// Power symbols as (rail_name, x, y, is_ground), flags excluded.
    fn power_symbols(content: &str) -> Vec<(String, f64, f64, bool)> {
        let mut out = Vec::new();
        for block in content.split("\t(symbol\n").skip(1) {
            let Some(lib) = block
                .lines()
                .find(|l| l.trim_start().starts_with("(lib_id \"pcb_power:"))
            else {
                continue;
            };
            let rail = lib
                .trim()
                .trim_start_matches("(lib_id \"pcb_power:")
                .trim_end_matches("\")")
                .to_string();
            if rail == "PWR_FLAG" {
                continue;
            }
            let Some(at) = block.lines().find(|l| l.trim_start().starts_with("(at ")) else {
                continue;
            };
            let mut it = at.trim().trim_start_matches("(at ").split_whitespace();
            let x: f64 = it.next().unwrap().parse().unwrap();
            let y: f64 = it.next().unwrap().parse().unwrap();
            out.push((rail.clone(), x, y, rail == "GND"));
        }
        out
    }

    #[test]
    fn labels_read_outward_from_their_pin() {
        // #3: a label at the end of a stub faces its pin — the connection point
        // (and a global label's flag) toward the wire, the text reading AWAY
        // from the pin. Regression: labels used to be flipped 180° (text over
        // the wire, the flag pointing out of the sheet).
        let out = generate_schematic(&decoupled_adc(), &SchOptions::new("lbl")).unwrap();
        let c = &out.files[0].content;
        let segs = wire_segments(c);
        let mut checked = 0;
        for kind in ["global_label", "label"] {
            for (name, x, y, rot) in labels_of(c, kind) {
                // Direction of the wire leaving the label anchor (toward the pin).
                let Some(dir) = segs.iter().find_map(|(a, b)| {
                    if (a.0 - x).abs() < 1e-3 && (a.1 - y).abs() < 1e-3 {
                        Some((b.0 - a.0, b.1 - a.1))
                    } else if (b.0 - x).abs() < 1e-3 && (b.1 - y).abs() < 1e-3 {
                        Some((a.0 - b.0, a.1 - b.1))
                    } else {
                        None
                    }
                }) else {
                    continue;
                };
                if dir.0.abs() <= dir.1.abs() {
                    continue; // vertical stub: no horizontal reading to check
                }
                // Text extends +x at rot 0, -x at rot 180 — always opposite the
                // wire, which runs toward the pin.
                let text_dir = if rot == 0 { 1.0 } else { -1.0 };
                assert!(
                    text_dir * dir.0 < 0.0,
                    "{kind} {name}: reads into its wire (rot {rot}, wire dx {})",
                    dir.0
                );
                checked += 1;
            }
        }
        assert!(checked > 0, "expected a horizontal label to check");
    }

    #[test]
    fn zones_enclose_their_power_symbols() {
        // #4: no drawn element pokes out of its zone. Power symbols are the
        // classic offender — a GND once hung below the Functional rectangle.
        let out = generate_schematic(&decoupled_adc(), &SchOptions::new("zc")).unwrap();
        let c = &out.files[0].content;
        let zones = zone_rects(c);
        assert_eq!(zones.len(), 3, "functional + decoupling + erc");
        let inside = |x1: f64, y1: f64, x2: f64, y2: f64| {
            zones
                .iter()
                .any(|z| z.0 - 0.1 <= x1 && x2 <= z.2 + 0.1 && z.1 - 0.1 <= y1 && y2 <= z.3 + 0.1)
        };
        for (name, x, y, ground) in power_symbols(c) {
            let ht = (name.chars().count() as f64 * 0.762 + 1.27).max(2.54);
            let (y1, y2) = if ground { (y, y + 6.35) } else { (y - 7.62, y) };
            assert!(
                inside(x - ht, y1, x + ht, y2),
                "power symbol {name} at ({x},{y}) escapes every zone"
            );
        }
    }

    #[test]
    fn zones_form_a_shared_edge_grid() {
        // #5: the three zones tile a grid — functional on the left, decoupling
        // over erc in the right column — sharing their dividing edges.
        let out = generate_schematic(&decoupled_adc(), &SchOptions::new("grid")).unwrap();
        let z = zone_rects(&out.files[0].content);
        assert_eq!(z.len(), 3);
        let func = z[0]; // leftmost cell
        let mut right = [z[1], z[2]];
        right.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap()); // top first
        let (top, bot) = (right[0], right[1]); // decoupling over erc
        let close = |a: f64, b: f64| (a - b).abs() < 0.01;
        // The right column's two cells share their left and right edges.
        assert!(close(top.0, bot.0), "utility cells share the left edge");
        assert!(close(top.2, bot.2), "utility cells share the right edge");
        // Functional spans the full height; the outer top/bottom are shared.
        assert!(close(func.1, top.1), "functional/decoupling share the top");
        assert!(close(func.3, bot.3), "functional/erc share the bottom");
        // Column and row seams abut with NO gap: cells touch on a shared edge
        // (clear channel) or overlap slightly (a stray stub reaching in), but
        // never leave a hole between them.
        assert!(
            func.2 >= top.0 - 0.01,
            "gap between functional and utility column ({} vs {})",
            func.2,
            top.0
        );
        assert!(
            top.3 >= bot.1 - 0.01,
            "gap between decoupling and erc ({} vs {})",
            top.3,
            bot.1
        );
    }

    #[test]
    fn unconnected_pin_gets_no_connect() {
        let mut sch = divider();
        sch.nets.remove("GND");
        let out = generate_schematic(&sch, &SchOptions::new("t")).unwrap();
        assert!(out.files[0].content.contains("(no_connect"));
    }

    #[test]
    fn single_endpoint_signal_net_gets_global_label() {
        let mut sch = divider();
        sch.net_mut("MID")
            .unwrap()
            .ports
            .retain(|p| p.instance_path.first().map(String::as_str) == Some("R1"));
        let out = generate_schematic(&sch, &SchOptions::new("t")).unwrap();
        let text = &out.files[0].content;
        assert_eq!(text.matches("(global_label \"MID\"").count(), 1);
        assert_eq!(text.matches("(label \"MID\"").count(), 0);
        assert!(text.contains("(no_connect"));
    }

    #[test]
    fn box_symbol_synthesized_when_symbol_missing() {
        let mut sch = divider();
        for inst in sch.instances.values_mut() {
            inst.attributes.remove(ATTR_SYMBOL_VALUE);
        }
        let out = generate_schematic(&sch, &SchOptions::new("t")).unwrap();
        assert_eq!(out.warnings.len(), 2);
        assert!(out.files[0].content.contains("_BOX"));
    }

    #[test]
    fn hierarchical_design_emits_two_linked_sheets() {
        let sch = hierarchical_design();
        let opts = SchOptions::new("top");
        let out = generate_schematic(&sch, &opts).unwrap();
        assert_eq!(out.files.len(), 2);
        assert_eq!(out.files[0].file_name, "top.kicad_sch");
        assert_eq!(out.files[1].file_name, "big.kicad_sch");

        let root = &out.files[0].content;
        let child = &out.files[1].content;
        // The root references the child sheet with a matching sheet pin.
        assert!(root.contains("(sheet\n"));
        assert!(root.contains("\"big.kicad_sch\""));
        assert!(root.contains("(pin \"SIG\""));
        // The child carries the matching hierarchical label...
        assert!(child.contains("(hierarchical_label \"SIG\""));
        // ...and no sheet_instances block (root only).
        assert!(!child.contains("(sheet_instances"));
        assert!(root.contains("(sheet_instances"));
        // Power symbols never become sheet pins.
        assert!(!root.contains("(pin \"VCC\""));
        // Sub-sheet symbols use the extended instance path.
        assert!(child.contains("(path \"/"));

        // Determinism across runs.
        let again = generate_schematic(&sch, &opts).unwrap();
        assert_eq!(out.files[0].content, again.files[0].content);
        assert_eq!(out.files[1].content, again.files[1].content);
    }

    #[test]
    fn manual_positions_pin_components() {
        let mut sch = divider();
        // Manually position R1 (viewer coordinates, 0.1 mm units).
        let root = sch.root_ref.clone().unwrap();
        let root_inst = sch.instance_mut(&root).unwrap();
        root_inst.symbol_positions.insert(
            "comp:R1".to_string(),
            pcb_sch::position::Position {
                x: 1016.0, // 101.6 mm
                y: 508.0,  // 50.8 mm
                rotation: 0.0,
                mirror: None,
            },
        );
        let out = generate_schematic(&sch, &SchOptions::new("t")).unwrap();
        // The uniform page translation is reported when pinning is active.
        let text = &out.files[0].content;
        assert!(text.contains("\"R1\""));
        // R1 and R2 both placed; deterministic output.
        let again = generate_schematic(&sch, &SchOptions::new("t")).unwrap();
        assert_eq!(text, &again.files[0].content);
    }
}
