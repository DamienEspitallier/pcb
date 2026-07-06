//! Procedural generator of human-readable KiCad schematics (`.kicad_sch`)
//! from a [`pcb_sch::Schematic`] netlist.
//!
//! The crate consumes the evaluated IR **read-only** and emits KiCad 9/10
//! s-expression documents through [`pcb_sexpr`]. Symbols are embedded
//! verbatim from the `__symbol_value` attribute each Zener component
//! carries; power and ground rails become generated global power symbols;
//! every UUID is deterministic so repeated runs are byte-identical.
//!
//! Entry point: [`generate_schematic`] plans the sheet hierarchy
//! ([`sheets`]), places every sheet by signal flow with satellites stuck to
//! their anchors ([`place`]), routes label stubs and power symbols
//! ([`route`]) and emits the documents through [`writer`]. Manual
//! `# pcb:sch` positions are honored as hard constraints.

pub mod config;
pub mod geometry;
pub mod uuid;
pub mod writer;

mod generate;
mod model;
mod place;
mod route;
mod sheets;
#[cfg(test)]
mod testkit;

pub use config::{Paper, SchConfig};
pub use generate::{GeneratedSchematic, SchFile, SchOptions, generate_schematic};

use anyhow::{Context, Result};

/// Round a coordinate to 4 decimal places, normalizing negative zero
/// (KiCad-style number formatting).
pub(crate) fn round4(v: f64) -> f64 {
    let r = (v * 10000.0).round() / 10000.0;
    if r == 0.0 { 0.0 } else { r }
}

/// ERC rule severities that a generated project must override: embedded
/// symbols necessarily come from libraries unknown to (or diverging from)
/// the local KiCad installation, which otherwise raises unavoidable
/// `lib_symbol_issues` / `lib_symbol_mismatch` warnings; footprint
/// assignments likewise reference libraries that may not be in the local
/// footprint table (`footprint_link_issues`) — `pcb layout` resolves
/// footprints through its own pipeline (verified with kicad-cli 10.0.3).
const ERC_IGNORED_RULES: &[&str] = &[
    "lib_symbol_issues",
    "lib_symbol_mismatch",
    "footprint_link_issues",
];

/// Build (or patch) the `.kicad_pro` content next to a generated schematic.
///
/// With `existing = None` a minimal project file is created. With an
/// existing project (e.g. the one `pcb layout` maintains) only the
/// `erc.rule_severities` entries are inserted — everything else is
/// preserved verbatim.
pub fn kicad_pro_content(existing: Option<&str>, project_name: &str) -> Result<String> {
    let mut project: serde_json::Value = match existing {
        Some(text) => {
            serde_json::from_str(text).context("failed to parse existing .kicad_pro JSON")?
        }
        None => serde_json::json!({
            "meta": {
                "filename": format!("{project_name}.kicad_pro"),
                "version": 3,
            },
        }),
    };

    let root = project
        .as_object_mut()
        .context(".kicad_pro root is not a JSON object")?;
    let erc = root
        .entry("erc")
        .or_insert_with(|| serde_json::json!({}))
        .as_object_mut()
        .context(".kicad_pro `erc` is not a JSON object")?;
    let severities = erc
        .entry("rule_severities")
        .or_insert_with(|| serde_json::json!({}))
        .as_object_mut()
        .context(".kicad_pro `erc.rule_severities` is not a JSON object")?;
    for rule in ERC_IGNORED_RULES {
        severities.insert(rule.to_string(), serde_json::json!("ignore"));
    }

    let mut out = serde_json::to_string_pretty(&project)?;
    out.push('\n');
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round4_normalizes() {
        assert_eq!(round4(60.959999999999994), 60.96);
        assert_eq!(round4(-0.00001), 0.0);
        assert_eq!(round4(-1.27), -1.27);
    }

    #[test]
    fn kicad_pro_created_from_scratch() {
        let text = kicad_pro_content(None, "blinky").unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["meta"]["filename"], "blinky.kicad_pro");
        assert_eq!(v["erc"]["rule_severities"]["lib_symbol_issues"], "ignore");
        assert_eq!(v["erc"]["rule_severities"]["lib_symbol_mismatch"], "ignore");
    }

    #[test]
    fn kicad_pro_patch_preserves_existing_content() {
        let existing = r#"{
            "meta": {"filename": "layout.kicad_pro", "version": 3},
            "board": {"design_settings": {"rules": {"min_clearance": 0.1}}},
            "erc": {"rule_severities": {"pin_not_connected": "error"}}
        }"#;
        let text = kicad_pro_content(Some(existing), "layout").unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        // Existing keys survive.
        assert_eq!(v["board"]["design_settings"]["rules"]["min_clearance"], 0.1);
        assert_eq!(v["erc"]["rule_severities"]["pin_not_connected"], "error");
        // Our overrides are added.
        assert_eq!(v["erc"]["rule_severities"]["lib_symbol_issues"], "ignore");
    }
}
