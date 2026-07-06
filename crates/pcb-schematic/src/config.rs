//! Layout configuration for the KiCad schematic generator.
//!
//! All distances are in millimeters. Spacing values are multiples of the
//! KiCad schematic grid (1.27 mm / 50 mil). Golden rule: every electrical
//! connection point (pin end, wire endpoint, sheet pin) must stay on that
//! grid, otherwise eeschema reports `endpoint_off_grid` violations.
//!
//! The defaults were tuned on a validated TypeScript proof of concept whose
//! output was reviewed rule by rule in eeschema by an electronics engineer.

use serde::Deserialize;

/// Paper size selection for generated sheets.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "UPPERCASE")]
pub enum Paper {
    /// Pick the smallest size that fits the content (A4, then A3, then A2).
    #[serde(rename = "auto")]
    #[default]
    Auto,
    A4,
    A3,
    A2,
}

impl Paper {
    /// Usable page sizes in landscape orientation, smallest first.
    pub(crate) const SIZES: &'static [(Paper, f64, f64)] = &[
        (Paper::A4, 297.0, 210.0),
        (Paper::A3, 420.0, 297.0),
        (Paper::A2, 594.0, 420.0),
    ];

    /// KiCad name of a fixed paper size (`Auto` has no name by itself).
    pub(crate) fn kicad_name(self) -> &'static str {
        match self {
            Paper::Auto => "A4",
            Paper::A4 => "A4",
            Paper::A3 => "A3",
            Paper::A2 => "A2",
        }
    }
}

/// Configuration of the schematic generator (sheet planning + placement +
/// wiring). Deserializable so a user rules file can override any subset;
/// unspecified fields keep the proven defaults.
#[derive(Debug, Clone, Deserialize)]
#[serde(default, rename_all = "kebab-case", deny_unknown_fields)]
pub struct SchConfig {
    /// Base KiCad schematic grid (50 mil). Do not change: KiCad contract.
    pub grid_mm: f64,
    /// Horizontal channel between columns of major components.
    pub col_gap_mm: f64,
    /// Vertical spacing between major components in the same column.
    pub row_gap_mm: f64,
    /// Clearance from an anchor's edge to its satellite when no offset is given.
    pub satellite_gap_mm: f64,
    /// Horizontal pitch of satellite rows (decoupling caps, pulls, ...).
    pub satellite_pitch_mm: f64,
    /// Distance from an IC edge to its decoupling/pull row (no offset given).
    pub role_row_gap_mm: f64,
    /// Signal stub length (pin end to net label).
    pub stub_mm: f64,
    /// Minimum length, in grid steps, of the wire carrying a net label. The
    /// stub is always at least `label_stub_min_grid_steps * grid_mm` and
    /// longer when the text is wider (the wire must underline the full text).
    pub label_stub_min_grid_steps: u32,
    /// Vertical gap, in grid steps, of the Value text below the component
    /// (below the lowest pin end; falls back under the body when the symbol
    /// has no downward pin).
    pub value_gap_grid_steps: u32,
    /// Vertical gap, in grid steps, of the Reference text above the top-left
    /// corner of the component body.
    pub ref_gap_grid_steps: u32,
    /// Power stub length (pin end to power symbol).
    pub power_stub_mm: f64,
    /// Power symbol row alignment: two stubs of the same net whose symbols
    /// would land within this X/Y window are aligned on a common Y (the
    /// shorter stub is stretched).
    pub power_align_max_dx_mm: f64,
    pub power_align_max_dy_mm: f64,
    /// Horizontal elbow used to keep net labels of vertical pins horizontal.
    pub label_elbow_mm: f64,
    /// Clearance from an MCU OSC pin to the crystal cluster axis.
    pub crystal_gap_mm: f64,
    /// Above this distance, two aligned pins get net labels instead of wires.
    pub direct_wire_max_mm: f64,
    /// Hub threshold: a component with more visible pins than this is a hub
    /// (MCU, big connector...); its signal nets break into net labels instead
    /// of real wires. Applied per component, not per net.
    pub hub_pin_count_threshold: usize,
    /// Analog wiring break threshold: an analog net is wired as a continuous
    /// wire and is allowed to break into a label only when it lands on a
    /// symbol with more visible pins than this (a many-pin part where a
    /// continuous wire would be unreadable). Smaller symbols keep the wire.
    /// A net leaving directly on a hierarchical/global port always keeps its
    /// wire regardless of this threshold.
    pub analog_break_pin_count: usize,
    /// Collision margin added around component bounding boxes.
    pub component_pad_mm: f64,
    /// Content margins of a sheet.
    pub margin_left_mm: f64,
    pub margin_top_mm: f64,
    /// Width reserved for the hierarchical port column.
    pub port_column_mm: f64,
    /// Vertical pitch between hierarchical ports.
    pub port_pitch_mm: f64,
    /// Sub-sheet blocks placed on the parent sheet.
    pub sheet_block_min_width_mm: f64,
    pub sheet_block_gap_x_mm: f64,
    pub sheet_block_gap_y_mm: f64,
    pub sheet_region_top_gap_mm: f64,
    /// Paper size (auto picks A4 then A3 then A2 based on content extent).
    pub paper: Paper,
    /// Sheet planner: a module with fewer physical parts than this threshold
    /// is a candidate for inlining into its caller's sheet.
    pub sheet_min_parts: usize,
    /// Sheet planner: inline modules whose parts are all satellites
    /// (passives, decoupling, pulls, LEDs) regardless of size.
    pub inline_passive_only_modules: bool,
    /// Sheet planner: a small module is only inlined when it contains at most
    /// this many "major" parts (connector, regulator, mcu, sensor, ...).
    pub inline_max_major_parts: usize,
    /// Sheet planner: warn when a sheet holds more parts than this.
    pub max_parts_per_sheet: usize,
    /// Role heuristics: a rail-to-rail capacitor at or above this value (in
    /// microfarads) is classified `bulk` instead of `decoupling`.
    pub bulk_capacitance_uf: f64,
    /// Proximity grouping: a series two-pin passive on signal-only nets (an
    /// input/output filter element) rides next to the part it feeds instead
    /// of floating into the column flow, but only when that part carries at
    /// least this many visible pins (a real IC, not another two-pin part).
    /// Its input filter is then re-seated as a tidy aligned cluster (series
    /// resistors on a shared X column, shunt caps on a shared Y row).
    pub input_chain_min_pins: usize,
    /// Relegation: pull auto-placed rail-to-rail capacitors (decoupling and
    /// bulk) and the undriven-rail `PWR_FLAG`s out of the functional flow and
    /// into a dedicated utility band on the right — decoupling aligned in a
    /// row at the top of the band, the flags stacked below it — and outline
    /// the functional / decoupling / ERC areas with discreet graphic
    /// rectangles. Only sheets carrying a real IC (a part with at least
    /// `input_chain_min_pins` pins) are relegated; a passive-only sheet keeps
    /// its parts in place. Manually positioned (`# pcb:sch`) parts are never
    /// relegated. Purely a placement/annotation change: the netlist is
    /// unaffected (power symbols and flags are not netlist nodes).
    pub relegate_utility: bool,
    /// Horizontal clearance between the functional flow's right edge and the
    /// relegated utility band.
    pub utility_gap_mm: f64,
    /// Pitch between relegated items (decoupling caps along the row, flags
    /// down the column).
    pub utility_pitch_mm: f64,
    /// Padding added around a zone's contents when drawing its outline
    /// rectangle.
    pub zone_margin_mm: f64,
}

impl Default for SchConfig {
    fn default() -> Self {
        Self {
            grid_mm: 1.27,
            col_gap_mm: 19.05,
            row_gap_mm: 15.24,
            satellite_gap_mm: 7.62,
            satellite_pitch_mm: 11.43,
            role_row_gap_mm: 19.05,
            stub_mm: 5.08,
            label_stub_min_grid_steps: 4,
            value_gap_grid_steps: 1,
            ref_gap_grid_steps: 1,
            power_stub_mm: 2.54,
            power_align_max_dx_mm: 25.4,
            power_align_max_dy_mm: 7.62,
            label_elbow_mm: 2.54,
            crystal_gap_mm: 8.89,
            direct_wire_max_mm: 50.8,
            hub_pin_count_threshold: 10,
            analog_break_pin_count: 8,
            component_pad_mm: 1.27,
            margin_left_mm: 25.4,
            margin_top_mm: 22.86,
            port_column_mm: 20.32,
            port_pitch_mm: 7.62,
            sheet_block_min_width_mm: 30.48,
            sheet_block_gap_x_mm: 44.45,
            sheet_block_gap_y_mm: 17.78,
            sheet_region_top_gap_mm: 20.32,
            paper: Paper::Auto,
            sheet_min_parts: 4,
            inline_passive_only_modules: true,
            inline_max_major_parts: 1,
            max_parts_per_sheet: 40,
            bulk_capacitance_uf: 10.0,
            input_chain_min_pins: 3,
            relegate_utility: true,
            utility_gap_mm: 12.7,
            utility_pitch_mm: 12.7,
            zone_margin_mm: 3.81,
        }
    }
}

impl SchConfig {
    /// Snap a coordinate onto the schematic grid (nearest step).
    pub fn snap(&self, v: f64) -> f64 {
        crate::round4((v / self.grid_mm).round() * self.grid_mm)
    }

    /// Snap a length up to the next grid step.
    pub fn snap_up(&self, v: f64) -> f64 {
        crate::round4((v / self.grid_mm - 1e-9).ceil() * self.grid_mm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_on_grid() {
        let c = SchConfig::default();
        for v in [
            c.col_gap_mm,
            c.row_gap_mm,
            c.satellite_gap_mm,
            c.stub_mm,
            c.power_stub_mm,
            c.margin_left_mm,
            c.margin_top_mm,
        ] {
            let steps = v / c.grid_mm;
            assert!(
                (steps - steps.round()).abs() < 1e-6,
                "{v} is not a grid multiple"
            );
        }
    }

    #[test]
    fn snap_rounds_to_grid() {
        let c = SchConfig::default();
        assert_eq!(c.snap(2.6), 2.54);
        assert_eq!(c.snap(0.0), 0.0);
        assert_eq!(c.snap_up(0.1), 1.27);
        assert_eq!(c.snap_up(1.27), 1.27);
    }

    #[test]
    fn config_overrides_from_toml_subset() {
        let cfg: SchConfig =
            toml_str_subset("grid-mm = 1.27\nhub-pin-count-threshold = 12\npaper = \"A3\"\n");
        assert_eq!(cfg.hub_pin_count_threshold, 12);
        assert_eq!(cfg.paper, Paper::A3);
        // Untouched fields keep defaults.
        assert_eq!(cfg.stub_mm, 5.08);
        assert_eq!(cfg.analog_break_pin_count, 8);
        // The analog break threshold is overridable like any other field.
        let over: SchConfig = toml_str_subset("analog-break-pin-count = 16\n");
        assert_eq!(over.analog_break_pin_count, 16);
        // New foundation knob, kebab-case, defaulted and overridable.
        assert_eq!(cfg.input_chain_min_pins, 3);
        let ic: SchConfig = toml_str_subset("input-chain-min-pins = 5\n");
        assert_eq!(ic.input_chain_min_pins, 5);
        // Relegation knobs, kebab-case, defaulted and overridable.
        assert!(cfg.relegate_utility);
        assert_eq!(cfg.utility_gap_mm, 12.7);
        let rel: SchConfig = toml_str_subset("relegate-utility = false\nutility-gap-mm = 19.05\n");
        assert!(!rel.relegate_utility);
        assert_eq!(rel.utility_gap_mm, 19.05);
    }

    fn toml_str_subset(s: &str) -> SchConfig {
        // serde_json round-trip via toml is not available here; use the
        // serde_json Value path to exercise the Deserialize impl.
        let value: serde_json::Value = {
            // Minimal TOML-ish parsing for the test: key = value lines.
            let mut map = serde_json::Map::new();
            for line in s.lines().filter(|l| !l.trim().is_empty()) {
                let (k, v) = line.split_once('=').unwrap();
                let k = k.trim().to_string();
                let v = v.trim();
                let jv = if !v.contains('.') && v.parse::<u64>().is_ok() {
                    serde_json::json!(v.parse::<u64>().unwrap())
                } else if let Ok(n) = v.parse::<f64>() {
                    serde_json::json!(n)
                } else if let Ok(b) = v.parse::<bool>() {
                    serde_json::json!(b)
                } else {
                    serde_json::json!(v.trim_matches('"'))
                };
                map.insert(k, jv);
            }
            serde_json::Value::Object(map)
        };
        serde_json::from_value(value).unwrap()
    }
}
