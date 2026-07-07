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
    /// Horizontal air between an IC's input edge and its re-seated input-filter
    /// resistor column, on top of the room reserved for the shunt-cap row. The
    /// sheet is mostly empty, so the filter is spread well off the IC: the
    /// filtered nets then run as long continuous wires across the open band and
    /// a pin-seated pull-up at the IC edge stays clear of the filter caps. A
    /// tight gap tasses the whole input against the IC (the reference AD7171
    /// pushes its filter far to the left instead, with the IC well to the
    /// right). The spread is no longer bounded by any wire-length limit — the
    /// filtered analog wire stays continuous at any length — so this defaults
    /// generously to match the reference layout; keep it sensible relative to
    /// the useful content (no spread for its own sake).
    pub input_filter_gap_mm: f64,
    /// Horizontal pitch of satellite rows (decoupling caps, pulls, ...).
    pub satellite_pitch_mm: f64,
    /// Distance from an IC edge to its decoupling/pull row (no offset given).
    pub role_row_gap_mm: f64,
    /// Offset, along a pulled pin's exit direction, from that IC pin to the
    /// upright pull resistor's drop. A pull anchored to an IC is re-seated
    /// directly over the pin it pulls so its free leg drops straight onto the
    /// pin's exit stub as one continuous wire (instead of floating over the IC
    /// body where the net would break into a label). Kept small so the pull
    /// hugs the pin just outside the body — two grid steps by default.
    pub pullup_pin_gap_mm: f64,
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
    /// Minimum straight pin exit, in grid steps. Every wire (signal stub,
    /// power/ground dogleg, PWR_FLAG/ERC leg, stacked-pin bus tap) must leave
    /// its component pin in a straight line, collinear with the pin, for at
    /// least this many grid steps before its first right-angle bend. Keeps the
    /// schematic free of right angles struck right at a pin (the engineer's
    /// "at least two grid steps out of every pin" rule).
    pub min_pin_exit_steps: usize,
    /// Minimum spacing, in grid steps, kept between two parallel wires (or a
    /// net label's text) of DIFFERENT nets. Harmonizes the whole sheet on the
    /// same coarse pitch the power exits already use (the engineer's "two grid
    /// steps between nets by default, like out of power" rule): a power-bus
    /// column peels off two steps clear of the neighbouring net instead of
    /// hugging it at one, and a foreign net label keeps two steps of air from
    /// a parallel wire. Matches `min_pin_exit_steps` so the pin exit and the
    /// inter-net channel read as one grid.
    pub min_net_spacing_steps: usize,
    /// Power stub length (pin end to power symbol).
    pub power_stub_mm: f64,
    /// Power symbol row alignment: two stubs of the same net whose symbols
    /// would land within this X/Y window are aligned on a common Y (the
    /// shorter stub is stretched).
    pub power_align_max_dx_mm: f64,
    pub power_align_max_dy_mm: f64,
    /// Order the power symbols of a cluster by electrical potential on the
    /// vertical axis and align same-potential symbols on a shared ordinate:
    /// positive rails (VDD/VCC/+…) ride at the top, grounds (GND/VSS) at the
    /// bottom, negative rails (VEE/-…) lower still, and every symbol of the
    /// same potential in one alignment window shares a common Y so the sheet
    /// reads as horizontal power bands. Purely a symbol-placement rule — the
    /// netlist is unaffected. Set false to keep the plain same-net alignment.
    pub align_power_by_potential: bool,
    /// Horizontal elbow used to keep net labels of vertical pins horizontal.
    pub label_elbow_mm: f64,
    /// Clearance from an MCU OSC pin to the crystal cluster axis.
    pub crystal_gap_mm: f64,
    /// Anti-absurd upper guard on the length of a single continuous wire, in
    /// millimeters. Wire length is NOT a wire-vs-label criterion: an analog net
    /// is wired as one continuous wire whatever its length. The decision to
    /// wire a net rests on its class (analog -> wire; digital -> label ok), hub
    /// membership (`hub_pin_count_threshold` / `analog_break_pin_count`) and
    /// clean routability (no frank crossing beyond the existing cost penalty) —
    /// never on how long the wire would be. This guard only rejects a candidate
    /// whose length is physically nonsensical: longer than any wire could
    /// sensibly span on the biggest supported sheet (A2 is 594x420 mm, a
    /// Manhattan reach of ~1014 mm), where a route that long is a routing bug,
    /// not a real net. Kept generous so it never gates a legitimate spread; set
    /// lower only to deliberately cap the router.
    pub wire_length_guard_mm: f64,
    /// Local reach, in millimeters, of the orientation engine's routability
    /// probe: at placement time a component is turned toward a group only when
    /// one of its pins can reach that group with a wire no longer than this.
    /// This is a placement PROXIMITY heuristic ("is the group within local
    /// reach so the pin should face it"), NOT the wire-vs-label gate — once a
    /// net is committed to a wire it runs at any length (`wire_length_guard_mm`).
    /// Kept at the proven local value so orientation (and the placement it
    /// drives) is unchanged.
    pub orient_probe_reach_mm: f64,
    /// De-duplicate the labels of a signal net onto a single boundary port.
    /// A net must carry only ONE port/name label (the module-boundary marker):
    /// the reader should not chase the same name across two floating labels.
    /// When a signal net's endpoints scatter into two or more homonym labels
    /// (its crossing-free tree could not join them), the router keeps a single
    /// label on one endpoint and wires the OTHER endpoints to it as continuous
    /// wires — accepting electrically-harmless frank crossings if that is the
    /// only way to reach the label, exactly as the engineer draws a DOUT pull-up
    /// dropping straight onto the pin across the analog input wires. The change
    /// is purely graphical (labels and wires never move a netlist node): the
    /// exported `(ref.pin)` net partition is unchanged. Set false to keep one
    /// homonym label per stranded endpoint.
    pub dedup_signal_labels: bool,
    /// Wiring cost penalty, in millimeters of equivalent path length, charged
    /// for each frank crossing of a foreign net on a candidate wire. Frank
    /// crossings are electrically harmless in KiCad (perpendicular wires that
    /// only touch at an interior point make no junction, hence no connection),
    /// so they are allowed — but this penalty keeps the router avoiding them: a
    /// crossing-free candidate is always preferred, and a crossing is accepted
    /// only when it is the sole way to keep a net on one continuous wire instead
    /// of breaking it into labels. Same-net crossings (a missing junction) stay
    /// forbidden. Set to 0 to treat crossings as free.
    pub crossing_penalty_mm: f64,
    /// Wiring cost penalty, in millimeters of equivalent path length, charged
    /// for each right-angle bend on a candidate wire. Biases the router toward
    /// straight wires: a signal's main path runs straight into its target IC
    /// pin and the bends are pushed onto the shunt/branch derivations (a shunt
    /// cap tees off a straight backbone instead of sitting on its corner).
    pub bend_penalty_mm: f64,
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
    /// Gap between adjacent cells of the zone grid. Zones are laid out as a
    /// table (functional flow on the left, the utility column split into
    /// decoupling over ERC on the right); with a gap of 0 the neighbouring
    /// cells share their dividing edge (the tidy look of the reference
    /// layout), a positive value opens a channel between them. The gap never
    /// eats into a cell's content — it is capped by the free space between the
    /// two contents it separates.
    pub zone_gap_mm: f64,
    /// Zone outlines snap their outer edges outward to this grid so the cells
    /// line up cleanly. Set to 0 to disable snapping. The inner dividers stay
    /// centered in the free space between contents, so cells always abut.
    pub zone_snap_mm: f64,
}

impl Default for SchConfig {
    fn default() -> Self {
        Self {
            grid_mm: 1.27,
            col_gap_mm: 19.05,
            row_gap_mm: 15.24,
            satellite_gap_mm: 7.62,
            input_filter_gap_mm: 62.23,
            satellite_pitch_mm: 11.43,
            role_row_gap_mm: 19.05,
            pullup_pin_gap_mm: 2.54,
            stub_mm: 5.08,
            min_pin_exit_steps: 2,
            min_net_spacing_steps: 2,
            label_stub_min_grid_steps: 4,
            value_gap_grid_steps: 1,
            ref_gap_grid_steps: 1,
            power_stub_mm: 2.54,
            power_align_max_dx_mm: 25.4,
            power_align_max_dy_mm: 7.62,
            align_power_by_potential: true,
            label_elbow_mm: 2.54,
            crystal_gap_mm: 8.89,
            wire_length_guard_mm: 1016.0,
            orient_probe_reach_mm: 50.8,
            dedup_signal_labels: true,
            crossing_penalty_mm: 20.0,
            bend_penalty_mm: 5.08,
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
            zone_gap_mm: 0.0,
            zone_snap_mm: 1.27,
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

    /// Minimum straight pin-exit length in millimeters (`min_pin_exit_steps`
    /// grid steps). A pin's first bend must sit at least this far from the pin.
    pub fn min_pin_exit_mm(&self) -> f64 {
        crate::round4(self.min_pin_exit_steps as f64 * self.grid_mm)
    }

    /// Minimum inter-net spacing in millimeters (`min_net_spacing_steps` grid
    /// steps). Two parallel wires of different nets, or a foreign net label and
    /// a wire, stay at least this far apart.
    pub fn min_net_spacing_mm(&self) -> f64 {
        crate::round4(self.min_net_spacing_steps as f64 * self.grid_mm)
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
        // Router cost knobs, kebab-case, defaulted and overridable.
        assert_eq!(cfg.crossing_penalty_mm, 20.0);
        assert_eq!(cfg.bend_penalty_mm, 5.08);
        let pen: SchConfig =
            toml_str_subset("crossing-penalty-mm = 30.0\nbend-penalty-mm = 2.54\n");
        assert_eq!(pen.crossing_penalty_mm, 30.0);
        assert_eq!(pen.bend_penalty_mm, 2.54);
        // Pin-exit knob, kebab-case, defaulted and overridable.
        assert_eq!(cfg.min_pin_exit_steps, 2);
        assert_eq!(cfg.min_pin_exit_mm(), 2.54);
        let pe: SchConfig = toml_str_subset("min-pin-exit-steps = 3\n");
        assert_eq!(pe.min_pin_exit_steps, 3);
        assert_eq!(pe.min_pin_exit_mm(), 3.81);
        // Inter-net spacing knob, kebab-case, defaulted and overridable. Its
        // default matches the pin-exit guard (two grid steps) so the sheet
        // reads on one harmonized coarse grid.
        assert_eq!(cfg.min_net_spacing_steps, 2);
        assert_eq!(cfg.min_net_spacing_mm(), 2.54);
        let ns: SchConfig = toml_str_subset("min-net-spacing-steps = 3\n");
        assert_eq!(ns.min_net_spacing_steps, 3);
        assert_eq!(ns.min_net_spacing_mm(), 3.81);
        // Potential-ordering knob, kebab-case, defaulted and overridable.
        assert!(cfg.align_power_by_potential);
        let ap: SchConfig = toml_str_subset("align-power-by-potential = false\n");
        assert!(!ap.align_power_by_potential);
        // New foundation knob, kebab-case, defaulted and overridable.
        assert_eq!(cfg.input_chain_min_pins, 3);
        let ic: SchConfig = toml_str_subset("input-chain-min-pins = 5\n");
        assert_eq!(ic.input_chain_min_pins, 5);
        // Pull-up seating knob, kebab-case, defaulted and overridable.
        assert_eq!(cfg.pullup_pin_gap_mm, 2.54);
        let pu: SchConfig = toml_str_subset("pullup-pin-gap-mm = 5.08\n");
        assert_eq!(pu.pullup_pin_gap_mm, 5.08);
        // Input-filter spread knob, kebab-case, defaulted and overridable. The
        // default spreads the filter well off the IC (matching the reference
        // AD7171, whose IC sits far right) now that no wire-length limit caps it.
        assert_eq!(cfg.input_filter_gap_mm, 62.23);
        let ifg: SchConfig = toml_str_subset("input-filter-gap-mm = 25.4\n");
        assert_eq!(ifg.input_filter_gap_mm, 25.4);
        // Wire-length guard: an anti-absurd bound, NOT a wire/label gate. Kept
        // generous (~A2 Manhattan span) so length never breaks a continuous wire.
        assert_eq!(cfg.wire_length_guard_mm, 1016.0);
        let wg: SchConfig = toml_str_subset("wire-length-guard-mm = 500.0\n");
        assert_eq!(wg.wire_length_guard_mm, 500.0);
        // Orientation-probe reach: a placement proximity heuristic, distinct
        // from the wire/label decision; defaulted and overridable.
        assert_eq!(cfg.orient_probe_reach_mm, 50.8);
        let opr: SchConfig = toml_str_subset("orient-probe-reach-mm = 38.1\n");
        assert_eq!(opr.orient_probe_reach_mm, 38.1);
        // Relegation knobs, kebab-case, defaulted and overridable.
        assert!(cfg.relegate_utility);
        assert_eq!(cfg.utility_gap_mm, 12.7);
        let rel: SchConfig = toml_str_subset("relegate-utility = false\nutility-gap-mm = 19.05\n");
        assert!(!rel.relegate_utility);
        assert_eq!(rel.utility_gap_mm, 19.05);
        // Label de-duplication knob, kebab-case, defaulted and overridable.
        assert!(cfg.dedup_signal_labels);
        let dd: SchConfig = toml_str_subset("dedup-signal-labels = false\n");
        assert!(!dd.dedup_signal_labels);
        // Zone-grid knobs, kebab-case, defaulted and overridable.
        assert_eq!(cfg.zone_gap_mm, 0.0);
        assert_eq!(cfg.zone_snap_mm, 1.27);
        let zg: SchConfig = toml_str_subset("zone-gap-mm = 2.54\nzone-snap-mm = 2.54\n");
        assert_eq!(zg.zone_gap_mm, 2.54);
        assert_eq!(zg.zone_snap_mm, 2.54);
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
