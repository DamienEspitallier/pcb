//! Low-level writer for a single `.kicad_sch` sheet.
//!
//! Target format: KiCad 9/10, `(version 20250114)` — validated empirically
//! against kicad-cli 10.0.3 (`sch erc` + `sch export svg` + `sch export
//! netlist`) on handcrafted probe files. Verified format points:
//!
//! * embedded `lib_symbols` accept raw blocks lifted from KiCad 10 libraries
//!   (`in_pos_files`, `duplicate_pin_numbers_are_jumpers`, `embedded_fonts`
//!   fields included), renamed to a `nickname:name` lib id;
//! * `(instances ...)` on symbols/sheets and `(sheet_instances ...)` are
//!   accepted; the displayed reference comes from the `Reference` property;
//! * sheet properties are `Sheetname` / `Sheetfile` (no space);
//! * an embedded power symbol carries `(power global)`; its **Value creates
//!   the global net** — no extra label required;
//! * a power net driven by no `power_out` pin needs exactly one `PWR_FLAG`
//!   (pin type `power_out`), otherwise ERC raises `power_pin_not_driven`;
//! * a `Sheetfile` pointing to a file that is never written silently renders
//!   a blank page — generators must verify every referenced file is emitted.
//!
//! All UUIDs are deterministic (see [`crate::uuid`]): identical content plus
//! identical seed serializes byte-for-byte identically.

use std::collections::BTreeMap;

use pcb_sch::position::MirrorAxis;
use pcb_sexpr::Sexpr;
use pcb_sexpr::formatter::{FormatMode, format_tree};

use crate::round4;
use crate::uuid::UuidGen;

/// Schematic file format version emitted (KiCad 9/10).
pub const KICAD_SCH_VERSION: i64 = 20250114;

/// Library nickname used for generated power/ground symbols. Deliberately
/// not a stock KiCad library name ("power") so eeschema never tries to sync
/// our generated graphics against the locally installed library.
pub const POWER_LIB_NICKNAME: &str = "pcb_power";

/// Direction of a hierarchical port (sheet pin / hierarchical label shape).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDirection {
    Input,
    Output,
    Bidirectional,
}

impl PortDirection {
    fn shape(self) -> &'static str {
        match self {
            PortDirection::Input => "input",
            PortDirection::Output => "output",
            PortDirection::Bidirectional => "bidirectional",
        }
    }
}

/// Visual text justification tokens (subset used by the generator).
pub type Justify = &'static [&'static str];

/// Options for placing one symbol instance.
#[derive(Debug, Clone)]
pub struct PlaceSymbol<'a> {
    /// Lib id of a symbol previously registered with `add_lib_symbol`.
    pub lib_id: &'a str,
    /// Reference designator (also drives the `Reference` property).
    pub reference: &'a str,
    pub value: &'a str,
    /// Instance anchor on the sheet (grid-aligned).
    pub at: (f64, f64),
    /// CCW rotation in degrees (0/90/180/270).
    pub rotation: i32,
    pub mirror: Option<MirrorAxis>,
    pub unit: i32,
    pub dnp: bool,
    /// Hidden standard fields (emitted only when provided).
    pub footprint: Option<&'a str>,
    pub datasheet: Option<&'a str>,
    pub description: Option<&'a str>,
    pub mpn: Option<&'a str>,
    /// Stable key for UUID derivation: the hierarchical instance path
    /// relative to the root module (machine independent).
    pub uuid_key: &'a str,
    /// Absolute position of the Reference text (default: right of anchor).
    pub ref_at: Option<(f64, f64)>,
    /// Absolute position of the Value text (default: right of anchor).
    pub value_at: Option<(f64, f64)>,
    /// Visual justification of the Reference (the writer compensates for
    /// rotation/mirror so the render matches).
    pub ref_justify: Justify,
    pub value_justify: Justify,
    pub hide_ref: bool,
    pub hide_value: bool,
}

impl<'a> PlaceSymbol<'a> {
    pub fn new(lib_id: &'a str, reference: &'a str, value: &'a str, at: (f64, f64)) -> Self {
        Self {
            lib_id,
            reference,
            value,
            at,
            rotation: 0,
            mirror: None,
            unit: 1,
            dnp: false,
            footprint: None,
            datasheet: None,
            description: None,
            mpn: None,
            uuid_key: reference,
            ref_at: None,
            value_at: None,
            ref_justify: &["left"],
            value_justify: &["left"],
            hide_ref: false,
            hide_value: false,
        }
    }
}

/// One pin on a sub-sheet block.
#[derive(Debug, Clone)]
pub struct SheetRefPin {
    pub name: String,
    pub direction: PortDirection,
    /// Position on the sheet block outline (grid-aligned).
    pub at: (f64, f64),
}

/// A sub-sheet block placed on this sheet.
#[derive(Debug, Clone)]
pub struct SheetRef {
    pub file_name: String,
    pub title: String,
    pub at: (f64, f64),
    pub size: (f64, f64),
    pub pins: Vec<SheetRefPin>,
    /// Page number recorded in the sheet instance block ("2", "3", ...).
    pub page: String,
}

/// Options for creating a [`SheetWriter`].
#[derive(Debug, Clone)]
pub struct SheetWriterOptions {
    /// Sheet title (title block).
    pub title: String,
    /// Output file name; also the seed of all deterministic UUIDs.
    pub file_name: String,
    /// KiCad paper name ("A4", "A3", ...).
    pub paper: String,
    pub generator: String,
    pub generator_version: String,
    /// KiCad project name (base name of the root file, no extension).
    pub project_name: String,
    /// `true` for the root sheet: emits `(sheet_instances (path "/" ...))`.
    pub is_root: bool,
    /// Hierarchical instance path prefix for symbols on this sheet
    /// (`/<root-uuid>` for the root, `/<root-uuid>/<sheet-uuid>` for a
    /// child). `None` derives the root prefix from this sheet's own UUID.
    pub instance_path_prefix: Option<String>,
    /// Starting offset of the `#PWRnn` / `#FLGnn` reference counters so
    /// power references stay unique across the sheets of one project.
    pub power_ref_offset: u32,
}

struct LibEntry {
    sexpr: Sexpr,
    pin_numbers: Vec<String>,
}

/// Writer building one `.kicad_sch` document.
pub struct SheetWriter {
    title: String,
    paper: String,
    generator: String,
    generator_version: String,
    project_name: String,
    is_root: bool,
    instance_path_prefix: Option<String>,
    power_ref_offset: u32,
    uuids: UuidGen,
    libs: BTreeMap<String, LibEntry>,
    /// Maps a power/ground net name to its generated lib id.
    power_lib_ids: BTreeMap<String, String>,
    junctions: Vec<Sexpr>,
    no_connects: Vec<Sexpr>,
    wires: Vec<Sexpr>,
    labels: Vec<Sexpr>,
    global_labels: Vec<Sexpr>,
    hier_labels: Vec<Sexpr>,
    texts: Vec<Sexpr>,
    graphics: Vec<Sexpr>,
    symbols: Vec<Sexpr>,
    sheets: Vec<Sexpr>,
    pwr_count: u32,
    flag_count: u32,
}

impl SheetWriter {
    pub fn new(options: SheetWriterOptions) -> Self {
        Self {
            title: options.title,
            paper: options.paper,
            generator: options.generator,
            generator_version: options.generator_version,
            project_name: options.project_name,
            is_root: options.is_root,
            instance_path_prefix: options.instance_path_prefix,
            power_ref_offset: options.power_ref_offset,
            uuids: UuidGen::new(options.file_name),
            libs: BTreeMap::new(),
            power_lib_ids: BTreeMap::new(),
            junctions: Vec::new(),
            no_connects: Vec::new(),
            wires: Vec::new(),
            labels: Vec::new(),
            global_labels: Vec::new(),
            hier_labels: Vec::new(),
            texts: Vec::new(),
            graphics: Vec::new(),
            symbols: Vec::new(),
            sheets: Vec::new(),
            pwr_count: 0,
            flag_count: 0,
        }
    }

    /// Stable UUID of this sheet document.
    pub fn root_uuid(&self) -> String {
        self.uuids.root_uuid()
    }

    /// Hierarchical path prefix used in symbol instance blocks.
    fn path_prefix(&self) -> String {
        self.instance_path_prefix
            .clone()
            .unwrap_or_else(|| format!("/{}", self.root_uuid()))
    }

    /// Public accessor of the hierarchical path prefix (child sheet
    /// prefixes are built from the parent prefix plus the block uuid).
    pub fn path_prefix_public(&self) -> String {
        self.path_prefix()
    }

    // ------------------------------------------------------------------
    // Embedded library
    // ------------------------------------------------------------------

    pub fn has_lib_symbol(&self, lib_id: &str) -> bool {
        self.libs.contains_key(lib_id)
    }

    /// Register a parsed library symbol (idempotent by lib id).
    pub fn add_lib_symbol(&mut self, geom: &crate::geometry::SymbolGeom) {
        self.libs
            .entry(geom.lib_id.clone())
            .or_insert_with(|| LibEntry {
                sexpr: geom.sexpr.clone(),
                pin_numbers: geom.pins.iter().map(|p| p.number.clone()).collect(),
            });
    }

    // ------------------------------------------------------------------
    // Sheet elements
    // ------------------------------------------------------------------

    /// Place a symbol instance. The lib symbol must be registered first.
    pub fn place_symbol(&mut self, opts: PlaceSymbol<'_>) -> anyhow::Result<()> {
        let lib = self
            .libs
            .get(opts.lib_id)
            .ok_or_else(|| anyhow::anyhow!("symbol \"{}\" not registered", opts.lib_id))?;
        let pin_numbers = lib.pin_numbers.clone();

        let (x, y) = (round4(opts.at.0), round4(opts.at.1));
        let text_angle = text_angle(opts.rotation);
        let ref_at = opts.ref_at.unwrap_or((x + 2.54, y - 1.27));
        let value_at = opts.value_at.unwrap_or((x + 2.54, y + 1.27));
        let uuid = self.uuids.next(&["symbol", opts.uuid_key]);

        let mut items = vec![
            sym("symbol"),
            node("lib_id", vec![sstr(opts.lib_id)]),
            node("at", vec![num(x), num(y), int(opts.rotation as i64)]),
        ];
        if let Some(axis) = opts.mirror {
            items.push(node("mirror", vec![sym(axis.as_comment_value())]));
        }
        items.extend([
            node("unit", vec![int(opts.unit as i64)]),
            node("exclude_from_sim", vec![sym("no")]),
            node("in_bom", vec![sym("yes")]),
            node("on_board", vec![sym("yes")]),
            node("dnp", vec![sym(if opts.dnp { "yes" } else { "no" })]),
            node("uuid", vec![sstr(&uuid)]),
            property(
                "Reference",
                opts.reference,
                ref_at,
                text_angle,
                &render_justify(opts.ref_justify, opts.rotation, opts.mirror),
                opts.hide_ref,
            ),
            property(
                "Value",
                opts.value,
                value_at,
                text_angle,
                &render_justify(opts.value_justify, opts.rotation, opts.mirror),
                opts.hide_value,
            ),
        ]);
        for (name, value) in [
            ("Footprint", opts.footprint),
            ("Datasheet", opts.datasheet),
            ("Description", opts.description),
            ("MPN", opts.mpn),
        ] {
            if let Some(value) = value {
                items.push(property(name, value, (x, y), 0, &[], true));
            }
        }
        for number in &pin_numbers {
            let pin_uuid = self.uuids.next(&["pin", opts.uuid_key, number]);
            items.push(node(
                "pin",
                vec![sstr(number), node("uuid", vec![sstr(&pin_uuid)])],
            ));
        }
        items.push(self.instances_block(&[
            node("reference", vec![sstr(opts.reference)]),
            node("unit", vec![int(opts.unit as i64)]),
        ]));

        self.symbols.push(Sexpr::list(items));
        Ok(())
    }

    /// Polyline wire: N points produce N-1 `(wire ...)` segments.
    pub fn add_wire(&mut self, points: &[(f64, f64)]) {
        assert!(points.len() >= 2, "add_wire requires at least 2 points");
        for pair in points.windows(2) {
            let (x1, y1) = (round4(pair[0].0), round4(pair[0].1));
            let (x2, y2) = (round4(pair[1].0), round4(pair[1].1));
            let uuid = self
                .uuids
                .next(&["wire", &format!("{x1},{y1}"), &format!("{x2},{y2}")]);
            self.wires.push(node(
                "wire",
                vec![
                    node(
                        "pts",
                        vec![
                            node("xy", vec![num(x1), num(y1)]),
                            node("xy", vec![num(x2), num(y2)]),
                        ],
                    ),
                    stroke(),
                    node("uuid", vec![sstr(&uuid)]),
                ],
            ));
        }
    }

    /// Junction dot — required on 3-way tees, forbidden on 2-way corners.
    pub fn add_junction(&mut self, at: (f64, f64)) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["junction", &format!("{x},{y}")]);
        self.junctions.push(node(
            "junction",
            vec![
                node("at", vec![num(x), num(y)]),
                node("diameter", vec![int(0)]),
                node("color", vec![int(0), int(0), int(0), int(0)]),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// No-connect marker on an intentionally unused pin end.
    pub fn add_no_connect(&mut self, at: (f64, f64)) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["no_connect", &format!("{x},{y}")]);
        self.no_connects.push(node(
            "no_connect",
            vec![
                node("at", vec![num(x), num(y)]),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Local net label. Rotation follows the wire direction; the text stays
    /// readable (KiCad renders 180 as horizontal with flipped anchoring).
    pub fn add_net_label(&mut self, name: &str, at: (f64, f64), rotation: i32) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["label", name, &format!("{x},{y}")]);
        self.labels.push(node(
            "label",
            vec![
                sstr(name),
                node("at", vec![num(x), num(y), int(rotation as i64)]),
                effects(&label_justify(rotation), false),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Global label (design-wide net). Reserved for fallback rails and
    /// high-fanout intra-sheet nets — never for sheet ports (risk of
    /// shorting sibling sheet instances): use `add_hier_label` instead.
    pub fn add_global_label(
        &mut self,
        name: &str,
        at: (f64, f64),
        rotation: i32,
        direction: PortDirection,
    ) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self
            .uuids
            .next(&["global_label", name, &format!("{x},{y}")]);
        self.global_labels.push(node(
            "global_label",
            vec![
                sstr(name),
                node("shape", vec![sym(direction.shape())]),
                node("at", vec![num(x), num(y), int(rotation as i64)]),
                effects(&flag_label_justify(rotation), false),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Hierarchical label — child side of a sheet pin (same-name contract).
    pub fn add_hier_label(
        &mut self,
        name: &str,
        direction: PortDirection,
        at: (f64, f64),
        rotation: i32,
    ) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["hier_label", name, &format!("{x},{y}")]);
        self.hier_labels.push(node(
            "hierarchical_label",
            vec![
                sstr(name),
                node("shape", vec![sym(direction.shape())]),
                node("at", vec![num(x), num(y), int(rotation as i64)]),
                effects(&flag_label_justify(rotation), false),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Free text annotation.
    pub fn add_text(&mut self, text: &str, at: (f64, f64), rotation: i32) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["text", text, &format!("{x},{y}")]);
        self.texts.push(node(
            "text",
            vec![
                sstr(text),
                node("exclude_from_sim", vec![sym("no")]),
                node("at", vec![num(x), num(y), int(rotation as i64)]),
                effects(&[], false),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Graphic zone outline: a thin dashed rectangle on the notes/graphic
    /// layer grouping a functional/decoupling/ERC area. Purely visual — a
    /// schematic graphic shape carries no connectivity, so it never affects
    /// the netlist or ERC. Drawn discreetly (hairline, dashed, mid-gray).
    pub fn add_zone_rect(&mut self, start: (f64, f64), end: (f64, f64)) {
        let (x1, y1) = (round4(start.0), round4(start.1));
        let (x2, y2) = (round4(end.0), round4(end.1));
        let uuid = self
            .uuids
            .next(&["zone", &format!("{x1},{y1}"), &format!("{x2},{y2}")]);
        self.graphics.push(node(
            "rectangle",
            vec![
                node("start", vec![num(x1), num(y1)]),
                node("end", vec![num(x2), num(y2)]),
                node(
                    "stroke",
                    vec![
                        node("width", vec![num(0.127)]),
                        node("type", vec![sym("dash")]),
                        node("color", vec![int(130), int(130), int(130), num(1.0)]),
                    ],
                ),
                node("fill", vec![node("type", vec![sym("none")])]),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Discreet title for a zone, left/bottom anchored just above its top-left
    /// corner on the graphic layer. Purely visual — no connectivity.
    pub fn add_zone_title(&mut self, text: &str, at: (f64, f64)) {
        let (x, y) = (round4(at.0), round4(at.1));
        let uuid = self.uuids.next(&["ztitle", text, &format!("{x},{y}")]);
        self.texts.push(node(
            "text",
            vec![
                sstr(text),
                node("exclude_from_sim", vec![sym("no")]),
                node("at", vec![num(x), num(y), int(0)]),
                effects(&["left", "bottom"], false),
                node("uuid", vec![sstr(&uuid)]),
            ],
        ));
    }

    /// Power/ground symbol. The Value creates the global net — no label
    /// needed. `down` selects the ground glyph (triangle below the origin,
    /// pin pointing up into the wire); otherwise an upward VCC-style arrow.
    /// Returns the allocated `#PWRnn` reference.
    pub fn add_power_symbol(&mut self, net_name: &str, at: (f64, f64), down: bool) -> String {
        let lib_id = self.power_lib_id(net_name);
        if !self.libs.contains_key(&lib_id) {
            let entry = build_power_lib_symbol(&lib_id, net_name, down);
            self.libs.insert(lib_id.clone(), entry);
        }
        self.pwr_count += 1;
        let reference = format!("#PWR{:02}", self.power_ref_offset + self.pwr_count);
        let (x, y) = (round4(at.0), round4(at.1));
        // Value sits beyond the glyph, in the outgoing direction.
        let value_at = if down { (x, y + 3.81) } else { (x, y - 3.556) };
        let ref_at = if down { (x, y + 6.35) } else { (x, y - 6.35) };

        let uuid = self.uuids.next(&["symbol", &reference]);
        let pin_uuid = self.uuids.next(&["pin", &reference, "1"]);
        let mut items = vec![
            sym("symbol"),
            node("lib_id", vec![sstr(&lib_id)]),
            node("at", vec![num(x), num(y), int(0)]),
            node("unit", vec![int(1)]),
            node("exclude_from_sim", vec![sym("no")]),
            node("in_bom", vec![sym("yes")]),
            node("on_board", vec![sym("yes")]),
            node("dnp", vec![sym("no")]),
            node("uuid", vec![sstr(&uuid)]),
            property("Reference", &reference, ref_at, 0, &[], true),
            property("Value", net_name, value_at, 0, &[], false),
            node("pin", vec![sstr("1"), node("uuid", vec![sstr(&pin_uuid)])]),
        ];
        items.push(self.instances_block(&[
            node("reference", vec![sstr(&reference)]),
            node("unit", vec![int(1)]),
        ]));
        self.symbols.push(Sexpr::list(items));
        reference
    }

    /// PWR_FLAG (hidden `power_out` pin): place exactly one per power/ground
    /// net not driven by a real `power_out` pin, otherwise `kicad-cli sch
    /// erc` reports `power_pin_not_driven` (error). Never place two on the
    /// same net. Returns the allocated `#FLGnn` reference.
    pub fn add_pwr_flag(&mut self, at: (f64, f64), rot: i32) -> String {
        let lib_id = format!("{POWER_LIB_NICKNAME}:PWR_FLAG");
        if !self.libs.contains_key(&lib_id) {
            let entry = build_pwr_flag_lib_symbol(&lib_id);
            self.libs.insert(lib_id.clone(), entry);
        }
        self.flag_count += 1;
        let reference = format!("#FLG{:02}", self.power_ref_offset + self.flag_count);
        let (x, y) = (round4(at.0), round4(at.1));

        let uuid = self.uuids.next(&["symbol", &reference]);
        let pin_uuid = self.uuids.next(&["pin", &reference, "1"]);
        let mut items = vec![
            sym("symbol"),
            node("lib_id", vec![sstr(&lib_id)]),
            node("at", vec![num(x), num(y), int(rot as i64)]),
            node("unit", vec![int(1)]),
            node("exclude_from_sim", vec![sym("no")]),
            node("in_bom", vec![sym("yes")]),
            node("on_board", vec![sym("yes")]),
            node("dnp", vec![sym("no")]),
            node("uuid", vec![sstr(&uuid)]),
            property("Reference", &reference, (x, y - 1.905), 0, &[], true),
            property("Value", "PWR_FLAG", (x, y - 3.81), 0, &[], true),
            node("pin", vec![sstr("1"), node("uuid", vec![sstr(&pin_uuid)])]),
        ];
        items.push(self.instances_block(&[
            node("reference", vec![sstr(&reference)]),
            node("unit", vec![int(1)]),
        ]));
        self.symbols.push(Sexpr::list(items));
        reference
    }

    /// Reference block to a sub-sheet. The generator must guarantee that the
    /// referenced file is produced (dangling `Sheetfile` = silent blank page).
    pub fn add_sheet_ref(&mut self, opts: &SheetRef) -> String {
        let (x, y) = (round4(opts.at.0), round4(opts.at.1));
        let (w, h) = (round4(opts.size.0), round4(opts.size.1));
        let sheet_uuid = self.uuids.next(&["sheet", &opts.file_name, &opts.title]);

        let mut items = vec![
            sym("sheet"),
            node("at", vec![num(x), num(y)]),
            node("size", vec![num(w), num(h)]),
            node("exclude_from_sim", vec![sym("no")]),
            node("in_bom", vec![sym("yes")]),
            node("on_board", vec![sym("yes")]),
            node("dnp", vec![sym("no")]),
            node(
                "stroke",
                vec![
                    node("width", vec![num(0.1524)]),
                    node("type", vec![sym("solid")]),
                ],
            ),
            node(
                "fill",
                vec![node("color", vec![int(0), int(0), int(0), int(0)])],
            ),
            node("uuid", vec![sstr(&sheet_uuid)]),
            property(
                "Sheetname",
                &opts.title,
                (x, y - 0.7116),
                0,
                &["left", "bottom"],
                false,
            ),
            property(
                "Sheetfile",
                &opts.file_name,
                (x, y + h + 0.5842),
                0,
                &["left", "top"],
                false,
            ),
        ];

        for pin in &opts.pins {
            let rotation = infer_sheet_pin_rotation(pin.at, x, y, w);
            // Verified with kicad-cli 10.0.3 SVG probes: the pin NAME must
            // extend toward the INSIDE of the rectangle — right edge (rot 0)
            // and top (rot 90) justify right; left edge (rot 180) and bottom
            // (rot 270) justify left. The opposite pushes the text outside.
            let justify: Justify = if rotation == 0 || rotation == 90 {
                &["right"]
            } else {
                &["left"]
            };
            let uuid = self.uuids.next(&["sheet_pin", &opts.file_name, &pin.name]);
            items.push(node(
                "pin",
                vec![
                    sstr(&pin.name),
                    sym(pin.direction.shape()),
                    node(
                        "at",
                        vec![
                            num(round4(pin.at.0)),
                            num(round4(pin.at.1)),
                            int(rotation as i64),
                        ],
                    ),
                    effects(justify, false),
                    node("uuid", vec![sstr(&uuid)]),
                ],
            ));
        }

        items.push(self.instances_block(&[node("page", vec![sstr(&opts.page)])]));
        self.sheets.push(Sexpr::list(items));
        sheet_uuid
    }

    // ------------------------------------------------------------------
    // Serialization
    // ------------------------------------------------------------------

    /// Serialize the complete `.kicad_sch` document (deterministic for equal
    /// content and seed).
    pub fn serialize(&self) -> String {
        let mut items = vec![
            sym("kicad_sch"),
            node("version", vec![int(KICAD_SCH_VERSION)]),
            node("generator", vec![sstr(&self.generator)]),
            node("generator_version", vec![sstr(&self.generator_version)]),
            node("uuid", vec![sstr(&self.root_uuid())]),
            node("paper", vec![sstr(&self.paper)]),
            node("title_block", vec![node("title", vec![sstr(&self.title)])]),
            Sexpr::list(
                std::iter::once(sym("lib_symbols"))
                    .chain(self.libs.values().map(|l| l.sexpr.clone()))
                    .collect(),
            ),
        ];
        items.extend(self.junctions.iter().cloned());
        items.extend(self.no_connects.iter().cloned());
        items.extend(self.wires.iter().cloned());
        items.extend(self.labels.iter().cloned());
        items.extend(self.global_labels.iter().cloned());
        items.extend(self.hier_labels.iter().cloned());
        items.extend(self.texts.iter().cloned());
        // Zone outlines before symbols: they draw as a backdrop, symbols on top.
        items.extend(self.graphics.iter().cloned());
        items.extend(self.symbols.iter().cloned());
        items.extend(self.sheets.iter().cloned());
        if self.is_root {
            items.push(node(
                "sheet_instances",
                vec![node("path", vec![sstr("/"), node("page", vec![sstr("1")])])],
            ));
        }
        items.push(node("embedded_fonts", vec![sym("no")]));

        format_tree(&Sexpr::list(items), FormatMode::Normal)
    }

    fn instances_block(&self, path_children: &[Sexpr]) -> Sexpr {
        let mut path_items = vec![sym("path"), sstr(&self.path_prefix())];
        path_items.extend(path_children.iter().cloned());
        node(
            "instances",
            vec![node(
                "project",
                vec![sstr(&self.project_name), Sexpr::list(path_items)],
            )],
        )
    }

    /// Lib id of the generated power symbol for a net (deduplicated when two
    /// distinct net names sanitize to the same identifier).
    fn power_lib_id(&mut self, net_name: &str) -> String {
        if let Some(id) = self.power_lib_ids.get(net_name) {
            return id.clone();
        }
        let base = sanitize_name(net_name);
        let mut candidate = format!("{POWER_LIB_NICKNAME}:{base}");
        let mut n = 1;
        while self.power_lib_ids.values().any(|v| *v == candidate) {
            n += 1;
            candidate = format!("{POWER_LIB_NICKNAME}:{base}_{n}");
        }
        self.power_lib_ids
            .insert(net_name.to_string(), candidate.clone());
        candidate
    }
}

// ----------------------------------------------------------------------
// S-expression helpers
// ----------------------------------------------------------------------

fn sym(s: &str) -> Sexpr {
    Sexpr::symbol(s)
}

fn sstr(s: &str) -> Sexpr {
    Sexpr::string(s)
}

fn num(v: f64) -> Sexpr {
    Sexpr::float(round4(v))
}

fn int(v: i64) -> Sexpr {
    Sexpr::int(v)
}

fn node(name: &str, mut items: Vec<Sexpr>) -> Sexpr {
    let mut all = Vec::with_capacity(items.len() + 1);
    all.push(sym(name));
    all.append(&mut items);
    Sexpr::list(all)
}

fn font() -> Sexpr {
    node("font", vec![node("size", vec![num(1.27), num(1.27)])])
}

fn effects(justify: &[&str], hide: bool) -> Sexpr {
    let mut items = vec![font()];
    if !justify.is_empty() {
        items.push(node("justify", justify.iter().map(|j| sym(j)).collect()));
    }
    if hide {
        items.push(node("hide", vec![sym("yes")]));
    }
    node("effects", items)
}

fn stroke() -> Sexpr {
    node(
        "stroke",
        vec![
            node("width", vec![int(0)]),
            node("type", vec![sym("default")]),
        ],
    )
}

fn property(
    name: &str,
    value: &str,
    at: (f64, f64),
    angle: i32,
    justify: &[&str],
    hide: bool,
) -> Sexpr {
    node(
        "property",
        vec![
            sstr(name),
            sstr(value),
            node(
                "at",
                vec![num(round4(at.0)), num(round4(at.1)), int(angle as i64)],
            ),
            effects(justify, hide),
        ],
    )
}

/// Empirical mapping: text angle keeping a property horizontal while the
/// symbol rotates (kicad-cli 10.0.3 SVG probe: rendered angle is symbol
/// rotation + property angle; with these values it is 0 except at symbol
/// rotation 180 where KiCad re-uprights the text).
fn text_angle(rotation: i32) -> i32 {
    match rotation.rem_euclid(360) {
        90 => 270,
        270 => 90,
        _ => 0,
    }
}

/// Convert a VISUAL justification into the one to WRITE for a property of a
/// rotated/mirrored symbol. Empirical (kicad-cli 10.0.3 SVG probe): with
/// [`text_angle`] the render honors justification as-is at 0/90/270 but
/// INVERTS it (horizontal AND vertical) at 180; a mirror additionally flips
/// the corresponding axis.
fn render_justify(
    justify: &[&str],
    rotation: i32,
    mirror: Option<MirrorAxis>,
) -> Vec<&'static str> {
    let rot180 = rotation.rem_euclid(360) == 180;
    let h_flip = rot180 != matches!(mirror, Some(MirrorAxis::Y));
    let v_flip = rot180 != matches!(mirror, Some(MirrorAxis::X));
    justify
        .iter()
        .map(|j| match *j {
            "left" => {
                if h_flip {
                    "right"
                } else {
                    "left"
                }
            }
            "right" => {
                if h_flip {
                    "left"
                } else {
                    "right"
                }
            }
            "top" => {
                if v_flip {
                    "bottom"
                } else {
                    "top"
                }
            }
            _ => {
                if v_flip {
                    "top"
                } else {
                    "bottom"
                }
            }
        })
        .collect()
}

/// Local net-label anchoring by rotation (labels are written so the text
/// sits above its carrying wire, hence the `bottom` component).
fn label_justify(rotation: i32) -> Vec<&'static str> {
    if rotation.rem_euclid(360) == 180 || rotation.rem_euclid(360) == 270 {
        vec!["right", "bottom"]
    } else {
        vec!["left", "bottom"]
    }
}

/// Flag-label anchoring by rotation, for the directional labels (global and
/// hierarchical) whose text is centered inside a flag glyph. Only the reading
/// side matters — no vertical component (matches KiCad's autoplaced ports):
/// rotation 180/270 reads leftward (`right` justify), otherwise rightward.
fn flag_label_justify(rotation: i32) -> Vec<&'static str> {
    if rotation.rem_euclid(360) == 180 || rotation.rem_euclid(360) == 270 {
        vec!["right"]
    } else {
        vec!["left"]
    }
}

fn infer_sheet_pin_rotation(at: (f64, f64), x: f64, y: f64, w: f64) -> i32 {
    if (at.0 - x).abs() < 0.01 {
        180 // left edge
    } else if (at.0 - (x + w)).abs() < 0.01 {
        0 // right edge
    } else if (at.1 - y).abs() < 0.01 {
        90 // top edge
    } else {
        270 // bottom edge
    }
}

fn sanitize_name(name: &str) -> String {
    let out: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '_' | '+' | '.' | '-') {
                c
            } else {
                '_'
            }
        })
        .collect();
    if out.is_empty() {
        "NET".to_string()
    } else {
        out
    }
}

// ----------------------------------------------------------------------
// Generated power symbols
// ----------------------------------------------------------------------

fn polyline(points: &[(f64, f64)]) -> Sexpr {
    node(
        "polyline",
        vec![
            node(
                "pts",
                points
                    .iter()
                    .map(|(x, y)| node("xy", vec![num(*x), num(*y)]))
                    .collect(),
            ),
            stroke(),
            node("fill", vec![node("type", vec![sym("none")])]),
        ],
    )
}

fn unit_base_name(lib_id: &str) -> &str {
    lib_id.split_once(':').map(|(_, n)| n).unwrap_or(lib_id)
}

/// Drawn power symbol: ground triangle (down) or VCC-style up arrow.
/// `(power global)` makes the Value create the global net.
fn build_power_lib_symbol(lib_id: &str, net_name: &str, down: bool) -> LibEntry {
    let base = unit_base_name(lib_id);
    let graphics: Vec<Sexpr> = if down {
        vec![polyline(&[
            (0.0, 0.0),
            (0.0, -1.27),
            (1.27, -1.27),
            (0.0, -2.54),
            (-1.27, -1.27),
            (0.0, -1.27),
        ])]
    } else {
        vec![
            polyline(&[(-0.762, 1.27), (0.0, 2.54)]),
            polyline(&[(0.0, 2.54), (0.762, 1.27)]),
            polyline(&[(0.0, 0.0), (0.0, 2.54)]),
        ]
    };
    let pin_angle = if down { 270 } else { 90 };
    let (ref_y, value_y) = if down { (-6.35, -3.81) } else { (-3.81, 3.556) };

    let sexpr = Sexpr::list(vec![
        sym("symbol"),
        sstr(lib_id),
        node("power", vec![sym("global")]),
        node("pin_numbers", vec![node("hide", vec![sym("yes")])]),
        node(
            "pin_names",
            vec![node("offset", vec![int(0)]), node("hide", vec![sym("yes")])],
        ),
        node("exclude_from_sim", vec![sym("no")]),
        node("in_bom", vec![sym("yes")]),
        node("on_board", vec![sym("yes")]),
        property("Reference", "#PWR", (0.0, ref_y), 0, &[], true),
        property("Value", net_name, (0.0, value_y), 0, &[], false),
        Sexpr::list(
            std::iter::once(sym("symbol"))
                .chain(std::iter::once(sstr(&format!("{base}_0_1"))))
                .chain(graphics)
                .collect(),
        ),
        node(
            "symbol",
            vec![
                sstr(&format!("{base}_1_1")),
                power_pin("power_in", pin_angle),
            ],
        ),
    ]);
    LibEntry {
        sexpr,
        pin_numbers: vec!["1".to_string()],
    }
}

fn build_pwr_flag_lib_symbol(lib_id: &str) -> LibEntry {
    let sexpr = Sexpr::list(vec![
        sym("symbol"),
        sstr(lib_id),
        node("power", vec![sym("global")]),
        node("pin_numbers", vec![node("hide", vec![sym("yes")])]),
        node(
            "pin_names",
            vec![node("offset", vec![int(0)]), node("hide", vec![sym("yes")])],
        ),
        node("exclude_from_sim", vec![sym("no")]),
        node("in_bom", vec![sym("yes")]),
        node("on_board", vec![sym("yes")]),
        property("Reference", "#FLG", (0.0, 1.905), 0, &[], true),
        property("Value", "PWR_FLAG", (0.0, 3.81), 0, &[], true),
        node(
            "symbol",
            vec![
                sstr("PWR_FLAG_0_1"),
                polyline(&[
                    (0.0, 0.0),
                    (0.0, 1.27),
                    (-1.016, 1.905),
                    (0.0, 2.54),
                    (1.016, 1.905),
                    (0.0, 1.27),
                ]),
            ],
        ),
        node(
            "symbol",
            vec![sstr("PWR_FLAG_1_1"), power_pin("power_out", 90)],
        ),
    ]);
    LibEntry {
        sexpr,
        pin_numbers: vec!["1".to_string()],
    }
}

fn power_pin(etype: &str, angle: i32) -> Sexpr {
    node(
        "pin",
        vec![
            sym(etype),
            sym("line"),
            node("at", vec![int(0), int(0), int(angle as i64)]),
            node("length", vec![int(0)]),
            node("name", vec![sstr(""), effects(&[], false)]),
            node("number", vec![sstr("1"), effects(&[], false)]),
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::parse_lib_symbol;

    const R_SMALL: &str = r#"(symbol "R_Small"
        (property "Reference" "R" (at 0 0 90) (effects (font (size 1.016 1.016))))
        (symbol "R_Small_0_1"
            (rectangle (start -0.762 1.778) (end 0.762 -1.778)
                (stroke (width 0.2032) (type default)) (fill (type none))))
        (symbol "R_Small_1_1"
            (pin passive line (at 0 2.54 270) (length 0.762)
                (name "" (effects (font (size 1.27 1.27))))
                (number "1" (effects (font (size 1.27 1.27)))))
            (pin passive line (at 0 -2.54 90) (length 0.762)
                (name "" (effects (font (size 1.27 1.27))))
                (number "2" (effects (font (size 1.27 1.27)))))))"#;

    fn writer() -> SheetWriter {
        SheetWriter::new(SheetWriterOptions {
            title: "test".into(),
            file_name: "test.kicad_sch".into(),
            paper: "A4".into(),
            generator: "pcb".into(),
            generator_version: "0.0.0".into(),
            project_name: "test".into(),
            is_root: true,
            instance_path_prefix: None,
            power_ref_offset: 0,
        })
    }

    #[test]
    fn serialization_is_deterministic() {
        let build = || {
            let mut w = writer();
            let geom = parse_lib_symbol(R_SMALL, Some("Device:R_Small")).unwrap();
            w.add_lib_symbol(&geom);
            w.place_symbol(PlaceSymbol::new(
                "Device:R_Small",
                "R1",
                "1k",
                (127.0, 63.5),
            ))
            .unwrap();
            w.add_wire(&[(127.0, 60.96), (127.0, 58.42)]);
            w.add_power_symbol("VCC", (127.0, 58.42), false);
            w.add_net_label("OUT", (127.0, 66.04), 0);
            w.serialize()
        };
        assert_eq!(build(), build());
    }

    #[test]
    fn document_structure_and_ordering() {
        let mut w = writer();
        let geom = parse_lib_symbol(R_SMALL, Some("Device:R_Small")).unwrap();
        w.add_lib_symbol(&geom);
        w.place_symbol(PlaceSymbol::new(
            "Device:R_Small",
            "R1",
            "1k",
            (127.0, 63.5),
        ))
        .unwrap();
        w.add_pwr_flag((127.0, 58.42), 0);
        let text = w.serialize();

        assert!(text.starts_with("(kicad_sch\n"));
        assert!(text.contains("(version 20250114)"));
        assert!(text.contains("(paper \"A4\")"));
        assert!(text.contains("(lib_symbols"));
        assert!(text.contains("\"Device:R_Small\""));
        assert!(text.contains(&format!("\"{POWER_LIB_NICKNAME}:PWR_FLAG\"")));
        assert!(text.contains("(reference \"R1\")"));
        assert!(text.contains("(sheet_instances"));
        assert!(text.ends_with("(embedded_fonts no)\n)\n"));
        // Pin instance uuids present for both pins.
        assert!(text.contains("(pin \"1\""));
        assert!(text.contains("(pin \"2\""));
    }

    #[test]
    fn zone_rect_emits_dashed_graphic_rectangle() {
        let mut w = writer();
        w.add_zone_rect((170.0, 20.0), (210.0, 60.0));
        let text = w.serialize();
        // A graphic rectangle on the notes layer: dashed, no fill.
        assert!(text.contains("(rectangle"));
        assert!(text.contains("(start 170 20)"));
        assert!(text.contains("(end 210 60)"));
        assert!(text.contains("(type dash)"));
        assert!(text.contains("(type none)"));
        // Deterministic across builds.
        let mut w2 = writer();
        w2.add_zone_rect((170.0, 20.0), (210.0, 60.0));
        assert_eq!(text, w2.serialize());
    }

    #[test]
    fn place_symbol_requires_registered_lib() {
        let mut w = writer();
        let err = w
            .place_symbol(PlaceSymbol::new("Device:R_Small", "R1", "1k", (0.0, 0.0)))
            .unwrap_err();
        assert!(err.to_string().contains("not registered"));
    }

    #[test]
    fn power_lib_ids_deduplicate_on_sanitize_collision() {
        let mut w = writer();
        w.add_power_symbol("N$1", (0.0, 0.0), false);
        w.add_power_symbol("N 1", (2.54, 0.0), false);
        let text = w.serialize();
        assert!(text.contains(&format!("\"{POWER_LIB_NICKNAME}:N_1\"")));
        assert!(text.contains(&format!("\"{POWER_LIB_NICKNAME}:N_1_2\"")));
        // Both Values keep the EXACT net names (the Value creates the net).
        assert!(text.contains("\"N$1\""));
        assert!(text.contains("\"N 1\""));
    }

    #[test]
    fn render_justify_flips_at_180_and_mirror() {
        assert_eq!(render_justify(&["left"], 0, None), vec!["left"]);
        assert_eq!(render_justify(&["left"], 180, None), vec!["right"]);
        assert_eq!(
            render_justify(&["left", "bottom"], 180, None),
            vec!["right", "top"]
        );
        assert_eq!(
            render_justify(&["left"], 0, Some(MirrorAxis::Y)),
            vec!["right"]
        );
        assert_eq!(
            render_justify(&["left"], 180, Some(MirrorAxis::Y)),
            vec!["left"]
        );
        assert_eq!(
            render_justify(&["bottom"], 0, Some(MirrorAxis::X)),
            vec!["top"]
        );
    }

    #[test]
    fn flag_label_justify_is_horizontal_only() {
        // Global and hierarchical labels read outward with NO vertical
        // component (their flag centers the text): rot 0/90 justify left,
        // rot 180/270 justify right — never the local-label `bottom`.
        assert_eq!(flag_label_justify(0), vec!["left"]);
        assert_eq!(flag_label_justify(180), vec!["right"]);
        assert_eq!(flag_label_justify(90), vec!["left"]);
        assert_eq!(flag_label_justify(270), vec!["right"]);
        assert!(!flag_label_justify(0).contains(&"bottom"));
        assert!(!flag_label_justify(180).contains(&"bottom"));
        // Local labels keep the `bottom` component (text above the wire).
        assert!(label_justify(0).contains(&"bottom"));
    }

    #[test]
    fn sheet_pin_rotation_and_justify() {
        let mut w = writer();
        w.add_sheet_ref(&SheetRef {
            file_name: "sub.kicad_sch".into(),
            title: "Sub".into(),
            at: (152.4, 63.5),
            size: (25.4, 12.7),
            pins: vec![
                SheetRefPin {
                    name: "L".into(),
                    direction: PortDirection::Input,
                    at: (152.4, 66.04),
                },
                SheetRefPin {
                    name: "R".into(),
                    direction: PortDirection::Output,
                    at: (177.8, 66.04),
                },
            ],
            page: "2".into(),
        });
        let text = w.serialize();
        // Left edge pin: rotation 180, justify left; right edge: 0, right.
        assert!(text.contains("(at 152.4 66.04 180)"));
        assert!(text.contains("(at 177.8 66.04 0)"));
        assert!(text.contains("(page \"2\")"));
        assert!(text.contains("\"Sheetname\""));
        assert!(text.contains("\"Sheetfile\""));
    }
}
