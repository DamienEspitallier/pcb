//! Symbol geometry extracted from raw `.kicad_sym` s-expression blocks.
//!
//! Zener components carry their KiCad symbol verbatim in the `__symbol_value`
//! attribute. This module parses that block once to expose what the layout
//! engine needs: pin connection points, pin orientations and the body
//! bounding box — all in symbol-local coordinates (millimeters, +Y up, origin
//! at the symbol anchor).
//!
//! Golden rule (wire/pin coincidence): never derive a wire endpoint from
//! layout arithmetic. Use [`SymbolGeom::pin_position`], which applies the
//! local pin geometry (mirror on library axes, then CCW rotation, then the
//! lib-to-sheet Y flip).

use anyhow::{Context, Result, bail};
use pcb_sch::position::MirrorAxis;
use pcb_sexpr::{Sexpr, SexprKind};

use crate::round4;

/// One pin of a library symbol, in symbol-local coordinates (+Y up).
#[derive(Debug, Clone)]
pub struct PinGeom {
    /// Pin number (KiCad "number", equals the footprint pad in Zener parts).
    pub number: String,
    /// Pin name (signal-ish label, may be empty).
    pub name: String,
    /// Electrical type: passive, input, output, power_in, power_out, ...
    pub etype: String,
    /// Connection point (this is where wires must land).
    pub x: f64,
    pub y: f64,
    /// Orientation in degrees: the direction the pin body points, i.e. from
    /// the connection point toward the symbol body (0 = +X, 90 = +Y, ...).
    pub angle: i32,
    /// Pin body length.
    pub length: f64,
    /// Hidden pin (stacked duplicates, internal power pins).
    pub hidden: bool,
}

/// Axis-aligned bounding box in symbol-local coordinates (+Y up).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BBox {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
}

impl BBox {
    pub const EMPTY: BBox = BBox {
        x1: f64::INFINITY,
        y1: f64::INFINITY,
        x2: f64::NEG_INFINITY,
        y2: f64::NEG_INFINITY,
    };

    pub fn is_empty(&self) -> bool {
        self.x1 > self.x2 || self.y1 > self.y2
    }

    pub fn include(&mut self, x: f64, y: f64) {
        self.x1 = self.x1.min(x);
        self.y1 = self.y1.min(y);
        self.x2 = self.x2.max(x);
        self.y2 = self.y2.max(y);
    }

    pub fn union(&mut self, other: &BBox) {
        if !other.is_empty() {
            self.include(other.x1, other.y1);
            self.include(other.x2, other.y2);
        }
    }

    pub fn width(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            self.x2 - self.x1
        }
    }

    pub fn height(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            self.y2 - self.y1
        }
    }

    /// Fallback box for symbols without graphics.
    fn or_default(self) -> BBox {
        if self.is_empty() {
            BBox {
                x1: -2.54,
                y1: -2.54,
                x2: 2.54,
                y2: 2.54,
            }
        } else {
            self
        }
    }
}

/// A parsed library symbol: the (renamed) raw s-expression ready to embed in
/// `(lib_symbols ...)` plus the geometry the layout engine needs.
#[derive(Debug, Clone)]
pub struct SymbolGeom {
    /// Full lib id, e.g. `Device:R_Small` (first string atom of the block).
    pub lib_id: String,
    /// The s-expression tree of the `(symbol ...)` block, already renamed.
    pub sexpr: Sexpr,
    /// All pins (visible and hidden), symbol-local coordinates.
    pub pins: Vec<PinGeom>,
    /// Bounding box of the drawn body (graphics only, no pins).
    pub body_bbox: BBox,
    /// Bounding box including pin connection points.
    pub full_bbox: BBox,
}

impl SymbolGeom {
    pub fn pin(&self, number: &str) -> Option<&PinGeom> {
        self.pins.iter().find(|p| p.number == number)
    }

    /// Sheet coordinates (Y down) of a pin's connection point for an instance
    /// placed at `at` with `rotation` (CCW degrees) and optional mirror.
    ///
    /// Transform order (validated against kicad-cli renders in the PoC):
    /// mirror on library axes, then CCW rotation, then Y flip (library +Y up
    /// to sheet +Y down).
    pub fn pin_position(
        &self,
        number: &str,
        at: (f64, f64),
        rotation: i32,
        mirror: Option<MirrorAxis>,
    ) -> Option<(f64, f64)> {
        let pin = self.pin(number)?;
        let (lx, ly) = apply_mirror(pin.x, pin.y, mirror);
        let (rx, ry) = rotate_ccw(lx, ly, rotation);
        Some((round4(at.0 + rx), round4(at.1 - ry)))
    }

    /// Outward unit direction of a pin on the sheet (Y down): the direction
    /// pointing away from the symbol body, i.e. where a stub wire should go.
    pub fn pin_outward(
        &self,
        number: &str,
        rotation: i32,
        mirror: Option<MirrorAxis>,
    ) -> Option<(f64, f64)> {
        let pin = self.pin(number)?;
        // Pin angle points toward the body; outward is the opposite.
        let outward = (pin.angle + 180).rem_euclid(360);
        let (dx, dy) = match outward {
            0 => (1.0, 0.0),
            90 => (0.0, 1.0),
            180 => (-1.0, 0.0),
            270 => (0.0, -1.0),
            _ => (1.0, 0.0),
        };
        let (mx, my) = apply_mirror(dx, dy, mirror);
        let (rx, ry) = rotate_ccw(mx, my, rotation);
        // Lib +Y up becomes sheet -Y (up on screen).
        Some((round4(rx), round4(-ry)))
    }
}

fn apply_mirror(x: f64, y: f64, mirror: Option<MirrorAxis>) -> (f64, f64) {
    match mirror {
        Some(MirrorAxis::X) => (x, -y),
        Some(MirrorAxis::Y) => (-x, y),
        None => (x, y),
    }
}

fn rotate_ccw(x: f64, y: f64, rotation: i32) -> (f64, f64) {
    match rotation.rem_euclid(360) {
        0 => (x, y),
        90 => (-y, x),
        180 => (-x, -y),
        270 => (y, -x),
        other => {
            // Non-quadrant rotations never occur in KiCad schematics; fall
            // back to identity rather than accumulating float error.
            log::warn!("unsupported symbol rotation {other}, treating as 0");
            (x, y)
        }
    }
}

/// Parse a raw `(symbol "NAME" ...)` block from a `.kicad_sym` library (as
/// stored in the `__symbol_value` attribute) and extract its geometry.
///
/// `rename_to` rewrites the symbol name (e.g. `R_Small` -> `Device:R_Small`)
/// so the block can be embedded under a library nickname; nested unit
/// symbols (`R_Small_0_1`, ...) keep their names per KiCad convention.
pub fn parse_lib_symbol(raw: &str, rename_to: Option<&str>) -> Result<SymbolGeom> {
    let mut root = pcb_sexpr::parse(raw).context("failed to parse symbol s-expression")?;

    let items = root.as_list_mut().context("symbol block is not a list")?;
    if items.first().and_then(Sexpr::as_sym) != Some("symbol") {
        bail!("expected a (symbol ...) block");
    }
    let name_atom = items.get_mut(1).context("(symbol ...) block has no name")?;
    let SexprKind::String(name) = &mut name_atom.kind else {
        bail!("(symbol ...) name is not a string");
    };
    if let Some(new_name) = rename_to {
        *name = new_name.to_string();
    }
    let lib_id = name.clone();

    let mut pins = Vec::new();
    let mut body_bbox = BBox::EMPTY;
    collect_geometry(&root, &mut pins, &mut body_bbox);
    let body_bbox = body_bbox.or_default();

    let mut full_bbox = body_bbox;
    for pin in &pins {
        full_bbox.include(pin.x, pin.y);
    }

    Ok(SymbolGeom {
        lib_id,
        sexpr: root,
        pins,
        body_bbox,
        full_bbox,
    })
}

/// Recursively collect pins and graphic extents from a symbol tree.
fn collect_geometry(node: &Sexpr, pins: &mut Vec<PinGeom>, bbox: &mut BBox) {
    let Some(items) = node.as_list() else { return };
    let tag = items.first().and_then(Sexpr::as_sym);

    match tag {
        Some("pin") => {
            if let Some(pin) = parse_pin(items) {
                pins.push(pin);
            }
            return;
        }
        Some("rectangle") => {
            for key in ["start", "end"] {
                if let Some((x, y)) = point_of(items, key) {
                    bbox.include(x, y);
                }
            }
            return;
        }
        Some("polyline") | Some("bezier") => {
            if let Some(pts) = pcb_sexpr::find_child_list(items, "pts") {
                for xy in pcb_sexpr::find_all_child_lists(pts, "xy") {
                    if let (Some(x), Some(y)) = (num_at(xy, 1), num_at(xy, 2)) {
                        bbox.include(x, y);
                    }
                }
            }
            return;
        }
        Some("circle") => {
            if let (Some((cx, cy)), Some(r)) = (
                point_of(items, "center"),
                pcb_sexpr::find_child_list(items, "radius").and_then(|r| num_at(r, 1)),
            ) {
                bbox.include(cx - r, cy - r);
                bbox.include(cx + r, cy + r);
            }
            return;
        }
        Some("arc") => {
            for key in ["start", "mid", "end"] {
                if let Some((x, y)) = point_of(items, key) {
                    bbox.include(x, y);
                }
            }
            return;
        }
        // Skip properties: their text positions must not inflate the body box.
        Some("property") => return,
        _ => {}
    }

    for child in items {
        collect_geometry(child, pins, bbox);
    }
}

fn parse_pin(items: &[Sexpr]) -> Option<PinGeom> {
    let etype = items
        .get(1)
        .and_then(Sexpr::as_sym)
        .unwrap_or("passive")
        .to_string();
    // Hidden pins appear either as a bare `hide` symbol (KiCad <= 7) or as a
    // `(hide yes)` child list (KiCad 8+).
    let hidden = items.iter().any(|it| it.as_sym() == Some("hide"))
        || pcb_sexpr::find_child_list(items, "hide")
            .and_then(|h| h.get(1))
            .and_then(Sexpr::as_sym)
            == Some("yes");

    let at = pcb_sexpr::find_child_list(items, "at")?;
    let x = num_at(at, 1)?;
    let y = num_at(at, 2)?;
    let angle = num_at(at, 3).unwrap_or(0.0) as i32;
    let length = pcb_sexpr::find_child_list(items, "length")
        .and_then(|l| num_at(l, 1))
        .unwrap_or(0.0);
    let name = pcb_sexpr::find_child_list(items, "name")
        .and_then(|n| n.get(1))
        .and_then(Sexpr::as_str)
        .unwrap_or("")
        .to_string();
    let number = pcb_sexpr::find_child_list(items, "number")
        .and_then(|n| n.get(1))
        .and_then(Sexpr::as_str)?
        .to_string();

    Some(PinGeom {
        number,
        name,
        etype,
        x: round4(x),
        y: round4(y),
        angle,
        length,
        hidden,
    })
}

fn point_of(items: &[Sexpr], key: &str) -> Option<(f64, f64)> {
    let list = pcb_sexpr::find_child_list(items, key)?;
    Some((num_at(list, 1)?, num_at(list, 2)?))
}

fn num_at(items: &[Sexpr], idx: usize) -> Option<f64> {
    let node = items.get(idx)?;
    node.as_float().or_else(|| node.as_int().map(|v| v as f64))
}

#[cfg(test)]
mod tests {
    use super::*;

    const R_SMALL: &str = r#"(symbol "R_Small"
        (pin_numbers (hide yes))
        (pin_names (offset 0.254) (hide yes))
        (exclude_from_sim no) (in_bom yes) (on_board yes)
        (property "Reference" "R" (at 0 0 90) (effects (font (size 1.016 1.016))))
        (property "Value" "R_Small" (at 1.778 0 90) (effects (font (size 1.27 1.27))))
        (symbol "R_Small_0_1"
            (rectangle (start -0.762 1.778) (end 0.762 -1.778)
                (stroke (width 0.2032) (type default)) (fill (type none)))
        )
        (symbol "R_Small_1_1"
            (pin passive line (at 0 2.54 270) (length 0.762)
                (name "" (effects (font (size 1.27 1.27))))
                (number "1" (effects (font (size 1.27 1.27)))))
            (pin passive line (at 0 -2.54 90) (length 0.762)
                (name "" (effects (font (size 1.27 1.27))))
                (number "2" (effects (font (size 1.27 1.27)))))
        )
        (embedded_fonts no)
    )"#;

    #[test]
    fn parses_pins_and_bbox() {
        let geom = parse_lib_symbol(R_SMALL, Some("Device:R_Small")).unwrap();
        assert_eq!(geom.lib_id, "Device:R_Small");
        assert_eq!(geom.pins.len(), 2);

        let p1 = geom.pin("1").unwrap();
        assert_eq!((p1.x, p1.y, p1.angle), (0.0, 2.54, 270));
        assert_eq!(p1.etype, "passive");
        assert!(!p1.hidden);

        assert_eq!(
            geom.body_bbox,
            BBox {
                x1: -0.762,
                y1: -1.778,
                x2: 0.762,
                y2: 1.778
            }
        );
        assert_eq!(geom.full_bbox.y2, 2.54);

        // The rename must be visible in the serialized tree.
        let text = geom.sexpr.to_string();
        assert!(text.contains("\"Device:R_Small\""));
        assert!(text.contains("\"R_Small_0_1\""));
    }

    #[test]
    fn pin_position_applies_rotation_mirror_and_y_flip() {
        let geom = parse_lib_symbol(R_SMALL, None).unwrap();
        // Rotation 0: pin 1 (lib top) lands above the anchor on the sheet.
        assert_eq!(
            geom.pin_position("1", (127.0, 63.5), 0, None),
            Some((127.0, 60.96))
        );
        assert_eq!(
            geom.pin_position("2", (127.0, 63.5), 0, None),
            Some((127.0, 66.04))
        );
        // Rotation 90 (CCW in lib coords): pin 1 goes to the left on the sheet.
        assert_eq!(
            geom.pin_position("1", (127.0, 63.5), 90, None),
            Some((124.46, 63.5))
        );
        // Mirror X flips the library Y axis.
        assert_eq!(
            geom.pin_position("1", (127.0, 63.5), 0, Some(MirrorAxis::X)),
            Some((127.0, 66.04))
        );
    }

    #[test]
    fn pin_outward_points_away_from_body() {
        let geom = parse_lib_symbol(R_SMALL, None).unwrap();
        // Pin 1 is on top of the body (lib +Y): outward is up on the sheet.
        assert_eq!(geom.pin_outward("1", 0, None), Some((0.0, -1.0)));
        assert_eq!(geom.pin_outward("2", 0, None), Some((0.0, 1.0)));
        assert_eq!(geom.pin_outward("1", 90, None), Some((-1.0, 0.0)));
    }

    #[test]
    fn hidden_pin_detection_both_syntaxes() {
        let old = r#"(symbol "X" (symbol "X_1_1"
            (pin power_in line (at 0 0 90) (length 0) hide
                (name "VSS" (effects (font (size 1.27 1.27))))
                (number "3" (effects (font (size 1.27 1.27)))))))"#;
        let new = r#"(symbol "X" (symbol "X_1_1"
            (pin power_in line (at 0 0 90) (length 0) (hide yes)
                (name "VSS" (effects (font (size 1.27 1.27))))
                (number "3" (effects (font (size 1.27 1.27)))))))"#;
        for raw in [old, new] {
            let geom = parse_lib_symbol(raw, None).unwrap();
            assert!(geom.pins[0].hidden, "hidden not detected in: {raw}");
        }
    }
}
