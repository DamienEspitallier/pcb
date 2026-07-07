//! Shared in-memory design fixtures for unit tests.

#![cfg(test)]

use pcb_sch::{AttributeValue, Instance, InstanceRef, ModuleRef, Net, Schematic};
use std::path::Path;

use crate::model::{ATTR_PADS, ATTR_SYMBOL_VALUE};

pub const R_SMALL: &str = r#"(symbol "R_Small"
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

/// Six-pin IC-style box (3 pins per side), pin 1 declared input.
pub const BOX6: &str = r#"(symbol "Box6"
    (symbol "Box6_0_1"
        (rectangle (start -5.08 5.08) (end 5.08 -5.08)
            (stroke (width 0.254) (type default)) (fill (type background))))
    (symbol "Box6_1_1"
        (pin input line (at -7.62 2.54 0) (length 2.54)
            (name "IN" (effects (font (size 1.27 1.27))))
            (number "1" (effects (font (size 1.27 1.27)))))
        (pin passive line (at -7.62 0 0) (length 2.54)
            (name "P2" (effects (font (size 1.27 1.27))))
            (number "2" (effects (font (size 1.27 1.27)))))
        (pin power_in line (at -7.62 -2.54 0) (length 2.54)
            (name "VCC" (effects (font (size 1.27 1.27))))
            (number "3" (effects (font (size 1.27 1.27)))))
        (pin output line (at 7.62 2.54 180) (length 2.54)
            (name "OUT" (effects (font (size 1.27 1.27))))
            (number "4" (effects (font (size 1.27 1.27)))))
        (pin passive line (at 7.62 0 180) (length 2.54)
            (name "P5" (effects (font (size 1.27 1.27))))
            (number "5" (effects (font (size 1.27 1.27)))))
        (pin power_in line (at 7.62 -2.54 180) (length 2.54)
            (name "GND" (effects (font (size 1.27 1.27))))
            (number "6" (effects (font (size 1.27 1.27)))))))"#;

/// An IC-style box whose two right-edge pins sit only ONE grid step apart
/// (1.27 mm), plus one left pin. Used to force a text collision when two
/// sibling ports on those pins are pulled onto a common X column: their labels
/// would stack a single grid step apart, a real overlap the alignment must
/// refuse.
pub const TIGHT_PORTS: &str = r#"(symbol "TightPorts"
    (symbol "TightPorts_0_1"
        (rectangle (start -5.08 5.08) (end 5.08 -5.08)
            (stroke (width 0.254) (type default)) (fill (type background))))
    (symbol "TightPorts_1_1"
        (pin input line (at -7.62 0 0) (length 2.54)
            (name "IN" (effects (font (size 1.27 1.27))))
            (number "1" (effects (font (size 1.27 1.27)))))
        (pin output line (at 7.62 1.27 180) (length 2.54)
            (name "CK" (effects (font (size 1.27 1.27))))
            (number "2" (effects (font (size 1.27 1.27)))))
        (pin output line (at 7.62 0 180) (length 2.54)
            (name "RST" (effects (font (size 1.27 1.27))))
            (number "3" (effects (font (size 1.27 1.27)))))))"#;

fn module_ref() -> ModuleRef {
    ModuleRef::from_path(Path::new("/test.zen"), "<root>")
}

/// Add a two-pin resistor-style component under `path`, wiring pad 1 to
/// `p1_net` and pad 2 to `p2_net` (nets added separately).
pub fn add_r(sch: &mut Schematic, path: &[&str], value: &str) -> InstanceRef {
    add_component(
        sch,
        path,
        R_SMALL,
        &[("1", "1"), ("2", "2")],
        value,
        Some("resistor"),
    )
}

/// Add a component with an explicit symbol and (signal, pad) pin map.
pub fn add_component(
    sch: &mut Schematic,
    path: &[&str],
    symbol: &str,
    pins: &[(&str, &str)],
    value: &str,
    type_attr: Option<&str>,
) -> InstanceRef {
    let module = module_ref();
    let comp_ref = InstanceRef::new(module.clone(), path.iter().map(|s| s.to_string()).collect());
    let mut comp = Instance::component(module.clone())
        .with_attribute(ATTR_SYMBOL_VALUE, symbol.to_string())
        .with_attribute("value", value.to_string());
    if let Some(t) = type_attr {
        comp.add_attribute("type", AttributeValue::String(t.into()));
    }
    for (signal, pad) in pins {
        let port_ref = comp_ref.append(signal.to_string());
        let mut port = Instance::port(module.clone());
        port.add_attribute(
            ATTR_PADS,
            AttributeValue::Array(vec![AttributeValue::String(pad.to_string())]),
        );
        sch.add_instance(port_ref.clone(), port);
        comp.add_child(signal.to_string(), port_ref);
    }
    sch.add_instance(comp_ref.clone(), comp);
    comp_ref
}

/// Add a symbol-LESS component: with `__symbol_value` absent the model
/// synthesizes a box from the pads, i.e. an "unknown" IC box (the flag the
/// analog/digital classifier keys on). `pins` = (signal, pad).
pub fn add_box(sch: &mut Schematic, path: &[&str], pins: &[(&str, &str)]) -> InstanceRef {
    let module = module_ref();
    let comp_ref = InstanceRef::new(module.clone(), path.iter().map(|s| s.to_string()).collect());
    let mut comp = Instance::component(module.clone());
    for (signal, pad) in pins {
        let port_ref = comp_ref.append(signal.to_string());
        let mut port = Instance::port(module.clone());
        port.add_attribute(
            ATTR_PADS,
            AttributeValue::Array(vec![AttributeValue::String(pad.to_string())]),
        );
        sch.add_instance(port_ref.clone(), port);
        comp.add_child(signal.to_string(), port_ref);
    }
    sch.add_instance(comp_ref.clone(), comp);
    comp_ref
}

/// Add a two-pin capacitor-style component (pads 1/2), `type = capacitor`.
pub fn add_c(sch: &mut Schematic, path: &[&str], value: &str) -> InstanceRef {
    add_component(
        sch,
        path,
        R_SMALL,
        &[("1", "1"), ("2", "2")],
        value,
        Some("capacitor"),
    )
}

pub fn port_ref(comp: &[&str], signal: &str) -> InstanceRef {
    let mut path: Vec<String> = comp.iter().map(|s| s.to_string()).collect();
    path.push(signal.to_string());
    InstanceRef::new(module_ref(), path)
}

/// VCC - R1 - MID - R2 - GND, flat root (same as the phase-1 fixture).
pub fn divider() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());

    for name in ["R1", "R2"] {
        let comp_ref = add_r(&mut sch, &[name], "10k");
        root_inst.add_child(name.to_string(), comp_ref);
    }
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);

    sch.add_net(Net::new("Power".to_string(), "VCC", 1).with_port(port_ref(&["R1"], "1")));
    sch.add_net(
        Net::new("Net".to_string(), "MID", 2)
            .with_port(port_ref(&["R1"], "2"))
            .with_port(port_ref(&["R2"], "1")),
    );
    sch.add_net(Net::new("Ground".to_string(), "GND", 3).with_port(port_ref(&["R2"], "2")));
    sch.assign_reference_designators();
    sch
}

/// Differential input filter: a synthesized IC box `U1` (four pins: AINP,
/// AINN on the left edge, VDD/GND on the right) fed through two series
/// resistors (`RP`, `RN`) with two shunt capacitors (`CP`, `CN`) to ground.
/// Exercises the input-chain proximity + alignment rules.
pub fn diff_filter() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("AINP", "1"), ("AINN", "2"), ("VDD", "3"), ("GND", "4")],
    );
    let rp = add_r(&mut sch, &["RP"], "6k");
    let rn = add_r(&mut sch, &["RN"], "6k");
    let cp = add_c(&mut sch, &["CP"], "100pF");
    let cn = add_c(&mut sch, &["CN"], "100pF");
    for (n, r) in [("U1", u1), ("RP", rp), ("RN", rn), ("CP", cp), ("CN", cn)] {
        root_inst.add_child(n.to_string(), r);
    }
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    // External differential inputs (single-endpoint terminals).
    sch.add_net(Net::new("Net".to_string(), "AINP_EXT", 1).with_port(port_ref(&["RP"], "1")));
    sch.add_net(Net::new("Net".to_string(), "AINN_EXT", 2).with_port(port_ref(&["RN"], "1")));
    // Filtered nodes: series R + shunt C into the IC input pin.
    sch.add_net(
        Net::new("Net".to_string(), "AINP_FILT", 3)
            .with_port(port_ref(&["RP"], "2"))
            .with_port(port_ref(&["CP"], "1"))
            .with_port(port_ref(&["U1"], "AINP")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "AINN_FILT", 4)
            .with_port(port_ref(&["RN"], "2"))
            .with_port(port_ref(&["CN"], "1"))
            .with_port(port_ref(&["U1"], "AINN")),
    );
    sch.add_net(Net::new("Power".to_string(), "VDD", 5).with_port(port_ref(&["U1"], "VDD")));
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 6)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["CP"], "2"))
            .with_port(port_ref(&["CN"], "2")),
    );
    sch.assign_reference_designators();
    sch
}

/// Synthesized IC box `U1` whose left edge carries two VDD pins split by a
/// signal pin (VDD, SIG, VDD) and whose right edge carries two adjacent GND
/// pins. Exercises the shared power-symbol rule (non-adjacent bank merge +
/// adjacent stack collapse).
pub fn power_bank() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    // Pads 1..3 land on the left edge (VDD, SIG, VDD), 4..6 on the right
    // (OUT, GND, GND) after the box synthesizer's split.
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[
            ("VDDA", "1"),
            ("SIG", "2"),
            ("VDDB", "3"),
            ("OUT", "4"),
            ("GNDA", "5"),
            ("GNDB", "6"),
        ],
    );
    root_inst.add_child("U1".to_string(), u1);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(
        Net::new("Power".to_string(), "VDD", 1)
            .with_port(port_ref(&["U1"], "VDDA"))
            .with_port(port_ref(&["U1"], "VDDB")),
    );
    sch.add_net(Net::new("Net".to_string(), "SIG", 2).with_port(port_ref(&["U1"], "SIG")));
    sch.add_net(Net::new("Net".to_string(), "OUT", 3).with_port(port_ref(&["U1"], "OUT")));
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 4)
            .with_port(port_ref(&["U1"], "GNDA"))
            .with_port(port_ref(&["U1"], "GNDB")),
    );
    sch.assign_reference_designators();
    sch
}

/// The AD7171 input corner: an IC whose left edge stacks AIN+, AIN- and a
/// DOUT/RDY output at the 2-step pin pitch, with a differential input filter
/// (series R + shunt C on each of AIN+/AIN-) crowding the edge, plus a pull-up
/// resistor on the DOUT net. The DOUT net (`MISO`, two endpoints: the IC pin
/// and the pull-up) is boxed in — a local label on the DOUT pin collides with
/// the AIN- filter column — so it must promote to a self-contained global
/// label. Everything lives on the root sheet.
pub fn adc_dout_congested() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    // Left edge (pads 1..3): AIN+, AIN-, DOUT. Right edge (pads 4..6).
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[
            ("AINP", "1"),
            ("AINN", "2"),
            ("DOUT", "3"),
            ("GND", "4"),
            ("VDD", "5"),
            ("OUT", "6"),
        ],
    );
    let rp = add_r(&mut sch, &["RP"], "6k");
    let rn = add_r(&mut sch, &["RN"], "6k");
    let cp = add_c(&mut sch, &["CP"], "100pF");
    let cn = add_c(&mut sch, &["CN"], "100pF");
    let rpu = add_r(&mut sch, &["RPU"], "10k");
    for (n, r) in [
        ("U1", u1),
        ("RP", rp),
        ("RN", rn),
        ("CP", cp),
        ("CN", cn),
        ("RPU", rpu),
    ] {
        root_inst.add_child(n.to_string(), r);
    }
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(Net::new("Net".to_string(), "AINP_EXT", 1).with_port(port_ref(&["RP"], "1")));
    sch.add_net(Net::new("Net".to_string(), "AINN_EXT", 2).with_port(port_ref(&["RN"], "1")));
    sch.add_net(
        Net::new("Net".to_string(), "AINP_FILT", 3)
            .with_port(port_ref(&["RP"], "2"))
            .with_port(port_ref(&["CP"], "1"))
            .with_port(port_ref(&["U1"], "AINP")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "AINN_FILT", 4)
            .with_port(port_ref(&["RN"], "2"))
            .with_port(port_ref(&["CN"], "1"))
            .with_port(port_ref(&["U1"], "AINN")),
    );
    // DOUT signal with a pull-up: two endpoints (IC pin + pull-up resistor).
    sch.add_net(
        Net::new("Net".to_string(), "MISO", 5)
            .with_port(port_ref(&["U1"], "DOUT"))
            .with_port(port_ref(&["RPU"], "1")),
    );
    sch.add_net(
        Net::new("Power".to_string(), "VDD", 6)
            .with_port(port_ref(&["U1"], "VDD"))
            .with_port(port_ref(&["RPU"], "2")),
    );
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 7)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["CP"], "2"))
            .with_port(port_ref(&["CN"], "2")),
    );
    sch.assign_reference_designators();
    sch
}

/// A spread ADC box whose horizontal DOUT output is pulled up to VDD, with one
/// filtered analog input entering just above DOUT. On the default (generous)
/// grid the pull-up re-seats directly over the DOUT pin, close enough to drop a
/// continuous wire — but that drop must cross the horizontal analog-input wire,
/// so the crossing-free digital tree gives up. Rule #1 de-duplication then
/// accepts the electrically-harmless frank crossing to keep DOUT on ONE wire
/// under a single label (the reference AD7171 R3 → DOUT/RDY across AIN+/AIN-),
/// instead of scattering a homonym label onto each endpoint.
pub fn adc_pullup_over_analog() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    // Left edge (pads 1..2): AIN above DOUT. Right edge (pads 3..4): VDD, GND.
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("AIN", "1"), ("DOUT", "2"), ("VDD", "3"), ("GND", "4")],
    );
    let rin = add_r(&mut sch, &["RIN"], "1k"); // series input resistor (analog)
    let cin = add_c(&mut sch, &["CIN"], "100pF"); // shunt cap: makes AIN a 3-pin analog net
    let rpu = add_r(&mut sch, &["RPU"], "10k"); // DOUT pull-up
    for (n, r) in [("U1", u1), ("RIN", rin), ("CIN", cin), ("RPU", rpu)] {
        root_inst.add_child(n.to_string(), r);
    }
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(Net::new("Net".to_string(), "AIN_EXT", 1).with_port(port_ref(&["RIN"], "1")));
    // Three endpoints (series R, shunt C, IC pin) → analog: wired as a
    // continuous tree BEFORE the digital DOUT net, so the pull-up drop then
    // meets a committed analog wire it must cross (as on the real AD7171).
    sch.add_net(
        Net::new("Net".to_string(), "AIN_FILT", 2)
            .with_port(port_ref(&["RIN"], "2"))
            .with_port(port_ref(&["CIN"], "1"))
            .with_port(port_ref(&["U1"], "AIN")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "MISO", 3)
            .with_port(port_ref(&["U1"], "DOUT"))
            .with_port(port_ref(&["RPU"], "1")),
    );
    sch.add_net(
        Net::new("Power".to_string(), "VDD", 4)
            .with_port(port_ref(&["U1"], "VDD"))
            .with_port(port_ref(&["RPU"], "2")),
    );
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 5)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["CIN"], "2")),
    );
    sch.assign_reference_designators();
    sch
}

/// An IC whose right edge carries a split VDD rail: two VDD pins (pads 5 and 8)
/// separated by two foreign pins (a ground and a signal), the AD7171
/// REFIN+/VDD pattern. A single rail symbol crowns both under a short offset
/// bus (`merge_power_banks`), so this exercises the merged-rail-bank symbol
/// placement (the one-grid-step lift). GND is undriven; relegation is left to
/// the caller.
pub fn split_rail_ic() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    // Pads 1..4 land on the left edge, 5..8 on the right after the box
    // synthesizer's split. VDD on 5 and 8 (split by GND on 6 and SIG on 7).
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[
            ("IN1", "1"),
            ("IN2", "2"),
            ("IN3", "3"),
            ("IN4", "4"),
            ("REFP", "5"),
            ("REFN", "6"),
            ("SCK", "7"),
            ("VDDPIN", "8"),
        ],
    );
    root_inst.add_child("U1".to_string(), u1);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    for (sig, pad) in [
        ("IN1", 1u64),
        ("IN2", 2),
        ("IN3", 3),
        ("IN4", 4),
        ("REFN", 6),
        ("SCK", 7),
    ] {
        sch.add_net(Net::new("Net".to_string(), sig, pad).with_port(port_ref(&["U1"], sig)));
    }
    sch.add_net(
        Net::new("Power".to_string(), "VDD", 5)
            .with_port(port_ref(&["U1"], "REFP"))
            .with_port(port_ref(&["U1"], "VDDPIN")),
    );
    sch.assign_reference_designators();
    sch
}

/// An IC `U1` whose two edges each carry a pair of single-endpoint signal
/// ports of DIFFERENT name lengths, so their natural stub lengths differ and
/// the boundary labels land at different X. The left edge stacks `A_LONG` over
/// `B`, the right edge `CLK` over `PDRST` — the AD7171 CLK/PDRST pattern.
/// Exercises the sibling-port column alignment (both edges should collapse onto
/// one X). The sheet is otherwise empty, so alignment is always clean.
pub fn sibling_ports_ic() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    // Pads 1..2 land on the left edge, 3..4 on the right.
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("A_LONG", "1"), ("B", "2"), ("CLK", "3"), ("PDRST", "4")],
    );
    root_inst.add_child("U1".to_string(), u1);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    for (sig, pad) in [("A_LONG", 1u64), ("B", 2), ("CLK", 3), ("PDRST", 4)] {
        sch.add_net(Net::new("Net".to_string(), sig, pad).with_port(port_ref(&["U1"], sig)));
    }
    sch.assign_reference_designators();
    sch
}

/// Two single-endpoint ports (`CLK`, `PDRESET`) leave the right edge of an IC
/// whose two right pins are only one grid step apart. Their natural stubs
/// differ in length (different name widths), so the alignment would pull them
/// onto one X — but that stacks their labels a single grid step apart, a real
/// overlap. The soft pass must therefore SKIP this group, leaving the two
/// labels at their routed (different) X.
pub fn sibling_ports_collide() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    let u1 = add_component(
        &mut sch,
        &["U1"],
        TIGHT_PORTS,
        &[("IN", "1"), ("CLK", "2"), ("PDRESET", "3")],
        "IC",
        None,
    );
    root_inst.add_child("U1".to_string(), u1);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    for (sig, pad) in [("IN", 1u64), ("CLK", 2), ("PDRESET", 3)] {
        sch.add_net(Net::new("Net".to_string(), sig, pad).with_port(port_ref(&["U1"], sig)));
    }
    sch.assign_reference_designators();
    sch
}

/// A minimal ADC-like sheet: an IC `U1` (signal input + VDD + GND) with a
/// rail-to-rail decoupling capacitor `C1` (VDD to GND, 100nF). VDD and GND are
/// undriven (no `power_out` pin), so each needs a `PWR_FLAG`. Exercises utility
/// relegation (#6 decoupling, #7 flags) and zone outlining (#8): the sheet
/// carries a real IC, so relegation is active by default.
pub fn decoupled_adc() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("IN", "1"), ("VDD", "2"), ("GND", "3")],
    );
    let c1 = add_c(&mut sch, &["C1"], "100nF");
    root_inst.add_child("U1".to_string(), u1);
    root_inst.add_child("C1".to_string(), c1);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(Net::new("Net".to_string(), "IN", 1).with_port(port_ref(&["U1"], "IN")));
    sch.add_net(
        Net::new("Power".to_string(), "VDD", 2)
            .with_port(port_ref(&["U1"], "VDD"))
            .with_port(port_ref(&["C1"], "1")),
    );
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 3)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["C1"], "2")),
    );
    sch.assign_reference_designators();
    sch
}

/// Two synthesized IC boxes joined by signal net `BUS`, plus a pull-up
/// resistor `RP` (BUS to VCC). BUS = IC-IC + one pull -> DIGITAL.
pub fn digital_bus() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("BUS", "1"), ("VCC", "2"), ("GND", "3")],
    );
    let u2 = add_box(
        &mut sch,
        &["U2"],
        &[("BUS", "1"), ("VCC", "2"), ("GND", "3")],
    );
    let rp = add_r(&mut sch, &["RP"], "10k");
    root_inst.add_child("U1".to_string(), u1);
    root_inst.add_child("U2".to_string(), u2);
    root_inst.add_child("RP".to_string(), rp);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(
        Net::new("Net".to_string(), "BUS", 1)
            .with_port(port_ref(&["U1"], "BUS"))
            .with_port(port_ref(&["U2"], "BUS"))
            .with_port(port_ref(&["RP"], "1")),
    );
    sch.add_net(
        Net::new("Power".to_string(), "VCC", 2)
            .with_port(port_ref(&["U1"], "VCC"))
            .with_port(port_ref(&["U2"], "VCC"))
            .with_port(port_ref(&["RP"], "2")),
    );
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 3)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["U2"], "GND")),
    );
    sch.assign_reference_designators();
    sch
}

/// Synthesized box `U1.AIN` behind a series resistor `RF` with a shunt cap
/// `CF` to GND — an RC input filter. `FILT` (AIN + RF + CF) -> ANALOG.
pub fn analog_filter() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);
    let mut root_inst = Instance::module(module.clone());
    let u1 = add_box(
        &mut sch,
        &["U1"],
        &[("AIN", "1"), ("VCC", "2"), ("GND", "3")],
    );
    let rf = add_r(&mut sch, &["RF"], "1k");
    let cf = add_c(&mut sch, &["CF"], "100pF");
    root_inst.add_child("U1".to_string(), u1);
    root_inst.add_child("RF".to_string(), rf);
    root_inst.add_child("CF".to_string(), cf);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);
    sch.add_net(Net::new("Net".to_string(), "AIN_EXT", 1).with_port(port_ref(&["RF"], "1")));
    sch.add_net(
        Net::new("Net".to_string(), "FILT", 2)
            .with_port(port_ref(&["U1"], "AIN"))
            .with_port(port_ref(&["RF"], "2"))
            .with_port(port_ref(&["CF"], "1")),
    );
    sch.add_net(Net::new("Power".to_string(), "VCC", 3).with_port(port_ref(&["U1"], "VCC")));
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 4)
            .with_port(port_ref(&["U1"], "GND"))
            .with_port(port_ref(&["CF"], "2")),
    );
    sch.assign_reference_designators();
    sch
}

/// Root with an inlined two-resistor module (`div`) and a bigger module
/// (`big`: one IC + four resistors) that keeps its own sheet. Signal `SIG`
/// crosses from the divider into `big`; VCC/GND span the design.
pub fn hierarchical_design() -> Schematic {
    let module = module_ref();
    let mut sch = Schematic::new();
    let root = InstanceRef::new(module.clone(), vec![]);

    let mut div = Instance::module(module.clone());
    for name in ["R1", "R2"] {
        let comp_ref = add_r(&mut sch, &["div", name], "10k");
        div.add_child(name.to_string(), comp_ref);
    }
    let div_ref = InstanceRef::new(module.clone(), vec!["div".to_string()]);
    sch.add_instance(div_ref.clone(), div);

    let mut big = Instance::module(module.clone());
    let ic_ref = add_component(
        &mut sch,
        &["big", "U1"],
        BOX6,
        &[
            ("IN", "1"),
            ("P2", "2"),
            ("VCC", "3"),
            ("OUT", "4"),
            ("P5", "5"),
            ("GND", "6"),
        ],
        "IC",
        None,
    );
    big.add_child("U1".to_string(), ic_ref);
    for name in ["RA", "RB", "RC", "RD"] {
        let comp_ref = add_r(&mut sch, &["big", name], "1k");
        big.add_child(name.to_string(), comp_ref);
    }
    let big_ref = InstanceRef::new(module.clone(), vec!["big".to_string()]);
    sch.add_instance(big_ref.clone(), big);

    let mut root_inst = Instance::module(module.clone());
    root_inst.add_child("div".to_string(), div_ref);
    root_inst.add_child("big".to_string(), big_ref);
    sch.add_instance(root.clone(), root_inst);
    sch.set_root_ref(root);

    sch.add_net(
        Net::new("Power".to_string(), "VCC", 1)
            .with_port(port_ref(&["div", "R1"], "1"))
            .with_port(port_ref(&["big", "U1"], "VCC")),
    );
    sch.add_net(
        Net::new("Ground".to_string(), "GND", 2)
            .with_port(port_ref(&["div", "R2"], "2"))
            .with_port(port_ref(&["big", "U1"], "GND")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "SIG", 3)
            .with_port(port_ref(&["div", "R1"], "2"))
            .with_port(port_ref(&["div", "R2"], "1"))
            .with_port(port_ref(&["big", "U1"], "IN")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "CHAIN", 4)
            .with_port(port_ref(&["big", "U1"], "OUT"))
            .with_port(port_ref(&["big", "RA"], "1")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "C2", 5)
            .with_port(port_ref(&["big", "RA"], "2"))
            .with_port(port_ref(&["big", "RB"], "1")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "C3", 6)
            .with_port(port_ref(&["big", "RB"], "2"))
            .with_port(port_ref(&["big", "RC"], "1")),
    );
    sch.add_net(
        Net::new("Net".to_string(), "C4", 7)
            .with_port(port_ref(&["big", "RC"], "2"))
            .with_port(port_ref(&["big", "RD"], "1")),
    );
    sch.add_net(Net::new("Net".to_string(), "C5", 8).with_port(port_ref(&["big", "RD"], "2")));
    sch.assign_reference_designators();
    sch
}
