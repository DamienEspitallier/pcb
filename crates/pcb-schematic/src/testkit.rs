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
