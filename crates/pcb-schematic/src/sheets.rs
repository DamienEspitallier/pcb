//! Sheet planning: partition the module-instance hierarchy into KiCad
//! sheets (port of the validated `sheet_planner.ts`).
//!
//! Rules:
//! * the root module is always the top sheet;
//! * every module instance becomes a dedicated sheet **unless** it is
//!   inlined into its caller's sheet:
//!   - `Module(..., schematic="embed")` (attribute `embed`) forces inlining,
//!   - `Module(..., schematic="collapse")` (attribute `collapse`) forces a
//!     dedicated sheet,
//!   - auto + module containing a sub-sheet stays a sheet (structural
//!     container),
//!   - auto + 100% satellite parts (passives, decoupling, pulls, LEDs) is
//!     inlined regardless of size,
//!   - auto + fewer than `sheet_min_parts` physical parts with at most
//!     `inline_max_major_parts` major parts is inlined,
//!   - anything else keeps its sheet;
//! * sheet ports carry only **signal** nets that actually cross the sheet
//!   boundary (endpoints both inside the subtree and outside); power and
//!   ground rails never become ports (global power symbols instead);
//! * port direction aggregates the electrical types of the pins connected
//!   inside the subtree (bidirectional dominates, output-only -> output,
//!   input-only -> input, default bidirectional).
//!
//! Unlike the PoC (one call per module), the `pcb` IR instantiates modules
//! any number of times; every module **instance** gets its own sheet with a
//! unique file name — KiCad sheet-file reuse is not attempted.

use std::collections::BTreeSet;

use pcb_sch::{InstanceKind, InstanceRef, Schematic};

use crate::config::SchConfig;
use crate::model::{DesignModel, NetClass};
use crate::writer::PortDirection;

/// One planned sheet.
pub struct SheetDef {
    /// Instance path of the owning module (empty for the root). Kept for
    /// the next phase (`# pcb:sch` write-back maps sheets to modules).
    #[allow(dead_code)]
    pub module_path: Vec<String>,
    /// Sheet title (root: project name; child: module instance name).
    pub title: String,
    /// Output file name (unique across the plan).
    pub file_name: String,
    /// Page number recorded in sheet instance blocks ("1", "2", ...).
    pub page: String,
    /// Indices of the components owned by this sheet (into `DesignModel`).
    pub comps: Vec<usize>,
    /// Hierarchical ports: signal nets (design net indices) crossing this
    /// sheet's boundary, with their aggregated direction.
    pub ports: Vec<SheetPort>,
    /// Index of the parent sheet in the plan (None for the root).
    pub parent: Option<usize>,
    /// Indices of the child sheets, in plan order.
    pub children: Vec<usize>,
}

#[derive(Debug, Clone, Copy)]
pub struct SheetPort {
    /// Design net index (net names make the KiCad connection).
    pub net: usize,
    pub direction: PortDirection,
}

/// The full plan; `sheets[0]` is the root.
pub struct SheetPlan {
    pub sheets: Vec<SheetDef>,
    /// Owning sheet of every design component.
    pub comp_sheet: Vec<usize>,
}

#[derive(Clone, Copy, PartialEq)]
enum Status {
    Sheet,
    Inline,
}

/// A node of the module-instance tree.
struct Node {
    inst_ref: InstanceRef,
    /// Child module nodes, natural order by instance name.
    children: Vec<usize>,
    /// Direct components (design indices), natural order.
    comps: Vec<usize>,
    name: String,
}

/// Plan the sheets of an evaluated design.
pub fn plan_sheets(
    sch: &Schematic,
    design: &DesignModel,
    cfg: &SchConfig,
    project_name: &str,
    warnings: &mut Vec<String>,
) -> SheetPlan {
    // ------------------------------------------------------------------
    // 1. Module-instance tree (deterministic: children sorted by name).
    // ------------------------------------------------------------------
    let mut nodes: Vec<Node> = Vec::new();
    let root_ref = sch
        .root_ref
        .clone()
        .expect("schematic has no root instance");
    build_tree(sch, design, &root_ref, "<root>", &mut nodes);

    // ------------------------------------------------------------------
    // 2. Inline / sheet decision (post-order).
    // ------------------------------------------------------------------
    let mut status: Vec<Status> = vec![Status::Sheet; nodes.len()];
    decide(sch, design, cfg, &nodes, 0, true, &mut status);

    // ------------------------------------------------------------------
    // 3. Assemble sheets (pre-order) and map components to sheets.
    // ------------------------------------------------------------------
    let mut sheets: Vec<SheetDef> = Vec::new();
    let mut file_names: BTreeSet<String> = BTreeSet::new();
    let root_file = format!("{project_name}.kicad_sch");
    file_names.insert(root_file.clone());
    emit_sheet(
        &nodes,
        &status,
        0,
        None,
        project_name,
        Some(root_file),
        &mut file_names,
        &mut sheets,
    );
    for (i, sheet) in sheets.iter_mut().enumerate() {
        sheet.page = (i + 1).to_string();
        if sheet.comps.len() > cfg.max_parts_per_sheet {
            warnings.push(format!(
                "sheet \"{}\": {} components exceed max-parts-per-sheet ({}) — automatic split is out of scope, the sheet will be dense",
                sheet.title,
                sheet.comps.len(),
                cfg.max_parts_per_sheet
            ));
        }
    }

    let mut comp_sheet = vec![0usize; design.comps.len()];
    for (si, sheet) in sheets.iter().enumerate() {
        for &ci in &sheet.comps {
            comp_sheet[ci] = si;
        }
    }

    // ------------------------------------------------------------------
    // 4. Ports: signal nets crossing each sheet's subtree boundary.
    // ------------------------------------------------------------------
    // Subtree component sets (a sheet's subtree = its comps + descendants').
    let mut subtree: Vec<BTreeSet<usize>> = sheets
        .iter()
        .map(|s| s.comps.iter().copied().collect())
        .collect();
    // Children come after their parent in plan order: fold right-to-left.
    for si in (0..sheets.len()).rev() {
        for &child in &sheets[si].children.clone() {
            let child_set = subtree[child].clone();
            subtree[si].extend(child_set);
        }
    }

    for si in 1..sheets.len() {
        let inside = &subtree[si];
        let mut ports: Vec<SheetPort> = Vec::new();
        for (ni, net) in design.nets.iter().enumerate() {
            if net.class != NetClass::Signal {
                continue; // power/ground: global symbols, never ports
            }
            let mut has_inside = false;
            let mut has_outside = false;
            for (ci, _) in &net.endpoints {
                if inside.contains(ci) {
                    has_inside = true;
                } else {
                    has_outside = true;
                }
            }
            if !has_inside || !has_outside {
                continue;
            }
            ports.push(SheetPort {
                net: ni,
                direction: port_direction(design, inside, ni),
            });
        }
        sheets[si].ports = ports;
    }

    SheetPlan { sheets, comp_sheet }
}

/// Recursively build the module tree. Returns the node index.
fn build_tree(
    sch: &Schematic,
    design: &DesignModel,
    inst_ref: &InstanceRef,
    name: &str,
    nodes: &mut Vec<Node>,
) -> usize {
    let idx = nodes.len();
    nodes.push(Node {
        inst_ref: inst_ref.clone(),
        children: Vec::new(),
        comps: Vec::new(),
        name: name.to_string(),
    });

    let inst = &sch.instances[inst_ref];
    let mut children: Vec<(&String, &InstanceRef)> = inst.children.iter().collect();
    children.sort_by(|a, b| natord::compare(a.0, b.0));

    for (child_name, child_ref) in children {
        let Some(child) = sch.instances.get(child_ref) else {
            continue;
        };
        match child.kind {
            InstanceKind::Module => {
                let child_idx = build_tree(sch, design, child_ref, child_name, nodes);
                nodes[idx].children.push(child_idx);
            }
            InstanceKind::Component => {
                if let Some(&ci) = design.by_path.get(&child_ref.instance_path.join(".")) {
                    nodes[idx].comps.push(ci);
                }
            }
            _ => {}
        }
    }
    idx
}

/// Post-order status decision for `node`.
fn decide(
    sch: &Schematic,
    design: &DesignModel,
    cfg: &SchConfig,
    nodes: &[Node],
    node: usize,
    is_root: bool,
    status: &mut Vec<Status>,
) {
    for &child in &nodes[node].children {
        decide(sch, design, cfg, nodes, child, false, status);
    }
    let inst = &sch.instances[&nodes[node].inst_ref];
    status[node] = if is_root {
        Status::Sheet
    } else if inst.boolean_attr(&[crate::model::ATTR_EMBED]) == Some(true) {
        Status::Inline
    } else if inst.boolean_attr(&[crate::model::ATTR_COLLAPSE]) == Some(true) {
        Status::Sheet
    } else {
        auto_status(design, cfg, nodes, node, status)
    };
}

fn auto_status(
    design: &DesignModel,
    cfg: &SchConfig,
    nodes: &[Node],
    node: usize,
    status: &[Status],
) -> Status {
    // A module containing a sub-sheet is a structural container.
    if has_sheet_descendant_through_inlined(nodes, status, node) {
        return Status::Sheet;
    }
    let mut physical = 0usize;
    let mut majors = 0usize;
    collect_effective_counts(design, nodes, status, node, &mut physical, &mut majors);
    if cfg.inline_passive_only_modules && majors == 0 {
        return Status::Inline;
    }
    if physical < cfg.sheet_min_parts && majors <= cfg.inline_max_major_parts {
        return Status::Inline;
    }
    Status::Sheet
}

/// Does this module (through inlined children only) contain a sheet child?
fn has_sheet_descendant_through_inlined(nodes: &[Node], status: &[Status], node: usize) -> bool {
    for &child in &nodes[node].children {
        match status[child] {
            Status::Sheet => return true,
            Status::Inline => {
                if has_sheet_descendant_through_inlined(nodes, status, child) {
                    return true;
                }
            }
        }
    }
    false
}

/// Physical/major part counts of a module plus its inlined descendants.
fn collect_effective_counts(
    design: &DesignModel,
    nodes: &[Node],
    status: &[Status],
    node: usize,
    physical: &mut usize,
    majors: &mut usize,
) {
    for &ci in &nodes[node].comps {
        *physical += 1;
        if !design.comps[ci].role.is_satellite() {
            *majors += 1;
        }
    }
    for &child in &nodes[node].children {
        if status[child] == Status::Inline {
            collect_effective_counts(design, nodes, status, child, physical, majors);
        }
    }
}

/// Pre-order emission of the sheet list.
#[allow(clippy::too_many_arguments)]
fn emit_sheet(
    nodes: &[Node],
    status: &[Status],
    node: usize,
    parent: Option<usize>,
    title: &str,
    fixed_file: Option<String>,
    file_names: &mut BTreeSet<String>,
    sheets: &mut Vec<SheetDef>,
) -> usize {
    let file_name = fixed_file.unwrap_or_else(|| {
        let base = slugify(&nodes[node].name);
        let base = if base.is_empty() { "sheet" } else { &base };
        let mut candidate = format!("{base}.kicad_sch");
        let mut n = 1;
        while file_names.contains(&candidate) {
            n += 1;
            candidate = format!("{base}_{n}.kicad_sch");
        }
        candidate
    });
    file_names.insert(file_name.clone());

    let idx = sheets.len();
    sheets.push(SheetDef {
        module_path: nodes[node].inst_ref.instance_path.clone(),
        title: title.to_string(),
        file_name,
        page: String::new(),
        comps: Vec::new(),
        ports: Vec::new(),
        parent,
        children: Vec::new(),
    });

    // Components: this module plus inlined descendants, pre-order.
    let mut comps = Vec::new();
    collect_effective_comps(nodes, status, node, &mut comps);
    sheets[idx].comps = comps;

    // Child sheets: sheet-status modules reachable through inlined ones.
    let mut sheet_children = Vec::new();
    collect_sheet_children(nodes, status, node, &mut sheet_children);
    for child in sheet_children {
        let child_idx = emit_sheet(
            nodes,
            status,
            child,
            Some(idx),
            &nodes[child].name,
            None,
            file_names,
            sheets,
        );
        sheets[idx].children.push(child_idx);
    }
    idx
}

fn collect_effective_comps(nodes: &[Node], status: &[Status], node: usize, out: &mut Vec<usize>) {
    out.extend(nodes[node].comps.iter().copied());
    for &child in &nodes[node].children {
        if status[child] == Status::Inline {
            collect_effective_comps(nodes, status, child, out);
        }
    }
}

fn collect_sheet_children(nodes: &[Node], status: &[Status], node: usize, out: &mut Vec<usize>) {
    for &child in &nodes[node].children {
        match status[child] {
            Status::Sheet => out.push(child),
            Status::Inline => collect_sheet_children(nodes, status, child, out),
        }
    }
}

/// Aggregate the direction of a port net from the pin electrical types
/// connected inside the subtree.
fn port_direction(design: &DesignModel, inside: &BTreeSet<usize>, net: usize) -> PortDirection {
    let mut has_in = false;
    let mut has_out = false;
    for (ci, pad) in &design.nets[net].endpoints {
        if !inside.contains(ci) {
            continue;
        }
        let Some(pin) = design.comps[*ci].geom.pin(pad) else {
            continue;
        };
        match pin.etype.as_str() {
            "bidirectional" => return PortDirection::Bidirectional,
            "input" | "power_in" => has_in = true,
            "output" | "power_out" => has_out = true,
            _ => {}
        }
    }
    if has_out && !has_in {
        PortDirection::Output
    } else if has_in && !has_out {
        PortDirection::Input
    } else {
        PortDirection::Bidirectional
    }
}

/// Safe file name: lowercase ASCII alphanumerics, `_` for everything else.
fn slugify(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    let mut last_us = true; // trim leading underscores
    for c in value.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
            last_us = false;
        } else if !last_us {
            out.push('_');
            last_us = true;
        }
    }
    while out.ends_with('_') {
        out.pop();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testkit::hierarchical_design;

    #[test]
    fn slugify_names() {
        assert_eq!(slugify("R_SHUNT"), "r_shunt");
        assert_eq!(slugify("Défi éße"), "d_fi_e");
        assert_eq!(slugify("__x__"), "x");
    }

    #[test]
    fn small_modules_are_inlined_and_big_ones_get_sheets() {
        let sch = hierarchical_design();
        let design = DesignModel::build(&sch, &SchConfig::default()).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(&sch, &design, &SchConfig::default(), "top", &mut warnings);

        // Root plus the big module; the two-resistor divider is inlined.
        assert_eq!(plan.sheets.len(), 2);
        assert_eq!(plan.sheets[0].file_name, "top.kicad_sch");
        assert_eq!(plan.sheets[1].title, "big");
        assert_eq!(plan.sheets[1].parent, Some(0));
        assert_eq!(plan.sheets[0].children, vec![1]);

        // The inlined divider's resistors live on the root sheet.
        let root_refdes: Vec<&str> = plan.sheets[0]
            .comps
            .iter()
            .map(|&ci| design.comps[ci].refdes.as_str())
            .collect();
        assert!(root_refdes.len() >= 2, "root sheet holds inlined divider");

        // The signal crossing into `big` is a port with a direction.
        assert_eq!(plan.sheets[1].ports.len(), 1);
        let port = plan.sheets[1].ports[0];
        assert_eq!(design.nets[port.net].name, "SIG");
    }

    #[test]
    fn schematic_attribute_overrides_auto_status() {
        use pcb_sch::AttributeValue;
        // collapse: the small inlined divider gets a dedicated sheet.
        let mut sch = hierarchical_design();
        let root = sch.root_ref.clone().unwrap();
        let div_ref = sch.instances[&root].children["div"].clone();
        sch.instance_mut(&div_ref)
            .unwrap()
            .add_attribute(crate::model::ATTR_COLLAPSE, AttributeValue::Boolean(true));
        let design = DesignModel::build(&sch, &SchConfig::default()).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(&sch, &design, &SchConfig::default(), "top", &mut warnings);
        assert_eq!(plan.sheets.len(), 3);
        assert!(plan.sheets.iter().any(|s| s.title == "div"));

        // embed: the big module is inlined despite its size.
        let mut sch = hierarchical_design();
        let root = sch.root_ref.clone().unwrap();
        let big_ref = sch.instances[&root].children["big"].clone();
        sch.instance_mut(&big_ref)
            .unwrap()
            .add_attribute(crate::model::ATTR_EMBED, AttributeValue::Boolean(true));
        let design = DesignModel::build(&sch, &SchConfig::default()).unwrap();
        let mut warnings = Vec::new();
        let plan = plan_sheets(&sch, &design, &SchConfig::default(), "top", &mut warnings);
        assert_eq!(plan.sheets.len(), 1);
    }
}
