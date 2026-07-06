use anyhow::{Context, Result};
use clap::Args;
use pcb_layout::utils as layout_utils;
use pcb_sch::Schematic;
use pcb_ui::prelude::*;
use std::path::{Path, PathBuf};

use crate::build::{build, create_diagnostics_passes};
use crate::config_input::{CONFIG_ARG_HELP, parse_config_overrides};

#[derive(Args, Debug, Default, Clone)]
#[command(about = "Generate a KiCad schematic (.kicad_sch) from a .zen file")]
pub struct SchArgs {
    /// Path to .zen file
    #[arg(value_name = "FILE", value_hint = clap::ValueHint::FilePath)]
    pub file: PathBuf,

    #[arg(long = "config", value_name = "KEY=VALUE", help = CONFIG_ARG_HELP)]
    pub config: Vec<String>,

    /// Output path of the root .kicad_sch (defaults to the layout directory
    /// when the design declares one, otherwise next to the .zen file)
    #[arg(short = 'o', long = "output", value_name = "PATH")]
    pub output: Option<PathBuf>,

    /// Run KiCad ERC on the generated schematic (requires kicad-cli)
    #[arg(long = "check")]
    pub check: bool,

    /// Disable network access (offline mode) - only use vendored dependencies
    #[arg(long = "offline")]
    pub offline: bool,
}

pub fn execute(args: SchArgs) -> Result<()> {
    crate::file_walker::require_zen_file(&args.file)?;
    let config_inputs = parse_config_overrides(&args.config)?;

    // Resolve dependencies before building (same pipeline as `pcb layout`).
    let resolution_result = crate::resolve::resolve(Some(&args.file), args.offline)?;

    let zen_path = &args.file;
    let file_name = zen_path.file_name().unwrap().to_string_lossy().to_string();

    let Some(schematic) = build(
        zen_path,
        config_inputs,
        create_diagnostics_passes(&[], &[]),
        false,
        &mut false.clone(),
        &mut false.clone(),
        resolution_result,
    ) else {
        anyhow::bail!("Build failed");
    };

    let sch_path = resolve_output_path(&args, zen_path, &schematic)?;
    let project_name = sch_path
        .file_stem()
        .context("output path has no file name")?
        .to_string_lossy()
        .to_string();
    let out_dir = sch_path
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));

    let spinner = Spinner::builder(format!("{file_name}: Generating schematic")).start();
    let options = pcb_schematic::SchOptions::new(project_name.clone());
    let generated = pcb_schematic::generate_schematic(&schematic, &options)?;
    spinner.finish();

    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("Failed to create {}", out_dir.display()))?;
    for file in &generated.files {
        let path = out_dir.join(&file.file_name);
        std::fs::write(&path, &file.content)
            .with_context(|| format!("Failed to write {}", path.display()))?;
    }

    // The generated sheets need a project file that downgrades the two
    // unavoidable embedded-library ERC warnings; patch an existing project
    // (e.g. the one `pcb layout` maintains) or create a minimal one.
    let pro_path = out_dir.join(format!("{project_name}.kicad_pro"));
    let existing = match std::fs::read_to_string(&pro_path) {
        Ok(text) => Some(text),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => None,
        Err(err) => {
            return Err(err).with_context(|| format!("Failed to read {}", pro_path.display()));
        }
    };
    let pro_content = pcb_schematic::kicad_pro_content(existing.as_deref(), &project_name)?;
    std::fs::write(&pro_path, pro_content)
        .with_context(|| format!("Failed to write {}", pro_path.display()))?;

    for warning in &generated.warnings {
        eprintln!(
            "{} {}",
            "Warning:".with_style(Style::Yellow).bold(),
            warning
        );
    }

    let display_path = zen_path
        .parent()
        .and_then(|parent| sch_path.strip_prefix(parent).ok())
        .unwrap_or(&sch_path);
    println!(
        "{} {} ({})",
        pcb_ui::icons::success(),
        file_name.as_str().with_style(Style::Green).bold(),
        display_path.display()
    );

    if args.check {
        run_erc_check(&sch_path, &file_name)?;
    }

    Ok(())
}

/// Pick the output path of the root sheet: explicit -o first, then the
/// design's layout directory (sharing the KiCad project base name), then a
/// sibling of the source .zen file.
fn resolve_output_path(args: &SchArgs, zen_path: &Path, schematic: &Schematic) -> Result<PathBuf> {
    if let Some(output) = &args.output {
        if output.extension().and_then(|e| e.to_str()) == Some("kicad_sch") {
            return Ok(output.clone());
        }
        // Treat anything else as a directory.
        let stem = zen_path
            .file_stem()
            .context("input file has no name")?
            .to_string_lossy()
            .to_string();
        return Ok(output.join(format!("{stem}.kicad_sch")));
    }

    if let Some(layout_dir) = layout_utils::resolve_layout_dir(schematic)? {
        let files = layout_utils::resolve_kicad_files(&layout_dir)?;
        return Ok(files.kicad_sch());
    }

    Ok(zen_path.with_extension("kicad_sch"))
}

/// Run kicad-cli ERC on the generated schematic and fail on any violation.
fn run_erc_check(sch_path: &Path, file_name: &str) -> Result<()> {
    let spinner = Spinner::builder(format!("{file_name}: Running ERC checks")).start();
    let report = pcb_kicad::run_erc_report(sch_path, sch_path.parent())?;
    spinner.finish();

    let mut error_count = 0usize;
    let mut warning_count = 0usize;
    for sheet in &report.sheets {
        for violation in &sheet.violations {
            if violation.excluded {
                continue;
            }
            if violation.severity == "error" {
                error_count += 1;
            } else {
                warning_count += 1;
            }
            eprintln!(
                "{} [{}] {} (sheet {})",
                match violation.severity.as_str() {
                    "error" => "Error:".with_style(Style::Red).bold().to_string(),
                    _ => "Warning:".with_style(Style::Yellow).bold().to_string(),
                },
                violation.violation_type,
                violation.description,
                sheet.path
            );
        }
    }

    // Same convention as `pcb layout --check` DRC: errors are fatal,
    // warnings are reported but do not fail the command.
    if error_count > 0 {
        anyhow::bail!("ERC reported {error_count} error(s), {warning_count} warning(s)");
    }
    if warning_count > 0 {
        println!(
            "{} {} (ERC: 0 errors, {warning_count} warning(s))",
            pcb_ui::icons::success(),
            file_name.with_style(Style::Green).bold()
        );
    } else {
        println!(
            "{} {} (ERC clean)",
            pcb_ui::icons::success(),
            file_name.with_style(Style::Green).bold()
        );
    }
    Ok(())
}
