//! Deterministic UUIDs for generated schematics.
//!
//! Every `(uuid ...)` emitted in a `.kicad_sch` is derived from stable,
//! machine-independent keys (sheet file name, hierarchical instance path
//! relative to the root module, element role, coordinates). Two generations
//! of the same design produce byte-identical files — no randomness anywhere.
//!
//! UUIDs are RFC 4122 version 5 (SHA-1) under a fixed private namespace.

use std::collections::HashMap;
use uuid::Uuid;

/// Separator used when joining key parts. A control character cannot appear
/// in net names, reference designators or file names, so joined keys are
/// unambiguous ("a" + "bc" never collides with "ab" + "c").
const SEP: char = '\u{1f}';

/// Fixed namespace for all pcb-schematic UUIDs.
fn namespace() -> Uuid {
    Uuid::new_v5(&Uuid::NAMESPACE_URL, b"pcb-schematic")
}

/// Stateless deterministic UUID for a unique key (e.g. the sheet root).
pub fn stable_uuid(key: &str) -> String {
    Uuid::new_v5(&namespace(), key.as_bytes()).to_string()
}

/// Per-sheet UUID generator.
///
/// UUIDs are derived from `seed` (the sheet file name) plus the caller's key
/// parts. Identical keys requested multiple times receive distinct UUIDs via
/// an occurrence counter, so repeated geometry (e.g. two wires between the
/// same points) never collides while staying fully deterministic.
#[derive(Debug)]
pub struct UuidGen {
    seed: String,
    counts: HashMap<String, u32>,
}

impl UuidGen {
    pub fn new(seed: impl Into<String>) -> Self {
        Self {
            seed: seed.into(),
            counts: HashMap::new(),
        }
    }

    /// Deterministic UUID for `parts`, unique per identical call.
    pub fn next(&mut self, parts: &[&str]) -> String {
        let key = parts.join(&SEP.to_string());
        let n = self.counts.entry(key.clone()).or_insert(0);
        let uuid = Uuid::new_v5(
            &namespace(),
            format!("{}{SEP}{key}{SEP}{n}", self.seed).as_bytes(),
        );
        *n += 1;
        uuid.to_string()
    }

    /// UUID of the sheet document itself (stable, derived from the seed only).
    pub fn root_uuid(&self) -> String {
        stable_uuid(&format!("{}{SEP}__root__", self.seed))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_key_gets_distinct_sequential_uuids() {
        let mut a = UuidGen::new("sheet.kicad_sch");
        let u1 = a.next(&["wire", "1,2", "3,4"]);
        let u2 = a.next(&["wire", "1,2", "3,4"]);
        assert_ne!(u1, u2);

        // A fresh generator replays the exact same sequence.
        let mut b = UuidGen::new("sheet.kicad_sch");
        assert_eq!(b.next(&["wire", "1,2", "3,4"]), u1);
        assert_eq!(b.next(&["wire", "1,2", "3,4"]), u2);
    }

    #[test]
    fn seed_isolates_sheets() {
        let mut a = UuidGen::new("a.kicad_sch");
        let mut b = UuidGen::new("b.kicad_sch");
        assert_ne!(a.next(&["symbol", "R1"]), b.next(&["symbol", "R1"]));
    }

    #[test]
    fn part_boundaries_are_unambiguous() {
        let mut g = UuidGen::new("s");
        let u1 = g.next(&["ab", "c"]);
        let mut g = UuidGen::new("s");
        let u2 = g.next(&["a", "bc"]);
        assert_ne!(u1, u2);
    }

    #[test]
    fn uuids_are_rfc4122_v5() {
        let u = stable_uuid("anything");
        let parsed = Uuid::parse_str(&u).unwrap();
        assert_eq!(parsed.get_version_num(), 5);
    }
}
