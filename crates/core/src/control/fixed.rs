use std::collections::BTreeMap;

use super::{Controller, DecisionInput};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};

/// Plain fixed-time plan, same at every junction, no offsets between them.
///
/// Asks for phase k until its planned green is used up, then for the next.
/// The envelope adds yellow + all-red in between, so the cycle is
/// sum(greens) + one clearance per phase.
pub struct FixedTime {
    junction_ids: Vec<String>,
    green_ends: Vec<f64>,
    cycle: f64,
}

impl FixedTime {
    pub fn new(corridor: &CorridorIndex, greens: &[f64], clearance_s: f64) -> Result<Self> {
        for j in &corridor.corridor.junctions {
            if j.phases.len() != greens.len() {
                return Err(Error::Config(format!("{} has {} phases, plan has {}", j.id, j.phases.len(), greens.len())));
            }
        }
        if greens.iter().any(|g| !g.is_finite() || *g <= 0.0) {
            return Err(Error::Config("fixed plan greens must be > 0".into()));
        }
        let (mut t, mut green_ends) = (0.0, Vec::with_capacity(greens.len()));
        for g in greens {
            t += g;
            green_ends.push(t);
            t += clearance_s;
        }
        let junction_ids = corridor.corridor.junctions.iter().map(|j| j.id.clone()).collect();
        Ok(FixedTime { junction_ids, green_ends, cycle: t })
    }

    fn phase_at(&self, t: f64) -> usize {
        let pos = t % self.cycle;
        self.green_ends.iter().position(|&end| pos < end).unwrap_or(0)
    }
}

impl Controller for FixedTime {
    fn name(&self) -> &str {
        "fixed"
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        let phase = self.phase_at(input.t);
        self.junction_ids.iter().map(|j| (j.clone(), phase)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corridor::tests::two_junctions;

    #[test]
    fn plan_positions() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let f = FixedTime::new(&ix, &[40.0, 20.0], 5.0).unwrap();
        assert_eq!(f.cycle, 70.0);
        assert_eq!(f.phase_at(0.0), 0);
        assert_eq!(f.phase_at(39.0), 0);
        assert_eq!(f.phase_at(40.0), 1);
        assert_eq!(f.phase_at(64.0), 1);
        assert_eq!(f.phase_at(65.0), 0); // clearance after the last green asks for phase 0
        assert_eq!(f.phase_at(110.0), 1);
    }

    #[test]
    fn plan_must_match_phase_count() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        assert!(FixedTime::new(&ix, &[40.0], 5.0).is_err());
        assert!(FixedTime::new(&ix, &[40.0, 0.0], 5.0).is_err());
    }
}
