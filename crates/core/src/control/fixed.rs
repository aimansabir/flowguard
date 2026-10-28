use std::collections::BTreeMap;

use super::{Controller, DecisionInput};
use crate::corridor::CorridorIndex;
use crate::error::{Error, Result};

/// Fixed-time plan, the same greens at every junction.
///
/// Asks for phase k until its planned green is used up, then for the next.
/// The envelope adds yellow + all-red in between, so the cycle is
/// sum(greens) + one clearance per phase. A junction's offset shifts its
/// plan later: at time t it asks for what the plan says at t - offset
/// (green wave, D21). No offsets = every junction in step.
pub struct FixedTime {
    junction_ids: Vec<String>,
    offsets: Vec<f64>,
    green_ends: Vec<f64>,
    cycle: f64,
}

impl FixedTime {
    pub fn new(
        corridor: &CorridorIndex,
        greens: &[f64],
        offsets_s: &BTreeMap<String, f64>,
        clearance_s: f64,
    ) -> Result<Self> {
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
        let junction_ids: Vec<String> = corridor.corridor.junctions.iter().map(|j| j.id.clone()).collect();
        for (id, off) in offsets_s {
            if !junction_ids.contains(id) {
                return Err(Error::Config(format!("fixed plan offset for unknown junction {id}")));
            }
            if !off.is_finite() {
                return Err(Error::Config(format!("fixed plan offset for {id} must be finite")));
            }
        }
        let offsets = junction_ids.iter().map(|id| offsets_s.get(id).copied().unwrap_or(0.0)).collect();
        Ok(FixedTime { junction_ids, offsets, green_ends, cycle: t })
    }

    fn phase_at(&self, t: f64) -> usize {
        let pos = t.rem_euclid(self.cycle);
        self.green_ends.iter().position(|&end| pos < end).unwrap_or(0)
    }
}

impl Controller for FixedTime {
    fn name(&self) -> &str {
        "fixed"
    }

    fn decide(&mut self, input: &DecisionInput<'_>) -> BTreeMap<String, usize> {
        self.junction_ids.iter().zip(&self.offsets).map(|(j, off)| (j.clone(), self.phase_at(input.t - off))).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::corridor::tests::two_junctions;

    #[test]
    fn plan_positions() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let f = FixedTime::new(&ix, &[40.0, 20.0], &BTreeMap::new(), 5.0).unwrap();
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
        let none = BTreeMap::new();
        assert!(FixedTime::new(&ix, &[40.0], &none, 5.0).is_err());
        assert!(FixedTime::new(&ix, &[40.0, 0.0], &none, 5.0).is_err());
    }

    fn offsets(pairs: &[(&str, f64)]) -> BTreeMap<String, f64> {
        pairs.iter().map(|(j, o)| ((*j).to_string(), *o)).collect()
    }

    #[test]
    fn offsets_shift_each_junction() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        let mut f = FixedTime::new(&ix, &[40.0, 20.0], &offsets(&[("B", 20.0)]), 5.0).unwrap();
        let ask = |f: &mut FixedTime, t: f64| {
            let last = [None, None];
            let input = DecisionInput { t, corridor: &ix, states: &[None, None, None, None], last_commands: &last };
            let r = f.decide(&input);
            (r["A"], r["B"])
        };
        assert_eq!(ask(&mut f, 45.0), (1, 0)); // B is 20 s behind: still in its main green
        assert_eq!(ask(&mut f, 60.0), (1, 1));
        // before its first cycle starts, B is where its plan was one cycle earlier
        assert_eq!(ask(&mut f, 0.0), (0, 1));
        // an offset of a whole cycle changes nothing
        let mut whole = FixedTime::new(&ix, &[40.0, 20.0], &offsets(&[("B", 70.0)]), 5.0).unwrap();
        assert_eq!(ask(&mut whole, 45.0), (1, 1));
    }

    #[test]
    fn offsets_must_name_known_junctions() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        assert!(FixedTime::new(&ix, &[40.0, 20.0], &offsets(&[("Z", 5.0)]), 5.0).is_err());
        assert!(FixedTime::new(&ix, &[40.0, 20.0], &offsets(&[("A", f64::INFINITY)]), 5.0).is_err());
    }
}
