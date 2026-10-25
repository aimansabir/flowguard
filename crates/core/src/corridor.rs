//! Validated, indexed view of the corridor map. Built once in `Core::new`; the
//! rest of the core works on indices instead of string lookups.

use std::collections::{BTreeSet, HashMap};

use crate::contracts::{check_version, Corridor, Movement};
use crate::error::{Error, Result};

#[derive(Debug, Clone)]
pub struct CorridorIndex {
    pub corridor: Corridor,
    approach_idx: HashMap<String, usize>,
    /// Approach index -> index of the approach its straight-through traffic
    /// joins at the next junction (None = leaves the corridor).
    pub downstream: Vec<Option<usize>>,
    /// Junction index -> phase -> approach indices that get green.
    pub phases: Vec<Vec<Vec<usize>>>,
    /// Approach index -> junction id each movement leads to, if another junction.
    pub movement_neighbour: Vec<[Option<String>; 3]>,
    /// Junction index -> sorted ids of neighbouring junctions.
    pub neighbours: Vec<Vec<String>>,
}

fn slot(m: Movement) -> usize {
    match m {
        Movement::Left => 0,
        Movement::Straight => 1,
        Movement::Right => 2,
    }
}

impl CorridorIndex {
    pub fn new(corridor: Corridor) -> Result<Self> {
        check_version(&corridor.schema_version)?;
        let bad = |m: String| Err(Error::Corridor(m));

        let mut junction_idx = HashMap::new();
        for (i, j) in corridor.junctions.iter().enumerate() {
            if junction_idx.insert(j.id.clone(), i).is_some() {
                return bad(format!("duplicate junction {}", j.id));
            }
        }
        if junction_idx.is_empty() {
            return bad("no junctions".into());
        }

        let mut approach_idx = HashMap::new();
        for (i, a) in corridor.approaches.iter().enumerate() {
            if approach_idx.insert(a.id.clone(), i).is_some() {
                return bad(format!("duplicate approach {}", a.id));
            }
            if !junction_idx.contains_key(&a.junction) {
                return bad(format!("approach {} names unknown junction {}", a.id, a.junction));
            }
            if !(a.length_m.is_finite() && a.length_m > 0.0) || a.lanes == 0 {
                return bad(format!("approach {} needs length_m > 0 and lanes >= 1", a.id));
            }
        }

        let belongs = |approach: &str, jid: &str| {
            approach_idx.get(approach).is_some_and(|&i| corridor.approaches[i].junction == jid)
        };
        let mut phases = Vec::with_capacity(corridor.junctions.len());
        for j in &corridor.junctions {
            if j.phases.is_empty() || j.phases.iter().any(Vec::is_empty) {
                return bad(format!("{} needs at least one phase and no empty phases", j.id));
            }
            for a in j.phases.iter().flatten().chain(j.links.iter().map(|l| &l.approach)) {
                if !belongs(a, &j.id) {
                    return bad(format!("{} refers to {a}, which is not one of its approaches", j.id));
                }
            }
            phases.push(j.phases.iter().map(|p| p.iter().map(|a| approach_idx[a]).collect()).collect());
        }

        let mut movement_neighbour = Vec::with_capacity(corridor.approaches.len());
        let mut downstream = Vec::with_capacity(corridor.approaches.len());
        let mut neighbours = vec![BTreeSet::new(); corridor.junctions.len()];
        for a in &corridor.approaches {
            let mut row: [Option<String>; 3] = Default::default();
            for m in Movement::ALL {
                let Some(edge) = a.movements.get(m) else { continue };
                if let Some(&k) = approach_idx.get(edge) {
                    let next = &corridor.approaches[k].junction;
                    if *next != a.junction {
                        neighbours[junction_idx[&a.junction]].insert(next.clone());
                        row[slot(m)] = Some(next.clone());
                    }
                }
            }
            downstream.push(a.movements.straight.as_ref().and_then(|e| approach_idx.get(e).copied()));
            movement_neighbour.push(row);
        }

        Ok(CorridorIndex {
            approach_idx,
            downstream,
            phases,
            movement_neighbour,
            neighbours: neighbours.into_iter().map(|s| s.into_iter().collect()).collect(),
            corridor,
        })
    }

    pub fn approach(&self, id: &str) -> Option<usize> {
        self.approach_idx.get(id).copied()
    }

    pub fn junction_of(&self, approach: usize) -> usize {
        let jid = &self.corridor.approaches[approach].junction;
        self.corridor.junctions.iter().position(|j| &j.id == jid).expect("validated in new")
    }

    pub fn neighbour_via(&self, approach: usize, m: Movement) -> Option<&String> {
        self.movement_neighbour[approach][slot(m)].as_ref()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::contracts::{Approach, Junction, MovementTargets, SignalLink};

    fn approach(id: &str, j: &str, straight: Option<&str>) -> Approach {
        Approach {
            id: id.into(),
            junction: j.into(),
            length_m: 300.0,
            lanes: 2,
            movements: MovementTargets { left: None, straight: straight.map(Into::into), right: None },
        }
    }

    /// Two junctions A and B; a_w feeds b_w straight through, side streets leave.
    pub(crate) fn two_junctions() -> Corridor {
        let junction = |id: &str, main: &str, side: &str| Junction {
            id: id.into(),
            phases: vec![vec![main.into()], vec![side.into()]],
            links: vec![
                SignalLink { approach: main.into(), direction: "s".into() },
                SignalLink { approach: side.into(), direction: "s".into() },
            ],
        };
        Corridor {
            schema_version: "2.0".into(),
            junctions: vec![junction("A", "a_w", "a_n"), junction("B", "b_w", "b_n")],
            approaches: vec![
                approach("a_w", "A", Some("b_w")),
                approach("a_n", "A", Some("out_a")),
                approach("b_w", "B", Some("out_e")),
                approach("b_n", "B", None),
            ],
            entries: vec![],
            exits: vec![],
        }
    }

    #[test]
    fn indexes_downstream_and_neighbours() {
        let ix = CorridorIndex::new(two_junctions()).unwrap();
        assert_eq!(ix.downstream, vec![Some(2), None, None, None]);
        assert_eq!(ix.neighbours, vec![vec!["B".to_string()], vec![]]);
        assert_eq!(ix.neighbour_via(0, Movement::Straight), Some(&"B".to_string()));
        assert_eq!(ix.phases[1], vec![vec![2], vec![3]]);
        assert_eq!(ix.junction_of(3), 1);
    }

    #[test]
    fn rejects_phase_with_foreign_approach() {
        let mut c = two_junctions();
        c.junctions[0].phases[0].push("b_w".into());
        assert!(matches!(CorridorIndex::new(c), Err(Error::Corridor(_))));
    }

    #[test]
    fn rejects_duplicates_and_bad_lengths() {
        let mut c = two_junctions();
        c.approaches.push(approach("a_w", "A", None));
        assert!(CorridorIndex::new(c).is_err());
        let mut c = two_junctions();
        c.approaches[0].length_m = 0.0;
        assert!(CorridorIndex::new(c).is_err());
        let mut c = two_junctions();
        c.schema_version = "1.0".into();
        assert!(matches!(CorridorIndex::new(c), Err(Error::Version(_))));
    }
}
