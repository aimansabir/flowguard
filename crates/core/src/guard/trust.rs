//! G2 (docs/specs/guard.md): one score per approach from the G1 evidence,
//! and the TRUSTED / SUSPECT / FAULTY state machine with hysteresis. Rules
//! and defaults: docs/decisions.md D26.

use crate::contracts::{CheckWeights, TrustConfig, TrustState};
use crate::error::{Error, Result};
use crate::guard::evidence::Evidence;

/// Noisy-OR of the checks that could judge this second:
/// `1 - product(1 - weight x score)`. Checks with no score (None) are left
/// out, not taken as clean. None when no check could judge at all.
///
/// Independent signs add up (two at 0.5 give 0.75), a single check is never
/// inflated, and the result stays in 0-1. The weak checks keep their caps
/// (E1, E2 on a standing queue and E4 stalled green at 0.5), so even all
/// three together reach 1 - 0.5^3 = 0.875: below the default
/// `faulty_score` 0.9, so weak evidence alone never makes an approach FAULTY.
pub fn combine(e: &Evidence, w: &CheckWeights) -> Option<f64> {
    let checks = [
        (e.quality, w.quality),
        (e.frozen, w.frozen),
        (e.conservation, w.conservation),
        (e.cross_junction, w.cross_junction),
        (e.stalled_green, w.stalled_green),
        (e.red_departures, w.red_departures),
        (e.silence, w.silence),
    ];
    let mut clean = 1.0;
    let mut judged = false;
    for (score, weight) in checks {
        if let Some(s) = score {
            judged = true;
            clean *= 1.0 - (weight * s).clamp(0.0, 1.0);
        }
    }
    judged.then_some(1.0 - clean)
}

/// Whether a low score may count toward recovery: the camera's own
/// consistency check (E5) must be able to judge. Right after a gap, before
/// E5 has history, a low score proves nothing and holds the recovery timer.
/// E3 is not required: it depends on neighbouring cameras.
pub fn can_confirm_clean(e: &Evidence) -> bool {
    e.conservation.is_some()
}

pub fn validate(c: &TrustConfig, w: &CheckWeights) -> Result<()> {
    let fraction = |x: f64| (0.0..=1.0).contains(&x);
    let weights =
        [w.quality, w.frozen, w.conservation, w.cross_junction, w.stalled_green, w.red_departures, w.silence];
    let ok = c.suspect_score > 0.0
        && c.suspect_score <= c.faulty_score
        && c.faulty_score <= 1.0
        && [c.suspect_s, c.faulty_s, c.recover_s].iter().all(|&s| s >= 0.0)
        && weights.into_iter().all(fraction);
    if !ok {
        return Err(Error::Config("invalid guard trust or combine parameters".into()));
    }
    Ok(())
}

/// Trust state of one approach.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Trust {
    pub state: TrustState,
    /// Consecutive seconds with score >= suspect_score.
    suspect_s: f64,
    /// Consecutive seconds with score >= faulty_score.
    faulty_s: f64,
    /// Seconds below suspect_score that could be confirmed clean, since the
    /// last score at or above it.
    clean_s: f64,
}

impl Default for Trust {
    fn default() -> Self {
        Trust { state: TrustState::Trusted, suspect_s: 0.0, faulty_s: 0.0, clean_s: 0.0 }
    }
}

impl Trust {
    /// One interval. `score` None (nothing could judge, or no measurement)
    /// holds every timer and the state. A score at or above suspect_score
    /// grows the suspicion timers and resets recovery; a lower one resets
    /// them and, if `confirmed`, grows recovery (otherwise holds it).
    pub fn update(&mut self, score: Option<f64>, confirmed: bool, interval: f64, c: &TrustConfig) -> TrustState {
        let Some(score) = score else { return self.state };
        if score >= c.suspect_score {
            self.suspect_s += interval;
            self.clean_s = 0.0;
        } else {
            self.suspect_s = 0.0;
            if confirmed {
                self.clean_s += interval;
            }
        }
        if score >= c.faulty_score {
            self.faulty_s += interval;
        } else {
            self.faulty_s = 0.0;
        }

        // strong evidence counts from when it began, also while still TRUSTED,
        // but FAULTY is only reached through SUSPECT
        self.state = match self.state {
            TrustState::Trusted if self.suspect_s >= c.suspect_s => TrustState::Suspect,
            TrustState::Suspect if self.faulty_s >= c.faulty_s => TrustState::Faulty,
            TrustState::Suspect | TrustState::Faulty if self.clean_s >= c.recover_s => TrustState::Trusted,
            s => s,
        };
        if self.state == TrustState::Trusted && self.suspect_s == 0.0 {
            self.clean_s = 0.0;
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> TrustConfig {
        TrustConfig::default() // suspect 0.5 for 5 s, faulty 0.9 for 10 s, recover 30 s
    }

    /// Feeds (score, confirmed) per second; returns the state after each.
    fn run(trust: &mut Trust, rows: impl IntoIterator<Item = (Option<f64>, bool)>) -> Vec<TrustState> {
        rows.into_iter().map(|(s, ok)| trust.update(s, ok, 1.0, &cfg())).collect()
    }

    fn steady(score: f64, n: usize) -> Vec<(Option<f64>, bool)> {
        vec![(Some(score), true); n]
    }

    use TrustState::{Faulty, Suspect, Trusted};

    // ------------------------------------------------------------ combine

    fn ev(scores: [Option<f64>; 6]) -> Evidence {
        Evidence {
            quality: scores[0],
            frozen: scores[1],
            conservation: scores[2],
            cross_junction: scores[3],
            stalled_green: scores[4],
            red_departures: scores[5],
            silence: None,
        }
    }

    #[test]
    fn combine_is_noisy_or_of_available_checks() {
        let w = CheckWeights::default();
        assert_eq!(combine(&ev([None; 6]), &w), None);
        assert_eq!(combine(&ev([None, Some(0.0), Some(0.0), None, Some(0.0), Some(0.0)]), &w), Some(0.0));
        assert_eq!(combine(&ev([None, Some(1.0), None, None, None, None]), &w), Some(1.0));
        assert_eq!(combine(&ev([Some(0.5), Some(0.5), None, None, None, None]), &w), Some(0.75));
        // a missing check is not a clean one: same score as without it
        assert_eq!(
            combine(&ev([None, Some(0.6), None, None, None, None]), &w),
            combine(&ev([None, Some(0.6), Some(0.0), None, None, None]), &w)
        );
    }

    #[test]
    fn weak_checks_alone_cannot_reach_faulty() {
        let w = CheckWeights::default();
        let all_weak = combine(&ev([Some(0.5), Some(0.5), Some(0.0), Some(0.0), Some(0.5), Some(0.0)]), &w).unwrap();
        assert_eq!(all_weak, 0.875);
        assert!(all_weak < cfg().faulty_score);
        let mut t = Trust::default();
        let states = run(&mut t, steady(all_weak, 600));
        assert_eq!(*states.last().unwrap(), Suspect);
    }

    #[test]
    fn weights_scale_and_drop_checks() {
        let w = CheckWeights { frozen: 0.0, ..Default::default() };
        assert_eq!(combine(&ev([None, Some(1.0), Some(0.0), None, None, None]), &w), Some(0.0));
        let w = CheckWeights { frozen: 0.5, ..Default::default() };
        assert_eq!(combine(&ev([None, Some(1.0), None, None, None, None]), &w), Some(0.5));
    }

    // ------------------------------------------------------------ state machine

    #[test]
    fn clean_approach_stays_trusted() {
        let mut t = Trust::default();
        assert!(run(&mut t, steady(0.1, 600)).iter().all(|&s| s == Trusted));
    }

    #[test]
    fn transitions_at_exact_boundaries() {
        // suspicion from the first second: SUSPECT on the 5th, FAULTY on the 10th
        let mut t = Trust::default();
        let s = run(&mut t, steady(1.0, 12));
        assert_eq!(&s[..4], &[Trusted; 4]);
        assert_eq!(s[4], Suspect);
        assert_eq!(&s[5..9], &[Suspect; 4]);
        assert_eq!(s[9], Faulty);
        // recovery on the 30th clean second
        let s = run(&mut t, steady(0.0, 31));
        assert_eq!(&s[..29], &[Faulty; 29]);
        assert_eq!(s[29], Trusted);
    }

    #[test]
    fn weak_evidence_enters_suspect_only() {
        let mut t = Trust::default();
        let s = run(&mut t, steady(0.5, 300));
        assert_eq!(s[4], Suspect);
        assert!(s.iter().all(|&x| x != Faulty));
    }

    #[test]
    fn short_spike_does_not_transition() {
        let mut t = Trust::default();
        let mut rows = steady(0.0, 10);
        rows.extend(steady(1.0, 4));
        rows.extend(steady(0.0, 10));
        rows.extend(steady(1.0, 4));
        assert!(run(&mut t, rows).iter().all(|&s| s == Trusted));
    }

    #[test]
    fn strong_evidence_must_persist_for_faulty() {
        // in SUSPECT, strong spikes shorter than faulty_s never reach FAULTY
        let mut t = Trust::default();
        run(&mut t, steady(0.6, 5));
        assert_eq!(t.state, Suspect);
        for _ in 0..20 {
            run(&mut t, steady(1.0, 9));
            run(&mut t, steady(0.6, 1));
        }
        assert_eq!(t.state, Suspect);
    }

    #[test]
    fn missing_score_holds_state_and_timers() {
        let mut t = Trust::default();
        run(&mut t, steady(1.0, 3));
        run(&mut t, vec![(None, false); 50]);
        assert_eq!(t.state, Trusted);
        // the 3 seconds already counted still count: 2 more reach SUSPECT
        assert_eq!(run(&mut t, steady(1.0, 2))[1], Suspect);
        // a long gap does not count as clean recovery
        run(&mut t, steady(1.0, 10));
        assert_eq!(t.state, Faulty);
        run(&mut t, vec![(None, false); 100]);
        assert_eq!(t.state, Faulty);
    }

    #[test]
    fn unconfirmed_clean_seconds_hold_recovery() {
        let mut t = Trust::default();
        run(&mut t, steady(1.0, 10));
        assert_eq!(t.state, Faulty);
        // low scores while E5 cannot judge (just after a gap) hold the timer
        run(&mut t, vec![(Some(0.0), false); 100]);
        assert_eq!(t.state, Faulty);
        let s = run(&mut t, steady(0.0, 30));
        assert_eq!(s[28], Faulty);
        assert_eq!(s[29], Trusted);
    }

    #[test]
    fn recovery_timer_resets_when_evidence_returns() {
        let mut t = Trust::default();
        run(&mut t, steady(1.0, 10));
        run(&mut t, steady(0.0, 29));
        assert_eq!(t.state, Faulty);
        run(&mut t, steady(0.7, 1));
        let s = run(&mut t, steady(0.0, 30));
        assert_eq!(s[28], Faulty);
        assert_eq!(s[29], Trusted);
    }

    #[test]
    fn suspect_recovers_without_faulty() {
        let mut t = Trust::default();
        run(&mut t, steady(0.6, 5));
        assert_eq!(t.state, Suspect);
        let s = run(&mut t, steady(0.0, 30));
        assert_eq!(s[29], Trusted);
    }

    #[test]
    fn approaches_are_independent() {
        let (mut a, mut b) = (Trust::default(), Trust::default());
        for _ in 0..20 {
            a.update(Some(1.0), true, 1.0, &cfg());
            b.update(Some(0.0), true, 1.0, &cfg());
        }
        assert_eq!((a.state, b.state), (Faulty, Trusted));
    }

    #[test]
    fn rejects_bad_config() {
        let w = CheckWeights::default();
        assert!(validate(&cfg(), &w).is_ok());
        assert!(validate(&TrustConfig { suspect_score: 0.95, ..cfg() }, &w).is_err());
        assert!(validate(&TrustConfig { recover_s: -1.0, ..cfg() }, &w).is_err());
        assert!(validate(&cfg(), &CheckWeights { quality: 1.5, ..w }).is_err());
    }
}
