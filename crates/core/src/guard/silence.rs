//! E6 (docs/decisions.md D28): improbable silence. A camera that reports
//! nothing at all (no vehicle in the zone, none arriving, none leaving) for
//! T seconds on an approach whose trusted arrival rate is r would, with
//! arrivals at random (Poisson), be right with probability p = exp(-r T).
//! The smaller p, the stronger the evidence that the camera is blind rather
//! than the road empty. This is what separates a blackout from an empty
//! road on approaches no other check covers (entries), and it only claims
//! what the traffic volume supports: on a quiet road a short blackout stays
//! plausible and scores nothing.

use crate::contracts::{Measurement, SilenceEvidence};
use crate::error::{Error, Result};

pub fn validate(c: &SilenceEvidence) -> Result<()> {
    if !(0.0 < c.p_full && c.p_full < c.p_start && c.p_start < 1.0) {
        return Err(Error::Config("silence evidence needs 0 < p_full < p_start < 1".into()));
    }
    Ok(())
}

/// Nothing in the zone, nothing arriving, nothing leaving.
pub fn is_empty(m: &Measurement) -> bool {
    m.zone_counts.total() == 0
        && m.queue_counts.total() == 0
        && m.arrivals.total() == 0
        && m.departures.total().total() == 0
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Silence {
    /// Seconds of consecutive empty measurements.
    silent_s: f64,
    /// Trusted arrival rate when the silence began, PCU/s; held for the
    /// whole silence, so the silence cannot talk its own rate down. None if
    /// there was no settled rate then: the whole silence goes unjudged.
    rate: Option<f64>,
}

impl Silence {
    /// One measured interval. `rate` is the approach's settled trusted
    /// arrival rate now (G3), in PCU/s: fewer vehicles than PCU when
    /// motorcycles are counted, so p comes out larger, i.e. conservative.
    /// Returns the E6 score: 0 when something was seen, None for a silence
    /// that began without a settled rate.
    pub fn update(&mut self, empty: bool, interval: f64, rate: Option<f64>, c: &SilenceEvidence) -> Option<f64> {
        if !empty {
            self.silent_s = 0.0;
            return Some(0.0);
        }
        if self.silent_s <= 0.0 {
            self.rate = rate;
        }
        self.silent_s += interval;
        // -log10 p for p = exp(-rate x seconds)
        let surprise = self.rate? * self.silent_s / std::f64::consts::LN_10;
        let (start, full) = (-c.p_start.log10(), -c.p_full.log10());
        Some(((surprise - start) / (full - start)).clamp(0.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Scores of `n` empty seconds at a settled trusted rate.
    fn silence(rate: f64, n: usize) -> Vec<f64> {
        let mut s = Silence::default();
        (0..n).map(|_| s.update(true, 1.0, Some(rate), &SilenceEvidence::default()).unwrap()).collect()
    }

    #[test]
    fn no_settled_rate_cannot_judge() {
        let mut s = Silence::default();
        let c = SilenceEvidence::default();
        assert!((0..3600).all(|_| s.update(true, 1.0, None, &c).is_none()));
        // a rate that settles mid-silence does not start judging it
        assert_eq!(s.update(true, 1.0, Some(5.0), &c), None);
        assert_eq!(s.update(false, 1.0, Some(5.0), &c), Some(0.0));
    }

    #[test]
    fn a_road_with_no_traffic_on_record_is_no_evidence() {
        assert!(silence(0.0, 3600).iter().all(|&x| x == 0.0));
    }

    #[test]
    fn busy_road_gone_silent_builds_evidence() {
        // 0.2 PCU/s: p = 1e-3 after 34.5 s, 1e-5 after 57.6 s
        let s = silence(0.2, 60);
        assert_eq!(s[33], 0.0);
        assert!(s[45] > 0.4 && s[45] < 0.6, "{}", s[45]);
        assert_eq!(s[57], 1.0);
    }

    #[test]
    fn quiet_road_two_minutes_empty_is_plausible() {
        // 0.04 PCU/s (about 150 vehicles an hour): two empty minutes have p of
        // about 0.008, which happens on a real side street
        assert!(silence(0.04, 120).iter().all(|&x| x == 0.0));
    }

    #[test]
    fn any_vehicle_resets_and_the_rate_is_held() {
        let c = SilenceEvidence::default();
        let mut s = Silence::default();
        for _ in 0..50 {
            s.update(true, 1.0, Some(0.2), &c);
        }
        assert_eq!(s.update(false, 1.0, Some(0.2), &c), Some(0.0));
        // the rate passed in later does not matter until the next silence starts
        let mut s = Silence::default();
        s.update(true, 1.0, Some(0.2), &c);
        let late = (1..60).map(|_| s.update(true, 1.0, Some(0.0), &c)).last().unwrap();
        assert_eq!(late, Some(1.0));
    }

    #[test]
    fn rejects_bad_config() {
        assert!(validate(&SilenceEvidence::default()).is_ok());
        assert!(validate(&SilenceEvidence { p_start: 1e-5, p_full: 1e-3 }).is_err());
        assert!(validate(&SilenceEvidence { p_start: 1.5, p_full: 1e-3 }).is_err());
    }
}
