//! E4, command response: under green a queue should discharge, under red
//! nothing should cross the stop line. Judged against the envelope's
//! Command, never the controller's request. Thresholds: docs/decisions.md D25.

use std::collections::VecDeque;

use super::{EvidenceParams, Signal};

/// One interval of an approach, as E4 needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ResponseInput {
    pub signal: Signal,
    pub interval: f64,
    /// Stopped vehicles in the zone at the start of the interval; None when
    /// the previous measurement is missing.
    pub queue_before: Option<u32>,
    /// Stop-line departures this interval: left, straight, right.
    pub departures: [u32; 3],
    /// Whether a link this approach feeds is full (spillback); None = cannot tell.
    pub blocked: Option<bool>,
}

#[derive(Debug, Clone, Default)]
pub struct Response {
    /// Seconds of green with a queue and no departure, since the last departure under green.
    stalled_s: f64,
    /// (time, interval, red, departures by movement), last `red_window_s`.
    history: VecDeque<(f64, f64, bool, [u32; 3])>,
}

impl Response {
    /// (stalled green, red departures) scores.
    pub fn update(&mut self, t: f64, x: &ResponseInput, p: &EvidenceParams) -> (f64, f64) {
        // a stall counts only after the start-up delay, with a queue waiting
        // and the exit known to be free; red and clearing hold the count
        if let Signal::Green { elapsed } = x.signal {
            if x.departures.iter().sum::<u32>() > 0 {
                self.stalled_s = 0.0;
            } else if elapsed > p.startup_s && x.queue_before.is_some_and(|q| q > 0) && x.blocked == Some(false) {
                self.stalled_s += x.interval;
            }
        }
        let stalled = (self.stalled_s / p.stall_s).min(1.0) * p.stall_weight;

        self.history.push_back((t, x.interval, x.signal == Signal::Red, x.departures));
        while self.history.front().is_some_and(|h| t - h.0 >= p.red_window_s) {
            self.history.pop_front();
        }
        let mut red_s = 0.0;
        let mut departed = [0u32; 3];
        for &(_, dt, red, d) in &self.history {
            if red {
                red_s += dt;
                for (n, k) in departed.iter_mut().zip(d) {
                    *n += k;
                }
            }
        }
        let mut red = 0.0f64;
        for (&n, &tol) in departed.iter().zip(&p.red_tolerance_per_min) {
            let per_min = f64::from(n) * 60.0 / red_s.max(p.red_floor_s);
            red = red.max(((per_min - tol) / tol).clamp(0.0, 1.0));
        }
        (stalled, red)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn green(elapsed: f64) -> Signal {
        Signal::Green { elapsed }
    }

    fn input(signal: Signal, queue_before: u32, departures: [u32; 3]) -> ResponseInput {
        ResponseInput { signal, interval: 1.0, queue_before: Some(queue_before), departures, blocked: Some(false) }
    }

    /// Feeds a green phase that started at t = 0; returns the last scores.
    fn green_run(n: usize, f: impl Fn(usize) -> ResponseInput) -> (f64, f64) {
        let p = EvidenceParams::default();
        let mut r = Response::default();
        let mut out = (0.0, 0.0);
        for t in 1..=n {
            out = r.update(t as f64, &f(t), &p);
        }
        out
    }

    #[test]
    fn healthy_green_discharge() {
        let s = green_run(60, |t| input(green(t as f64), 8, if t % 2 == 0 { [0, 1, 0] } else { [0; 3] }));
        assert_eq!(s, (0.0, 0.0));
    }

    #[test]
    fn arrivals_during_healthy_green_do_not_matter() {
        // the queue grows (more arrive than leave) but departures keep coming
        let s = green_run(60, |t| input(green(t as f64), 8 + t as u32 / 3, if t % 3 == 0 { [1, 0, 0] } else { [0; 3] }));
        assert_eq!(s.0, 0.0);
    }

    #[test]
    fn persistent_queue_without_departures_is_weak_evidence() {
        let p = EvidenceParams::default();
        let s = green_run(60, |t| input(green(t as f64), 8, [0; 3]));
        assert_eq!(s.0, p.stall_weight);
    }

    #[test]
    fn startup_delay_is_not_a_stall() {
        // no departures in the first startup_s seconds, then normal discharge
        let s = green_run(30, |t| input(green(t as f64), 8, if t > 4 && t % 2 == 0 { [0, 1, 0] } else { [0; 3] }));
        assert_eq!(s.0, 0.0);
        // a whole short green spent in start-up builds nothing either
        let s = green_run(4, |t| input(green(t as f64), 8, [0; 3]));
        assert_eq!(s.0, 0.0);
    }

    #[test]
    fn empty_approach_under_green_is_fine() {
        assert_eq!(green_run(60, |t| input(green(t as f64), 0, [0; 3])), (0.0, 0.0));
    }

    #[test]
    fn blocked_or_unknown_exit_holds_the_stall() {
        let p = EvidenceParams::default();
        for blocked in [Some(true), None] {
            let s = green_run(60, |t| ResponseInput { blocked, ..input(green(t as f64), 8, [0; 3]) });
            assert_eq!(s.0, 0.0);
        }
        // a departure under green resets what was built up
        let mut r = Response::default();
        for t in 1..=30 {
            r.update(f64::from(t), &input(green(f64::from(t)), 8, [0; 3]), &p);
        }
        assert_eq!(r.update(31.0, &input(green(31.0), 8, [0, 1, 0]), &p).0, 0.0);
    }

    #[test]
    fn missing_previous_measurement_holds_the_stall() {
        let s = green_run(60, |t| ResponseInput { queue_before: None, ..input(green(t as f64), 8, [0; 3]) });
        assert_eq!(s.0, 0.0);
    }

    #[test]
    fn healthy_red_scores_zero() {
        assert_eq!(green_run(120, |_| input(Signal::Red, 8, [0; 3])), (0.0, 0.0));
    }

    #[test]
    fn departures_under_red_are_evidence() {
        // a vehicle every 10 s over red: 6 per minute, three times the tolerance
        let s = green_run(120, |t| input(Signal::Red, 8, if t % 10 == 0 { [0, 1, 0] } else { [0; 3] }));
        assert_eq!(s.1, 1.0);
    }

    #[test]
    fn an_occasional_red_runner_is_tolerated() {
        let s = green_run(120, |t| input(Signal::Red, 8, if t == 60 { [1, 0, 0] } else { [0; 3] }));
        assert_eq!(s.1, 0.0);
    }

    #[test]
    fn red_tolerance_is_per_movement() {
        // a free left turn allowed for in the tolerance, straight-through still judged
        let p = EvidenceParams { red_tolerance_per_min: [20.0, 2.0, 2.0], ..Default::default() };
        let mut r = Response::default();
        let mut out = (0.0, 0.0);
        for t in 1..=120 {
            out = r.update(f64::from(t), &input(Signal::Red, 8, if t % 5 == 0 { [1, 0, 0] } else { [0; 3] }), &p);
        }
        assert_eq!(out.1, 0.0);
        for t in 121..=240 {
            out = r.update(f64::from(t), &input(Signal::Red, 8, if t % 5 == 0 { [0, 1, 0] } else { [0; 3] }), &p);
        }
        assert_eq!(out.1, 1.0);
    }

    #[test]
    fn yellow_and_all_red_are_neither_green_nor_red() {
        let s = green_run(120, |t| input(Signal::Clearing, 8, if t % 2 == 0 { [0, 1, 0] } else { [0; 3] }));
        assert_eq!(s, (0.0, 0.0));
        let s = green_run(120, |_| input(Signal::Clearing, 8, [0; 3]));
        assert_eq!(s, (0.0, 0.0));
    }
}
