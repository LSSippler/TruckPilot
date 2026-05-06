use std::thread;
use std::time::{Duration, Instant};

use truckpilot::vjoy::ControlOutput;

struct TestTimings {
    steering_duration: Duration,
    throttle_duration: Duration,
    brake_tap_duration: Duration,
    tick: Duration,
}

impl Default for TestTimings {
    fn default() -> Self {
        Self {
            steering_duration: Duration::from_secs(10),
            throttle_duration: Duration::from_secs(20),
            brake_tap_duration: Duration::from_millis(500),
            tick: Duration::from_millis(50),
        }
    }
}

#[cfg(not(windows))]
struct MockOutput {
    steering: f64,
    throttle: f64,
    brake: f64,
}

#[cfg(not(windows))]
impl MockOutput {
    fn new() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
            brake: 0.0,
        }
    }
}

#[cfg(not(windows))]
impl ControlOutput for MockOutput {
    fn set_steering(&mut self, value: f64) {
        self.steering = value;
    }

    fn set_throttle(&mut self, value: f64) {
        self.throttle = value;
    }

    fn set_brake(&mut self, value: f64) {
        self.brake = value;
    }

    fn flush(&mut self) {}
}

fn run_vjoy_test(output: &mut dyn ControlOutput, timings: &TestTimings) {
    println!("Lenkung: Sinus-Welle gestartet");
    let start = Instant::now();
    while start.elapsed() < timings.steering_duration {
        let t = start.elapsed().as_secs_f64();
        let steering =
            (t * std::f64::consts::PI * 2.0 / timings.steering_duration.as_secs_f64()).sin();
        output.set_steering(steering);
        output.set_throttle(0.0);
        output.set_brake(0.0);
        output.flush();
        thread::sleep(timings.tick);
    }

    println!("Gas: Rampe gestartet");
    let start = Instant::now();
    let half = timings.throttle_duration.as_secs_f64() / 2.0;
    while start.elapsed() < timings.throttle_duration {
        let elapsed = start.elapsed().as_secs_f64();
        let throttle = if elapsed <= half {
            (elapsed / half).clamp(0.0, 1.0)
        } else {
            ((timings.throttle_duration.as_secs_f64() - elapsed) / half).clamp(0.0, 1.0)
        };
        output.set_steering(0.0);
        output.set_throttle(throttle);
        output.set_brake(0.0);
        output.flush();
        thread::sleep(timings.tick);
    }
    println!("Gas: 100% erreicht und zurückgeführt");

    println!("Bremse: 100% Tap für 0.5s");
    let start = Instant::now();
    while start.elapsed() < timings.brake_tap_duration {
        output.set_steering(0.0);
        output.set_throttle(0.0);
        output.set_brake(1.0);
        output.flush();
        thread::sleep(timings.tick);
    }

    output.set_steering(0.0);
    output.set_throttle(0.0);
    output.set_brake(0.0);
    output.flush();
    println!("Achsen auf Mittelstellung zurückgesetzt");
    println!("vJoy-Test abgeschlossen. Alle Achsen getestet.");
}

fn main() {
    #[cfg(windows)]
    {
        println!("vJoy-Test startet auf Gerät 1");
        if let Some(mut vjoy) = truckpilot::vjoy::VJoyOutput::try_acquire(1) {
            run_vjoy_test(&mut vjoy, &TestTimings::default());
        } else {
            eprintln!("vJoy Gerät 1 nicht verfügbar. Test wird mit ConsoleOutput ausgeführt.");
            let mut fallback = truckpilot::vjoy::ConsoleOutput;
            run_vjoy_test(&mut fallback, &TestTimings::default());
        }
    }

    #[cfg(not(windows))]
    {
        println!("Kein Windows/vJoy erkannt. Test läuft im Mock-Modus.");
        let mut mock = MockOutput::new();
        run_vjoy_test(&mut mock, &TestTimings::default());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestOutput {
        flush_count: usize,
    }

    impl TestOutput {
        fn new() -> Self {
            Self { flush_count: 0 }
        }
    }

    impl ControlOutput for TestOutput {
        fn set_steering(&mut self, _value: f64) {}
        fn set_throttle(&mut self, _value: f64) {}
        fn set_brake(&mut self, _value: f64) {}
        fn flush(&mut self) {
            self.flush_count += 1;
        }
    }

    #[test]
    fn test_vjoy_test_runs() {
        let mut out = TestOutput::new();
        let timings = TestTimings {
            steering_duration: Duration::from_millis(20),
            throttle_duration: Duration::from_millis(20),
            brake_tap_duration: Duration::from_millis(20),
            tick: Duration::from_millis(1),
        };

        run_vjoy_test(&mut out, &timings);
        assert!(out.flush_count > 0);
    }
}
