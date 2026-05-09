//! TruckPilot Telemetry Probe
//!
//! Reads one telemetry frame and prints all fields.
//! Useful to verify the DLL is running and SHM data is correct.
//!
//! Usage:
//!   telemetry-probe           # read once and exit
//!   telemetry-probe --watch   # print every second until Ctrl+C

use std::time::Duration;

fn main() {
    let watch = std::env::args().any(|a| a == "--watch");

    loop {
        match truckpilot_telemetry::shm::ShmReader::open() {
            Ok(mut reader) => match reader.read() {
                Some(t) => {
                    println!("=== TruckPilot Telemetry ===");
                    println!("  Source       : SHM (truckpilot_telemetry.dll)");
                    println!(
                        "  Position     : x={:.1}  y={:.1}  z={:.1}",
                        t.position[0], t.position[1], t.position[2]
                    );
                    println!(
                        "  Heading      : {:.3} rad  ({:.1}°)",
                        t.heading,
                        t.heading.to_degrees()
                    );
                    println!("  Speed        : {:.1} km/h", t.speed_ms * 3.6);
                    println!("  Engine RPM   : {:.0}", t.engine_rpm);
                    println!(
                        "  Cruise ctrl  : {} km/h",
                        if t.cruise_control_kmh > 0.0 {
                            format!("{:.0}", t.cruise_control_kmh)
                        } else {
                            "off".into()
                        }
                    );
                    println!(
                        "  Nav limit    : {}",
                        if t.nav_speed_limit_kmh > 0.0 {
                            format!("{:.0} km/h", t.nav_speed_limit_kmh)
                        } else {
                            "unknown".into()
                        }
                    );
                    println!("  Fuel         : {:.1} L", t.accel_longitudinal); // placeholder
                    println!("  Accel (long) : {:.2} m/s²", t.accel_longitudinal);
                    if !watch {
                        return;
                    }
                    println!();
                }
                None => {
                    eprintln!("SHM region found but data invalid — DLL loaded but no frame yet.");
                    eprintln!("Make sure ETS2 is running and you are in-game (not in the menu).");
                    if !watch {
                        std::process::exit(1);
                    }
                }
            },
            Err(e) => {
                eprintln!("SHM not available: {e}");
                eprintln!("→ Copy truckpilot_telemetry.dll to ETS2/bin/win_x64/plugins/");
                eprintln!("→ Start ETS2 and load a save");
                if !watch {
                    std::process::exit(1);
                }
            }
        }

        if watch {
            std::thread::sleep(Duration::from_secs(1));
        }
    }
}
