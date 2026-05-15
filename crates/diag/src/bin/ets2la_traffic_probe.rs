//! ETS2LA Traffic Probe — verifies the `Local\ETS2LATraffic` shared-memory
//! buffer is opened by the ETS2LA-Fork DLL and (optionally) contains
//! non-zero vehicle slot data. Closes V4/V5 of the ETS2LA Pre-Check.
//!
//! Raw `extern "system"` FFI per spec — no new crate dependencies.

#[cfg(windows)]
fn main() {
    windows_impl::run();
}

#[cfg(not(windows))]
fn main() {
    println!("SKIP: ETS2LA traffic probe requires Windows.");
    println!("This binary reads the ETS2LATraffic shared memory buffer");
    println!("which only exists on Windows with ETS2 running.");
    std::process::exit(0);
}

#[cfg(windows)]
mod windows_impl {
    use std::ffi::c_void;

    extern "system" {
        fn OpenFileMappingA(
            dwDesiredAccess: u32,
            bInheritHandle: i32,
            lpName: *const u8,
        ) -> *mut c_void;

        fn MapViewOfFile(
            hFileMappingObject: *mut c_void,
            dwDesiredAccess: u32,
            dwFileOffsetHigh: u32,
            dwFileOffsetLow: u32,
            dwNumberOfBytesToMap: usize,
        ) -> *mut u8;

        fn UnmapViewOfFile(lpBaseAddress: *const u8) -> i32;
        fn CloseHandle(hObject: *mut c_void) -> i32;
        fn GetLastError() -> u32;
    }

    const FILE_MAP_READ: u32 = 4;
    const TRAFFIC_BUFFER_NAME: &[u8] = b"Local\\ETS2LATraffic\0";
    const PARKED_BUFFER_NAME: &[u8] = b"Local\\ETS2LAParkedVehicles\0";
    const TRAFFIC_BUFFER_SIZE: usize = 6960;
    const PARKED_BUFFER_SIZE: usize = 1720;
    const VEHICLE_SLOT_SIZE: usize = 174;
    const NUM_VEHICLE_SLOTS: usize = 40;

    /// RAII wrapper over an `OpenFileMappingA` + `MapViewOfFile` pair so
    /// the view + handle get released on every exit path (early `return`,
    /// `process::exit` won't run drop — we manually drop before exit).
    struct MappedView {
        handle: *mut c_void,
        ptr: *mut u8,
        size: usize,
    }

    impl Drop for MappedView {
        fn drop(&mut self) {
            unsafe {
                if !self.ptr.is_null() {
                    UnmapViewOfFile(self.ptr);
                }
                if !self.handle.is_null() {
                    CloseHandle(self.handle);
                }
            }
        }
    }

    fn open_shm_buffer(name: &[u8], size: usize) -> Option<MappedView> {
        unsafe {
            let handle = OpenFileMappingA(FILE_MAP_READ, 0, name.as_ptr());
            if handle.is_null() {
                eprintln!(
                    "OpenFileMappingA failed (GetLastError = {})",
                    GetLastError()
                );
                return None;
            }
            let ptr = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size);
            if ptr.is_null() {
                eprintln!("MapViewOfFile failed (GetLastError = {})", GetLastError());
                CloseHandle(handle);
                return None;
            }
            Some(MappedView { handle, ptr, size })
        }
    }

    fn hex_dump(buffer: &[u8], num_bytes: usize) {
        let limit = num_bytes.min(buffer.len());
        for row_start in (0..limit).step_by(16) {
            let row_end = (row_start + 16).min(limit);
            let row = &buffer[row_start..row_end];
            let mut hex = String::with_capacity(48);
            let mut ascii = String::with_capacity(16);
            for byte in row {
                hex.push_str(&format!("{byte:02x} "));
                ascii.push(if (0x20..=0x7e).contains(byte) {
                    *byte as char
                } else {
                    '.'
                });
            }
            for _ in row.len()..16 {
                hex.push_str("   ");
            }
            println!("{row_start:08x}  {hex} {ascii}");
        }
    }

    fn count_active_slots(buffer: &[u8]) -> usize {
        let mut active = 0;
        for slot_idx in 0..NUM_VEHICLE_SLOTS {
            let slot_start = slot_idx * VEHICLE_SLOT_SIZE;
            if slot_start + 12 > buffer.len() {
                break;
            }
            let Ok(x_bytes) = buffer[slot_start..slot_start + 4].try_into() else {
                continue;
            };
            let Ok(y_bytes) = buffer[slot_start + 4..slot_start + 8].try_into() else {
                continue;
            };
            let Ok(z_bytes) = buffer[slot_start + 8..slot_start + 12].try_into() else {
                continue;
            };
            let x = f32::from_le_bytes(x_bytes);
            let y = f32::from_le_bytes(y_bytes);
            let z = f32::from_le_bytes(z_bytes);
            if x == 0.0 && y == 0.0 && z == 0.0 {
                continue;
            }
            active += 1;
            let implausible =
                x.abs() > 1_000_000.0 || y.abs() > 1_000_000.0 || z.abs() > 1_000_000.0;
            if implausible {
                println!(
                    "WARN: Slot {slot_idx} has implausible position ({x:.1}, {y:.1}, {z:.1})"
                );
            }
        }
        active
    }

    pub fn run() {
        println!("ETS2LA Traffic Probe v1.0");

        let traffic = match open_shm_buffer(TRAFFIC_BUFFER_NAME, TRAFFIC_BUFFER_SIZE) {
            Some(view) => view,
            None => {
                println!("FAIL: ETS2LATraffic buffer not found");
                println!("  - Is ETS2 running?");
                println!("  - Is ets2la_plugin.dll in plugins/?");
                println!("  - Are you in-game (not main menu)?");
                std::process::exit(1);
            }
        };
        println!("PASS: ETS2LATraffic buffer found ({} bytes)", traffic.size);

        // `slice` borrows from the mapped view; both go out of scope at
        // the explicit `drop(traffic)` below.
        let slice: &[u8] = unsafe { std::slice::from_raw_parts(traffic.ptr, traffic.size) };

        println!("--- Hex dump (first 256 bytes) ---");
        hex_dump(slice, 256);

        let active = count_active_slots(slice);
        println!("INFO: Non-zero vehicle slots: {active}/{NUM_VEHICLE_SLOTS}");
        if active == 0 {
            println!("WARN: Buffer is all zeros — DLL may need backend trigger");
        } else {
            println!("PASS: Vehicle slots contain data — DLL is standalone");
        }

        drop(traffic);

        match open_shm_buffer(PARKED_BUFFER_NAME, PARKED_BUFFER_SIZE) {
            Some(parked) => {
                println!(
                    "PASS: ETS2LAParkedVehicles buffer found ({} bytes)",
                    parked.size
                );
                drop(parked);
            }
            None => {
                println!("INFO: ETS2LAParkedVehicles buffer not found (optional)");
            }
        }

        println!("============================================");
        println!("Probe Result: PASS");
        println!("Pfad B1-GPL: GO");
        println!("============================================");
        std::process::exit(0);
    }
}
