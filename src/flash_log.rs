//! Offline log storage: saves messages to flash during a run so they survive disconnection.
//!
//! Workflow:
//!   1. At boot call `startup_dump(&mut flash)` — replays any saved log via RTT then erases.
//!   2. Use `flash_log!("fmt {}", value)` anywhere during the run (writes to a 32 KB RAM buffer).
//!   3. After the run call `flush(&mut flash)` — writes the buffer to flash (takes ~1–4 s to erase
//!      the sector first, so keep the robot powered until done).
//!
//! Flash sector used: sector 7 on STM32F446RE, offset 0x0006_0000, 128 KB.
//! Entry on-flash format: [u32 msg_len LE][message bytes zero-padded to 4-byte boundary].
//! Magic word 0xFEED_CAFE at sector offset 0 marks a populated log.
//!
//! Buffer capacity: ~32 KB ≈ 190 entries of 160 chars at 20 ms intervals ≈ 3.8 s of full-rate
//! straight-line logging.  Increase RAM_CAPACITY if you need more (mind the 128 KB SRAM limit).

use core::fmt::Write as FmtWrite;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use defmt::{info, println};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_time::Instant;

/// Master switch.  When `false`: `flash_log!` is a no-op (no RTT print, no RAM buffer write),
/// and `startup_dump`/`flush` return immediately without touching flash.
pub const ENABLED: bool = true;

const SECTOR_OFFSET: u32 = 0x0006_0000; // sector 7 start, relative to flash base
const SECTOR_SIZE: u32 = 128 * 1024;
const FLASH_BASE: u32 = 0x0800_0000;
const MAGIC: [u8; 4] = [0xFE, 0xED, 0xCA, 0xFE];
const WRITE_SIZE: usize = 4; // STM32F4 minimum write granularity (x32 mode)

const RAM_CAPACITY: usize = 32 * 1024;

static mut RAM_BUFFER: [u8; RAM_CAPACITY] = [0u8; RAM_CAPACITY];
static RAM_LEN: AtomicUsize = AtomicUsize::new(0);
static OVERFLOW: AtomicBool = AtomicBool::new(false);

struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl FmtWrite for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let n = b.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + n].copy_from_slice(&b[..n]);
        self.pos += n;
        Ok(())
    }
}

/// Format `args` with a millisecond timestamp prefix and append to the RAM buffer.
/// Called by the `flash_log!` macro.
pub fn write_fmt(args: core::fmt::Arguments) {
    if OVERFLOW.load(Ordering::Relaxed) {
        return;
    }
    let mut tmp = [0u8; 256];
    let mut w = SliceWriter { buf: &mut tmp, pos: 0 };
    core::write!(w, "[{}ms] ", Instant::now().as_millis()).ok();
    w.write_fmt(args).ok();
    let msg_len = w.pos;

    // Pad message body to WRITE_SIZE so every entry is a multiple of 4 bytes.
    let padded = (msg_len + WRITE_SIZE - 1) & !(WRITE_SIZE - 1);
    let entry = WRITE_SIZE + padded; // 4-byte length header + padded body

    critical_section::with(|_| {
        let pos = RAM_LEN.load(Ordering::Relaxed);
        if pos + entry > RAM_CAPACITY {
            OVERFLOW.store(true, Ordering::Relaxed);
            return;
        }
        unsafe {
            RAM_BUFFER[pos..pos + 4].copy_from_slice(&(msg_len as u32).to_le_bytes());
            RAM_BUFFER[pos + 4..pos + 4 + msg_len].copy_from_slice(&tmp[..msg_len]);
            for b in &mut RAM_BUFFER[pos + 4 + msg_len..pos + entry] {
                *b = 0;
            }
        }
        RAM_LEN.store(pos + entry, Ordering::Relaxed);
    });
}

/// If sector 7 contains a previous run's log, replay every entry via RTT.
/// Does NOT erase — the sector is only erased when `flush` is called at the end of a maneuver,
/// so accidental power-on will never wipe unread data.
/// Call once at the top of `main` before spawning tasks.
pub fn startup_dump(flash: &mut Flash<'_, Blocking>) {
    if !ENABLED {
        let _ = flash;
        return;
    }
    // Flash is memory-mapped; read via volatile pointer — safe for data (not instruction) reads.
    let base = (FLASH_BASE + SECTOR_OFFSET) as *const u8;
    let magic = unsafe { core::slice::from_raw_parts(base, 4) };
    if magic != MAGIC {
        return;
    }

    info!("=== flash log from previous run ===");
    let mut count = 0u32;
    let mut offset = WRITE_SIZE; // skip 4-byte magic
    loop {
        if offset + WRITE_SIZE > SECTOR_SIZE as usize {
            break;
        }
        let len_bytes = unsafe {
            core::ptr::read_volatile(base.add(offset) as *const [u8; 4])
        };
        let msg_len = u32::from_le_bytes(len_bytes) as usize;
        if msg_len == 0xFFFF_FFFF || msg_len == 0 {
            break; // erased flash or end of entries
        }
        let padded = (msg_len + WRITE_SIZE - 1) & !(WRITE_SIZE - 1);
        if offset + WRITE_SIZE + padded > SECTOR_SIZE as usize {
            break; // corrupted entry
        }
        let msg_bytes = unsafe {
            core::slice::from_raw_parts(base.add(offset + WRITE_SIZE), msg_len)
        };
        if let Ok(s) = core::str::from_utf8(msg_bytes) {
            println!("{}", s);
        }
        count += 1;
        offset += WRITE_SIZE + padded;
    }
    info!("=== end — {} entries (sector kept, will be erased on next flush) ===", count);
    let _ = flash; // consumed for API symmetry; erase deferred to flush()
}

/// Write the RAM buffer to flash.  Call after the run (motors stopped, robot still powered).
/// Erases the sector first (~1–4 s), then writes; signals completion via RTT.
pub fn flush(flash: &mut Flash<'_, Blocking>) {
    if !ENABLED {
        let _ = flash;
        return;
    }
    let len = RAM_LEN.load(Ordering::Relaxed);
    if len == 0 {
        return;
    }
    flash.blocking_erase(SECTOR_OFFSET, SECTOR_OFFSET + SECTOR_SIZE).ok();
    flash.blocking_write(SECTOR_OFFSET, &MAGIC).ok();
    // len is always a multiple of WRITE_SIZE by construction in write_fmt
    flash
        .blocking_write(SECTOR_OFFSET + WRITE_SIZE as u32, unsafe { &RAM_BUFFER[..len] })
        .ok();
    if OVERFLOW.load(Ordering::Relaxed) {
        defmt::warn!("flash_log: RAM buffer overflowed — log is truncated");
    }
    info!("flash_log: {} bytes saved to flash — safe to disconnect", len);
}

#[macro_export]
macro_rules! flash_log {
    ($($arg:tt)*) => {{
        if $crate::flash_log::ENABLED {
            defmt::println!($($arg)*);
            $crate::flash_log::write_fmt(format_args!($($arg)*));
        }
    }};
}
