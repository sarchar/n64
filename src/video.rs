use std::sync::atomic::Ordering;

#[allow(unused_imports)]
use tracing::{debug,error,trace,warn,info};

use crate::*;
use mips::{InterruptUpdate, InterruptUpdateMode, IMask_VI};

const PAL_BURST: u32 = 0x0404233A;
const NTSC_BURST: u32 = 0x03E52239;

pub struct VideoInterface {
    comms: SystemCommunication,

    resolution_changed: u32,
    cycle_count: u64,

    // VI_CTRL control flags
    dedither_filter_enable: u8,
    pixel_advance: u8,
    kill_we: u8,
    aa_mode: u8,
    test_mode: u8,
    serrate: u8,
    vbus_clock_enable: u8,
    divot_enable: u8,
    gamma_enable: u8,
    gamma_dither_enable: u8,
    pixel_type: u8,

    // VI_ORIGIN frame buffer
    origin: u32,

    // VI_WIDTH buffer width
    frame_buffer_width: u32,

    // VI_V_INTR interrupt line
    interrupt_line: u32,

    // VI_V_CURRENT current line
    current_line: u32,

    // VI_BURST won't be used for the longest time
    burst: u32,

    // VI_V_SYNC vsync line start?
    vsync: u32,

    // VI_H_SYNC hsync pixel start?
    leap_pattern: u8,
    hsync: u32,

    // VI_H_SYNC_LEAP leap lines
    leap_a: u16,
    leap_b: u16,

    // VI_H_VIDEO horizontal scanline
    h_start: u16,
    h_end: u16,

    // VI_V_VIDEO field size?
    v_start: u16,
    v_end: u16,

    // VI_V_BURST not going to be used for a while?
    v_burst: u32,

    // VI_X_SCALE
    x_offset: u16,
    x_scale: u16,

    // VI_Y_SCALE
    y_offset: u16,
    y_scale: u16,
}

impl VideoInterface {
    // CPU runs at 93.75MHz, so at 60fps we should render a frame every 1,562,500 cpu cycles
    // Just assuming a 320x240 resolution, we should complete a scanline every ~6510 cycles
    const NUM_LINES: u64 = 280;
    const CPU_FREQ : u64 = 93_750_000;

    // this FPS value "simulates" the VI interrupt occuring at this framerate
    const FPS      : u64 = 140; // some demos run really nicely with this VI at 200+

    const CYC_PER_FRAME: u64 = Self::CPU_FREQ / Self::FPS;
    const CYC_PER_SCANLINE: u64 = Self::CYC_PER_FRAME / Self::NUM_LINES;

    pub fn new(comms: SystemCommunication) -> VideoInterface {
        VideoInterface {
            comms: comms,

            resolution_changed: 0,
            cycle_count: 0,

            // VI_CTRL
            dedither_filter_enable: 0,
            pixel_advance: 0,
            kill_we: 0,
            aa_mode: 0,
            test_mode: 0,
            serrate: 0,
            vbus_clock_enable: 0,
            divot_enable: 0,
            gamma_enable: 0,
            gamma_dither_enable: 0,
            pixel_type: 0,

            // VI_ORIGIN
            origin: 0,

            // VI_WIDTH
            frame_buffer_width: 0,

            // VI_V_INTR
            interrupt_line: 0x3FF,

            // VI_V_CURRENT
            current_line: 0,

            // VI_BURST
            burst: 0x01,

            // VI_V_SYNC
            vsync: 0,

            // VI_H_SYNC,
            leap_pattern: 0,
            hsync: 2047,

            // VI_H_SYNC_LEAP
            leap_a: 3182, // for PAL (not used on NTSC)
            leap_b: 3183, // for PAL

            // VI_H_VIDEO
            h_start: 0,
            h_end: 0,

            // VI_V_VIDEO
            v_start: 0,
            v_end: 0,

            // VI_V_BURST
            v_burst: 0,

            // VI_X_SCALE
            x_offset: 0,
            x_scale: 0,

            // VI_Y_SCALE
            y_offset: 0,
            y_scale: 0,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "VI", "reset");

        self.resolution_changed = 0;
        self.cycle_count = 0;

        // VI_CTRL
        self.dedither_filter_enable = 0;
        self.pixel_advance = 0;
        self.kill_we = 0;
        self.aa_mode = 0;
        self.test_mode = 0;
        self.serrate = 0;
        self.vbus_clock_enable = 0;
        self.divot_enable = 0;
        self.gamma_enable = 0;
        self.gamma_dither_enable = 0;
        self.pixel_type = 0;
        self.comms.vi_format.store(self.pixel_type as u32, Ordering::SeqCst);

        // VI_ORIGIN
        self.origin = 0;
        self.comms.vi_origin.store(self.origin, Ordering::SeqCst);

        // VI_WIDTH
        self.frame_buffer_width = 0;
        self.comms.vi_width.store(self.frame_buffer_width, Ordering::SeqCst);

        // VI_V_INTR
        self.interrupt_line = 0x3FF;

        // VI_V_CURRENT
        self.current_line = 0;

        // VI_BURST
        self.burst = 0x01;

        // VI_V_SYNC
        self.vsync = 0;

        // VI_H_SYNC;
        self.leap_pattern = 0;
        self.hsync = 2047;

        // VI_H_SYNC_LEAP
        self.leap_a = 3182; // for PAL (not used on NTSC)
        self.leap_b = 3183; // for PAL

        // VI_H_VIDEO
        self.h_start = 0;
        self.h_end = 0;

        // VI_V_VIDEO
        self.v_start = 0;
        self.v_end = 0;

        // VI_V_BURST
        self.v_burst = 0;

        // VI_X_SCALE
        self.x_offset = 0;
        self.x_scale = 0;

        // VI_Y_SCALE
        self.y_offset = 0;
        self.y_scale = 0;
    }

    fn _is_ntsc(&self) -> bool {
        self.burst == NTSC_BURST
    }

    fn _is_pal(&self) -> bool {
        self.burst == PAL_BURST
    }

    pub fn origin(&self) -> u32 {
        self.origin
    }

    pub fn calculate_free_cycles(&self) -> u64 {
        return Self::CYC_PER_SCANLINE - self.cycle_count;
    }

    pub fn step(&mut self, cpu_cycles_elapsed: u64) {
        if self.resolution_changed > 0 {
            self.resolution_changed = 0;

            // emit some signal?
            let width  = ((self.h_end - self.h_start) as f32 * (self.x_scale as f32 / 1024.0)) as u32;
            let height = ((((self.v_end - self.v_start) as f32) / 2.0) * (self.y_scale as f32 / 1024.0)) as u32;
            let depth  = if self.pixel_type == 3 { 32 } else if self.pixel_type == 2 { 16 } else { 0 };
            debug!(target: "VI", "resolution changed to {width}x{height}x{depth}");
        }

        self.cycle_count += cpu_cycles_elapsed;
        while self.cycle_count >= Self::CYC_PER_SCANLINE { 
            self.cycle_count -= Self::CYC_PER_SCANLINE;
            self.current_line = (self.current_line + 1) % (Self::NUM_LINES as u32);

            if self.current_line == self.interrupt_line {
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_VI, InterruptUpdateMode::SetInterrupt)).unwrap();
            }
        }
    }
}

impl Addressable for VideoInterface {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        trace!(target: "VI", "read32 offset=${:08X}", offset);

        let result = match offset {
            // VI_CTRL
            0x0_0000 => {
                ((self.dedither_filter_enable as u32) << 16)
                | ((self.pixel_advance as u32) << 12)
                | ((self.kill_we as u32) << 11)
                | ((self.aa_mode as u32) << 8)
                | ((self.test_mode as u32) << 7)
                | ((self.serrate as u32) << 6)
                | ((self.vbus_clock_enable as u32) << 5)
                | ((self.divot_enable as u32) << 4)
                | ((self.gamma_enable as u32) << 3)
                | ((self.gamma_dither_enable as u32) << 2)
                | (self.pixel_type as u32)
            },

            // VI_ORIGIN
            0x0_0004 => self.origin,

            // VI_WIDTH
            0x0_0008 => self.frame_buffer_width,

            // VI_V_INTR
            0x0_000C => self.interrupt_line,

            // VI_V_CURRENT
            0x0_0010 => {
                trace!(target: "VI", "current_line = {}", self.current_line);
                self.current_line * 2
            },

            // VI_BURST
            0x0_0014 => self.burst,

            // VI_V_SYNC
            0x0_0018 => self.vsync,

            // VI_H_SYNC
            0x0_001C => ((self.leap_pattern as u32) << 16) | self.hsync,

            // VI_H_SYNC_LEAP
            0x0_0020 => ((self.leap_a as u32) << 16) | (self.leap_b as u32),

            // VI_H_VIDEO
            0x0_0024 => ((self.h_start as u32) << 16) | (self.h_end as u32),

            // VI_V_VIDEO
            0x0_0028 => ((self.v_start as u32) << 16) | (self.v_end as u32),

            // VI_V_BURST
            0x0_002C => self.v_burst,

            // VI_X_SCALE
            0x0_0030 => ((self.x_offset as u32) << 16) | (self.x_scale as u32),
            
            // VI_Y_SCALE
            0x0_0034 => ((self.y_offset as u32) << 16) | (self.y_scale as u32),

            // VI_TEST_ADDR diagnostics only
            0x0_0038 => 0,

            // VI_STAGED_DATA diagnostics only
            0x0_003C => 0,

            _ => {
                error!(target: "VI", "unsupported/unimplemented read32 offset=${:08X}", offset);
                0
            }
        };

        Ok(result)
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        trace!(target: "VI", "write32 value=${:08X} offset=${:08X}", value, offset);

        match offset {
            // VI_CTRL 
            0x0_0000 => {
                self.pixel_type             = ((value >> 0) & 0x03) as u8;
                self.gamma_dither_enable    = ((value >> 2) & 0x01) as u8;
                self.gamma_enable           = ((value >> 3) & 0x01) as u8;
                self.divot_enable           = ((value >> 4) & 0x01) as u8;
                self.vbus_clock_enable      = ((value >> 5) & 0x01) as u8;
                self.serrate                = ((value >> 6) & 0x01) as u8;
                self.test_mode              = ((value >> 7) & 0x03) as u8;
                self.aa_mode                = ((value >> 8) & 0x03) as u8;
                self.kill_we                = ((value >> 11) & 0x01) as u8;
                self.pixel_advance          = ((value >> 12) & 0x0F) as u8;
                self.dedither_filter_enable = ((value >> 16) & 0x01) as u8;
                assert!(self.vbus_clock_enable == 0); // crash if vbus clock is ever enabled, simulating the real thing
                debug!(target: "VI", "setting pixel type to {}", self.pixel_type);
                self.comms.vi_format.store(self.pixel_type as u32, Ordering::SeqCst);
                Ok(WriteReturnSignal::None)
            },

            // VI_ORIGIN
            0x0_0004 => {
                self.origin = value & 0x00FF_FFFF;
                debug!(target: "VI", "setting RDRAM origin to ${:08X}", self.origin);

                // notify renderer of the framebuffer change
                self.comms.vi_origin.store(self.origin, Ordering::SeqCst);

                Ok(WriteReturnSignal::None)
            },

            // VI_WIDTH
            0x0_0008 => {
                self.frame_buffer_width = value & 0x0FFF;
                debug!(target: "VI", "setting framebuffer width to {}", self.frame_buffer_width);

                // notify renderer of the framebuffer size
                self.comms.vi_width.store(self.frame_buffer_width, Ordering::SeqCst);

                Ok(WriteReturnSignal::None)
            },

            // VI_V_INTR
            0x0_000C => {
                self.interrupt_line = value & 0x3FF;
                debug!(target: "VI", "setting interrupt line to {}", self.interrupt_line);
                Ok(WriteReturnSignal::None)
            },

            // VI_V_CURRENT
            0x0_0010 => {
                self.comms.mi_interrupts_tx.as_ref().unwrap().send(InterruptUpdate(IMask_VI, InterruptUpdateMode::ClearInterrupt)).unwrap();
                debug!(target: "VI", "setting current line to {} (int line={})", self.current_line, self.interrupt_line);
                Ok(WriteReturnSignal::None)
            },

            // VI_BURST
            0x0_0014 => {
                self.burst = value & 0x3FFF_FFFF;
                match self.burst {
                    NTSC_BURST => debug!(target: "VI", "NTSC signal detected"),
                    PAL_BURST  => debug!(target: "VI", "PAL signal detected"),
                    _          => warn!(target: "VI", "non-NTSC/PAL burst signal set value=${:08X}", self.burst),
                };
                Ok(WriteReturnSignal::None)
            },

            // VI_V_SYNC
            0x0_0018 => {
                self.vsync = value & 0x3FF;
                debug!(target: "VI", "setting vsync start to {}", self.vsync);
                Ok(WriteReturnSignal::None)
            },

            // VI_H_SYNC
            0x0_001C => {
                self.leap_pattern = ((value >> 16) & 0x1F) as u8;
                self.hsync = value & 0xFFF;
                debug!(target: "VI", "setting hsync start to {}", self.hsync);
                Ok(WriteReturnSignal::None)
            },

            // VI_H_SYNC_LEAP
            0x0_0020 => {
                self.leap_a = ((value >> 16) & 0x0FFF) as u16;
                self.leap_b = (value & 0x0FFF) as u16;
                Ok(WriteReturnSignal::None)
            },

            // VI_H_VIDEO
            0x0_0024 => {
                self.h_start = ((value >> 16) & 0x3FF) as u16;
                self.h_end = (value & 0x3FF) as u16;
                self.resolution_changed += 1;
                debug!(target: "VI", "setting scanline to pixels {}..{}", self.h_start, self.h_end);
                Ok(WriteReturnSignal::None)
            },

            // VI_V_VIDEO
            0x0_0028 => {
                self.v_start = ((value >> 16) & 0x3FF) as u16;
                self.v_end   = (value & 0x3FF) as u16;
                self.resolution_changed += 1;
                debug!(target: "VI", "setting field size to lines {}..{}", self.v_start, self.v_end);
                Ok(WriteReturnSignal::None)
            },

            // VI_V_BURST
            0x0_002C => {
                self.v_burst = value & 0x03FF_03FF;
                Ok(WriteReturnSignal::None)
            },

            // VI_X_SCALE
            0x0_0030 => {
                self.x_offset = ((value >> 16) & 0x0FFF) as u16;
                self.x_scale = (value & 0x0FFF) as u16;
                self.resolution_changed += 1;
                debug!(target: "VI", "setting x_scale={} x_offset={}", self.x_scale, self.x_offset);
                Ok(WriteReturnSignal::None)
            },
            
            // VI_Y_SCALE
            0x0_0034 => {
                self.y_offset = ((value >> 16) & 0x0FFF) as u16;
                self.y_scale = (value & 0x0FFF) as u16;
                self.resolution_changed += 1;
                debug!(target: "VI", "setting y_scale={} y_offset={}", self.y_scale, self.y_offset);
                Ok(WriteReturnSignal::None)
            },

            // VI_TEST_ADDR diagnostics only
            0x0_0038 => Ok(WriteReturnSignal::None),

            // VI_STAGED_DATA diagnostics only
            0x0_003C => Ok(WriteReturnSignal::None),

            _ => {
                error!(target: "VI", "unsupported/unimplemented write32 value=${:08X} offset=${:08X}", value, offset);
                Ok(WriteReturnSignal::None)
            }
        }
    }
}
