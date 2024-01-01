#[allow(unused_imports)]
use tracing::{debug, error, info, trace};

use crate::*;

/// N64 PIF-ROM, where the boot rom is stored
/// boot_rom is big endian data
/// Essentially part of the PeripheralInterface but the PIF-ROM and features abstracted out
pub struct PifRom {
    boot_rom: Vec<u8>,    
    ram: Vec<u32>,
    command_finished: bool,
}

impl PifRom {
    pub fn new(boot_rom: Vec<u8>, pi: &mut peripheral::PeripheralInterface) -> PifRom {
        let mut ram = vec![0u32; 16]; // 64 byte RAM

        // CRC the IPL3 code, which starts after the 64 byte header and goes up to address 0x1000
        let mut crc: u64 = 0;
        for i in (0x40..0x1000).step_by(4) {
            crc += match pi.read_u32((0x1000_0000 | i) as usize) {
                Ok(value) => value as u64,
                Err(_) => panic!("could not read cartridge, this shouldn't happen"),
            }
        }

        // HACK! To simulate the CIC exchange, we need certain seed values at 0x7E4 in PIF ram
        match crc {
            0xD057C85244 => { // CIC 6102, GoldenEye
                info!(target: "PIF", "CIC 6102 detected");
                ram[9] = 0x00003F3F;
            },

            0x11A49F60E96 => { // CIC CIC 6105, Ocarina of Time, Majora's Mask
                info!(target: "PIF", "CIC 6105 detected");
                ram[9] = 0x00029100;
            },

            _ => {
                info!(target: "PIF", "unknown IPL3/CIC checksum ${:08X}. game probably won't run", crc);
            }
        };

        PifRom {
            boot_rom: boot_rom,
            ram: ram,
            command_finished: false,
        }
    }

    fn update_control_write(&mut self) {
        let value = self.ram[0x0F];

        //panic!("PIF: write command port");
        if (value & 0x01) != 0 {
            self.do_joybus();
        } else if (value & 0xFE) != 0 {
            if (value & 0x10) != 0 {
                debug!(target: "PIF", "disable PIF-ROM access");
            } else if (value & 0x20) != 0 {
                debug!(target: "PIF", "CPU checksum ready");
                self.command_finished = true;
            } else if (value & 0x40) != 0 {
                debug!(target: "PIF", "run checksum");
            } else if (value & 0x08) != 0 {
                info!(target: "PIF", "IPL1 finished");
            } else if (value & 0x07) != 0 {
                panic!("PIF: not implemented PIF command ${:08X}", value);
            }
        }
    }

    fn do_joybus(&mut self) {
        trace!(target: "PIF", "running joybus protocol");

        let mut channel = 0;
        let mut i = 0;
        let mut cmd_count = 0;

        const CMDLEN_OFFSET: usize = 0;
        const RESLEN_OFFSET: usize = 1;
        const COMMAND_OFFSET: usize = 2;

        const JOYBUS_COMMAND_ID: u8 = 0x00;
        const JOYBUS_COMMAND_READ: u8 = 0x01;
        const JOYBUS_COMMAND_WRITE_ACCESSORY: u8 = 0x03;

        'cmd_loop: while i < 64 {
            let cmd_start = 0x7C0 + i;
            i += 1;

            let cmd_length = self.read_u8(cmd_start + CMDLEN_OFFSET).unwrap() & 0x3F;
            debug!(target: "JOY", "Command index {} (at offset {}), channel {}, length {}", cmd_count, cmd_start, channel, cmd_length);
            cmd_count += 1;

            match cmd_length {
                0 => { // no command for current channel, so move on to next
                    debug!(target: "JOY", "{}: no more commands on channel {}", cmd_count - 1, channel);
                    channel += 1;
                    continue 'cmd_loop;
                },

                0x3D => { // reset current channel
                    todo!("reset channel");
                },

                0x3E => { // end of commands
                    debug!(target: "JOY", "{}: end of commands", cmd_count - 1);
                    break 'cmd_loop;
                },

                0x3F => { // NOP/reserved space for response
                    debug!(target: "JOY", "{}: reserved space", cmd_count - 1);
                },

                _ => {
                    let mut res_length = self.read_u8(cmd_start + RESLEN_OFFSET).unwrap();                    
                    if res_length == 0xFE { // end of commands
                        break 'cmd_loop;
                    }

                    let original_res_length = res_length;
                    res_length &= 0x3F;
                    let res_addr = cmd_start + COMMAND_OFFSET + cmd_length as usize;

                    let cmd = self.read_u8(cmd_start + COMMAND_OFFSET).unwrap();
                    match cmd {
                        JOYBUS_COMMAND_ID => {
                            debug!(target: "JOY", "{}: JOYBUS_COMMAND_ID channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if cmd_length != 1 || res_length != 3 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }

                            // Identify a standard controller on port 1, and nothing on all the other ports
                            if channel == 0 {
                                self.write_u8_correct(0x05, res_addr + 0).unwrap();
                                self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                self.write_u8_correct(0x02, res_addr + 2).unwrap(); // 2 indicates no pak installed, 1 otherwise
                            } else {
                                self.write_u8_correct(0x00, res_addr + 0).unwrap();
                                self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                self.write_u8_correct(0x00, res_addr + 2).unwrap();
                                // setting bit 7 to the res length byte indicates device isn't present
                                self.write_u8_correct((original_res_length | 0x80) as u32, cmd_start + RESLEN_OFFSET).unwrap();
                            }

                            channel += 1;
                        },

                        JOYBUS_COMMAND_READ => { // read button state
                            debug!(target: "JOY", "{}: JOYBUS_COMMAND_READ channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if cmd_length != 1 || res_length != 4 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }

                            if channel == 0 {
                                // four bytes indicate buttons and two axes
                                self.write_u8_correct(0x00, res_addr + 0).unwrap();
                                self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                self.write_u8_correct(0x00, res_addr + 2).unwrap();
                                self.write_u8_correct(0x00, res_addr + 3).unwrap();
                            } else {
                                self.write_u8_correct(0x00, res_addr + 0).unwrap();
                                self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                self.write_u8_correct(0x00, res_addr + 2).unwrap();
                                self.write_u8_correct(0x00, res_addr + 3).unwrap();
                                // setting bit 7 to the res length byte indicates device isn't present
                                self.write_u8_correct((original_res_length | 0x80) as u32, cmd_start + RESLEN_OFFSET).unwrap();
                            }

                            channel += 1;
                        },

                        JOYBUS_COMMAND_WRITE_ACCESSORY => { // write to device accessory (pak)
                            debug!(target: "JOY", "{}: JOYBUS_COMMAND_WRITE_ACCESSORY channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if res_length != 1 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }
                        }

                        _ => {
                            error!(target: "JOY", "unhandled joybus command ${:02X} on channel {}", cmd, channel);
                            unimplemented!();
                            //break 'cmd_loop;
                        }
                    }

                    i += (1 + cmd_length + res_length) as usize;
                }
            }
        }

        self.ram[0x0F] &= !0xFF;

        //println!("Joybus Result:");
        //for j in 0..4 {
        //    print!("  ${:02X}: ", j*16);
        //    for i in 0..16 {
        //        let index = ((j * 16) + i) as usize;
        //        let w = self.ram[index >> 2];
        //        let shift = 24 - ((index & 0x03) << 3);
        //        print!("{:02X} ", (w >> shift) & 0xFF);
        //    }
        //    println!();
        //}
    }

    // because PIF doesn't implement write_u8 correctly, we have another copy here
    fn write_u8_correct(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        let word = self.read_u32(offset & !0x03)?;
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(((value & 0xFF) << shift) | (word & !(0xFFu32 << shift)), offset & !0x03)
    }
}

impl Addressable for PifRom {
    fn read_u32(&mut self, offset: usize) -> Result<u32, ReadWriteFault> {
        debug!(target: "PIF", "read32 offset=${:08X}", offset);

        if offset < 1984 {
            Ok(((self.boot_rom[offset+1] as u32) << 24)
               | ((self.boot_rom[offset+0] as u32) << 16)
               | ((self.boot_rom[offset+3] as u32) << 8)
               | (self.boot_rom[offset+2] as u32))
        } else if offset == 0x7FC {
            // HACK! data is always available (bit 7 set)
            if self.command_finished { 
                self.command_finished = false;
                Ok(0x00000080)
            } else { 
                Ok(0x00000000)
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                //info!(target: "PIF-RAM", "read offset=${:08X}", offset);
                Ok(self.ram[ram_offset as usize])
            } else {
                panic!("unhandled PIF read offset=${:08X}", offset)
            }
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        if offset >= 0x7C0 { // ignore writes to ROM
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                //info!(target: "PIF-RAM", "write value=${:08X} offset=${:08X}", value, offset);
                self.ram[ram_offset as usize] = value;

                if ram_offset == 0x0F {
                    self.update_control_write();
                }
            } else {
                panic!("PIF: invalid write value=${:08X} offset=${:08X}", value, offset);
            }
        }

        Ok(WriteReturnSignal::None)
    }

    fn write_u16(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SH incorrectly overwrites lower bytes with zeroes
        let offset = offset & !0x01;
        let shift = 16 - ((offset & 0x02) << 3);
        self.write_u32(value << shift, offset & !0x02)
    }

    fn write_u8(&mut self, value: u32, offset: usize) -> Result<WriteReturnSignal, ReadWriteFault> {
        // SB incorrectly overwrites lower bytes with zeroes
        let shift = 24 - ((offset & 0x03) << 3);
        self.write_u32(value << shift, offset & !0x03)
    }

    fn write_block(&mut self, address: usize, block: &[u32]) -> Result<WriteReturnSignal, ReadWriteFault> {
        if address != 0x7C0 || block.len() != 16 { todo!(); } // non-standard DMA

        //println!("DMA into PIF-RAM:");
        //for j in 0..4 {
        //    print!("  ${:02X}: ", j*16);
        //    for i in 0..16 {
        //        let index = ((j * 16) + i) as usize;
        //        let w = block[index >> 2];
        //        let shift = 24 - ((index & 0x03) << 3);
        //        print!("{:02X} ", (w >> shift) & 0xFF);
        //    }
        //    println!();
        //}

        self.ram.copy_from_slice(block);
        self.update_control_write();

        Ok(WriteReturnSignal::None)
    }

    fn read_block(&mut self, offset: usize, length: u32) -> Result<Vec<u32>, ReadWriteFault> {
        if offset != 0x7C0 || length != 64 { todo!(); }

        Ok(self.ram.to_owned())
    }
}


