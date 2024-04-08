#[allow(unused_imports)]
use tracing::{debug, error, info, trace, warn};

use crate::*;

const IPL3_START : usize = 0x40;
const IPL3_LENGTH: usize = 0x1000 - IPL3_START;

#[derive(Debug)]
enum CicType {
    Nus6101,
    Nus6102,
    Nus6103,
    Nus6105,
    Nus6106,
    Nus7101,
    Nus7102,
    Nus7103,
    Nus7105,
    Nus7106,
    _Nus8303,
    _Nus8401,
    _NusDdus,
    UnknownNTSC, // uses the 6102 seed
    UnknownPAL,  // uses the 7101 seed
}

#[derive(Copy, Clone, Debug)]
enum Eeprom {
    None,
    _4KiB,
    _16KiB,
}

/// N64 PIF-ROM, where the boot rom is stored
/// boot_rom is big endian data
/// Essentially part of the PeripheralInterface but the PIF-ROM and features abstracted out
pub struct PifRom {
    comms: SystemCommunication,

    boot_rom: Vec<u8>,    
    ram: Vec<u32>,
    joybus_ram_copy: Vec<u32>,
    command_finished: bool,
    seed: u32,
    _cic_type: CicType,
    eeprom: Eeprom,
}

impl PifRom {
    pub fn new(comms: SystemCommunication, boot_rom: Vec<u8>, pi: &mut peripheral::PeripheralInterface) -> PifRom {
        let mut ram = vec![0u32; 16]; // 64 byte RAM

        // Read game header
        let header = pi.read_block(0x1000_0000, 0x40).expect("error reading rom header");

        // Get the country code of the rom
        let cc = ((header[0x3C >> 2] >> 8) as u8) as char;
        let is_pal = match cc {
            'D' | 'F' | 'I' | 'P' | 'S' | 'U' | 'X' | 'Y' => true, // Germany, France, Italy, Europe, Spain, Australia
            _ => false,
        };

        // Determine the CIC. We need the IPL3 code.
        let ipl3 = pi.read_block(0x1000_0000 | IPL3_START, IPL3_LENGTH as u32).expect("error reading rom");
        let (cic_type, hash, seed) = PifRom::determine_cic(&ipl3, is_pal);
        info!(target: "PIF", "found CIC {cic_type:?}, hash ${hash:010X}, seed 0x{seed:02X}, region {cc}");

        // Seed the seeds at 0x7E6-7
        let seed = seed as u32;
        ram[9] = (seed << 8) | seed;

        // Get the game code
        let mut game_code = [0u8; 3];
        game_code[0] = header[0x38 >> 2] as u8;
        game_code[1] = (header[0x3C >> 2] >> 24) as u8;
        game_code[2] = (header[0x3C >> 2] >> 16) as u8;
        let game_code = String::from_utf8(game_code.into()).unwrap_or(String::from("???"));
        info!(target: "PIF", "Game code: {}", game_code);

        let eeprom = match game_code.as_str() {
            "NSM" => {
                info!(target: "PIF", "Game has 4KiB EEPROM");
                Eeprom::_4KiB
            },
            _ => Eeprom::None,
        };

        PifRom {
            comms: comms,

            boot_rom: boot_rom,
            ram: ram,
            joybus_ram_copy: vec![0u32; 16],
            command_finished: false,
            seed: seed,
            _cic_type: cic_type,
            eeprom: eeprom,
        }
    }

    pub fn reset(&mut self) {
        info!(target: "PIF-ROM", "reset");
        self.command_finished = false;

        self.ram[9] = (self.seed << 8) | self.seed;
    }

    // Calculate the hash used by IPL2 to verify the ROM.  The hash depends on the seed value,
    // which we don't really know until we know what CIC is used.  Basically, guess.
    fn determine_cic(data: &[u32], is_pal: bool) -> (CicType, u64, u8) {
        let seed = 0x3F;
        match PifRom::ipl2hash(data, seed) {
            x @ 0x45CC73EE317A => { return (CicType::Nus6101, x, seed); }, // only Star Fox 64
            x @ 0xA536C0F1D859 => { return (if is_pal { CicType::Nus7101 } else { CicType::Nus6102 }, x, seed); }, // Looootttssss of games.
            x @ 0x44160EC5D9AF => { return (CicType::Nus7102, x, seed); },
            x @ 0xDB9CAEFB5196 => { return (if is_pal { CicType::Nus7101 } else { CicType::Nus6102 }, x, seed); }, // NuSystems demos?
            _ => {},
        };

        let seed = 0x78;
        match PifRom::ipl2hash(data, seed) {
            x @ 0x586FD4709867 => { return (if is_pal { CicType::Nus7103 } else { CicType::Nus6103 }, x, seed); }, // Paper Mario, Pokemon Stadium, Super Smash Bros, and a few others
            _ => {},
        };

        let seed = 0x91;
        match PifRom::ipl2hash(data, seed) {
            x @ 0x8618A45BC2D3 => { return (if is_pal { CicType::Nus7105 } else { CicType::Nus6105 }, x, seed); }, // LoZ, a few others
            _ => {},
        };

        let seed = 0x85;
        match PifRom::ipl2hash(data, seed) {
            x @ 0x2BBAD4E6EB74 => { return (if is_pal { CicType::Nus7106 } else { CicType::Nus6106 }, x, seed); }, // Cruis'n World, F-Zero X, Yoshi's Story
            _ => {},
        };

        let seed = 0xDD; // 64DD? Niiice.
        match PifRom::ipl2hash(data, seed) {
            //x @ 0x32B294E2AB90 => (CicType::Nus8303, x, seed, false), // 64DD retail JP
            //x @ 0x6EE8D9E84970 => (CicType::Nus8401, x, seed, false), // 64DD dev
            //x @ 0x083C6C77E0B1 => (CicType::Nus5167, x, seed, false), // 64DD conversion cartridges ??
            //x @ 0x05BA2EF0A5f1 => (CicType::NusDdus, x, seed, false), // 64DD retail US
            _ => {},
        };

        // Unknown IPL3 checksum, default to 6102 with seed 0x3F
        let checksum = PifRom::ipl2hash(data, 0x3F);
        warn!(target: "PIF", "unknown IPL3/CIC checksum ${:010X}, game may not run.", checksum);
        (if is_pal { CicType::UnknownPAL } else { CicType::UnknownNTSC }, checksum, 0x3F)
    }

    // Many thanks to @korgeaux of Summer Cart 64 for the algorithm. Minor changes included
    // https://github.com/Polprzewodnikowy/SummerCart64/blob/main/sw/deployer/src/sc64/cic.rs#L6
    fn ipl2hash(data: &[u32], seed: u8) -> u64 {
        const MAGIC: u32 = 0x6C078965;
        assert!(data.len() == IPL3_LENGTH / 4);

        let add = |a: u32, b: u32| a.wrapping_add(b);
        let sub = |a: u32, b: u32| a.wrapping_sub(b);
        let rol = |a: u32, i: u32| a.rotate_left(i);
        let ror = |a: u32, i: u32| a.rotate_right(i);
        let mul = |a: u32, b: u32| a.wrapping_mul(b);
        let sum = |a: u32, b: u32, c: u32| {
            let prod = (a as u64).wrapping_mul((if b == 0 { c } else { b }) as u64);
            let hi = (prod >> 32) as u32;
            let lo = prod as u32;
            let diff = hi.wrapping_sub(lo);
            if diff == 0 { a } else { diff }
        };

        let init = add(mul(MAGIC, seed as u32), 1) ^ data[0];
        let mut buf = vec![init; 16];

        for i in 1..=1008 {
            let ii   = i as u32;
            let prev = data[ii.saturating_sub(2) as usize];
            let cur  = data[i - 1];

            buf[0] = add(buf[0], sum(sub(1007, ii), cur, ii));
            buf[1] = sum(buf[1], cur, ii);
            buf[2] = buf[2] ^ cur;
            buf[3] = add(buf[3], sum(add(cur, 5), MAGIC, ii));
            buf[4] = add(buf[4], ror(cur, prev & 0x1F));
            buf[5] = add(buf[5], rol(cur, prev >> 27));
            buf[6] = if cur < buf[6] {
                add(buf[3], buf[6]) ^ add(cur, ii)
            } else {
                add(buf[4], cur) ^ buf[6]
            };
            buf[7] = sum(buf[7], rol(cur, prev & 0x1F), ii);
            buf[8] = sum(buf[8], ror(cur, prev >> 27), ii);
            buf[9] = if prev < cur {
                sum(buf[9], cur, ii)
            } else {
                add(buf[9], cur)
            };

            if i == 1008 { break; }

            let next = data[i];

            buf[10] = sum(add(buf[10], cur), next, ii);
            buf[11] = sum(buf[11] ^ cur, next, ii);
            buf[12] = add(buf[12], buf[8] ^ cur);
            buf[13] = add(buf[13], add(ror(cur, cur & 0x1F), ror(next, next & 0x1F)));
            buf[14] = sum(sum(buf[14], ror(cur, prev & 0x1F), ii), ror(next, cur & 0x1F), ii);
            buf[15] = sum(sum(buf[15], rol(cur, prev >> 27), ii), rol(next, cur >> 27), ii);
        }

        let mut result = vec![buf[0]; 4];

        for i in 0..16 {
            let ii = i as u32;
            let cur = buf[i];

            result[0] = add(result[0], ror(cur, cur & 0x1F));
            result[1] = if cur < result[0] {
                add(result[1], cur)
            } else {
                sum(result[1], cur, ii)
            };
            result[2] = if ((cur & 0x02) >> 1) == (cur & 0x01) {
                add(result[2], cur)
            } else {
                sum(result[2], cur, ii)
            };
            result[3] = if (cur & 0x01) == 0x01 {
                result[3] ^ cur
            } else {
                sum(result[3], cur, ii)
            };
        }

        let result_sum = sum(result[0], result[1], 16);
        let result_xor = result[2] ^ result[3];

        // final checksum
        (((result_sum & 0xFFFF) as u64) << 32) | (result_xor as u64)
    }

    fn update_control_write(&mut self) {
        let value = self.ram[0x0F];

        //panic!("PIF: write command port");
        if (value & 0x01) != 0 {
            let ramcpy = self.ram.to_owned();
            self.joybus_ram_copy.copy_from_slice(&ramcpy);
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
        const JOYBUS_COMMAND_READ_EEPROM: u8 = 0x04;
        const JOYBUS_COMMAND_WRITE_EEPROM: u8 = 0x05;
        const JOYBUS_COMMAND_RESET: u8 = 0xFF;

        // copy over current ram with the joybus memory copy
        let ramcpy = self.joybus_ram_copy.to_owned();
        self.ram.copy_from_slice(&ramcpy);

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
                        JOYBUS_COMMAND_RESET | JOYBUS_COMMAND_ID => {
                            trace!(target: "JOY", "{}: JOYBUS_COMMAND_ID channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if cmd_length != 1 || res_length != 3 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }

                            // Identify a standard controller on all ports
                            if channel < 4 {
                                self.write_u8_correct(0x05, res_addr + 0).unwrap();
                                self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                self.write_u8_correct(0x02, res_addr + 2).unwrap(); // 2 indicates no pak installed, 1 otherwise
                            } else if channel == 4 { // Cart channel
                                match self.eeprom {
                                    Eeprom::_4KiB => {
                                        self.write_u8_correct(0x00, res_addr + 0).unwrap(); // 0x00C0 16Kbit 
                                        self.write_u8_correct(0x80, res_addr + 1).unwrap();
                                        self.write_u8_correct(0x00, res_addr + 2).unwrap(); // 0x80 set = write in progress
                                    },
                                    Eeprom::_16KiB => {
                                        self.write_u8_correct(0x00, res_addr + 0).unwrap(); // 0x00C0 16Kbit 
                                        self.write_u8_correct(0xC0, res_addr + 1).unwrap();
                                        self.write_u8_correct(0x00, res_addr + 2).unwrap(); // 0x80 set = write in progress
                                    },
                                    _ => {
                                        self.write_u8_correct(0x00, res_addr + 0).unwrap();
                                        self.write_u8_correct(0x00, res_addr + 1).unwrap();
                                        self.write_u8_correct(0x00, res_addr + 2).unwrap();
                                        // setting bit 7 to the res length byte indicates device isn't present
                                        self.write_u8_correct((original_res_length | 0x80) as u32, cmd_start + RESLEN_OFFSET).unwrap();
                                    },
                                }
                            }

                            channel += 1;
                        },

                        JOYBUS_COMMAND_READ => { // read button state
                            trace!(target: "JOY", "{}: JOYBUS_COMMAND_READ channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if cmd_length != 1 || res_length != 4 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }

                            if channel < 4 {
                                // copy ControllerState from comms channel
                                let cs = self.comms.controllers.read().unwrap()[channel as usize];

                                // four bytes indicate buttons and two axes
                                let b0 = ((cs.a.is_down() as u8) << 7) 
                                            | ((cs.b.is_down() as u8) << 6) 
                                            | ((cs.z.is_down() as u8) << 5)
                                            | ((cs.start.is_down() as u8) << 4)
                                            | ((cs.d_up.is_down() as u8) << 3)
                                            | ((cs.d_down.is_down() as u8) << 2)
                                            | ((cs.d_left.is_down() as u8) << 1)
                                            | ((cs.d_right.is_down() as u8) << 0);
                                self.write_u8_correct(b0 as u32, res_addr + 0).unwrap(); // from bit 7..0, ABZSdUdDdLdR

                                let b1 = ((cs.l_trigger.is_down() as u8) << 5)
                                            | ((cs.r_trigger.is_down() as u8) << 4)
                                            | ((cs.c_up.is_down() as u8) << 3)
                                            | ((cs.c_down.is_down() as u8) << 2)
                                            | ((cs.c_left.is_down() as u8) << 1)
                                            | ((cs.c_right.is_down() as u8) << 0);
                                self.write_u8_correct(b1 as u32, res_addr + 1).unwrap(); //              R_lTrTcUcDcLcR // R = reset, _ = zero

                                // convert -1..1 to -128..127 
                                let b2 = (if cs.x_axis < 0.0 { 128.0 * cs.x_axis } else { 127.0 * cs.x_axis }) as i8;
                                self.write_u8_correct((b2 as u8) as u32, res_addr + 2).unwrap(); // x-axis

                                let b3 = (if cs.y_axis < 0.0 { 128.0 * cs.y_axis } else { 127.0 * cs.y_axis }) as i8;
                                self.write_u8_correct((b3 as u8) as u32, res_addr + 3).unwrap(); // y-axis
                            }

                            channel += 1;
                        },

                        JOYBUS_COMMAND_WRITE_ACCESSORY => { // write to device accessory (pak)
                            debug!(target: "JOY", "{}: JOYBUS_COMMAND_WRITE_ACCESSORY channel={}, res_addr={}", cmd_count - 1, channel, res_addr - 0x7C0);
                            if res_length != 1 {
                                error!(target: "JOY", "unsupported/incorrect cmd_length ({}) or res_length ({})", cmd_length, res_length);
                                break 'cmd_loop;
                            }
                        },

                        JOYBUS_COMMAND_READ_EEPROM => { // read EEPROM block
                            let block = self.read_u8(cmd_start + COMMAND_OFFSET + 1).unwrap();
                            warn!(target: "JOY", "{}: JOYBUS_COMMAND_READ_EEPROM channel={}, block={}, res_addr={}", cmd_count - 1, channel, block, res_addr - 0x7C0);
                        },

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
            Ok(((self.boot_rom[offset+0] as u32) << 24)
               | ((self.boot_rom[offset+1] as u32) << 16)
               | ((self.boot_rom[offset+2] as u32) << 8)
               | (self.boot_rom[offset+3] as u32))
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
                //info!(target: "PIF-ROM", "read offset=${:08X}", offset);
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
                trace!(target: "PIF-ROM", "write value=${:08X} offset=${:08X}", value, offset);
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

    fn write_block(&mut self, address: usize, block: &[u32], length: u32) -> Result<WriteReturnSignal, ReadWriteFault> {
        if (block.len() * 4) as u32 != length { todo!(); }

        if address != 0x7C0 || block.len() != 16 { todo!(); } // non-standard DMA

        //println!("DMA into PIF-ROM:");
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

        self.do_joybus();

        Ok(self.ram.to_owned())
    }
}


