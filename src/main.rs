use std::env;
use std::fs;

enum MemorySegment {
    UserSpace,
    KSeg0,
    KSeg1,
    KSSeg,
    KSeg3,
}

pub trait Addressable {
    fn read_u32(&mut self, offset: usize) -> u32;
    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self;
}

/// N64 PIF, where the boot rom is stored
/// boot_rom is big endian data
struct Pif {
    boot_rom: Vec<u8>,    
    ram: Vec<u32>,
    command_finished: bool,
}

impl Pif {
    fn new(boot_rom_file: &str) -> Pif {
        let boot_rom_data = fs::read(boot_rom_file).expect("Boot rom not found");

        // HACK! To simulate the CIC exchange, we need seeds at location 0x7E4 in PIF memory
        let mut ram = vec![0u32; 16]; // 64 byte RAM
        ram[9] = 0x0000_3F3F;

        Pif {
            boot_rom: boot_rom_data,
            ram: ram,
            command_finished: false,
        }
    }
}

impl Addressable for Pif {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("PIF: read ${:08X}", offset);
        if offset < 1984 {
            ((self.boot_rom[offset+0] as u32) << 24)
            | ((self.boot_rom[offset+1] as u32) << 16)
            | ((self.boot_rom[offset+2] as u32) << 8)
            | (self.boot_rom[offset+3] as u32)
        } else if offset == 0x7FC {
            // HACK! data is always available (bit 7 set)
            if self.command_finished { 
                self.command_finished = false;
                0x00000080 
            } else { 
                0x00000000 
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                self.ram[ram_offset as usize]
            } else {
                panic!("unhandled PIF read")
            }
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        if offset < 0x7C0 {
            panic!("invalid PIF write");
        } else if offset == 0x7FC {
            //panic!("PIF: write command port");
            if (value & 0x10) != 0 {
                println!("PIF: disable PIF-ROM access");
            } else if (value & 0x20) != 0 {
                println!("PIF: CPU checksum ready");
                self.command_finished = true;
            } else if (value & 0x40) != 0 {
                println!("PIF: run checksum");
            } else if (value & 0x0F) != 0 {
                panic!("PIF: not implemented PIF command ${:08X}", value);
            }
        } else {
            let ram_offset = offset.wrapping_sub(0x7C0) >> 2;
            if ram_offset < 16 {
                self.ram[ram_offset as usize] = value;
            } else {
                panic!("invalid PIF write");
            }
        }

        self
    }
}

/// N64 Reality Control Processor
/// Contains the on-board RSP, RDP and manages the system bus
struct Rcp {
    rdp: Rdp,
    rsp: Rsp,

    // bus objects
    pif: Pif,
    pi: Pi,
}

impl Rcp {
    fn new(pif: Pif, pi: Pi) -> Rcp {
        Rcp {
            rdp: Rdp::new(),
            rsp: Rsp::new(),
            pif: pif,
            pi: pi,
        }
    }

    fn rcp_read_u32(&mut self, offset: usize) -> u32 {
        println!("RCP: read32 offset=${:08X}", offset);

        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => self.rsp.read_u32(offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x042F_FFFF
            1..=2 => self.rdp.read_u32(offset & 0x003F_FFFF),

            // MI 0x0430_0000-0x043F_FFFF
            3 => panic!("MIPS interface (MI) read"),

            // VI 0x0440_0000-0x044F_FFFF
            4 => panic!("Video interface (VI) read"),

            // AI 0x0450_0000-0x045F_FFFF
            5 => panic!("Audio interface (AI) read"),
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => panic!("Peripheral interface (PI) read"),

            // RDRAM 0x0470_0000-0x047F_FFFF
            7 => panic!("RDRAM interface (RI) read"),

            // SI 0x0480_0000-0x048F_FFFF
            8 => {
                println!("Serial interface (SI) read");
                0
            }

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP read"),
        }
    }

    fn rcp_write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        println!("RCP: write32 offset=${:08X}", offset);

        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => { self.rsp.write_u32(value, offset & 0x000F_FFFF); },

            // RDP 0x0410_0000-0x041F_FFFF
            1..=2 => { self.rdp.write_u32(value, offset & 0x003F_FFFF); },

            // MI 0x0430_0000-0x043F_FFFF
            3 => panic!("MIPS interface (MI) write"),

            // VI 0x0440_0000-0x044F_FFFF
            4 => {
                println!("VI: write32");
            }

            // AI 0x0450_0000-0x045F_FFFF
            5 => {
                println!("AI: write32");
            }
            
            // PI 0x0460_0000-0x046F_FFFF
            6 => {
                println!("PI: write32");
            }

            // RDRAM 0x0470_0000-0x047F_FFFF
            7 => panic!("RDRAM interface (RI) write"),

            // SI 0x0480_0000-0x048F_FFFF
            8 => panic!("Serial interface (SI) write"),

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP write"),
        };
        self
    }

    fn get_physical_address(&self, address: usize) -> (MemorySegment, usize) {
        // N64 memory is split into segments that are either cached or uncached, 
        // memory mapped or not, and user or kernel protected. 
        if (address & 0x8000_0000) == 0 {
            (MemorySegment::UserSpace, address & 0x7FFF_FFFF)
        } else {
            let s = match (address & 0x6000_0000) >> 29 {
                0 => MemorySegment::KSeg0,
                1 => MemorySegment::KSeg1,
                2 => MemorySegment::KSSeg,
                3 => MemorySegment::KSeg3,
                _ => panic!("not valid")
            };
            (s, address & 0x1FFF_FFFF)
        }
    }
}

impl Addressable for Rcp {
    // The RCP handles all bus arbitration, so that's why the primary bus access is
    // part of the Rcp module
    fn read_u32(&mut self, address: usize) -> u32 {
        let (_segment, physical_address) = self.get_physical_address(address);
        println!("BUS: read32 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => panic!("RDRAM read"),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u32(physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.read_u32(physical_address & 0x000FFFFF)
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.read_u32(physical_address & 0x7FFF_FFFF)
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }

    fn write_u32(&mut self, value: u32, address: usize) -> &mut Self {
        let (_segment, physical_address) = self.get_physical_address(address);
        println!("BUS: write32 value=${:08X} address=${:08X} physical=${:08X}", value, address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => panic!("RDRAM write"),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => { self.rcp_write_u32(value, physical_address & 0x00FF_FFFF); },

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.write_u32(value, physical_address & 0x000FFFFF);
                } else {
                    // PI external bus 0x05000000-0x7FFFFFFF
                    self.pi.write_u32(value, physical_address & 0x7FFF_FFFF);
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        };

        self
    }
}

/// N64 Reality Signal Processor
/// Resides on the die of the RCP.
struct Rsp {
    mem: Vec<u32>,
    si_status: u32,
}

impl Rsp {
    fn new() -> Rsp {
        Rsp {
            mem: vec![0u32; 2*1024], // 8KiB
            si_status: 0b0000_0000_0000_0001, // bit 0 (HALTED) set
        }
    }

    fn read_register(&mut self, offset: usize) -> u32 {
        match offset {
            // SP_STATUS
            0x4_0010 => {
                println!("RSP: read SP_STATUS");
                self.si_status
            },

            // SP_DMA_BUSY
            0x4_0018 => {
                println!("RSP: read SP_DMA_BUSY");

                // mirror of DMA_BUSY in self.si_status
                (self.si_status & 0x04) >> 2
            },

            _ => panic!("Unknown RSP register read ${:08X}", offset)
        }
    }

    fn write_register(&mut self, _value: u32, offset: usize) {
        match offset {
            // SP_STATUS 
            0x4_0010 => {
                println!("RSP: write SP_STATUS");
            },
            _ => panic!("Unknown RSP register write ${:08X}", offset)
        };
    }
}

impl Addressable for Rsp {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("RSP: read32 offset=${:08X}", offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                self.mem[mem_offset as usize]
            },

            0x0004_0000..=0x000B_FFFF => self.read_register(offset & 0x000F_FFFF),
            _ => panic!("invalid RSP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        println!("RSP: write32 value=${:08X} offset=${:08X}", value, offset);

        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => {
                let mem_offset = (offset & 0x1FFF) >> 2; // 8KiB, repeated
                self.mem[mem_offset as usize] = value;
            },

            0x0004_0000..=0x000B_FFFF => self.write_register(value, offset & 0x000F_FFFF),
            _ => panic!("invalid RSP write"),
        };
        self
    }
}

struct Rdp {
}

impl Rdp {
    fn new() -> Rdp {
        Rdp {}
    }
}

impl Addressable for Rdp {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("RDP: read32 offset=${:08X}", offset);
        match offset {
            // DP_STATUS 
            0x0010_000C => 0,
            _ => panic!("invalid RDP read"),
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        panic!("RDP: write32");
        self
    }
}

/// N64 Peripheral Interface
/// Connects EEPROM, cartridge, controllers, and more
struct Pi {
    cartridge_rom: Vec<u8>,
}

impl Pi {
    fn new(cartridge_rom_file: &str) -> Pi {
        let cartridge_rom = fs::read(cartridge_rom_file).expect("Could not open cartridge ROM file");

        Pi {
            cartridge_rom: cartridge_rom,
        }
    }
}

impl Addressable for Pi {
    fn read_u32(&mut self, offset: usize) -> u32 {
        println!("PI: read32 offset=${:08X}", offset);

        if offset < 0x0800_0000 {
            panic!("N64DD read")
        } else if offset < 0x1000_0000 {
            panic!("Cartridge SRAM/FlashRAM read")
        } else if offset < 0x1FC0_0000 {
            let cartridge_rom_offset = offset & 0x0FFF_FFFF;
            println!("CART: read32 offset=${:08X}", cartridge_rom_offset);

            if cartridge_rom_offset >= self.cartridge_rom.len() {
                0xFFFFFFFF
            } else {
                ((self.cartridge_rom[cartridge_rom_offset + 0] as u32) << 24)
                | ((self.cartridge_rom[cartridge_rom_offset + 1] as u32) << 16)
                | ((self.cartridge_rom[cartridge_rom_offset + 2] as u32) << 8)
                | (self.cartridge_rom[cartridge_rom_offset + 3] as u32)
            }
        } else {
            panic!("PI: invalid read")
        }
    }

    fn write_u32(&mut self, value: u32, offset: usize) -> &mut Self {
        panic!("PI: write32");
        self
    }
}

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        assert!(args.len() == 1);
        println!("Usage: {} file.z64", args[0]);
        return;
    }

    let pif = Pif::new("boot.rom");
    let pi = Pi::new(args[1].as_str());
    let mut rcp = Rcp::new(pif, pi);

    let mut gpr = [0u32; 32];
    let mut lo = 0u32;
    let mut hi = 0u32;
    let mut cp0r = [0u32; 32];

    let mut pc: u32 = 0xBFC00000;
    let mut next_instruction: u32; // simulates delay slot

    // fetch next_instruction before starting the loop
    next_instruction = rcp.read_u32(pc as usize);
    let mut next_instruction_pc = pc;
    pc += 4;

    loop {
        if pc == 0xA4001420 {
            let base = 2017;
            println!("m ${:08X} ${:08X} ${:08X} ${:08X}", rcp.rsp.mem[base+0] , rcp.rsp.mem[base+1] , rcp.rsp.mem[base+2] , rcp.rsp.mem[base+3]);
            println!("m ${:08X} ${:08X} ${:08X} ${:08X}", rcp.rsp.mem[base+4] , rcp.rsp.mem[base+5] , rcp.rsp.mem[base+6] , rcp.rsp.mem[base+7]);
            println!("m ${:08X} ${:08X} ${:08X} ${:08X}", rcp.rsp.mem[base+8] , rcp.rsp.mem[base+9] , rcp.rsp.mem[base+10], rcp.rsp.mem[base+11]);
            println!("m ${:08X} ${:08X} ${:08X} ${:08X}", rcp.rsp.mem[base+12], rcp.rsp.mem[base+13], rcp.rsp.mem[base+14], rcp.rsp.mem[base+15]);
        } else if pc == 0xB000_0000 {
            break;
        }

        // current instruction
        let inst = next_instruction;

        // next instruction fetch
        next_instruction = rcp.read_u32(pc as usize);

        // instruction decode
        let op     = inst >> 26;
        let rs     = (inst >> 21) & 0x1F;
        let rt     = (inst >> 16) & 0x1F;
        let rd     = (inst >> 11) & 0x1F;
        let imm    = inst & 0xFFFF;
        let signed_imm = (imm as i16) as u32;
        let _target = inst & 0x3FFFFFF;
        let sa     = (inst >> 6) & 0x1F;

        print!("i {:08X}: ", next_instruction_pc);
        //print!("{:08X}, op=0b{:06b}: ", 0xBFC00000+i, inst, op);
        next_instruction_pc = pc;
        pc += 4;

        // instruction execute
        match op {
            0b000_000 => {
                let special = inst & 0x3F;
                match special {
                    0b000_000 => {
                        println!("sll r{}, r{}, {}", rd, rt, sa);
                        gpr[rd as usize] = gpr[rt as usize] << sa;
                    },
                    0b000_010 => {
                        println!("srl r{}, r{}, {}", rd, rt, sa);
                        gpr[rd as usize] = gpr[rt as usize] >> sa;
                    },
                    0b000_100 => {
                        println!("sllv r{}, r{}, r{}", rd, rt, rs);
                        gpr[rd as usize] = gpr[rt as usize] << (gpr[rs as usize] & 0x1F);
                    },
                    0b000_110 => {
                        println!("srlv r{}, r{}, r{}", rd, rt, rs);
                        gpr[rd as usize] = gpr[rt as usize] >> (gpr[rs as usize] & 0x1F);
                    },
                    0b001_000 => {
                        println!("jr r{}", rs);
                        pc = gpr[rs as usize];
                    },
                    0b010_000 => {
                        println!("mfhi r{}", rd);
                        gpr[rd as usize] = hi;
                    },
                    0b010_010 => {
                        println!("mflo r{}", rd);
                        gpr[rd as usize] = lo;
                    },
                    0b011_001 => {
                        println!("multu r{}, r{}", rs, rt);
                        let result = (gpr[rs as usize] as u64) * (gpr[rt as usize] as u64);
                        // multu results are available in the next instruction since the multiply
                        // was started earlier in the pipeline
                        lo = (result & 0xFFFF_FFFF) as u32;
                        hi = ((result >> 32) & 0xFFFF_FFFF) as u32;
                    },
                    0b100_001 => {
                        println!("addu r{}, r{}, r{}", rd, rs, rt);
                        // addu does not cause an overflow exception
                        gpr[rd as usize] = gpr[rs as usize].wrapping_add(gpr[rt as usize]);
                    },
                    0b100_011 => {
                        println!("subu r{}, r{}, r{}", rd, rs, rt);
                        // subu does not cause an overflow exception
                        gpr[rd as usize] = gpr[rs as usize].wrapping_sub(gpr[rt as usize]);
                    },
                    0b100_100 => {
                        println!("and r{}, r{}, r{}", rd, rs, rt);

                        gpr[rd as usize] = gpr[rs as usize] & gpr[rt as usize];
                    },
                    0b100_101 => {
                        println!("or r{}, r{}, r{}", rd, rs, rt);

                        gpr[rd as usize] = gpr[rs as usize] | gpr[rt as usize];
                    },
                    0b100_110 => {
                        println!("xor r{}, r{}, r{}", rd, rs, rt);

                        gpr[rd as usize] = gpr[rs as usize] ^ gpr[rt as usize];
                    },
                    0b101_011 => {
                        println!("sltu r{}, r{}, r{}", rd, rs, rt);

                        // set rd to 1 if rs < rt, otherwise 0
                        gpr[rd as usize] = (gpr[rs as usize] < gpr[rt as usize]) as u32;
                    },
                    _ => panic!("Unknown special: 0b{:06b}", special)
                }
            },
            0b000_001 => {
                let regimm = (inst >> 16) & 0x1F;
                match regimm {
                    0b10_001 => {
                        println!("bgezal r{}, ${:04X}", rs, imm);

                        gpr[31] = pc; // unconditionally, the address after the delay slot is stored in the link register

                        if (gpr[rs as usize] & 0x8000_0000) == 0 {
                            pc = (pc - 4).wrapping_add(signed_imm << 2);
                        }
                    },

                    _ => panic!("Unknown regimm: 0b{:05b}", regimm)
                }
            },
            0b000_100 => {
                println!("beq r{}, r{}, ${:04X}", rs, rt, imm);

                // The only difference between this and beql is that
                // the delay slot is not discarded
                if gpr[rs as usize] == gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    pc = (pc - 4).wrapping_add(signed_imm << 2);
                }
            },
            0b000_101 => {
                println!("bne r{}, r{}, ${:04X}", rs, rt, imm);

                // The only difference between this and bnel is that
                // the delay slot is not discarded
                if gpr[rs as usize] != gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    pc = (pc - 4).wrapping_add(signed_imm << 2);
                }
            },
            0b001_000 => {
                println!("addi r{}, r{}, ${:04X}", rt, rs, imm);

                // integer overflow exception occurs with ADDI, unlike ADDIU
                let src = gpr[rs as usize];
                let result = src.wrapping_add(signed_imm);
                let is_pos = (imm & 0x8000) == 0;
                if imm != 0 && ((is_pos && result < src) || (!is_pos && result > src)) {
                    panic!("overflow detected: src=${:08X} imm=${:08X} result=${:08X}", src, signed_imm, result);
                }

                gpr[rt as usize] = result;
            },
            0b001_001 => {
                println!("addiu r{}, r{}, ${:04X}", rt, rs, imm);
                
                // no integer overflow exception occurs with ADDIU
                gpr[rt as usize] = gpr[rs as usize].wrapping_add(signed_imm);
            },
            0b001_100 => {
                println!("andi r{}, r{}, ${:04X}", rt, rs, imm);
                gpr[rt as usize] = gpr[rs as usize] & imm;
            },
            0b001_101 => {
                println!("ori r{}, r{}, ${:04X}", rt, rs, imm);
                gpr[rt as usize] = gpr[rs as usize] | imm;
            },
            0b001_111 => {
                println!("lui r{}, ${:04X}", rt, imm);
                gpr[rt as usize] = imm << 16;
            },
            0b010_000 => {
                let cop0 = (inst >> 21) & 0x1F;
                match cop0 {
                    0b00_100 => {
                        println!("mtc0 r{}, cp0r{} (r{}=${:08X})", rt, rd, rt, gpr[rt as usize]);

                        cp0r[rd as usize] = gpr[rt as usize];
                    },
                    _ => panic!("Unknown cop0: 0b{:05b}", cop0)
                }
            },
            0b010_001 => {
                panic!("cop1")
            },
            0b010_010 => {
                panic!("cop2")
            },
            0b010_100 => {
                println!("beql r{}, r{}, ${:04X}", rs, rt, imm);

                // branch only when the condition is true
                // if the branch is not taken, the delay slot instruction is discarded
                if gpr[rs as usize] == gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    pc = (pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    next_instruction = rcp.read_u32(pc as usize); // pc already points after the delay slot
                    pc += 4;
                }
            },
            0b010_101 => {
                println!("bnel r{}, r{}, ${:04X}", rs, rt, imm);

                // branch only when the condition is true
                // if the branch is not taken, the delay slot instruction is discarded
                if gpr[rs as usize] != gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    pc = (pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    next_instruction = rcp.read_u32(pc as usize); // pc already points after the delay slot
                    pc += 4;
                }
            },
            0b100_011 => {
                println!("lw r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = gpr[rs as usize].wrapping_add(signed_imm) as usize;
                if (address & 0x03) != 0 {
                    panic!("address exception!");
                }
                gpr[rt as usize] = rcp.read_u32(address);
            },
            0b101_011 => {
                println!("sw r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = gpr[rs as usize].wrapping_add(signed_imm) as usize;
                if (address & 0x03) != 0 {
                    panic!("address exception!");
                }
                rcp.write_u32(gpr[rt as usize], address);
            },
            0b111_111 => {
                panic!("<invalid>")
            },
            _ => panic!("Unknown op: 0b{:06b}", op)
        };

        // r0 must always be zero
        gpr[0] = 0;

        // dump all registers after each instruction
        for k in 0..4 {
            print!("r ");
            for j in 0..8 {
                print!("R{:02}: ${:08X} ", k*8+j, gpr[(k*8+j) as usize]);
            }
            println!("");
        }

        println!("-");
    }
}
