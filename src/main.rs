use std::fs;

enum MemorySegment {
    UserSpace,
    KSeg0,
    KSeg1,
    KSSeg,
    KSeg3,
}

/// N64 PIF, where the boot rom is stored
/// boot_rom is big endian data
struct Pif {
    boot_rom: Vec<u8>,    
}

impl Pif {
    fn new(boot_rom_file: &str) -> Pif {
        let boot_rom_data = fs::read(boot_rom_file).expect("Boot rom not found");
        Pif {
            boot_rom: boot_rom_data
        }
    }

    fn read_u32(&self, offset: usize) -> u32 {
        ((self.boot_rom[offset+0] as u32) << 24)
        | ((self.boot_rom[offset+1] as u32) << 16)
        | ((self.boot_rom[offset+2] as u32) << 8)
        | (self.boot_rom[offset+3] as u32)
    }
}

struct Rcp {
    rsp: Rsp,
    pif: Pif
}

impl Rcp {
    fn new(pif: Pif) -> Rcp {
        Rcp {
            rsp: Rsp::new(),
            pif: pif
        }
    }

    fn rcp_read_u32(&self, _segment: MemorySegment, offset: usize) -> u32 {
        println!("RCP: read32 offset=${:08X}", offset);

        match (offset & 0x00F0_0000) >> 20 {
            // RSP range 0x0400_0000-0x040F_FFFF 
            0 => self.rsp.read_u32(offset & 0x000F_FFFF),

            // RDP 0x0410_0000-0x041F_FFFF
            1 => panic!("RDP command registers read"),

            // RDP 0x0420_0000-0x042F_FFFF
            2 => panic!("RDP span registers read"),

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
            8 => panic!("Serial interface (SI) read"),

            // 0x0409_0000-0x04FF_FFFF unmapped
            _ => panic!("invalid RCP read"),
        }
    }

    // The RCP handles all bus arbitration, so that's why the primary bus access is
    // part of the Rcp module
    fn read_u32(&self, address: usize) -> u32 {
        // N64 memory is split into segments that are either cached or uncached, 
        // memory mapped or not, and user or kernel protected. 
        let (segment, physical_address) = if (address & 0x8000_0000) == 0 {
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
        };

        println!("BUS: read32 address=${:08X} physical=${:08X}", address, physical_address);

        match physical_address & 0xFC00_0000 {
            // RDRAM 0x00000000-0x03FFFFFF
            0x0000_0000 => panic!("RDRAM read"),

            // RCP 0x04000000-0x04FFFFFF
            0x0400_0000 => self.rcp_read_u32(segment, physical_address & 0x00FF_FFFF),

            // the SI external bus sits right in the middle of the PI external bus and needs further decode
            0x0500_0000..=0x7C00_0000 => {
                if (physical_address & 0xFFC0_0000) == 0x1FC0_0000 { 
                    // SI external bus 0x1FC00000-0x1FCFFFFF
                    // the SI external bus only has the PIF, so forward all access to it
                    self.pif.read_u32(physical_address & 0x000FFFFF)
                } else {
                    // PI external bus 0x05000000-0x7F000000
                    panic!("PI external bus read")
                }
            },

            // 0x8000_0000 and up not mapped
            _ => panic!("can't happen")
        }
    }
}

struct Rsp {
    si_status: u32,
}

impl Rsp {
    fn new() -> Rsp {
        Rsp {
            si_status: 0,
        }
    }

    fn read_u32(&self, offset: usize) -> u32 {
        match offset & 0x000F_0000 {
            0x0000_0000..=0x0003_FFFF => panic!("RSP DMEM/IMEM read"),
            0x0004_0000..=0x000B_FFFF => self.read_register(offset & 0x000F_FFFF),
            _ => panic!("invalid RSP read"),
        }
    }

    fn read_register(&self, offset: usize) -> u32 {
        match offset {
            // SI_STATUS
            0x4_0010 => {
                println!("RSP: read SI_STATUS");
                self.si_status
            }
            _ => panic!("Unknown RSP register read ${:08X}", offset)
        }
    }
}

fn main() {
    let pif = Pif::new("boot.rom");
    let rcp = Rcp::new(pif);

    let mut gpr = [0; 32];
    let mut cp0r = [0; 32];

    let mut pc: u32 = 0xBFC00000;
    loop {
        // instruction fetch
        let inst = rcp.read_u32(pc as usize);

        // instruction decode
        let op     = inst >> 26;
        let rs     = (inst >> 21) & 0x1F;
        let rt     = (inst >> 16) & 0x1F;
        let rd     = (inst >> 11) & 0x1F;
        let imm    = inst & 0xFFFF;
        let _target = inst & 0x3FFFFFF;
        let sa     = (inst >> 6) & 0x1F;

        //print!("{:08X}: {:08X}, op=0b{:06b}: ", 0xBFC00000+i, inst, op);
        print!("{:08X}: ", pc);
        pc += 4;

        // instruction execute
        match op {
            0b000_000 => {
                let special = inst & 0x3F;
                match special {
                    0b000_000 => {
                        panic!("sll r{}, r{}, {}", rd, rt, sa)
                    },
                    0b000_010 => {
                        panic!("srl r{}, r{}, {}", rd, rt, sa)
                    },
                    0b000_100 => {
                        panic!("sllv r{}, r{}, r{}", rd, rt, rs)
                    },
                    0b000_110 => {
                        panic!("srlv r{}, r{}, r{}", rd, rt, rs)
                    },
                    0b001_000 => {
                        panic!("jr r{}", rs)
                    },
                    0b010_000 => {
                        panic!("mfhi r{}", rd)
                    },
                    0b010_010 => {
                        panic!("mflo r{}", rd)
                    },
                    0b011_001 => {
                        panic!("multu r{}, r{}", rs, rt)
                    },
                    0b100_001 => {
                        panic!("addu r{}, r{}, r{}", rd, rs, rt)
                    },
                    0b100_011 => {
                        panic!("subu r{}, r{}, r{}", rd, rs, rt)
                    },
                    0b100_100 => {
                        panic!("and r{}, r{}, r{}", rd, rs, rt)
                    },
                    0b100_101 => {
                        panic!("or r{}, r{}, r{}", rd, rs, rt)
                    },
                    0b100_110 => {
                        panic!("xor r{}, r{}, r{}", rd, rs, rt)
                    },
                    0b101_011 => {
                        panic!("sltu r{}, r{}, r{}", rd, rs, rt)
                    },
                    _ => panic!("Unknown special: 0b{:06b}", special)
                }
            },
            0b000_001 => {
                let regimm = (inst >> 16) & 0x1F;
                match regimm {
                    0b10_001 => {
                        panic!("tgeiu r{}, ${:04X}", rs, imm)
                    },
                    _ => panic!("Unknown regimm: 0b{:05b}", regimm)
                }
            },
            0b000_100 => {
                panic!("beq r{}, r{}, ${:04X}", rs, rt, imm)
            },
            0b000_101 => {
                panic!("bne r{}, r{}, ${:04X}", rs, rt, imm)
            },
            0b001_000 => {
                panic!("addi r{}, r{}, ${:04X}", rt, rs, imm)
            },
            0b001_001 => {
                panic!("addiu r{}, r{}, ${:04X}", rt, rs, imm)
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
                        println!("mtc0 r{}, cp0r{}", rt, rd);

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
                panic!("beql r{}, r{}, ${:04X}", rs, rt, imm);

            },
            0b010_101 => {
                panic!("bnel r{}, r{}, ${:04X}", rs, rt, imm)
            },
            0b100_011 => {
                println!("lw r{}, 0x{:04X}(r{})", rt, imm, rs);

                let signed_imm = (imm as i16) as u32;
                gpr[rt as usize] = rcp.read_u32(gpr[rs as usize].wrapping_add(signed_imm) as usize);
            },
            0b101_011 => {
                panic!("sw r{}, 0x{:04X}(r{})", rt, imm, rs)
            },
            0b111_111 => {
                panic!("<invalid>")
            },
            _ => panic!("Unknown op: 0b{:06b}", op)
        };

        // dump all registers after each instruction
        for k in 0..4 {
            print!(" ");
            for j in 0..8 {
                print!("R{:02}: ${:08X} ", k*8+j, gpr[(k*8+j) as usize]);
            }
            println!("");
        }

        println!("-");
    }
}
