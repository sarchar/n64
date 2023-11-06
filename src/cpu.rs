use crate::Addressable;

pub struct Cpu<T: Addressable> {
    bus: T,

    pc: u32,

    gpr: [u32; 32],
    lo: u32,
    hi: u32,

    cp0gpr: [u32; 32],

    next_instruction: u32,    // emulates delay slot
    next_instruction_pc: u32, // for printing correct delay slot addresses
}

impl<T: Addressable> Cpu<T> {
    pub fn new(bus: T) -> Cpu<T> {
        let mut cpu = Cpu {
            bus: bus,

            gpr : [0u32; 32],
            lo  : 0,
            hi  : 0,
            pc  : 0,

            cp0gpr: [0u32; 32],

            next_instruction: 0,
            next_instruction_pc: 0,
        };
        
        cpu.reset();
        cpu
    }

    pub fn reset(&mut self) {
        self.pc = 0xBFC0_0000;

        // fetch next_instruction before starting the loop
        self.next_instruction = self.bus.read_u32(self.pc as usize);
        self.next_instruction_pc = self.pc;
        self.pc += 4;
    }

    pub fn pc(&self) -> &u32 {
        &self.pc
    }

    fn read_u8(&mut self, address: usize) -> u32 {
        let word = self.bus.read_u32(address & !0x03);
        let shift = 24 - ((address & 0x03) << 3);
        (word >> shift) & 0xFF
    }

    // The cpu would perform the external bus read if the address access
    // is uncached or there's a cache miss. otherwise, the value from cache would be used
    // so the programmer needs to be aware of side effects when reading/writing bytes
    // R-M-W for write_u8
    fn write_u8(&mut self, value: u8, address: usize) {
        let aligned_address = address & !0x03;

        // on cacheable addresses, we need to have the old data in order to change a byte
        let word = if (address & 0xF000_0000) != 0xA000_0000 {
            self.bus.read_u32(aligned_address)  // this read "simulates" the cache miss and fetch and 
                                                // doesn't happen for uncached addresses
        } else { 0 };

        let shift = 24 - ((address & 0x03) << 3);
        let mask = 0xFFu32 << shift;
        let nv = (word & !mask) | ((value as u32) << shift);
        self.bus.write_u32(nv, aligned_address);
    }

    pub fn step(&mut self) {
        assert!((self.pc & 0x03) == 0);

        if self.pc == 0xA4001420 {
            self.bus.print_debug_ipl2();
        } else if self.pc == 0x8000_0000 || self.pc == 0xB000_0000 {
            panic!("Starting cartridge ROM!");
        }

        // current instruction
        let inst = self.next_instruction;

        // next instruction fetch
        self.next_instruction = self.bus.read_u32(self.pc as usize);

        // instruction decode
        let op     = inst >> 26;
        let rs     = (inst >> 21) & 0x1F;
        let rt     = (inst >> 16) & 0x1F;
        let rd     = (inst >> 11) & 0x1F;
        let imm    = inst & 0xFFFF;
        let signed_imm = (imm as i16) as u32;
        let target = inst & 0x3FFFFFF;
        let sa     = (inst >> 6) & 0x1F;

        print!("i {:08X}: ", self.next_instruction_pc);
        //print!("{:08X}, op=0b{:06b}: ", 0xBFC00000+i, inst, op);
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        // instruction execute
        match op {
            0b000_000 => {
                let special = inst & 0x3F;
                match special {
                    0b000_000 => {
                        println!("sll r{}, r{}, {}", rd, rt, sa);
                        self.gpr[rd as usize] = self.gpr[rt as usize] << sa;
                    },
                    0b000_010 => {
                        println!("srl r{}, r{}, {}", rd, rt, sa);
                        self.gpr[rd as usize] = self.gpr[rt as usize] >> sa;
                    },
                    0b000_100 => {
                        println!("sllv r{}, r{}, r{}", rd, rt, rs);
                        self.gpr[rd as usize] = self.gpr[rt as usize] << (self.gpr[rs as usize] & 0x1F);
                    },
                    0b000_110 => {
                        println!("srlv r{}, r{}, r{}", rd, rt, rs);
                        self.gpr[rd as usize] = self.gpr[rt as usize] >> (self.gpr[rs as usize] & 0x1F);
                    },
                    0b001_000 => {
                        println!("jr r{}", rs);
                        self.pc = self.gpr[rs as usize];
                    },
                    0b010_000 => {
                        println!("mfhi r{}", rd);
                        self.gpr[rd as usize] = self.hi;
                    },
                    0b010_010 => {
                        println!("mflo r{}", rd);
                        self.gpr[rd as usize] = self.lo;
                    },
                    0b011_001 => {
                        println!("multu r{}, r{}", rs, rt);
                        let result = (self.gpr[rs as usize] as u64) * (self.gpr[rt as usize] as u64);
                        // multu results are available in the next instruction since the multiply
                        // was started earlier in the pipeline
                        self.lo = (result & 0xFFFF_FFFF) as u32;
                        self.hi = ((result >> 32) & 0xFFFF_FFFF) as u32;
                    },
                    0b100_001 => {
                        println!("addu r{}, r{}, r{}", rd, rs, rt);
                        // addu does not cause an overflow exception
                        self.gpr[rd as usize] = self.gpr[rs as usize].wrapping_add(self.gpr[rt as usize]);
                    },
                    0b100_011 => {
                        println!("subu r{}, r{}, r{}", rd, rs, rt);
                        // subu does not cause an overflow exception
                        self.gpr[rd as usize] = self.gpr[rs as usize].wrapping_sub(self.gpr[rt as usize]);
                    },
                    0b100_100 => {
                        println!("and r{}, r{}, r{}", rd, rs, rt);

                        self.gpr[rd as usize] = self.gpr[rs as usize] & self.gpr[rt as usize];
                    },
                    0b100_101 => {
                        println!("or r{}, r{}, r{}", rd, rs, rt);

                        self.gpr[rd as usize] = self.gpr[rs as usize] | self.gpr[rt as usize];
                    },
                    0b100_110 => {
                        println!("xor r{}, r{}, r{}", rd, rs, rt);

                        self.gpr[rd as usize] = self.gpr[rs as usize] ^ self.gpr[rt as usize];
                    },
                    0b101_011 => {
                        println!("sltu r{}, r{}, r{}", rd, rs, rt);

                        // set rd to 1 if rs < rt, otherwise 0
                        self.gpr[rd as usize] = (self.gpr[rs as usize] < self.gpr[rt as usize]) as u32;
                    },
                    _ => panic!("Unknown special: 0b{:06b}", special)
                }
            },
            0b000_001 => {
                let regimm = (inst >> 16) & 0x1F;
                match regimm {
                    0b00_011 => {
                        println!("bgezl r{}, ${:04X}", rs, imm);

                        if (self.gpr[rs as usize] & 0x8000_0000) == 0 {
                            self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                        } else {
                            // we need to throw away next_instruction
                            self.next_instruction = self.bus.read_u32(self.pc as usize); // pc already points after the delay slot
                            self.next_instruction_pc = self.pc;
                            self.pc += 4;
                        }
                    },
                    0b10_001 => {
                        println!("bgezal r{}, ${:04X}", rs, imm);

                        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register

                        if (self.gpr[rs as usize] & 0x8000_0000) == 0 {
                            self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                        }
                    },

                    _ => panic!("Unknown regimm: 0b{:05b}", regimm)
                }
            },
            0b000_011 => {
                let dest = ((self.pc - 4) & 0xF000_0000) | (target << 2);
                println!("jal ${:08X}", dest);
                self.gpr[31] = self.pc;
                self.pc = dest;
            },
            0b000_100 => {
                println!("beq r{}, r{}, ${:04X}", rs, rt, imm);

                // The only difference between this and beql is that
                // the delay slot is not discarded
                if self.gpr[rs as usize] == self.gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                }
            },
            0b000_101 => {
                println!("bne r{}, r{}, ${:04X}", rs, rt, imm);

                // The only difference between this and bnel is that
                // the delay slot is not discarded
                if self.gpr[rs as usize] != self.gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                }
            },
            0b001_000 => {
                println!("addi r{}, r{}, ${:04X}", rt, rs, imm);

                // integer overflow exception occurs with ADDI, unlike ADDIU
                let src = self.gpr[rs as usize];
                let result = src.wrapping_add(signed_imm);
                let is_pos = (imm & 0x8000) == 0;
                if imm != 0 && ((is_pos && result < src) || (!is_pos && result > src)) {
                    panic!("overflow detected: src=${:08X} imm=${:08X} result=${:08X}", src, signed_imm, result);
                }

                self.gpr[rt as usize] = result;
            },
            0b001_001 => {
                println!("addiu r{}, r{}, ${:04X}", rt, rs, imm);
                
                // no integer overflow exception occurs with ADDIU
                self.gpr[rt as usize] = self.gpr[rs as usize].wrapping_add(signed_imm);
            },
            0b001_010 => {
                println!("slti r{}, r{}, ${:04X}", rt, rs, imm);

                if (self.gpr[rs as usize] as i32) < (signed_imm as i32) {
                    self.gpr[rt as usize] = 1;
                } else {
                    self.gpr[rt as usize] = 0;
                }
            },
            0b001_100 => {
                println!("andi r{}, r{}, ${:04X}", rt, rs, imm);
                self.gpr[rt as usize] = self.gpr[rs as usize] & imm;
            },
            0b001_101 => {
                println!("ori r{}, r{}, ${:04X}", rt, rs, imm);
                self.gpr[rt as usize] = self.gpr[rs as usize] | imm;
            },
            0b001_110 => {
                println!("xori r{}, r{}, ${:04X}", rt, rs, imm);
                self.gpr[rt as usize] = self.gpr[rs as usize] ^ imm;
            },
            0b001_111 => {
                println!("lui r{}, ${:04X}", rt, imm);
                self.gpr[rt as usize] = imm << 16;
            },
            0b010_000 => {
                let cop0 = (inst >> 21) & 0x1F;
                match cop0 {
                    0b00_100 => {
                        println!("mtc0 r{}, cp0gpr{} (r{}=${:08X})", rt, rd, rt, self.gpr[rt as usize]);

                        self.cp0gpr[rd as usize] = self.gpr[rt as usize];
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
                if self.gpr[rs as usize] == self.gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    self.next_instruction = self.bus.read_u32(self.pc as usize); // pc already points after the delay slot
                    self.next_instruction_pc = self.pc;
                    self.pc += 4;
                }
            },
            0b010_101 => {
                println!("bnel r{}, r{}, ${:04X}", rs, rt, imm);

                // branch only when the condition is true
                // if the branch is not taken, the delay slot instruction is discarded
                if self.gpr[rs as usize] != self.gpr[rt as usize] {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    self.next_instruction = self.bus.read_u32(self.pc as usize); // pc already points after the delay slot
                    self.next_instruction_pc = self.pc;
                    self.pc += 4;
                }
            },
            0b010_110 => {
                println!("blezl r{}, ${:04X}", rs, imm);

                // branch only when the condition is true
                // if the branch is not taken, the delay slot instruction is discarded
                if (self.gpr[rs as usize] & 0x8000_0000) != 0 || (self.gpr[rs as usize] == 0) {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    self.pc = (self.pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    self.next_instruction = self.bus.read_u32(self.pc as usize); // pc already points after the delay slot
                    self.next_instruction_pc = self.pc;
                    self.pc += 4;
                }
            },
            0b100_011 => {
                println!("lw r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = self.gpr[rs as usize].wrapping_add(signed_imm) as usize;
                if (address & 0x03) != 0 {
                    panic!("address exception!");
                }
                self.gpr[rt as usize] = self.bus.read_u32(address);
            },
            0b100_100 => {
                println!("lbu r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = self.gpr[rs as usize].wrapping_add(signed_imm) as usize;
                self.gpr[rt as usize] = self.read_u8(address);
            },
            0b101_000 => {
                println!("sb r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = self.gpr[rs as usize].wrapping_add(signed_imm) as usize;
                self.write_u8((self.gpr[rt as usize] & 0xFF) as u8, address);
            },
            0b101_010 => {
                println!("sb r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = self.gpr[rs as usize].wrapping_add(signed_imm) as usize;
                self.write_u8((self.gpr[rt as usize] & 0xFF) as u8, address);
            },
            0b101_011 => {
                println!("sw r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = self.gpr[rs as usize].wrapping_add(signed_imm) as usize;
                if (address & 0x03) != 0 {
                    panic!("address exception!");
                }
                self.bus.write_u32(self.gpr[rt as usize], address);
            },
            0b111_111 => {
                panic!("<invalid>")
            },
            _ => panic!("Unknown op: 0b{:06b}", op)
        };

        // r0 must always be zero
        self.gpr[0] = 0;

        // dump all registers after each instruction
        for k in 0..4 {
            print!("r ");
            for j in 0..8 {
                print!("R{:02}: ${:08X} ", k*8+j, self.gpr[(k*8+j) as usize]);
            }
            println!("");
        }

        println!("-");
    }
}
