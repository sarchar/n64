use std::env;

use n64::Addressable;
use n64::pifrom::PifRom;
use n64::rcp::Rcp;
use n64::peripheral::PeripheralInterface;

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        assert!(args.len() == 1);
        println!("Usage: {} file.z64", args[0]);
        return;
    }

    let pif = PifRom::new("boot.rom");
    let pi = PeripheralInterface::new(args[1].as_str());
    let mut rcp = Rcp::new(pif, pi);

    let mut gpr = [0u32; 32];
    let mut lo = 0u32;
    let mut hi = 0u32;
    let mut cp0r = [0u32; 32];

    let mut pc: u32 = 0xBFC00000;
    let mut next_instruction: u32; // simulates delay slot

    let read_u8 = |rcp: &mut Rcp, address: usize| -> u32 {
        let word = rcp.read_u32(address & !0x03);
        let shift = 24 - ((address & 0x03) << 3);
        (word >> shift) & 0xFF
    };

    // The cpu would perform the external bus read if the address access
    // is uncached or there's a cache miss. otherwise, the value from cache would be used
    // so the programmer needs to be aware of side effects when reading/writing bytes
    // R-M-W for write_u8
    let write_u8 = |rcp: &mut Rcp, value: u8, address: usize| {
        let aligned_address = address & !0x03;

        // on cacheable addresses, we need to have the old data in order to change a byte
        let word = if (address & 0xF000_0000) != 0xA000_0000 {
            rcp.read_u32(aligned_address)  // this read "simulates" the cache miss and fetch and 
                                           // doesn't happen for uncached addresses
        } else { 0 };

        let shift = 24 - ((address & 0x03) << 3);
        let mask = 0xFFu32 << shift;
        let nv = (word & !mask) | ((value as u32) << shift);
        rcp.write_u32(nv, aligned_address);
    };

    // fetch next_instruction before starting the loop
    next_instruction = rcp.read_u32(pc as usize);
    let mut next_instruction_pc = pc;
    pc += 4;

    loop {
        assert!((pc & 0x03) == 0);
        if pc == 0xA4001420 {
            rcp.print_debug_ipl2();
        } else if pc == 0x8000_0000 || pc == 0xB000_0000 {
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
        let target = inst & 0x3FFFFFF;
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
                    0b00_011 => {
                        println!("bgezl r{}, ${:04X}", rs, imm);

                        if (gpr[rs as usize] & 0x8000_0000) == 0 {
                            pc = (pc - 4).wrapping_add(signed_imm << 2);
                        } else {
                            // we need to throw away next_instruction
                            next_instruction = rcp.read_u32(pc as usize); // pc already points after the delay slot
                            next_instruction_pc = pc;
                            pc += 4;
                        }
                    },
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
            0b000_011 => {
                let dest = ((pc - 4) & 0xF000_0000) | (target << 2);
                println!("jal ${:08X}", dest);
                gpr[31] = pc;
                pc = dest;
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
            0b001_010 => {
                println!("slti r{}, r{}, ${:04X}", rt, rs, imm);

                if (gpr[rs as usize] as i32) < (signed_imm as i32) {
                    gpr[rt as usize] = 1;
                } else {
                    gpr[rt as usize] = 0;
                }
            },
            0b001_100 => {
                println!("andi r{}, r{}, ${:04X}", rt, rs, imm);
                gpr[rt as usize] = gpr[rs as usize] & imm;
            },
            0b001_101 => {
                println!("ori r{}, r{}, ${:04X}", rt, rs, imm);
                gpr[rt as usize] = gpr[rs as usize] | imm;
            },
            0b001_110 => {
                println!("xori r{}, r{}, ${:04X}", rt, rs, imm);
                gpr[rt as usize] = gpr[rs as usize] ^ imm;
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
                    next_instruction_pc = pc;
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
                    next_instruction_pc = pc;
                    pc += 4;
                }
            },
            0b010_110 => {
                println!("blezl r{}, ${:04X}", rs, imm);

                // branch only when the condition is true
                // if the branch is not taken, the delay slot instruction is discarded
                if (gpr[rs as usize] & 0x8000_0000) != 0 || (gpr[rs as usize] == 0) {
                    // target is the sum of the address of the delay slot instruction
                    // plus the sign extended and left-shifted immediate offset
                    pc = (pc - 4).wrapping_add(signed_imm << 2);
                } else {
                    // we need to throw away next_instruction
                    next_instruction = rcp.read_u32(pc as usize); // pc already points after the delay slot
                    next_instruction_pc = pc;
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
            0b100_100 => {
                println!("lbu r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = gpr[rs as usize].wrapping_add(signed_imm) as usize;
                gpr[rt as usize] = read_u8(&mut rcp, address);
            },
            0b101_000 => {
                println!("sb r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = gpr[rs as usize].wrapping_add(signed_imm) as usize;
                write_u8(&mut rcp, (gpr[rt as usize] & 0xFF) as u8, address);
            },
            0b101_010 => {
                println!("sb r{}, 0x{:04X}(r{})", rt, imm, rs);

                let address = gpr[rs as usize].wrapping_add(signed_imm) as usize;
                write_u8(&mut rcp, (gpr[rt as usize] & 0xFF) as u8, address);
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
