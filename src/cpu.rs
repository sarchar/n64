use crate::*;

const COP0_STATUS: usize = 12;
//const Cop0_Config: usize = 16;
const COP0_LLADDR: usize = 17;

struct InstructionDecode {
    v : u32,      // full 32-bit instruction
    op: u32,      // 6-bit opcode field
    regimm: u32,  // 5-bit regimm op
    special: u32, // 6-bit special op

    // rd, rs, and rt are mostly used to index into self.gpr, so they will be usize
    // but they are 5 bits in the instruction
    rd: usize,
    rs: usize,
    rt: usize,

    // 16-bit immediate and sign extended value
    // the signed_imm value needs to be casted to i64/i32 for signed comparisons
    imm: u64,
    signed_imm: u64,

    // 5 bit shift amount
    sa: u32,

    // 27-bit jump targets
    target: u32,
}

pub struct Cpu<T: Addressable> {
    bus: T,

    pc: u32,

    gpr: [u64; 32],
    lo: u32,
    hi: u32,

    cp0gpr: [u32; 32],
    llbit: bool,

    fcr: [u32; 32],
    _fgpr: [f32; 32],

    next_instruction: u32,    // emulates delay slot
    next_instruction_pc: u32, // for printing correct delay slot addresses

    instruction_table: [CpuInstruction<T>; 64],
    special_table: [CpuInstruction<T>; 64],
    regimm_table: [CpuInstruction<T>; 32],

    // TODO - decide of one table can be used for both co-processors or not
    //cop0_table: [CpuInstruction<T>; 32],
    //cop1_table: [CpuInstruction<T>; 32],

    // instruction decode values
    inst: InstructionDecode,
}

type CpuInstruction<T> = fn(&mut Cpu<T>) -> ();

impl<T: Addressable> Cpu<T> {
    pub fn new(bus: T) -> Cpu<T> {
        let mut cpu = Cpu {
            bus: bus,

            gpr : [0u64; 32],
            lo  : 0,
            hi  : 0,
            pc  : 0,

            cp0gpr: [0u32; 32],
            llbit: false,

            fcr: [0u32; 32],
            _fgpr: [0f32; 32],

            next_instruction: 0,
            next_instruction_pc: 0,

            // Sorry for making these so wide, but it maps to the instruction decode table in the datasheet better!
            instruction_table: [
                    // _000                     _001                     _010                    _011                    _100                    _101                    _110                    _111
   /* 000_ */   Cpu::<T>::inst_special, Cpu::<T>::inst_regimm , Cpu::<T>::inst_j      , Cpu::<T>::inst_jal    , Cpu::<T>::inst_beq    , Cpu::<T>::inst_bne    , Cpu::<T>::inst_blez   , Cpu::<T>::inst_bgtz   ,
   /* 001_ */   Cpu::<T>::inst_addi   , Cpu::<T>::inst_addiu  , Cpu::<T>::inst_slti   , Cpu::<T>::inst_sltiu  , Cpu::<T>::inst_andi   , Cpu::<T>::inst_ori    , Cpu::<T>::inst_xori   , Cpu::<T>::inst_lui    ,
   /* 010_ */   Cpu::<T>::inst_cop0   , Cpu::<T>::inst_cop1   , Cpu::<T>::inst_unknown, Cpu::<T>::inst_invalid, Cpu::<T>::inst_beql   , Cpu::<T>::inst_bnel   , Cpu::<T>::inst_blezl  , Cpu::<T>::inst_unknown,
   /* 011_ */   Cpu::<T>::inst_unknown, Cpu::<T>::inst_daddiu , Cpu::<T>::inst_ldl    , Cpu::<T>::inst_ldr    , Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid,
   /* 100_ */   Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_lw     , Cpu::<T>::inst_lbu    , Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown,
   /* 101_ */   Cpu::<T>::inst_sb     , Cpu::<T>::inst_unknown, Cpu::<T>::inst_swl    , Cpu::<T>::inst_sw     , Cpu::<T>::inst_sdl    , Cpu::<T>::inst_sdr    , Cpu::<T>::inst_swr    , Cpu::<T>::inst_cache  ,
   /* 110_ */   Cpu::<T>::inst_ll     , Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_invalid, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown,
   /* 111_ */   Cpu::<T>::inst_sc     , Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_invalid, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_unknown, Cpu::<T>::inst_sd
            ],

            special_table: [
                    //   _000                       _001                       _010                       _011                       _100                       _101                       _110                       _111
   /* 000_ */   Cpu::<T>::special_sll    , Cpu::<T>::special_invalid, Cpu::<T>::special_srl    , Cpu::<T>::special_sra    , Cpu::<T>::special_sllv   , Cpu::<T>::special_invalid, Cpu::<T>::special_srlv   , Cpu::<T>::special_unknown,
   /* 001_ */   Cpu::<T>::special_jr     , Cpu::<T>::special_jalr   , Cpu::<T>::special_invalid, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_sync   ,
   /* 010_ */   Cpu::<T>::special_mfhi   , Cpu::<T>::special_unknown, Cpu::<T>::special_mflo   , Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown,
   /* 011_ */   Cpu::<T>::special_unknown, Cpu::<T>::special_multu  , Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown,
   /* 100_ */   Cpu::<T>::special_add    , Cpu::<T>::special_addu   , Cpu::<T>::special_unknown, Cpu::<T>::special_subu   , Cpu::<T>::special_and    , Cpu::<T>::special_or     , Cpu::<T>::special_xor    , Cpu::<T>::special_nor    ,
   /* 101_ */   Cpu::<T>::special_invalid, Cpu::<T>::special_invalid, Cpu::<T>::special_slt    , Cpu::<T>::special_sltu   , Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown,
   /* 110_ */   Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid,
   /* 111_ */   Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_dsll32 , Cpu::<T>::special_invalid, Cpu::<T>::special_dslr32 , Cpu::<T>::special_unknown,
            ],


            regimm_table: [
                    //   _000                      _001                      _010                      _011                      _100                      _101                      _110                      _111
   /* 00_ */    Cpu::<T>::regimm_bltz   , Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_bgezl  , Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid,
   /* 01_ */    Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_invalid,
   /* 10_ */    Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_bgezal , Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid,
   /* 11_ */    Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid,
            ],

            inst: InstructionDecode {
                v: 0, op: 0, regimm: 0, special: 0, rd: 0, rs: 0, rt: 0,
                imm: 0, signed_imm: 0, sa: 0, target: 0,
            }
        };
        
        cpu.reset();
        cpu
    }

    pub fn reset(&mut self) {
        self.pc = 0xBFC0_0000;

        // fetch next_instruction before starting the loop
        self.next_instruction = self.read_u32(self.pc as usize);
        self.next_instruction_pc = self.pc;
        self.pc += 4;
    }

    pub fn pc(&self) -> &u32 {
        &self.pc
    }

    fn read_u8(&mut self, address: usize) -> u32 {
        let word = self.read_u32(address & !0x03);
        let shift = 24 - ((address & 0x03) << 3);
        (word >> shift) & 0xFF
    }

    #[inline(always)]
    fn read_u32(&mut self, address: usize) -> u32 {
        self.bus.read_u32(address)
    }

    #[inline(always)]
    fn read_u64(&mut self, address: usize) -> u64 {
        ((self.read_u32(address) as u64) << 32) | (self.read_u32(address + 4) as u64)
    }

    // The cpu would perform the external bus read if the address access
    // is uncached or there's a cache miss. otherwise, the value from cache would be used
    // so the programmer needs to be aware of side effects when reading/writing bytes
    // R-M-W for write_u8
    fn write_u8(&mut self, value: u8, address: usize) -> WriteReturnSignal {
        let aligned_address = address & !0x03;

        // on cacheable addresses, we need to have the old data in order to change a byte
        let word = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u32(aligned_address)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { 0 };

        let shift = 24 - ((address & 0x03) << 3);
        let mask = 0xFFu32 << shift;
        let nv = (word & !mask) | ((value as u32) << shift);
        self.write_u32(nv, aligned_address)
    }

    #[inline(always)]
    fn write_u32(&mut self, value: u32, address: usize) -> WriteReturnSignal {
        self.bus.write_u32(value, address)
    }

    #[inline(always)]
    fn write_u64(&mut self, value: u64, address: usize) -> WriteReturnSignal {
        self.write_u32((value >> 32) as u32, address);
        self.write_u32(value as u32, address + 4)
    }

    #[inline(always)]
    fn branch(&mut self, condition: bool) {
        if condition {
            // target is the sum of the address of the delay slot instruction
            // plus the sign extended and left-shifted immediate offset
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u32) << 2);
        }
    }

    // branch likely instructions discards the delay slot when the branch is not taken
    #[inline(always)]
    fn branch_likely(&mut self, condition: bool) {
        if condition {
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u32) << 2);
        } else {
            // we need to throw away next_instruction when branch is not taken
            self.next_instruction = self.read_u32(self.pc as usize); // pc already points after the delay slot
            self.next_instruction_pc = self.pc;
            self.pc += 4;
        }
    }

    pub fn step(&mut self) {
        assert!((self.pc & 0x03) == 0);

        if self.pc == 0xA4001420 {
            self.bus.print_debug_ipl2();
        } else if self.pc == 0x8000_02B4 {
            println!("CPU: Starting cartridge ROM!");
        }

        // current instruction
        let inst = self.next_instruction;
        self.inst.v = inst;

        // next instruction fetch
        self.next_instruction = self.read_u32(self.pc as usize);

        // instruction decode
        self.inst.op         = inst >> 26;
        self.inst.rs         = ((inst >> 21) & 0x1F) as usize;
        self.inst.rt         = ((inst >> 16) & 0x1F) as usize;
        self.inst.rd         = ((inst >> 11) & 0x1F) as usize;
        self.inst.imm        = (inst & 0xFFFF) as u64;
        self.inst.signed_imm = (self.inst.imm as i16) as u64;
        self.inst.target     = inst & 0x3FFFFFF;
        self.inst.sa         = (inst >> 6) & 0x1F;

        print!("i {:08X}: ", self.next_instruction_pc);
        //print!("{:08X}, op=0b{:06b}: ", 0xBFC00000+i, inst, op);
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        self.instruction_table[self.inst.op as usize](self);

        // r0 must always be zero
        self.gpr[0] = 0;

        // dump all registers after each instruction
        //.for k in 0..8 {
        //.    print!("r ");
        //.    for j in 0..4 {
        //.        print!("R{:02}: ${:08X}_{:08X} ", k*4+j, self.gpr[(k*4+j) as usize] >> 32, self.gpr[(k*4+j) as usize] & 0xFFFF_FFFF);
        //.    }
        //.    println!("");
        //.}
        //.println!("-");
    }

    fn inst_invalid(&mut self) {
        panic!("CPU: invalid function ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
    }

    fn inst_unknown(&mut self) {
        panic!("CPU: unimplemented function ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
    }

    fn inst_addi(&mut self) {
        println!("addi r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);

        // integer overflow exception occurs with ADDI, unlike ADDIU
        let src = self.gpr[self.inst.rs] as u32;
        let result = src.wrapping_add(self.inst.signed_imm as u32);
        let is_pos = (self.inst.imm & 0x8000) == 0;
        if self.inst.imm != 0 && ((is_pos && result < src) || (!is_pos && result > src)) {
            panic!("overflow detected: src=${:08X} imm=${:04X} result=${:08X}", src, self.inst.imm as u32, result);
        }

        self.gpr[self.inst.rt] = (result as i32) as u64;
    }

    fn inst_addiu(&mut self) {
        println!("addiu r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
                
        // no integer overflow exception occurs with ADDIU
        self.gpr[self.inst.rt] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.inst.signed_imm as u32) as i32) as u64;
    }

    fn inst_andi(&mut self) {
        println!("andi r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] & self.inst.imm;
    }

    fn inst_cop0(&mut self) {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_100 => {
                println!("mtc0 r{}, cp0gpr{} (r{}=${:08X})", self.inst.rt, self.inst.rd, self.inst.rt, self.gpr[self.inst.rt] as u32);
                self.cp0gpr[self.inst.rd] = self.gpr[self.inst.rt] as u32;

                if self.inst.rd == COP0_STATUS {
                    if (self.cp0gpr[self.inst.rd] & 0x0200_00E0) != 0 {
                        panic!("CPU: unsupported 64-bit mode or little endian mode");
                    }
                }
            },

            0b10_000..=0b11_111 => {
                let special = self.inst.v & 0x3F;
                match special {
                    0b000_010 => {
                        println!("COP0: tlbwi");
                    },

                    _ => panic!("COP0: unknown cp0 function"),
                }
            },

            _ => panic!("CPU: unknown cop0 op: 0b{:02b}_{:03b} (0b{:032b}", cop0_op >> 3, cop0_op & 0x07, self.inst.v)
        }
    }

    fn inst_cop1(&mut self) {
        let cop1_op = (self.inst.v >> 21) & 0x1F;
        match cop1_op {
            0b00_110 => {
                println!("ctc1 r{}, fcr{}", self.inst.rt, self.inst.rd);
                self.fcr[self.inst.rd] = self.gpr[self.inst.rt] as u32;
            },
            _ => panic!("CPU: unknown cop1 op: 0b{:02b}_{:03b}", cop1_op >> 3, cop1_op & 0x07)
        }
    }

    fn inst_beq(&mut self) {
        println!("beq r{}, r{}, ${:04X}", self.inst.rs, self.inst.rt, self.inst.imm);
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch(condition);
    }

    fn inst_beql(&mut self) {
        println!("beql r{}, r{}, ${:04X}", self.inst.rs, self.inst.rt, self.inst.imm);
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch_likely(condition);
    }

    fn inst_bgtz(&mut self) {
        println!("bgtz r{}, ${:04X}", self.inst.rs, self.inst.imm);
        let condition = (self.gpr[self.inst.rs] as i64) > 0;
        self.branch(condition);
    }

    fn inst_blez(&mut self) {
        println!("blez r{}, ${:04X}", self.inst.rs, self.inst.imm);
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch(condition);
    }

    fn inst_blezl(&mut self) {
        println!("blezl r{}, ${:04X}", self.inst.rs, self.inst.imm);
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch_likely(condition);
    }

    fn inst_bne(&mut self) {
        println!("bne r{}, r{}, ${:04X}", self.inst.rs, self.inst.rt, self.inst.imm);
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch(condition);
    }

    fn inst_bnel(&mut self) {
        println!("bnel r{}, r{}, ${:04X}", self.inst.rs, self.inst.rt, self.inst.imm);
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch_likely(condition);
    }

    fn inst_cache(&mut self) {
        println!("cache ${:02X},${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
    }

    fn inst_daddiu(&mut self) {
        println!("daddiu r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
                
        // no integer overflow exception occurs with ADDIU
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
    }

    fn inst_j(&mut self) {
        let dest = ((self.pc - 4) & 0xF000_0000) | (self.inst.target << 2);
        println!("j ${:08X}", dest);
        self.pc = dest;
    }

    fn inst_jal(&mut self) {
        let dest = ((self.pc - 4) & 0xF000_0000) | (self.inst.target << 2);
        println!("jal ${:08X}", dest);
        self.gpr[31] = (self.pc as i32) as u64;
        self.pc = dest;
    }

    fn inst_lbu(&mut self) {
        println!("lbu r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = self.read_u8(address as usize) as u64;
    }

    fn inst_ldl(&mut self) {
        println!("ldl r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07);

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 0 {
            mem
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX >> (64 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = new; 
    }

    fn inst_ldr(&mut self) {
        println!("ldr r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07);

        // combine register and mem
        let shift = (address & 0x07) << 3;

        let new = if shift == 56 { // handle case where no shift occurs
            mem 
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX << (8 + shift))) | (mem >> (56 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = new; 
    }

    fn inst_ll(&mut self) {
        println!("ll r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            panic!("address exception!");
        }
        self.gpr[self.inst.rt] = (self.read_u32(address as usize) as i32) as u64;

        // the "linked part" sets the LLAddr register in cop0 to the physical address
        // of the read, and the LLbit to 1
        self.cp0gpr[COP0_LLADDR] = (address & 0x1FFF_FFFF) as u32; // TODO use proper physical address
        self.llbit = true;
    }

    fn inst_lui(&mut self) {
        println!("lui r{}, ${:04X}", self.inst.rt, self.inst.imm);
        self.gpr[self.inst.rt] = self.inst.signed_imm << 16;
    }

    fn inst_lw(&mut self) {
        println!("lw r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            panic!("address exception!");
        }

        self.gpr[self.inst.rt] = (self.read_u32(address as usize) as i32) as u64;
    }

    fn inst_ori(&mut self) {
        println!("ori r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] | self.inst.imm;
    }

    fn inst_regimm(&mut self) {
        self.inst.regimm = (self.inst.v >> 16) & 0x1F;
        self.regimm_table[self.inst.regimm as usize](self);
    }

    fn inst_sb(&mut self) {
        println!("sb r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.write_u8(self.gpr[self.inst.rt] as u8, address as usize);
    }

    fn inst_sc(&mut self) {
        println!("sc r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as u32;
        if (address & 0x03) != 0 {
            panic!("address exception!");
        }

        if self.llbit && ((address & 0x1FFF_FFFF) == self.cp0gpr[COP0_LLADDR]) {
            self.write_u32(self.gpr[self.inst.rt] as u32, address as usize);
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }
    }

    fn inst_sd(&mut self) {
        println!("sd r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            panic!("address exception!");
        }

        self.write_u64(self.gpr[self.inst.rt], address as usize);
    }

    fn inst_sdl(&mut self) {
        println!("sdl r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u64(address & !0x07)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { panic!("test"); 0 };

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] >> shift) | (mem & (u64::MAX << (64 - shift)))
        };

        // write new value
        println!("SDL: writing ${:016X} to address ${:016X}", new, address & !0x07);
        self.write_u64(new, address & !0x07);
    }

    fn inst_sdr(&mut self) {
        println!("sdr r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u64(address & !0x07)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { panic!("test"); 0 };

        // combine register and mem
        let shift = 56 - ((address & 0x07) << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] << shift) | (mem & (u64::MAX >> (64 - shift)))
        };

        // write new value
        self.write_u64(new, address & !0x07);
    }

    fn inst_slti(&mut self) {
        println!("slti r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);

        if (self.gpr[self.inst.rs] as i64) < (self.inst.signed_imm as i64) {
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }
    }

    fn inst_sltiu(&mut self) {
        println!("sltiu r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);

        self.gpr[self.inst.rt] = (self.gpr[self.inst.rs] < self.inst.signed_imm) as u64;
    }

    fn inst_special(&mut self) {
        self.inst.special = self.inst.v & 0x3F;
        self.special_table[self.inst.special as usize](self);
    }

    fn inst_sw(&mut self) {
        println!("sw r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            panic!("address exception!");
        }

        self.write_u32(self.gpr[self.inst.rt] as u32, address as usize);
    }

    fn inst_swl(&mut self) {
        println!("swl r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u32(address & !0x03)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { panic!("test"); 0 };

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) >> shift) | (mem & (u32::MAX << (32 - shift)))
        };

        // write new value
        self.write_u32(new, address & !0x03);
    }

    fn inst_swr(&mut self) {
        println!("swr r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u32(address & !0x03)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { panic!("test"); 0 };

        // combine register and mem
        let shift = 24 - ((address & 0x03) << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) << shift) | (mem & (u32::MAX >> (32 - shift)))
        };

        // write new value
        self.write_u32(new, address & !0x03);
    }

    fn inst_xori(&mut self) {
        println!("xori r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] ^ self.inst.imm;
    }

    fn regimm_unknown(&mut self) {
        panic!("CPU: unimplemented regimm op: 0b{:02b}_{:03b}", self.inst.regimm >> 3, self.inst.regimm & 0x07);
    }

    fn regimm_invalid(&mut self) {
        panic!("CPU: invalid regimm op: 0b{:02b}_{:03b}", self.inst.regimm >> 3, self.inst.regimm & 0x07);
    }

    fn regimm_bgezal(&mut self) {
        println!("bgezal r{}, ${:04X}", self.inst.rs, self.inst.imm);

        // addresses are sign extended
        self.gpr[31] = (self.pc as i32) as u64; // unconditionally, the address after the delay slot is stored in the link register

        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch(condition);
    }

    fn regimm_bgezl(&mut self) {
        println!("bgezl r{}, ${:04X}", self.inst.rs, self.inst.imm);

        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch_likely(condition);
    }

    fn regimm_bltz(&mut self) {
        println!("bltz r{}, ${:04X}", self.inst.rs, self.inst.imm);

        let condition = (self.gpr[self.inst.rs] as i64) < 0;
        self.branch(condition);
    }

    fn special_unknown(&mut self) {
        panic!("CPU: unimplemented special op: 0b{:03b}_{:03b}", self.inst.special >> 3, self.inst.special & 0x07);
    }

    fn special_invalid(&mut self) {
        panic!("CPU: invalid special op: 0b{:03b}_{:03b}", self.inst.special >> 3, self.inst.special & 0x07);
    }

    fn special_add(&mut self) {
        println!("add r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_add(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            panic!("overflow exception occurred");
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }
    }

    fn special_addu(&mut self) {
        println!("addu r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // addu does not cause an overflow exception
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.gpr[self.inst.rt] as u32) as i32) as u64;
    }

    fn special_and(&mut self) {
        println!("and r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);

        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] & self.gpr[self.inst.rt];
    }

    fn special_dsll32(&mut self) {
        println!("dsll32 r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (32 + self.inst.sa);
    }

    fn special_dslr32(&mut self) {
        println!("dslr32 r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (32 + self.inst.sa);
    }

    fn special_jalr(&mut self) {
        println!("jalr r{}, r{}", self.inst.rd, self.inst.rs);
        self.gpr[self.inst.rd] = (self.pc as i32) as u64; // pc pointing to after the delay slot already
        let dest = self.gpr[self.inst.rs] as u32;

        if (dest & 0x03) != 0 {
            panic!("address exception!");
        }

        self.pc = dest;
    }

    fn special_jr(&mut self) {
        println!("jr r{}", self.inst.rs);
        self.pc = self.gpr[self.inst.rs] as u32;
    }

    fn special_mfhi(&mut self) {
        println!("mfhi r{}", self.inst.rd);
        self.gpr[self.inst.rd] = (self.hi as i32) as u64;
    }

    fn special_mflo(&mut self) {
        println!("mflo r{}", self.inst.rd);
        self.gpr[self.inst.rd] = (self.lo as i32) as u64;
    }

    fn special_multu(&mut self) {
        println!("multu r{}, r{}", self.inst.rs, self.inst.rt);

        // must be 32-bit sign extended values
        let result = ((self.gpr[self.inst.rs] as i32) as u64) * ((self.gpr[self.inst.rt] as i32) as u64);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result as u32;
        self.hi = (result >> 32) as u32;
    }

    fn special_nor(&mut self) {
        println!("nor r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);

        self.gpr[self.inst.rd] = !(self.gpr[self.inst.rs] | self.gpr[self.inst.rt]);
    }

    fn special_or(&mut self) {
        println!("or r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);

        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] | self.gpr[self.inst.rt];
    }

    fn special_sll(&mut self) {
        println!("sll r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << self.inst.sa) as i32) as u64;
    }

    fn special_sllv(&mut self) {
        println!("sllv r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
    }

    fn special_slt(&mut self) {
        println!("slt r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);

        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as i64) < (self.gpr[self.inst.rt] as i64)) as u64;
    }

    fn special_sltu(&mut self) {
        println!("sltu r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);

        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs] < self.gpr[self.inst.rt]) as u64;
    }

    fn special_sra(&mut self) {
        println!("sra r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        // truncate to u32, convert to signed, shift (fills 1s in the upper bits) and sign extend to u64
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) as i32) >> self.inst.sa) as u64;

        // check for correctness (TODO remove later)
        if ((self.gpr[self.inst.rt] as u32) as i32) < 0 { assert!((self.gpr[self.inst.rd] as i64) < 0); }
    }

    fn special_srl(&mut self) {
        println!("srl r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as u32) >> self.inst.sa) as u64;
    }

    fn special_srlv(&mut self) {
        println!("srlv r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as u32) >> (self.gpr[self.inst.rs] & 0x1F)) as u64;
    }

    fn special_subu(&mut self) {
        println!("subu r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // subu does not cause an overflow exception
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]) as i32) as u64;
    }

    fn special_sync(&mut self) {
        println!("sync");
        // NOP on VR4300
    }

    fn special_xor(&mut self) {
        println!("xor r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] ^ self.gpr[self.inst.rt];
    }
}
