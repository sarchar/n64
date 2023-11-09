#![allow(non_upper_case_globals)]

use crate::*;

// Exception handling registers
const Cop0_Context : usize = 4;
const Cop0_BadVAddr: usize = 8; // Bad Virtual Address
const Cop0_Count   : usize = 9;
const Cop0_Compare : usize = 11;
const Cop0_Status  : usize = 12;
const Cop0_Cause   : usize = 13;
const Cop0_EPC     : usize = 14; // Exception Program Counter
const _COP0_WATCHLO : usize = 18;
const _COP0_WATCHHI : usize = 19;
const Cop0_XContext: usize = 20;
const _COP0_ERROREPC: usize = 30; // Error Exception Program Counter

const Cop0_Config: usize = 16;
const Cop0_LLAddr: usize = 17;

const STATUS_IM_TIMER_INTERRUPT_ENABLE_FLAG: u64 = 7;

const ExceptionCode_Int  : u64 = 0;  // interrupt
const _ExceptionCode_Mod  : u64 = 1;  // TLB modification exception
const _ExceptionCode_TLBL : u64 = 2;  // TLB Miss exception (load or instruction fetch)
const _ExceptionCode_TLBS : u64 = 3;  // TLB Miss exception (store)
const ExceptionCode_AdEL : u64 = 4;  // Address Error exception (load or instruction fetch)
const ExceptionCode_AdES : u64 = 5;  // Address Error exception (store)
const _ExceptionCode_IBE  : u64 = 6;  // Bus Error exception (instruction fetch)
const _ExceptionCode_DBE  : u64 = 7;  // Bus Error exception (data reference: load or store)
const _ExceptionCode_Sys  : u64 = 8;  // Syscall exception
const _ExceptionCode_Bp   : u64 = 9;  // Breakpoint exception
const _ExceptionCode_RI   : u64 = 10; // Reserved Instruction exception
const ExceptionCode_CpU  : u64 = 11; // Coprocessor Unusable exception
const ExceptionCode_Ov   : u64 = 12; // Arithmetic Overflow exception
const _ExceptionCode_Tr   : u64 = 13; // Trap instruction
const _ExceptionCode_FPE  : u64 = 15; // Floating-Point exception
const _ExceptionCode_WATCH: u64 = 23; // Watch exception

const InterruptCode_Timer: u64 = 0x80;

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
    next_instruction: u32,    // emulates delay slot
    next_instruction_pc: u32, // for printing correct delay slot addresses
    is_delay_slot: bool,      // true if the currently executing instruction is in a delay slot
    next_is_delay_slot: bool, // set to true on branching instructions

    gpr: [u64; 32],
    lo: u64,
    hi: u64,

    cp0gpr: [u64; 32],
    half_clock: u32,
    llbit: bool,

    cop1: cop1::Cop1,

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

            pc  : 0,
            next_instruction: 0,
            next_instruction_pc: 0,
            is_delay_slot: false,
            next_is_delay_slot: false,

            gpr : [0u64; 32],
            lo  : 0,
            hi  : 0,

            cp0gpr: [0u64; 32],
            half_clock: 0,
            llbit: false,

            cop1: cop1::Cop1::new(),

            // Sorry for making these so wide, but it maps to the instruction decode table in the datasheet better!
            instruction_table: [
                    // _000                     _001                     _010                    _011                    _100                    _101                    _110                    _111
   /* 000_ */   Cpu::<T>::inst_special, Cpu::<T>::inst_regimm , Cpu::<T>::inst_j      , Cpu::<T>::inst_jal    , Cpu::<T>::inst_beq    , Cpu::<T>::inst_bne    , Cpu::<T>::inst_blez   , Cpu::<T>::inst_bgtz   ,
   /* 001_ */   Cpu::<T>::inst_addi   , Cpu::<T>::inst_addiu  , Cpu::<T>::inst_slti   , Cpu::<T>::inst_sltiu  , Cpu::<T>::inst_andi   , Cpu::<T>::inst_ori    , Cpu::<T>::inst_xori   , Cpu::<T>::inst_lui    ,
   /* 010_ */   Cpu::<T>::inst_cop0   , Cpu::<T>::inst_cop    , Cpu::<T>::inst_cop2   , Cpu::<T>::inst_invalid, Cpu::<T>::inst_beql   , Cpu::<T>::inst_bnel   , Cpu::<T>::inst_blezl  , Cpu::<T>::inst_unknown,
   /* 011_ */   Cpu::<T>::inst_daddi  , Cpu::<T>::inst_daddiu , Cpu::<T>::inst_ldl    , Cpu::<T>::inst_ldr    , Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid, Cpu::<T>::inst_invalid,
   /* 100_ */   Cpu::<T>::inst_lb     , Cpu::<T>::inst_unknown, Cpu::<T>::inst_lwl    , Cpu::<T>::inst_lw     , Cpu::<T>::inst_lbu    , Cpu::<T>::inst_lhu    , Cpu::<T>::inst_lwr    , Cpu::<T>::inst_lwu    ,
   /* 101_ */   Cpu::<T>::inst_sb     , Cpu::<T>::inst_sh     , Cpu::<T>::inst_swl    , Cpu::<T>::inst_sw     , Cpu::<T>::inst_sdl    , Cpu::<T>::inst_sdr    , Cpu::<T>::inst_swr    , Cpu::<T>::inst_cache  ,
   /* 110_ */   Cpu::<T>::inst_ll     , Cpu::<T>::inst_lwc1   , Cpu::<T>::inst_unknown, Cpu::<T>::inst_invalid, Cpu::<T>::inst_unknown, Cpu::<T>::inst_ldc1   , Cpu::<T>::inst_unknown, Cpu::<T>::inst_ld     ,
   /* 111_ */   Cpu::<T>::inst_sc     , Cpu::<T>::inst_swc1   , Cpu::<T>::inst_unknown, Cpu::<T>::inst_invalid, Cpu::<T>::inst_unknown, Cpu::<T>::inst_sdc1   , Cpu::<T>::inst_unknown, Cpu::<T>::inst_sd
            ],

            special_table: [
                    //   _000                       _001                       _010                       _011                       _100                       _101                       _110                       _111
   /* 000_ */   Cpu::<T>::special_sll    , Cpu::<T>::special_invalid, Cpu::<T>::special_srl    , Cpu::<T>::special_sra    , Cpu::<T>::special_sllv   , Cpu::<T>::special_invalid, Cpu::<T>::special_srlv   , Cpu::<T>::special_srav   ,
   /* 001_ */   Cpu::<T>::special_jr     , Cpu::<T>::special_jalr   , Cpu::<T>::special_invalid, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_sync   ,
   /* 010_ */   Cpu::<T>::special_mfhi   , Cpu::<T>::special_mthi   , Cpu::<T>::special_mflo   , Cpu::<T>::special_mtlo   , Cpu::<T>::special_dsllv  , Cpu::<T>::special_invalid, Cpu::<T>::special_dsrlv  , Cpu::<T>::special_dsrav  ,
   /* 011_ */   Cpu::<T>::special_mult   , Cpu::<T>::special_multu  , Cpu::<T>::special_div    , Cpu::<T>::special_divu   , Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_ddiv   , Cpu::<T>::special_ddivu  ,
   /* 100_ */   Cpu::<T>::special_add    , Cpu::<T>::special_addu   , Cpu::<T>::special_sub    , Cpu::<T>::special_subu   , Cpu::<T>::special_and    , Cpu::<T>::special_or     , Cpu::<T>::special_xor    , Cpu::<T>::special_nor    ,
   /* 101_ */   Cpu::<T>::special_invalid, Cpu::<T>::special_invalid, Cpu::<T>::special_slt    , Cpu::<T>::special_sltu   , Cpu::<T>::special_dadd   , Cpu::<T>::special_daddu  , Cpu::<T>::special_dsub   , Cpu::<T>::special_dsubu  ,
   /* 110_ */   Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid, Cpu::<T>::special_unknown, Cpu::<T>::special_invalid,
   /* 111_ */   Cpu::<T>::special_dsll   , Cpu::<T>::special_invalid, Cpu::<T>::special_dsrl   , Cpu::<T>::special_dsra   , Cpu::<T>::special_dsll32 , Cpu::<T>::special_invalid, Cpu::<T>::special_dsrl32 , Cpu::<T>::special_dsra32 ,
            ],

            regimm_table: [
                    //   _000                      _001                      _010                      _011                      _100                      _101                      _110                      _111
   /* 00_ */    Cpu::<T>::regimm_bltz   , Cpu::<T>::regimm_bgez   , Cpu::<T>::regimm_unknown, Cpu::<T>::regimm_bgezl  , Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid, Cpu::<T>::regimm_invalid,
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

        // COP0
        // TODO see section 6.4.4 Cold Reset for a complete list of initial register values
        self.cp0gpr[Cop0_Config] = 0x7006E463; // EC=1:15, EP=0, BE=1 (big endian), CU=0 (RFU?), K0=3 (kseg0 cache enabled)
        self.cp0gpr[Cop0_Status] = 0x200004; // BEV=1, ERL=1, SR=0 (SR will need to be 1 on soft reset), IE=0

        // fetch next_instruction before starting the loop
        self.prefetch();
        self.next_is_delay_slot = false;
    }

    fn ie(&self) -> bool {
        (self.cp0gpr[Cop0_Status] & 0x01) != 0
    }

    fn interrupts_enabled(&self, interrupt_number: u64) -> bool {
        // interrupts are enabled when IE is set and EXL (exception level) and ERL (error level) are 0
        // (and also when the IM bit is set for the specified interrupt)
        let check = (0x100 << interrupt_number) | 0x07;
        (self.cp0gpr[Cop0_Status] & check) == (check & !0x06)
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

    // prefetch the next instruction
    fn prefetch(&mut self) {
        self.next_instruction = self.read_u32(self.pc as usize);
        self.next_instruction_pc = self.pc;
        self.pc += 4;
    }

    fn exception(&mut self, exception_code: u64) {
        // clear BD, exception code and bits that should be 0
        self.cp0gpr[Cop0_Cause] &= !0xC0FF_00FF;

        // set the exception code
        self.cp0gpr[Cop0_Cause] |= exception_code << 2;

        // set the EXL bit to prevent another exception
        self.cp0gpr[Cop0_Status] |= 0x02;

        // set the Exception Program Counter to the currently executing instruction
        // which is pc-8.  pc-8 will always be the currently executing instruction
        // because 1) branch instructions do not cause exceptions, 2) jump instructions 
        // only cause exceptions when the address is invalid (and therefore haven't 
        // changed pc yet), and 3) no other instructions modify pc. Moreover, if we're
        // in a delay slot, we need to subtract another 4
        self.cp0gpr[Cop0_EPC] = (if self.is_delay_slot {
            // also set the BD flag
            self.cp0gpr[Cop0_Cause] |= 0x8000_0000;
            self.pc.wrapping_sub(12)
        } else {
            self.pc.wrapping_sub(8)
        } as i32) as u64;

        // set PC and discard the delay slot. PC is set to the vector base determined by the BEV
        // bit in Cop0_Status.
        // TODO TLB miss needs to branch to base plus offset 0x000
        // TODO XTLB miss needs to branch to base plus offset 0x080
        self.pc = if (self.cp0gpr[Cop0_Status] & 0x200000) == 0 { 0x8000_0000 } else { 0xBFC0_0200 } + 0x180;

        // we need to throw away next_instruction, so prefetch the new instruction now
        self.prefetch();
    }

    fn address_exception(&mut self, address: u64, is_write: bool) {
        println!("CPU: address exception!");

        self.cp0gpr[Cop0_BadVAddr] = address;
        let bad_vpn2 = address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        let exception_code = if is_write { ExceptionCode_AdES } else { ExceptionCode_AdEL };
        self.exception(exception_code);
    }

    fn overflow_exception(&mut self) {
        self.exception(ExceptionCode_Ov);
    }

    fn coprocessor_unusable_exception(&mut self, coprocessor_number: u64) {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | (coprocessor_number << 28);
        self.exception(ExceptionCode_CpU);
        eprintln!("CPU: coprocessor unusable exception (Cop0_Status = ${:08X})!", self.cp0gpr[Cop0_Status]);
    }

    fn interrupt(&mut self, interrupt_signal: u64) {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0xFF0) | (interrupt_signal << 8);

        eprintln!("CPU: interrupt!");
        self.exception(ExceptionCode_Int);
    }

    #[inline(always)]
    fn timer_interrupt(&mut self) {
        self.interrupt(InterruptCode_Timer);
    }

    #[inline(always)]
    fn branch(&mut self, condition: bool) {
        if condition {
            // target is the sum of the address of the delay slot instruction
            // plus the sign extended and left-shifted immediate offset
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u32) << 2);
        }

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    // branch likely instructions discards the delay slot when the branch is not taken
    #[inline(always)]
    fn branch_likely(&mut self, condition: bool) {
        if condition {
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u32) << 2);

            // note that the next instruction to execute is a delay slot instruction
            self.next_is_delay_slot = true;
        } else {
            // we need to throw away next_instruction when branch is not taken
            self.prefetch();
        }
    }

    pub fn step(&mut self) {
        if self.pc == 0xA4001420 {
            self.bus.print_debug_ipl2();
        } else if self.pc == 0x8000_02B4 {
            println!("CPU: Starting cartridge ROM!");
        }

        // increment Cop0_Count at half PClock and trigger exception
        self.half_clock ^= 1;
        self.cp0gpr[Cop0_Count] = ((self.cp0gpr[Cop0_Count] as u32) + self.half_clock) as u64;
        if self.cp0gpr[Cop0_Compare] == self.cp0gpr[Cop0_Count] {
            // Timer interrupt enable, bit 7 of IM field 
            if self.interrupts_enabled(STATUS_IM_TIMER_INTERRUPT_ENABLE_FLAG) { // TODO move to self.timer_interrupt()
                eprintln!("COP0: timer interrupt");
                self.timer_interrupt();
            }
        }

        // TODO temp check until address exceptions are implement
        if (self.pc & 0x03) != 0 {
            println!("CPU: unaligned PC read at PC=${:08X}", self.pc);

            // I don't fully understand the pipeline, so this is a bit of a hack
            // to get the address exception correct when reading new instructions
            // Note, the address that caused the error and the EPC are different
            let bad_vaddr = self.pc; // for BadVAddr
            self.pc += 8;            // for Cop0_EPC
            self.address_exception((bad_vaddr as i32) as u64, false);

            return;
        }

        // in delay slot flag
        self.is_delay_slot = self.next_is_delay_slot;
        self.next_is_delay_slot = false;

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
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.inst.signed_imm as u32;
        let result = rs.wrapping_add(rt);
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            println!("CPU: addi overflow detected: rs=${:08X} imm=${:04X} result=${:08X}", rs, rt, result);
            self.overflow_exception();
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

    fn inst_cop(&mut self) {
        let copno = ((self.inst.v >> 26) & 0x03) as u64;
        let cop = match copno {
            1 => &mut self.cop1,
            _ => panic!("unsupported"),
        };

        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & (0x1000_0000 << copno)) == 0 {
            self.coprocessor_unusable_exception(copno);
            return;
        }

        if (self.inst.v & (1 << 25)) != 0 {
            cop.special(self.inst.v);
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => {
                    println!("mfc1 r{}, cgpr{}", self.inst.rt, self.inst.rd);

                    // TODO see datasheet for MFCz but this seemds weird. We use dmfc (double from
                    // cop) and then select low or high word based on the register index
                    self.gpr[self.inst.rt] = (if (self.inst.rd & 0x01) == 0 {
                        cop.dmfc(self.inst.rd & !0x01) & 0xFFFF_FFFF
                    } else {
                        (cop.dmfc(self.inst.rd & !0x01) >> 32) & 0xFFFF_FFFF
                    } as i32) as u64;
                },

                0b00_001 => {
                    println!("dmfc{} r{}, cgpr{}", copno, self.inst.rt, self.inst.rd);
                    self.gpr[self.inst.rt] = cop.dmfc(self.inst.rd);
                },

                0b00_010 => {
                    println!("cfc1 r{}, cr{}", self.inst.rt, self.inst.rd);
                    self.gpr[self.inst.rt] = cop.cfc(self.inst.rd);
                },

                0b00_100 => {
                    println!("mtc{} r{}, cgpr{} (r{}=${:08X})", copno, self.inst.rt, self.inst.rd, self.inst.rt, self.gpr[self.inst.rt]);
                    cop.mtc(self.gpr[self.inst.rt] as u32, self.inst.rd);
                },

                0b00_101 => {
                    println!("dmtc{} r{}, cgpr{} (r{}=${:16X})", copno, self.inst.rt, self.inst.rd, self.inst.rt, self.gpr[self.inst.rt]);
                    cop.dmtc(self.gpr[self.inst.rt], self.inst.rd);
                },

                0b00_110 => {
                    println!("ctc{} r{}, cr{}", copno, self.inst.rt, self.inst.rd);
                    cop.ctc(self.gpr[self.inst.rt], self.inst.rd);
                },

                /*
                0b01_000 => {
                    let branch = (self.inst.v >> 16) & 0x1F;
                    match branch {
                        _ => eprintln!("COP1: unhandled branch mode ${:05b}", branch),
                    };
                }
                */

                _ => panic!("CPU: unknown cop function 0b{:02b}_{:03b} (called on cop{})", func >> 3, func & 7, copno),
            };
        }
    }

    // convert into inst_cop at some point
    // all the cop should implement a common cop trait (mfc/mtc/ctc/etc)
    fn inst_cop0(&mut self) {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_000 => {
                println!("mfc0 r{}, cp0r{}", self.inst.rt, self.inst.rd);
                self.gpr[self.inst.rt] = (self.cp0gpr[self.inst.rd] as i32) as u64;
            },

            0b00_001 => {
                println!("dmfc0 r{}, cp0r{}", self.inst.rt, self.inst.rd);
                self.gpr[self.inst.rt] = self.cp0gpr[self.inst.rd];
            },

            0b00_100 => {
                println!("mtc0 r{}, cp0r{} (r{}=${:08X})", self.inst.rt, self.inst.rd, self.inst.rt, self.gpr[self.inst.rt] as u32);
                let val = self.gpr[self.inst.rt];

                // fix bits of the Config register
                self.cp0gpr[self.inst.rd] = (match self.inst.rd {
                    Cop0_Config => {
                        let read_only_mask = 0xF0FF7FF0u64;
                        (val & !read_only_mask) | (self.cp0gpr[Cop0_Config] & read_only_mask)
                    },

                    Cop0_Compare => {
                        // TODO clear timer interrupt see 6.3.4
                        // truncate to 32-bits
                        (val as u32) as u64
                    },

                    Cop0_Cause => {
                        // handle software interrupts 
                        if (val & 0x300) != 0 {
                            panic!("CPU: cop0 software interrupt");
                        }

                        // Only bits IP1 and IP0 are writable
                        (self.cp0gpr[Cop0_Cause] & !0x300) | (val & 0x300)
                    },

                    _ => { val },
                } as i32) as u64;

                // TODO testing
                match self.inst.rd {
                    Cop0_Status => {
                        if (self.cp0gpr[Cop0_Status] & 0x0200_00E0) != 0 {
                            panic!("CPU: unsupported 64-bit mode or little endian mode");
                        }

                        if (self.cp0gpr[Cop0_Status] & 0x18) != 0 {
                            panic!("CPU: unsupported user or supervisor mode");
                        }

                        if (self.cp0gpr[Cop0_Status] & 0x800000) != 0 {
                            panic!("CPU: unsupported instruction trace mode");
                        }

                        if (self.cp0gpr[Cop0_Status] & 0x200000) != 0 {
                            panic!("CPU: exception vector change");
                        }
                    },

                    _ => {},
                };
            },

            0b00_101 => {
                println!("dmtc0 r{}, cp0r{} (r{}=${:018X})", self.inst.rt, self.inst.rd, self.inst.rt, self.gpr[self.inst.rt]);
                let val = self.gpr[self.inst.rt];

                self.cp0gpr[self.inst.rd] = match self.inst.rd {
                    _ => { val },
                };

                if self.inst.rd == Cop0_Status {
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

                    0b011_000 => {
                        assert!(!self.is_delay_slot); // ERET must not be in a delay slot TODO error gracefully
                        println!("eret");

                        // If ERL bit is set, load the contents of ErrorEPC to the PC and clear the
                        // ERL bit. Otherwise, load the PC from EPC and clear the EXL bit.
                        if (self.cp0gpr[Cop0_Status] & 0x04) != 0 {
                            panic!("COP0: error bit set"); // TODO
                        } else {
                            self.pc = self.cp0gpr[Cop0_EPC] as u32;
                            self.cp0gpr[Cop0_Status] &= !0x02;
                        }

                        self.prefetch();

                        // clear LLbit so that SC writes fail
                        self.llbit = false;
                    },

                    _ => panic!("COP0: unknown cp0 function 0b{:03b}_{:03b}", special >> 3, special & 0x07),
                }
            },

            _ => panic!("CPU: unknown cop0 op: 0b{:02b}_{:03b} (0b{:032b})", cop0_op >> 3, cop0_op & 0x07, self.inst.v)
        }
    }

    fn inst_cop2(&mut self) {
        let cop2_op = (self.inst.v >> 21) & 0x1F;
        match cop2_op {
            _ => eprintln!("CPU: unknown cop2 op: 0b{:02b}_{:03b} (0b{:032b})", cop2_op >> 3, cop2_op & 0x07, self.inst.v)
        };
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

     fn inst_daddi(&mut self) {
        println!("daddi r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);

        // integer overflow exception occurs with ADDI, unlike ADDIU
        let rs = self.gpr[self.inst.rs] as u64;
        let rt = self.inst.signed_imm;
        let result = rs.wrapping_add(rt);
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000) != 0 {
            println!("CPU: daddi overflow detected: rs=${:16} imm=${:04X} result=${:16}", rs, rt, result);
            self.overflow_exception();
        }

        self.gpr[self.inst.rt] = result;
    }

   fn inst_daddiu(&mut self) {
        println!("daddiu r{}, r{}, ${:04X}", self.inst.rt, self.inst.rs, self.inst.imm);
                
        // no integer overflow exception occurs with DADDIU
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
    }

    fn inst_j(&mut self) {
        let dest = ((self.pc - 4) & 0xF000_0000) | (self.inst.target << 2);
        println!("j ${:08X}", dest);

        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    fn inst_jal(&mut self) {
        let dest = ((self.pc - 4) & 0xF000_0000) | (self.inst.target << 2);
        println!("jal ${:08X}", dest);

        self.gpr[31] = (self.pc as i32) as u64;
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    fn inst_lb(&mut self) {
        println!("lb r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = (self.read_u8(address as usize) as u8) as u64;
    }

    fn inst_lbu(&mut self) {
        println!("lbu r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = self.read_u8(address as usize) as u64;
    }

    fn inst_ld(&mut self) {
        println!("ld r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, false);
        }

        self.gpr[self.inst.rt] = self.read_u64(address as usize);
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

    fn inst_lhu(&mut self) {
        println!("lhu r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        if (address & 0x01) != 0 {
            self.address_exception(address, false);
        }

        let word = self.read_u32((address & !0x02) as usize) as u64;
        self.gpr[self.inst.rt] = (word >> (16 - ((address & 0x02) << 3))) & 0x0000_FFFF;
    }

    fn inst_ll(&mut self) {
        println!("ll r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false);
        }
        self.gpr[self.inst.rt] = (self.read_u32(address as usize) as i32) as u64;

        // the "linked part" sets the LLAddr register in cop0 to the physical address
        // of the read, and the LLbit to 1
        self.cp0gpr[Cop0_LLAddr] = address & 0x1FFF_FFFF; // TODO use proper physical address
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
            self.address_exception(address, false);
            return;
        }

        self.gpr[self.inst.rt] = (self.read_u32(address as usize) as i32) as u64;
    }

    fn inst_lwu(&mut self) {
        println!("lw r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false);
        }

        self.gpr[self.inst.rt] = self.read_u32(address as usize) as u64;
    }

    fn inst_lwl(&mut self) {
        println!("lwl r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03);

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 0 {
            mem
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX >> (32 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
    }

    fn inst_lwr(&mut self) {
        println!("lwr r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03);

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 24 { // handle case where no shift occurs
            mem 
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX << (8 + shift))) | (mem >> (24 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
    }

    fn inst_ldc1(&mut self) {
        println!("ldc1 r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x07) != 0 {
            self.address_exception(address, false);
        }

        let value = self.read_u64(address as usize);
        self.cop1.ldc(self.inst.rt, value);
    }

    fn inst_lwc1(&mut self) {
        println!("lwc1 r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x03) != 0 {
            self.address_exception(address, false);
        }

        let value = self.read_u32(address as usize);
        self.cop1.lwc(self.inst.rt, value);
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
    
    fn inst_sh(&mut self) {
        println!("sb r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        let word = self.read_u32((address & !0x01) as usize) as u64;

        if (address & 0x02) != 0 {
            self.write_u32((((self.gpr[self.inst.rt] & 0xFFFF) << 16) | (word & 0x0000FFFF)) as u32, (address & !0x02) as usize);
        } else {
            self.write_u32(((self.gpr[self.inst.rt] & 0xFFFF) | (word & 0xFFFF0000)) as u32, (address & !0x02) as usize);
        }
    }

    fn inst_sc(&mut self) {
        println!("sc r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true);
        }

        if self.llbit && ((address & 0x1FFF_FFFF) == self.cp0gpr[Cop0_LLAddr]) {
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
            self.address_exception(address, true);
        }

        self.write_u64(self.gpr[self.inst.rt], address as usize);
    }

    fn inst_sdc1(&mut self) {
        println!("sdc1 r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, true);
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.sdc(self.inst.rt);
        self.write_u64(value, address as usize);
    }

    fn inst_swc1(&mut self) {
        println!("swc1 r{}, 0x{:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true);
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.swc(self.inst.rt);
        self.write_u32(value, address as usize);
    }


    fn inst_sdl(&mut self) {
        println!("sdl r{}, ${:04X}(r{})", self.inst.rt, self.inst.imm, self.inst.rs);
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // need to fetch data on cache misses but not uncachable addresses
        let mem = if (address & 0xF000_0000) != 0xA000_0000 {
            self.read_u64(address & !0x07)  // this read "simulates" the cache miss and fetch and 
                                            // doesn't happen for uncached addresses
        } else { panic!("test"); /*0*/ };

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
        } else { panic!("test"); /*0*/ };

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
            self.address_exception(address, true);
            return;
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
        } else { panic!("test"); /*0*/ };

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
        } else { panic!("test"); /*0*/ };

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

    fn regimm_bgez(&mut self) {
        println!("bgez r{}, ${:04X}", self.inst.rs, self.inst.imm);

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
            self.overflow_exception();
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

    fn special_dadd(&mut self) {
        println!("dadd r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];
        let result = rs.wrapping_add(rt);

        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            eprintln!("CPU: dadd overflow exception occurred");
        } else {
            self.gpr[self.inst.rd] = result;
        }
    }

    fn special_daddu(&mut self) {
        println!("daddu r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // daddu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_add(self.gpr[self.inst.rt]);
    }

    fn divide(&mut self, dividend: i64, divisor: i64) {
        if divisor != 0 {
            if dividend == i64::MIN && divisor == -1 {
                (self.lo, self.hi) = (i64::MIN as u64, 0)
            } else {
                // ideally the compiler will optmize this to a divmod operation
                (self.lo, self.hi) = ((dividend / divisor) as u64, (dividend % divisor) as u64)
            }
        } else {
            // with divisor == 0, set self.lo (the quotient) to 1 or -1 basd on the sign
            // of the dividend
            self.lo = if dividend < 0 { 1 } else { u64::MAX };
            self.hi = dividend as u64;
        }
    }

    fn divide_unsigned(&mut self, dividend: u64, divisor: u64) {
        if divisor != 0 {
            // ideally the compiler will optmize this to a divmod operation
            (self.lo, self.hi) = (dividend / divisor, dividend % divisor)
        } else {
            self.lo = u64::MAX;
            self.hi = dividend as u64;
        }
    }

    fn special_ddiv(&mut self) {
        println!("ddiv r{}, r{}", self.inst.rs, self.inst.rt);

        let rs = self.gpr[self.inst.rs] as i64;
        let rt = self.gpr[self.inst.rt] as i64;
        self.divide(rs, rt);
    }

    fn special_ddivu(&mut self) {
        println!("ddivu r{}, r{}", self.inst.rs, self.inst.rt);

        // unsigned 64 bit values
        self.divide_unsigned(self.gpr[self.inst.rs], self.gpr[self.inst.rt]);
    }

    fn special_div(&mut self) {
        println!("div r{}, r{}", self.inst.rs, self.inst.rt);

        // must be 32-bit sign extended values
        let rs = (self.gpr[self.inst.rs] as i32) as i64;
        let rt = (self.gpr[self.inst.rt] as i32) as i64;
        self.divide(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;
    }

    fn special_divu(&mut self) {
        println!("divu r{}, r{}", self.inst.rs, self.inst.rt);

        // unsigned 32 bit values
        let rs = self.gpr[self.inst.rs] & 0xFFFF_FFFF;
        let rt = self.gpr[self.inst.rt] & 0xFFFF_FFFF;
        self.divide_unsigned(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;
    }

    fn special_dsll(&mut self) {
        println!("dsll r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << self.inst.sa;
    }

    fn special_dsllv(&mut self) {
        println!("dsll r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (self.gpr[self.inst.rs] & 0x3F);
    }

    fn special_dsll32(&mut self) {
        println!("dsll32 r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (32 + self.inst.sa);
    }

    fn special_dsra(&mut self) {
        println!("dsra r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> self.inst.sa) as u64;
    }

    fn special_dsrav(&mut self) {
        println!("dsrav r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (self.gpr[self.inst.rs] & 0x3F)) as u64;
    }

    fn special_dsra32(&mut self) {
        println!("dsra32 r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (32 + self.inst.sa)) as u64;
    }

    fn special_dsrl32(&mut self) {
        println!("dslr32 r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (32 + self.inst.sa);
    }

    fn special_dsrl(&mut self) {
        println!("dsrl r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> self.inst.sa;
    }

    fn special_dsrlv(&mut self) {
        println!("dsrlv r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x3F);
    }

    fn special_dsub(&mut self) {
        println!("dsub r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            eprintln!("CPU: sub overflow exception occurred");
        } else {
            self.gpr[self.inst.rd] = result;
        }
    }

    fn special_dsubu(&mut self) {
        println!("dsubu r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // dsubu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]);
    }

    fn special_jalr(&mut self) {
        println!("jalr r{}, r{}", self.inst.rd, self.inst.rs);
        self.gpr[self.inst.rd] = (self.pc as i32) as u64; // pc pointing to after the delay slot already
        let dest = self.gpr[self.inst.rs] as u32;

        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    fn special_jr(&mut self) {
        println!("jr r{}", self.inst.rs);
        let dest = self.gpr[self.inst.rs];

        self.pc = dest as u32;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    fn special_mfhi(&mut self) {
        println!("mfhi r{}", self.inst.rd);
        self.gpr[self.inst.rd] = self.hi;
    }

    fn special_mflo(&mut self) {
        println!("mflo r{}", self.inst.rd);
        self.gpr[self.inst.rd] = self.lo;
    }

    fn special_mthi(&mut self) {
        println!("mthi r{}", self.inst.rd);
        self.hi = self.gpr[self.inst.rd];
    }

    fn special_mtlo(&mut self) {
        println!("mtlo r{}", self.inst.rd);
        self.lo = self.gpr[self.inst.rd];
    }

    fn special_mult(&mut self) {
        println!("mult r{}, r{}", self.inst.rs, self.inst.rt);

        // must be 32-bit sign extended values
        let result = ((self.gpr[self.inst.rs] as i32) as u64) * ((self.gpr[self.inst.rt] as i32) as u64);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;
    }

    fn special_multu(&mut self) {
        println!("multu r{}, r{}", self.inst.rs, self.inst.rt);

        // must be 32-bit unsigned numbers
        let result = (self.gpr[self.inst.rs] & 0xFFFF_FFFF) * (self.gpr[self.inst.rt] & 0xFFFF_FFFF);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;
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

        // TODO I'm very confused here. The VR4300 datasheet is very clear that a 32-bit signed
        // integer is right shifted, and at the end 64-bit sign extended, but n64-systemtest
        // seems to check that it's just a 64 shift and then truncated to 32 bits and then sign
        // extended.  I've left the "working" one in place but the one I think is correct (at least
        // until someone explains to my why it isn't!) below.
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> self.inst.sa) as i32) as u64;

        // truncate to u32, convert to signed, shift (fills 1s in the upper bits) and sign extend to u64
        //self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) as i32) >> self.inst.sa) as u64;
    }

    fn special_srav(&mut self) {
        println!("srav r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
    }

    fn special_srl(&mut self) {
        println!("srl r{}, r{}, {}", self.inst.rd, self.inst.rt, self.inst.sa);
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> self.inst.sa) as i32) as u64;
    }

    fn special_srlv(&mut self) {
        println!("srlv r{}, r{}, r{}", self.inst.rd, self.inst.rt, self.inst.rs);
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
    }

    fn special_sub(&mut self) {
        println!("sub r{}, r{}, r{}", self.inst.rd, self.inst.rs, self.inst.rt);
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            eprintln!("CPU: sub overflow exception occurred");
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }
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
