#![allow(non_upper_case_globals)]

use cfg_if::cfg_if;

use std::arch::asm;
use std::cell::RefCell;
use std::rc::Rc;
use std::collections::HashMap;
use std::pin::Pin;

#[allow(unused_imports)]
use tracing::{debug, error, warn, info, trace};

use dynasmrt::{dynasm, DynasmApi, DynasmLabelApi};

use memoffset::offset_of;

use crate::*;

// Exception handling registers
const Cop0_Index   : usize = 0;
const Cop0_Random  : usize = 1;
const Cop0_EntryLo0: usize = 2;
const Cop0_EntryLo1: usize = 3;
const Cop0_Context : usize = 4;
const Cop0_PageMask: usize = 5;
const Cop0_Wired   : usize = 6;
const Cop0_BadVAddr: usize = 8; // Bad Virtual Address
const Cop0_Count   : usize = 9;
const Cop0_EntryHi : usize = 10;
const Cop0_Compare : usize = 11;
const Cop0_Status  : usize = 12;
const Cop0_Cause   : usize = 13;
const Cop0_EPC     : usize = 14; // Exception Program Counter
const Cop0_PRId    : usize = 15; // Processor Revision Identifier
const _COP0_WATCHLO : usize = 18;
const _COP0_WATCHHI : usize = 19;
const Cop0_XContext: usize = 20;
const Cop0_PErr    : usize = 26; // Parity Error
const Cop0_CacheErr: usize = 27;
const _COP0_ERROREPC: usize = 30; // Error Exception Program Counter

const Cop0_Config: usize = 16;
const Cop0_LLAddr: usize = 17;

const _STATUS_IM_TIMER_INTERRUPT_ENABLE_FLAG: u64 = 7;

const ExceptionCode_Int  : u64 = 0;  // interrupt
const ExceptionCode_Mod  : u64 = 1;  // TLB modification exception
const ExceptionCode_TLBL : u64 = 2;  // TLB Miss exception (load or instruction fetch)
const ExceptionCode_TLBS : u64 = 3;  // TLB Miss exception (store)
const ExceptionCode_AdEL : u64 = 4;  // Address Error exception (load or instruction fetch)
const ExceptionCode_AdES : u64 = 5;  // Address Error exception (store)
const _ExceptionCode_IBE  : u64 = 6;  // Bus Error exception (instruction fetch)
const _ExceptionCode_DBE  : u64 = 7;  // Bus Error exception (data reference: load or store)
const ExceptionCode_Sys  : u64 = 8;  // Syscall exception
const ExceptionCode_Bp   : u64 = 9;  // Breakpoint exception
const ExceptionCode_RI   : u64 = 10; // Reserved Instruction exception
const ExceptionCode_CpU  : u64 = 11; // Coprocessor Unusable exception
const ExceptionCode_Ov   : u64 = 12; // Arithmetic Overflow exception
const ExceptionCode_Tr   : u64 = 13; // Trap instruction
const ExceptionCode_FPE  : u64 = 15; // Floating-Point exception
const _ExceptionCode_WATCH: u64 = 23; // Watch exception

const InterruptCode_RPC: u64 = 0x04;
const InterruptCode_Timer: u64 = 0x80;

#[derive(Debug, Default)]
pub struct InstructionDecode {
    pub v : u32,      // full 32-bit instruction
    pub op: u32,      // 6-bit opcode field
    pub regimm: u32,  // 5-bit regimm op
    pub special: u32, // 6-bit special op

    // rd, rs, and rt are mostly used to index into self.gpr, so they will be usize
    // but they are 5 bits in the instruction
    pub rd: usize,
    pub rs: usize,
    pub rt: usize,

    // 16-bit immediate and sign extended value
    // the signed_imm value needs to be casted to i64/i32 for signed comparisons
    pub imm: u64,
    pub signed_imm: u64,

    // 5 bit shift amount
    pub sa: u32,

    // 27-bit jump targets
    pub target: u32,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum MemorySpace {
    User,
    Supervisor,
    XKPhys, // TODO
    Kernel
}

#[derive(Copy, Clone, Debug)]
pub struct Address {
    pub virtual_address: u64,
    pub physical_address: u64,
    pub cached: bool,
    pub mapped: bool,
    pub space: MemorySpace,
    pub tlb_index: Option<usize>, // tlb_index will be Some() when the virtual_address hits an EntryHi
                                  // in a mapped address
}

#[derive(Copy, Clone, Debug, Default)]
struct TlbEntry {
    page_mask: u64,
    entry_hi: u64,
    entry_lo1: u64,
    entry_lo0: u64
}

const JIT_BLOCK_MAX_INSTRUCTION_COUNT: usize = 512; //2048; // max instruction count per block

const CACHED_BLOCK_MAP_RDRAM_OFFSET  : usize = 0;
const CACHED_BLOCK_MAP_PIFROM_OFFSET : usize = CACHED_BLOCK_MAP_RDRAM_OFFSET   + (0x80_0000 >> 2);
const CACHED_BLOCK_MAP_RSP_MEM_OFFSET: usize = CACHED_BLOCK_MAP_PIFROM_OFFSET  + (0x800 >> 2);
const CACHED_BLOCK_MAP_SIZE          : usize = CACHED_BLOCK_MAP_RSP_MEM_OFFSET + (0x2000 >> 2);

#[derive(Copy, Clone, Debug)]
enum CachedBlockReference {
    CachedBlock(u64),              // official reference to the block
    AddressReference(usize, u64),  // usize is an offset into the cache, u64 is a block id
}

struct CompiledBlock {
    id: u64, // unique ID
    start_address: u64,
    _entry_point: dynasmrt::AssemblyOffset,
    _code_buffer: dynasmrt::mmap::ExecutableBuffer,
    jump_table: Pin<Box<Vec<u64>>>,
}

enum CachedBlockStatus {
    Uncompilable,
    NotCached,
    Cached(Rc<RefCell<CompiledBlock>>),
}

enum CompileInstructionResult {
    Continue, // Instruction compiled, keep going
    Stop,     // Instruction compiled, stop
    Cant,     // Instruction not compiled, exit block and interpret
}

// SysV64 calling convention/ABI:
// caller saved: rax, rdi, rsi, rcx, r8-r11, all xmm registers (volatile)
// callee saved: rbx, rbp, rsp, r12-r15 (non-volatile)
// TODO important: is fpcsr volatile? controls fp rounding and might need to be preserved
// rax is return value
// rsp is the stack pointer and must be 0x10 (16) bytes aligned before a function call
// rdi, rsi, rdx, rcx, r8, and r9 are the first six arguments to a function call -- these should never be
// aliased to r_ or v_ names, since they are used directly for function call parameters.
// further arguments are pushed to the stack in right-to-left order (left-most arguments are closer to rsp)
// at the end of a function, rsp must be equal to what it was at the beginning of a function, 
// unless the return value is in the stack, then rsp must be the original value minus the return value size
// optionally, functions can push rbp such that the return rip is 8 bytes above it, and then set rbp to the address
// of the saved rbp:
//    push rbp
//    mov rbp, rsp
macro_rules! letsgo {
    ($ops:ident $($t:tt)*) => {
        // for registers that we're using often, we want to use as many callee saved registers as
        // possible, otherwise we need to save them before every external call
        // 
        // convention: v_ are volatile registers, r_ are non-volatile, s_ are stack offsets
        // all the r_ registers must be saved in the prologue and restored in the epilogue of each
        // code block
        // we also do not alias rax (function pointer) or any of the arguments to function calls, (and 
        // all are valid to be used as another temporary storage register)
        dynasm!($ops
            ; .arch x64
            // saved registers
            ; .alias r_gpr, r12
            ; .alias r_cp0gpr, r13
            ; .alias r_cpu, r14
            ; .alias r_cond, ebx         // I want r_cond to be an "old" 32bit register for fast xor, also non-volatile
            ; .alias r_cond_64, rbx
            // volatile (non-saved) registers
            ; .alias v_tmp, r10          // I don't want to use any of the 6 function parameters since those are used directly
            ; .alias v_tmp_32, r10d
            ; .alias v_tmp2, r11
            // arguments to functions (also volatile)
            ; .alias v_arg0, rdi
            ; .alias v_arg1, rsi
            ; .alias v_arg2, rdx
            // ; .alias v_arg3, rcx    // not used
            // ; .alias v_arg0_32, edi // not used
            ; .alias v_arg1_32, esi
            ; .alias v_arg2_32, edx
            ; .alias v_arg3_32, ecx
            ; .alias v_arg1_8l, sil
            ; .alias v_arg2_8l, dl
            $($t)*
        );

        //trace!(target: "JIT-ASM", "\t{}", stringify!($($t)*).replace("\n", " ").replace("\r", "").replace(";", "\n\t;").trim());
    }
}

pub(crate) use letsgo;

// stack storage offsets, currently 0x18 bytes
// the trampoline uses call to jump into JIT code, so rip is sitting at the stack pointer, thus all these numbers have 8 added 
const s_jump_target: i32 = 0x00+8;  // pc jump target for jumps (64-bit)
const s_cycle_count: i32 = 0x08+8;  // number of r4300 instructions executed (32-bit)
const _s_unused    : i32 = 0x0C+8;  // unused space/padding (32-bit)
const s_tmp0       : i32 = 0x10+8;  // temp storage (64-bit)
const _s_next      : i32 = 0x18+8;  // next stack offset (dont forget to increase stack space)

// trigger a debugger breakpoint
#[allow(unused_macros)]
macro_rules! letsbreak {
    ($ops:ident) => {
        letsgo!($ops
            ;   int3
        );
    }
}

// use letscall! to call external "sysv64" functions
// this call uses the first parameter (Cpu) for all calls
// you can setup arguments before calling letscall!() in rsi, rdx, rcx...
macro_rules! letscall {
    ($ops:ident, $target:expr) => {
        letsgo!($ops
            ;   mov v_arg0, r_cpu                         // arg0 *mut Cpu
            ;   mov rax, QWORD $target as _               // function pointer
            ;   call rax                                  // make the call
        );
    }
}

// use letscop! to call external "sysv64" functions on other objects
// this call uses the first two parameters (Cpu, Cop1) for all calls
// you can setup arguments before calling letscop!() in rdx, rcx, r8, r8...
macro_rules! letscop {
    ($ops:ident, $cop:expr, $target:expr) => {
        letsgo!($ops
            ;   mov v_arg0, r_cpu                         // arg0 *mut Cpu
            ;   mov v_arg1, QWORD $cop as *mut _ as _     // arg1 *mut Cop1
            ;   mov rax, QWORD $target as _               // function pointer
            ;   call rax                                  // make the call
        );
    }
}


// place a u64 constant value into the specified register using as few instructions/bytes as possible
// signed ints smaller than u64 are sign-extended
// need to pass both the 64-bit and 32-bit register names
#[allow(unused_macros)]
macro_rules! letsset {
    ($ops:ident, $reg64:ident, $reg32:ident, $value:expr) => {
        let v = $value as u64;
        if v == 0 {
            letsgo!($ops
                ;   xor $reg64, $reg64
            );
        } else if (v >> 32) == 0u64 {
            letsgo!($ops
                ;   mov $reg32, DWORD v as i32 as _
            );
        } else if ((v as i32) as u64) == v {
            letsgo!($ops
                ;   mov $reg64, DWORD v as i32 as _
            );
        } else {
            letsgo!($ops
                ;   mov $reg64, QWORD v as _
            );
        }
    }
}

// set the link register to the self.pc, which is 8 bytes ahead of the current instruction pc
macro_rules! letslink {
    ($self:ident, $ops:ident) => {
        if (($self.pc as i32) as u64) == $self.pc {
            letsgo!($ops
                ;   mov QWORD [r_gpr + (31 * 8) as i32], DWORD $self.pc as i32 as _
            );
        } else {
            letsgo!($ops
                ;   mov v_tmp, QWORD $self.pc as _
                ;   mov QWORD [r_gpr + (31 * 8) as i32], v_tmp
            );
        }
    }
}

// add a signed immediate to a base register and store in a specific register
// used for stores and loads. $offset will be cast to 32-bits and sign-extended to the destination
// register (if 64-bit)
macro_rules! letsoffset {
    ($ops:ident, $base_reg:expr, $offset:expr, $reg:ident) => {
        if $base_reg == 0 {
            letsgo!($ops
                ;   xor $reg, $reg
            );
        } else {
            letsgo!($ops
                ;   mov $reg, QWORD [r_gpr + ($base_reg * 8) as i32]
            );
        }

        let v = $offset as i32;
        if v != 0 {
            letsgo!($ops
                ;   add $reg, DWORD v as _ // will do signed addition for both 32-bit
                                           // and 64-bit destination regs
            );
        }
    }
}

// prepares all variables needed for fn exception() to work properly
// set setup_cop_index to true if floating_point_exception is a possible exception
macro_rules! letssetupexcept {
    ($self:ident, $ops:ident, $setup_cop_index:expr) => {
        letsgo!($ops
            ;   mov BYTE [r_cpu + offset_of!(Cpu, is_delay_slot) as i32], BYTE $self.is_delay_slot as _
        );

        if (($self.current_instruction_pc as i32) as u64) == $self.current_instruction_pc {
            letsgo!($ops
                ;   mov QWORD [r_cpu + offset_of!(Cpu, current_instruction_pc) as i32], DWORD $self.current_instruction_pc as _
            );
        } else {
            letsgo!($ops
                ;   mov v_tmp2, QWORD $self.current_instruction_pc as _
                ;   mov QWORD [r_cpu + offset_of!(Cpu, current_instruction_pc) as i32], v_tmp2
            );
        }

        cfg_if! {
            if #[cfg(feature="jit-accuracy")] {
                if $setup_cop_index {
                    // this is a huge hack because next_instruction could be set to the instruction after a delay slot
                    // where an exception occurred. where really next_instruction should be the value from
                    // where the branch before this delay slot occurred. so hack alert: set the cop index
                    // to 0 on delay slot exceptions.  this will fail when the destination of the branch 
                    // has an instruction like MFC1 or MFC2 with when a co-processor exception occurs in
                    // the delay slot of that branch.
                    let next_cop_index = if !$self.is_delay_slot && ($self.next_instruction.unwrap() >> 29) == 0b010 { ($self.next_instruction.unwrap() >> 26) & 0b011 } else { 0 };
                    letsgo!($ops
                        ;   mov DWORD [r_cpu + offset_of!(Cpu, next_cop_index) as i32], DWORD next_cop_index as _
                    );
                }
            }
        }
    }
}

// calls the exception bridge function after setting up is_delay_slot and current_instruction_pc,
// which are required for exception() to work properly.  after, a jump to the epilog is executed.
// be sure to set up function arguments in rdx, r8, and r9 before calling letsexcept, which in
// turn calls letscall.
macro_rules! letsexcept {
    ($self:ident, $ops:ident, $exception_bridge:expr) => {
        letssetupexcept!($self, $ops, false);

        // this will call prefetch(), so we need to jump to the block epilog now.
        letscall!($ops, $exception_bridge);

        letsgo!($ops
            ;   dec DWORD [rsp+s_cycle_count]
            ;   jmp >epilog
        );
    }
}

// check the jit_other_exception flag after a bridged call that could generate exceptions
// if an exception occurred, the code jumps to the epilog of the current block
// expects:
//      0) jit_other_exception to be set if an exception was triggered
//      1) exception code in rax (return value from the rw call)
macro_rules! letscheck {
    ($ops:ident) => {
        // check if there was an exception in the previous letscall
        letsgo!($ops
            ;   cmp BYTE [r_cpu + offset_of!(Cpu, jit_other_exception) as i32], BYTE 0u8 as _
            // ;   jne >exit_for_exception
            ;   jne >epilog
        );

        // TODO: we could save and return an error code
    }
}

// The branch and branch likely instructions only vary in their comparison, so this macro encapsulates all the
// common code.  $not_cond needs to be the x64 instruction that would cause the branch NOT to be
// taken.  I.e., for branch-if-equal, use JNE (jump if not equal).  The reason is that the negated
// condition skips the instruction that sets the "should branch" flag to 1.  Set is_likely to true
// for the branch likely family
macro_rules! letsbranch {
    ($self:ident, $ops:ident, $cond_string:literal, $not_cond:ident, $is_likely:expr, $branch_taken_on_zero_zero:expr) => {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: {} r{}, r{}, ${:08X}", $self.current_instruction_pc as u32, $self.jit_current_assembler_offset, 
                                    $cond_string, $self.inst.rs, $self.inst.rt, ($self.inst.signed_imm << 2) as u32);

        // compute target jump and determine if the destination is within the current block
        // (self.pc is pointing at the delay slot)
        let dest = ($self.pc - 4).wrapping_add(($self.inst.signed_imm as u64) << 2);
        trace!(target: "JIT-BUILD", "** conditional branch to ${:08X}", dest as u32);

        // if we're in a delay slot, we should save r_cond before changing it
        // branching from delay slots is extremely uncommon, so this instruction isn't emitted very often
        // we save r_cond in case the instruction before this one set it
        if $self.is_delay_slot {
            trace!(target: "JIT-BUILD", "** conditional branch inside delay slot not compilable");
            return CompileInstructionResult::Cant;
        }

        // if both operands are zero we know the result of the branch in advance
        let mut is_unconditional = false;
        if $self.inst.rs == 0 && $self.inst.rt == 0 {
            is_unconditional = $branch_taken_on_zero_zero;
            if !is_unconditional {
                letsgo!($ops
                    ;   xor r_cond, r_cond
                );
            }
        } else {
            // clear r_cond
            letsgo!($ops
                ;   xor r_cond, r_cond
            );

            // rs!=0 and rt==0 is a common scenario
            if $self.inst.rs != 0 && $self.inst.rt == 0 {
                letsgo!($ops
                    ;   cmp QWORD [r_gpr + ($self.inst.rs * 8) as i32], BYTE 0u8 as _ // compare register to zero
                );
            } else {
                // if rs is 0, optimize to an xor instruction
                if $self.inst.rs == 0 {
                    letsgo!($ops
                        ;   xor v_tmp, v_tmp
                    );
                } else {
                    letsgo!($ops
                        ;   mov v_tmp, QWORD [r_gpr + ($self.inst.rs * 8) as i32]
                    );
                }

                // rt won't be zero here
                letsgo!($ops
                    ;   cmp v_tmp, QWORD [r_gpr + ($self.inst.rt * 8) as i32] // compare gpr[rs] to gpr[rt]
                );
            }

            letsgo!($ops
                ;   $not_cond >skip_set_cond                              // if !condition, don't set flag
                ;   inc r_cond                                            // otherwise set flag
                ;skip_set_cond:
            );
        }

        $self.jit_conditional_branch = Some((($self.inst.signed_imm as i64) << 2, $is_likely as bool, is_unconditional));
        $self.next_is_delay_slot = true;
    }
}

pub struct Cpu {
    comms: SystemCommunication,

    pub bus: Rc<RefCell<dyn Addressable>>,
    pc: u64,                       // lookahead PC
    current_instruction_pc: u64,   // actual PC of the currently executing instruction
                                   // only valid inside step()
    next_instruction: Option<u32>, // emulates delay slot (prefetch, next instruction), but can be None during JIT runs
    next_instruction_pc: u64,      // for printing correct delay slot addresses
    is_delay_slot: bool,           // true if the currently executing instruction is in a delay slot
    next_is_delay_slot: bool,      // set to true on branching instructions
    next_cop_index: u32,           // set to the cop index of the next instruction in the pipeline

    gpr: [u64; 32],
    lo: u64,
    hi: u64,

    cp0gpr: [u64; 32],
    cp0gpr_latch: u64,
    cp0gpr_random_delay: i64,
    tlb: [TlbEntry; 32],
    cp2gpr_latch: u64,

    // 32/64-bit addressing modes, set by Cop0_Status bits
    kernel_64bit_addressing: bool,

    llbit: bool,

    cop1: cop1::Cop1,

    instruction_table: [CpuInstruction; 64],
    special_table: [CpuInstruction; 64],
    regimm_table: [CpuInstruction; 32],

    // instruction decode values
    inst: InstructionDecode,

    num_steps: u64,

    // JIT
    cached_blocks: HashMap<u64, Rc<RefCell<CompiledBlock>>>,
    next_block_id: u64,

    // 8MiB for RDRAM
    // 2KiB for PIF-ROM
    // 8KiB for I/DMEM
    cached_block_map: Vec<Option<CachedBlockReference>>,

    jit_trampoline: Option<dynasmrt::mmap::ExecutableBuffer>,
    jit_trampoline_entry_point: dynasmrt::AssemblyOffset,
    jit_instruction_table: [CpuInstructionBuilder; 64],
    jit_special_table: [CpuInstructionBuilder; 64],
    jit_regimm_table: [CpuInstructionBuilder; 32],
    jit_block_start_pc: u64,
    jit_jump: bool, // true when a jump is happening
    jit_jump_no_delay: bool, // true when a jump happens without a delay slot
    jit_conditional_branch: Option<(i64, bool, bool)>, // branch_offset, is_branch_likely, is_unconditional
    jit_current_assembler_offset: usize,
    jit_executing: bool, // true when inside run_block()
    jit_other_exception: bool, // true when an InstructionFault::OtherException occurs inside a bridged function call
    jit_run_limit: u64, // starting run limit for each block that's run
    jit_idle_loop_detected: bool, // true if we exited a block due to an idle loop
    cp0_count_tracker: u64, // used to calculate Cop0_Count increment in JIT
    cp0_compare_distance: u32, // used to determine when the timer interrupt occurs in JIT
    cp0_random_tracker: u64, // used to calculate Cop0_Random decrement in JIT
}

// It may be worth mentioning that cpu exceptions are not InstructionFaults 
// InstructionFault is used for passing messages or interrupting cpu execution 
// within the emulator
#[derive(Debug)]
pub enum InstructionFault {
    Invalid,
    Unimplemented,
    Break,
    OtherException(u64),
    CoprocessorUnusable,
    FloatingPointException,
    ReadWrite(ReadWriteFault),
}

impl From<ReadWriteFault> for InstructionFault {
    fn from(value: ReadWriteFault) -> Self {
        InstructionFault::ReadWrite(value)
    }
}

type CpuInstruction = fn(&mut Cpu) -> Result<(), InstructionFault>;

pub type Assembler = dynasmrt::Assembler<dynasmrt::x64::X64Relocation>;
type CpuInstructionBuilder = fn(&mut Cpu, &mut Assembler) -> CompileInstructionResult;

impl Cpu {
    pub fn new(comms: SystemCommunication, bus: Rc<RefCell<dyn Addressable>>) -> Cpu {
        let mut cpu = Cpu {
            comms: comms,

            num_steps: 0,

            bus : bus,
            pc  : 0,
            current_instruction_pc: 0,
            next_instruction: None,
            next_instruction_pc: 0xFFFF_FFFF_FFFF_FFFF,
            is_delay_slot: false,
            next_is_delay_slot: false,
            next_cop_index: 0,

            gpr : [0u64; 32],
            lo  : 0,
            hi  : 0,

            cp0gpr: [0u64; 32],
            cp0gpr_latch: 0,
            cp0gpr_random_delay: 0,
            tlb: [TlbEntry::default(); 32],
            cp2gpr_latch: 0,
            kernel_64bit_addressing: false,
            llbit: false,

            cop1: cop1::Cop1::new(),

            // Sorry for making these so wide, but it maps to the instruction decode table in the datasheet better!
            instruction_table: [
                //  _000               _001              _010               _011                _100                _101                _110                _111
   /* 000_ */   Cpu::inst_special, Cpu::inst_regimm, Cpu::inst_j      , Cpu::inst_jal     , Cpu::inst_beq     , Cpu::inst_bne     , Cpu::inst_blez    , Cpu::inst_bgtz    ,
   /* 001_ */   Cpu::inst_addi   , Cpu::inst_addiu , Cpu::inst_slti   , Cpu::inst_sltiu   , Cpu::inst_andi    , Cpu::inst_ori     , Cpu::inst_xori    , Cpu::inst_lui     ,
   /* 010_ */   Cpu::inst_cop0   , Cpu::inst_cop   , Cpu::inst_cop2   , Cpu::inst_reserved, Cpu::inst_beql    , Cpu::inst_bnel    , Cpu::inst_blezl   , Cpu::inst_bgtzl   ,
   /* 011_ */   Cpu::inst_daddi  , Cpu::inst_daddiu, Cpu::inst_ldl    , Cpu::inst_ldr     , Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved, Cpu::inst_reserved,
   /* 100_ */   Cpu::inst_lb     , Cpu::inst_lh    , Cpu::inst_lwl    , Cpu::inst_lw      , Cpu::inst_lbu     , Cpu::inst_lhu     , Cpu::inst_lwr     , Cpu::inst_lwu     ,
   /* 101_ */   Cpu::inst_sb     , Cpu::inst_sh    , Cpu::inst_swl    , Cpu::inst_sw      , Cpu::inst_sdl     , Cpu::inst_sdr     , Cpu::inst_swr     , Cpu::inst_cache   ,
   /* 110_ */   Cpu::inst_ll     , Cpu::inst_lwc1  , Cpu::inst_unknown, Cpu::inst_reserved, Cpu::inst_lld     , Cpu::inst_ldc1    , Cpu::inst_unknown , Cpu::inst_ld      ,
   /* 111_ */   Cpu::inst_sc     , Cpu::inst_swc1  , Cpu::inst_unknown, Cpu::inst_reserved, Cpu::inst_scd     , Cpu::inst_sdc1    , Cpu::inst_unknown , Cpu::inst_sd
            ],

            special_table: [
               //   _000                  _001                  _010                  _011                  _100                  _101                  _110                  _111
   /* 000_ */   Cpu::special_sll    , Cpu::inst_reserved  , Cpu::special_srl    , Cpu::special_sra    , Cpu::special_sllv   , Cpu::inst_reserved  , Cpu::special_srlv   , Cpu::special_srav   ,
   /* 001_ */   Cpu::special_jr     , Cpu::special_jalr   , Cpu::inst_reserved  , Cpu::inst_reserved  , Cpu::special_syscall, Cpu::special_break  , Cpu::inst_reserved  , Cpu::special_sync   ,
   /* 010_ */   Cpu::special_mfhi   , Cpu::special_mthi   , Cpu::special_mflo   , Cpu::special_mtlo   , Cpu::special_dsllv  , Cpu::inst_reserved  , Cpu::special_dsrlv  , Cpu::special_dsrav  ,
   /* 011_ */   Cpu::special_mult   , Cpu::special_multu  , Cpu::special_div    , Cpu::special_divu   , Cpu::special_dmult  , Cpu::special_dmultu , Cpu::special_ddiv   , Cpu::special_ddivu  ,
   /* 100_ */   Cpu::special_add    , Cpu::special_addu   , Cpu::special_sub    , Cpu::special_subu   , Cpu::special_and    , Cpu::special_or     , Cpu::special_xor    , Cpu::special_nor    ,
   /* 101_ */   Cpu::inst_reserved  , Cpu::inst_reserved  , Cpu::special_slt    , Cpu::special_sltu   , Cpu::special_dadd   , Cpu::special_daddu  , Cpu::special_dsub   , Cpu::special_dsubu  ,
   /* 110_ */   Cpu::special_tge    , Cpu::special_tgeu   , Cpu::special_tlt    , Cpu::special_tltu   , Cpu::special_teq    , Cpu::inst_reserved  , Cpu::special_tne    , Cpu::inst_reserved  ,
   /* 111_ */   Cpu::special_dsll   , Cpu::inst_reserved  , Cpu::special_dsrl   , Cpu::special_dsra   , Cpu::special_dsll32 , Cpu::inst_reserved  , Cpu::special_dsrl32 , Cpu::special_dsra32 ,
            ],

            regimm_table: [
               //   _000                 _001                 _010                 _011                 _100                 _101                 _110                 _111
   /* 00_ */    Cpu::regimm_bltz   , Cpu::regimm_bgez   , Cpu::regimm_bltzl  , Cpu::regimm_bgezl  , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
   /* 01_ */    Cpu::regimm_tgei   , Cpu::regimm_tgeiu  , Cpu::regimm_tlti   , Cpu::regimm_tltiu  , Cpu::regimm_teqi   , Cpu::inst_reserved , Cpu::regimm_tnei   , Cpu::inst_reserved ,
   /* 10_ */    Cpu::regimm_unknown, Cpu::regimm_bgezal , Cpu::regimm_unknown, Cpu::regimm_bgezall, Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
   /* 11_ */    Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved , Cpu::inst_reserved ,
            ],

            inst: InstructionDecode {
                v: 0, op: 0, regimm: 0, special: 0, rd: 0, rs: 0, rt: 0,
                imm: 0, signed_imm: 0, sa: 0, target: 0,
            },

            // JIT
            cached_blocks: HashMap::new(),
            cached_block_map: vec![None; CACHED_BLOCK_MAP_SIZE],
            next_block_id: 0,

            jit_trampoline: None,
            jit_trampoline_entry_point: dynasmrt::AssemblyOffset(0),
            jit_instruction_table: [ 
               //  _000                     _001                     _010                     _011                     _100                     _101                     _110                     _111
    /* 000_ */  Cpu::build_inst_special, Cpu::build_inst_regimm , Cpu::build_inst_j      , Cpu::build_inst_jal     , Cpu::build_inst_beq     , Cpu::build_inst_bne     , Cpu::build_inst_blez    , Cpu::build_inst_bgtz    ,
    /* 001_ */  Cpu::build_inst_addi   , Cpu::build_inst_addiu  , Cpu::build_inst_slti   , Cpu::build_inst_sltiu   , Cpu::build_inst_andi    , Cpu::build_inst_ori     , Cpu::build_inst_xori    , Cpu::build_inst_lui     ,
    /* 010_ */  Cpu::build_inst_cop0   , Cpu::build_inst_cop    , Cpu::build_inst_cop2   , Cpu::build_inst_reserved, Cpu::build_inst_beql    , Cpu::build_inst_bnel    , Cpu::build_inst_blezl   , Cpu::build_inst_bgtzl   ,
    /* 011_ */  Cpu::build_inst_daddi  , Cpu::build_inst_daddiu , Cpu::build_inst_ldl    , Cpu::build_inst_ldr     , Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved,
    /* 100_ */  Cpu::build_inst_lb     , Cpu::build_inst_lh     , Cpu::build_inst_lwl    , Cpu::build_inst_lw      , Cpu::build_inst_lbu     , Cpu::build_inst_lhu     , Cpu::build_inst_lwr     , Cpu::build_inst_lwu     ,
    /* 101_ */  Cpu::build_inst_sb     , Cpu::build_inst_sh     , Cpu::build_inst_swl    , Cpu::build_inst_sw      , Cpu::build_inst_sdl     , Cpu::build_inst_sdr     , Cpu::build_inst_swr     , Cpu::build_inst_cache   ,
    /* 110_ */  Cpu::build_inst_ll     , Cpu::build_inst_lwc1   , Cpu::build_inst_unknown, Cpu::build_inst_unknown , Cpu::build_inst_lld     , Cpu::build_inst_ldc1    , Cpu::build_inst_unknown , Cpu::build_inst_ld      ,
    /* 111_ */  Cpu::build_inst_sc     , Cpu::build_inst_swc1   , Cpu::build_inst_unknown, Cpu::build_inst_unknown , Cpu::build_inst_scd     , Cpu::build_inst_sdc1    , Cpu::build_inst_unknown , Cpu::build_inst_sd      ,
            ],

            jit_special_table: [ 
               //  _000                     _001                      _010                     _011                     _100                       _101                     _110                     _111
    /* 000_ */  Cpu::build_special_sll , Cpu::build_inst_unknown , Cpu::build_special_srl , Cpu::build_special_sra , Cpu::build_special_sllv   , Cpu::build_inst_unknown  , Cpu::build_special_srlv  , Cpu::build_special_srav  ,
    /* 001_ */  Cpu::build_special_jr  , Cpu::build_special_jalr , Cpu::build_inst_unknown, Cpu::build_inst_unknown, Cpu::build_special_syscall, Cpu::build_special_break , Cpu::build_inst_unknown  , Cpu::build_special_sync  ,
    /* 010_ */  Cpu::build_special_mfhi, Cpu::build_special_mthi , Cpu::build_special_mflo, Cpu::build_special_mtlo, Cpu::build_special_dsllv  , Cpu::build_inst_unknown  , Cpu::build_special_dsrlv , Cpu::build_special_dsrav ,
    /* 011_ */  Cpu::build_special_mult, Cpu::build_special_multu, Cpu::build_special_div , Cpu::build_special_divu, Cpu::build_special_dmult  , Cpu::build_special_dmultu, Cpu::build_special_ddiv  , Cpu::build_special_ddivu ,
    /* 100_ */  Cpu::build_special_add , Cpu::build_special_addu , Cpu::build_special_sub , Cpu::build_special_subu, Cpu::build_special_and    , Cpu::build_special_or    , Cpu::build_special_xor   , Cpu::build_special_nor   ,
    /* 101_ */  Cpu::build_inst_unknown, Cpu::build_inst_unknown , Cpu::build_special_slt , Cpu::build_special_sltu, Cpu::build_special_dadd   , Cpu::build_special_daddu , Cpu::build_special_dsub  , Cpu::build_special_dsubu ,
    /* 110_ */  Cpu::build_special_tge , Cpu::build_special_tgeu , Cpu::build_special_tlt , Cpu::build_special_tltu, Cpu::build_special_teq    , Cpu::build_inst_unknown  , Cpu::build_special_tne   , Cpu::build_inst_unknown  ,
    /* 111_ */  Cpu::build_special_dsll, Cpu::build_inst_unknown , Cpu::build_special_dsrl, Cpu::build_special_dsra, Cpu::build_special_dsll32 , Cpu::build_inst_unknown  , Cpu::build_special_dsrl32, Cpu::build_special_dsra32,
            ],

            jit_regimm_table: [ 
               // _000                     _001                      _010                     _011                     _100                     _101                     _110                     _111
    /* 00_ */  Cpu::build_regimm_bltz  , Cpu::build_regimm_bgez  , Cpu::build_regimm_bltzl , Cpu::build_regimm_bgezl  , Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved,
    /* 01_ */  Cpu::build_regimm_tgei  , Cpu::build_regimm_tgeiu , Cpu::build_regimm_tlti  , Cpu::build_regimm_tltiu  , Cpu::build_regimm_teqi  , Cpu::build_inst_reserved, Cpu::build_regimm_tnei  , Cpu::build_inst_reserved,
    /* 10_ */  Cpu::build_inst_unknown , Cpu::build_regimm_bgezal, Cpu::build_inst_unknown , Cpu::build_regimm_bgezall, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved,
    /* 11_ */  Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved , Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved, Cpu::build_inst_reserved,
            ],


            jit_block_start_pc: 0,
            jit_jump: false,
            jit_jump_no_delay: false,
            jit_conditional_branch: None,
            jit_current_assembler_offset: 0,
            jit_executing: false,
            jit_other_exception: false,
            jit_run_limit: 0,
            jit_idle_loop_detected: false,
            cp0_count_tracker: 0,
            cp0_compare_distance: 0,
            cp0_random_tracker: 0,
        };
        
        let mut assembler = dynasmrt::x64::Assembler::new().unwrap();
        let trampoline_entry_offset = assembler.offset(); // is this always 0 ?

        // build the trampoline code
        // extern "sysv64" trampoline(cpu: *mut Cpu, compiled_instruction_address: u64, run_limit: u32) -> cycles_ran: u32
        #[allow(unused_assignments)]
        #[allow(unused_mut)]
        let mut stack_adjust = 0;

        cfg_if! [
            if #[cfg(feature="dev")] {
                // letsbreak!(assembler);
                letsgo!(assembler
                    ;   push rbp
                    ;   mov rbp, rsp
                );
                stack_adjust = 8;
            }
        ];

        letsgo!(assembler
            ;   push r_gpr
            ;   push r_cp0gpr
            ;   push r_cpu
            ;   push r_cond_64
            // cpu: *mut Cpu comes in rdi
            ;   mov r_cpu, v_arg0
            // shortcuts to gpr and cp0gpr
            ;   lea r_gpr, [r_cpu + offset_of!(Cpu, gpr) as i32]
            ;   lea r_cp0gpr, [r_cpu + offset_of!(Cpu, cp0gpr) as i32]
            // the stack is currently offset by 0x20/0x28 due to rip being pushed plus 4/5 register pushes
            // we want the stack to be 0x10 aligned /after/ the `call` because future calls will call sysv64 ABI functions
            // and this call below doesn't require the alignment. we can then align it using the call itself.
            ;   sub rsp, BYTE 0x20 + stack_adjust // setup the stack so function calls are aligned.. 0x20 bytes is enough room for the s_* variables
            // run_limit in v_arg2
            // NOTE: 8 is subtracted from the stack pointers because we're not inside the `call v_arg1` (JIT code) below
            ;   mov DWORD [rsp+(s_cycle_count-8)], v_arg2_32
            // v_arg1 contains the address of the target JIT code
            ;   call v_arg1
            // cycle count to return value
            ;   mov eax, DWORD [rsp+(s_cycle_count-8)]
            // restore the stack in reverse order
            ;   add rsp, BYTE 0x20 + stack_adjust // must match the sub rsp above
            ;   pop r_cond_64
            ;   pop r_cpu
            ;   pop r_cp0gpr
            ;   pop r_gpr
        );

        cfg_if! [
            if #[cfg(feature="dev")] {
                letsgo!(assembler
                    ;   mov rsp, rbp
                    ;   pop rbp
                );
            }
        ];

        letsgo!(assembler
            ;   ret
        );

        cpu.jit_trampoline = Some(assembler.finalize().unwrap());
        cpu.jit_trampoline_entry_point = trampoline_entry_offset;

        let _ = cpu.reset(false);
        cpu
    }

    pub fn reset(&mut self, is_soft: bool) -> Result<(), InstructionFault> {
        // set Cop0_EPC to the current PC
        self.cp0gpr[Cop0_EPC] = self.next_instruction_pc;
        self.cp0gpr[_COP0_ERROREPC] = self.next_instruction_pc;

        // all resets are vectored to the same address
        self.pc = 0xFFFF_FFFF_BFC0_0000;

        // COP0
        // TODO see section 6.4.4 Cold Reset for a complete list of initial register values
        self.cp0gpr[Cop0_Wired] = 0;
        self.cp0gpr[Cop0_Random] = 0x1F; // set to the upper bound (31) on reset
        self.cp0gpr_random_delay = 0;    
        self.cp0gpr[Cop0_PRId] = 0x0B22;
        self.cp0gpr[Cop0_Config] = 0x7006E463; // EC=1:15, EP=0, BE=1 (big endian), CU=0 (RFU?), K0=3 (kseg0 cache enabled)
        self.cp0gpr[Cop0_Status] = (1 << 22) | (1 << 2) | if is_soft { 1 << 20 } else { 0 }; // BEV=1, ERL=1, SR=0 (SR will be 1 on soft reset), IE=0

        // fetch next_instruction before starting the loop
        self.prefetch()?;
        self.next_is_delay_slot = false;
        self.next_cop_index = 0;

        // setup current_instruction_pc to be correct
        self.current_instruction_pc = self.next_instruction_pc;
        self.is_delay_slot = false;

        // reset some JIT flags
        self.jit_other_exception = false;

        // invalidate the exception handlers
        // TODO should actually invalidate everything?
        self.invalidate_block_cache(0, 0x80000);
        //self.cached_block_map[0] = None;
        //self.cached_block_map[0x80 >> 2] = None;
        //self.cached_block_map[0x180 >> 2] = None;
        //self.gpr[0] = 0;

        Ok(())
    }

    pub fn num_steps(&self) -> u64 {
        self.num_steps
    }

    pub fn current_instruction_pc(&self) -> u64 {
        self.current_instruction_pc
    }

    pub fn next_instruction(&mut self) -> Option<u32> {
        if self.next_instruction.is_none() {
            self.next_instruction = match self.translate_address(self.next_instruction_pc, false, false) {
                Ok(Some(address)) => Some(self.read_u32_phys(address).unwrap()),
                Ok(None) => None,
                Err(_) => None,
            };
        }
        self.next_instruction
    }

    pub fn next_instruction_pc(&self) -> u64 {
        self.next_instruction_pc
    }

    pub fn next_is_delay_slot(&self) -> bool {
        self.next_is_delay_slot
    }

    pub fn regs(&self) -> &[u64] {
        &self.gpr
    }

    pub fn abi_name(i: usize) -> &'static str {
        const NAMES: [&str; 32] = [
            "r0", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
            "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "t8", "t9", "k0", "k1", "gp", "sp", "s8", "ra"
        ];

        NAMES[i]
    }

    pub fn register_name(i: usize) -> &'static str {
        const NAMES: [&str; 32] = [
            "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
            "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23", "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31"
        ];

        NAMES[i]
    }

    // clear all references to compiled blocks for a given address range
    // just invalidate the cached_block_map without clearing self.cached_blocks
    // 1) if something else uses this address, then the old cached_blocks address will be freed.
    // 2) it doesn't really matter if that memory hangs around anyway.
    pub fn invalidate_block_cache(&mut self, physical_address: u64, length: usize) {
        if let Some(cached_block_map_index) = self.get_cached_block_map_index(physical_address) {
            //println!("invalidating ${:08X}..${:08X}", physical_address, physical_address + (length as u64));
            //self.cached_block_map[cached_block_map_index..][..aligned_length >> 2].fill(None);
            let aligned_length = (length + 7) & !7;
            for i in 0..(aligned_length >> 2) {
                match self.cached_block_map[cached_block_map_index + i] {
                    // invalidate all blocks contained in this region
                    Some(CachedBlockReference::CachedBlock(_)) => {
                        self.cached_block_map[cached_block_map_index + i] = None;
                    }
                    // any address that has an address reference to a parent block means the whole block is invaldated
                    Some(CachedBlockReference::AddressReference(index_reference, _)) => {
                        self.cached_block_map[index_reference] = None;
                    }
                    None => {},
                }
            }
        }
    }

    #[allow(dead_code)]
    fn interrupts_enabled(&self, interrupt_number: u64) -> bool {
        // interrupts are enabled when IE is set and EXL (exception level) and ERL (error level) are 0
        // (and also when the IM bit is set for the specified interrupt)
        let check = (0x100 << interrupt_number) | 0x07;
        (self.cp0gpr[Cop0_Status] & check) == (check & !0x06)
    }

    #[inline(always)]
    fn read_u8(&mut self, virtual_address: usize) -> Result<u8, InstructionFault> {
        let value = if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u8_phys(address)
        } else {
            // invalid TLB translation, no physical address present to read and no exception
            Ok(0)
        };

        //print!("DBG RD8 A:${:08X} V:${:08X} ", virtual_address as u32, value.as_ref().unwrap());
        //for i in 0..32 {
        //    print!("R{}:${:08X} ", i, self.gpr[i]);
        //}
        //println!();

        value
    }

    extern "sysv64" fn read_u8_bridge(cpu: *mut Cpu, virtual_address: u64) -> u32 {
        //trace!(target: "JIT", "read_u8_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        //if virtual_address == 2 { asm!("int3"); }
        match cpu.read_u8(virtual_address as usize) {
            Ok(value) => (value as u8) as u32,

            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_TLBL); // only valid exceptions here
                cpu.jit_other_exception = true;
                exception_code as u32
            }

            Err(e) => {
                // TODO handle errors better
                error!("error in read_u8_bridge: {:?}", e);
                unsafe { asm!("int3"); }
                0
            }
        }
    }

    #[inline(always)]
    fn read_u16(&mut self, virtual_address: usize) -> Result<u16, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u16_phys(address)
        } else {
            // invalid TLB translation, no physical address present to read and no exception
            Ok(0)
        }
    }

    extern "sysv64" fn read_u16_bridge(cpu: *mut Cpu, virtual_address: u64) -> u16 {
        //trace!(target: "JIT", "read_u16_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.read_u16(virtual_address as usize) {
            Ok(value) => value,
            Err(e) => {
                // TODO handle errors better
                panic!("error in read_u16_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn read_u32(&mut self, virtual_address: usize) -> Result<u32, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u32_phys(address)
        } else {
            // invalid TLB translation, no physical address present to read and no exception
            Ok(0)
        }
    }

    extern "sysv64" fn read_u32_bridge(cpu: *mut Cpu, virtual_address: u64) -> u32 {
        //trace!(target: "JIT", "read_u32_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.read_u32(virtual_address as usize) {
            Ok(value) => value,

            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_AdEL || exception_code == ExceptionCode_TLBL); // only valid exceptions here
                cpu.jit_other_exception = true;
                exception_code as u32
            },

            Err(e) => {
                // TODO handle errors better
                panic!("error in read_u32_bridge: {:?}", e);
            }
        }
    }

    // The VR4300 has to do two reads to get a doubleword
    #[inline(always)]
    fn read_u64(&mut self, virtual_address: usize) -> Result<u64, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            self.read_u64_phys(address)
        } else {
            Ok(0)
        }
    }

    extern "sysv64" fn read_u64_bridge(cpu: *mut Cpu, virtual_address: u64) -> u64 {
        //trace!(target: "JIT", "read_u64_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.read_u64(virtual_address as usize) {
            Ok(value) => value,
            Err(e) => {
                // TODO handle errors better
                panic!("error in read_u64_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn read_u8_phys(&mut self, address: Address) -> Result<u8, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u8(address.physical_address as usize)?)
    }

    #[inline(always)]
    fn read_u16_phys(&mut self, address: Address) -> Result<u16, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u16(address.physical_address as usize)?)
    }

    #[inline(always)]
    fn read_u32_phys(&mut self, address: Address) -> Result<u32, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u32(address.physical_address as usize)?)
    }
    
    #[inline(always)]
    fn read_u32_phys_direct(&mut self, physical_address: u64) -> Result<u32, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u32(physical_address as usize)?)
    }

    extern "sysv64" fn read_u32_phys_bridge(cpu: *mut Cpu, physical_address: u64) -> u32 {
        //trace!(target: "JIT", "read_u32_phys_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.read_u32_phys_direct(physical_address) {
            Ok(value) => value,

            Err(e) => {
                panic!("error in read_u32_phys_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn read_u64_phys(&mut self, address: Address) -> Result<u64, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u64(address.physical_address as usize)?)
    }

    #[inline(always)]
    fn read_u64_phys_direct(&mut self, physical_address: u64) -> Result<u64, InstructionFault> {
        Ok(self.bus.borrow_mut().read_u64(physical_address as usize)?)
    }

    extern "sysv64" fn read_u64_phys_bridge(cpu: *mut Cpu, physical_address: u64) -> u64 {
        //trace!(target: "JIT", "read_u64_phys_bridge from ${:08X}", virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.read_u64_phys_direct(physical_address) {
            Ok(value) => value,

            Err(e) => {
                panic!("error in read_u64_phys_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn write_u8(&mut self, value: u32, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        //print!("DBG WR8 A:${:08X} V:${:08X} ", virtual_address as u32, value);
        //for i in 0..32 {
        //    print!("R{}:${:08X} ", i, self.gpr[i]);
        //}
        //println!();

        if let Some(address) = self.translate_address(virtual_address as u64, true, true)? {
            if (address.physical_address & 0xFF800000) == 0 {
                let offs = CACHED_BLOCK_MAP_RDRAM_OFFSET + (address.physical_address >> 2) as usize;
                if self.cached_block_map[offs].is_some() {
                    self.invalidate_block_cache(address.physical_address, 1);
                }
            }
            self.write_u8_phys(value, address)
        } else {
            // invalid TLB translation, no physical address present to read
            Ok(WriteReturnSignal::None)
        }
    }

    extern "sysv64" fn write_u8_bridge(cpu: *mut Cpu, value: u32, virtual_address: u64) -> i32 {
        //trace!(target: "JIT", "write_u8_bridge(value=${:08X}, address=${:08X})", value, virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.write_u8(value, virtual_address as usize) {
            Ok(_) => 0, // TODO pass WriteReturnSignal along
            Err(e) => {
                // TODO handle errors better
                panic!("error in write_u8_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn write_u16(&mut self, value: u32, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, true)? {
            if (address.physical_address & 0xFF800000) == 0 {
                let offs = CACHED_BLOCK_MAP_RDRAM_OFFSET + (address.physical_address >> 2) as usize;
                if self.cached_block_map[offs].is_some() {
                    self.invalidate_block_cache(address.physical_address, 2);
                }
            }
            self.write_u16_phys(value, address)
        } else {
            // invalid TLB translation, no physical address present to read
            Ok(WriteReturnSignal::None)
        }
    }

    extern "sysv64" fn write_u16_bridge(cpu: *mut Cpu, value: u32, virtual_address: u64) -> i32 {
        //trace!(target: "JIT", "write_u16_bridge(value=${:08X}, address=${:08X})", value, virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.write_u16(value, virtual_address as usize) {
            Ok(_) => 0, // TODO pass WriteReturnSignal along
            Err(e) => {
                // TODO handle errors better
                panic!("error in write_u16_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn write_u32(&mut self, value: u32, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        //if (virtual_address as u32) == 0xA4001F9C && value == 0x0E468AD3 {
        //    println!("DBG current_instruction_pc=${:08X}", self.current_instruction_pc);
        //}
        //print!("DBG WR32 A:${:08X} V:${:08X} ", virtual_address as u32, value);
        //for i in 0..32 {
        //    print!("R{}:${:08X} ", i, self.gpr[i]);
        //}
        //println!();

        if let Some(address) = self.translate_address(virtual_address as u64, true, true)? {
            // invalidate RDRAM blocks
            if (address.physical_address & 0xFF800000) == 0 {
                let offs = CACHED_BLOCK_MAP_RDRAM_OFFSET + (address.physical_address >> 2) as usize;
                if self.cached_block_map[offs].is_some() {
                    self.invalidate_block_cache(address.physical_address, 4);
                }
            }
            self.write_u32_phys(value, address)
        } else {
            // invalid TLB translation, no physical address present to read
            Ok(WriteReturnSignal::None)
        }
    }

    extern "sysv64" fn write_u32_bridge(cpu: *mut Cpu, value: u32, virtual_address: u64) -> u32 {
        //trace!(target: "JIT", "write_u32_bridge(value=${:08X}, address=${:08X})", value, virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.write_u32(value, virtual_address as usize) {
            Ok(_) => 0, // TODO pass WriteReturnSignal along

            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_AdES || exception_code == ExceptionCode_TLBS || exception_code == ExceptionCode_Mod);
                cpu.jit_other_exception = true;
                exception_code as u32
            },

            Err(e) => {
                // TODO handle errors better
                error!("error in write_u32_bridge: {:?}", e);
                unsafe { asm!("int3"); }
                0
            }
        }
    }

    #[inline(always)]
    fn write_u64(&mut self, value: u64, virtual_address: usize) -> Result<WriteReturnSignal, InstructionFault> {
        if let Some(address) = self.translate_address(virtual_address as u64, true, false)? {
            if (address.physical_address & 0xFF800000) == 0 {
                let offs = CACHED_BLOCK_MAP_RDRAM_OFFSET + (address.physical_address >> 2) as usize;
                if self.cached_block_map[offs].is_some() || self.cached_block_map[offs+1].is_some() {
                    self.invalidate_block_cache(address.physical_address, 8);
                }
            }
            self.write_u64_phys(value, address)
        } else {
            Ok(WriteReturnSignal::None)
        }
    }

    extern "sysv64" fn write_u64_bridge(cpu: *mut Cpu, value: u64, virtual_address: u64) -> i32 {
        //trace!(target: "JIT", "write_u64_bridge(value=${:08X}, address=${:08X})", value, virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.write_u64(value, virtual_address as usize) {
            Ok(_) => 0, // TODO pass WriteReturnSignal along
            Err(e) => {
                // TODO handle errors better
                panic!("error in write_u64_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn write_u8_phys(&mut self, value: u32, address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u8(value, address.physical_address as usize)?)
    }

    #[inline(always)]
    fn write_u16_phys(&mut self, value: u32, address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u16(value, address.physical_address as usize)?)
    }

    #[inline(always)]
    fn write_u32_phys(&mut self, value: u32, address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u32(value, address.physical_address as usize)?)
    }

    #[inline(always)]
    fn write_u32_phys_direct(&mut self, value: u32, physical_address: u64) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u32(value, physical_address as usize)?)
    }

    extern "sysv64" fn write_u32_phys_bridge(cpu: *mut Cpu, value: u32, physical_address: u64) -> u32 {
        //trace!(target: "JIT", "write_u32_phys_bridge value=${:08X} to ${:08X}", value, virtual_address as u32);

        let cpu = unsafe { &mut *cpu };
        match cpu.write_u32_phys_direct(value, physical_address) {
            Ok(_) => 0,

            Err(e) => {
                panic!("error in write_u32_phys_bridge: {:?}", e);
            }
        }
    }

    #[inline(always)]
    fn write_u64_phys(&mut self, value: u64, address: Address) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u64(value, address.physical_address as usize)?)
    }

    #[inline(always)]
    fn write_u64_phys_direct(&mut self, value: u64, physical_address: u64) -> Result<WriteReturnSignal, InstructionFault> {
        Ok(self.bus.borrow_mut().write_u64(value, physical_address as usize)?)
    }

    extern "sysv64" fn write_u64_phys_bridge(cpu: *mut Cpu, value: u64, physical_address: u64) -> u32 {
        //trace!(target: "JIT", "write_u64_phys_bridge value=${:08X} to ${:08X}", value, virtual_address as u32);
        let cpu = unsafe { &mut *cpu };
        match cpu.write_u64_phys_direct(value, physical_address) {
            Ok(_) => 0,

            Err(e) => {
                panic!("error in write_u64_phys_bridge: {:?}", e);
            }
        }
    }

    // prefetch the next instruction
    #[inline(always)]
    fn prefetch(&mut self) -> Result<(), InstructionFault> {
        self.next_instruction = Some(self.read_u32(self.pc as usize)?); 
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        self.comms.increment_prefetch_counter();

        Ok(())
    }

    extern "sysv64" fn prefetch_bridge(cpu: *mut Self) {
        let cpu = unsafe { &mut *cpu };
        cpu.next_instruction = None;
        cpu.next_instruction_pc = cpu.pc;
        cpu.pc += 4;
    }

    fn exception(&mut self, exception_code: u64, use_special: bool) -> Result<(), InstructionFault> {
        // clear BD, exception code and bits that should be 0
        self.cp0gpr[Cop0_Cause] &= !0xC0FF_00FF;

        // set the exception code
        self.cp0gpr[Cop0_Cause] |= exception_code << 2;

        // set the EXL bit to prevent another exception
        let previous_exl = self.cp0gpr[Cop0_Status] & 0x02;
        self.cp0gpr[Cop0_Status] |= 0x02;

        // set the Exception Program Counter to the currently executing instruction
        self.cp0gpr[Cop0_EPC] = (if self.is_delay_slot {
            // also set the BD flag indicating the exception was in a delay slot
            self.cp0gpr[Cop0_Cause] |= 0x8000_0000;
            self.current_instruction_pc - 4
        } else {
            self.current_instruction_pc
        } as i32) as u64;

        // set PC and discard the delay slot. PC is set to the vector base determined by the BEV bit in Cop0_Status.
        // TLB and XTLB miss are vectored to offsets 0 and 0x80, respectively
        // all others go to offset 0x180
        self.pc = if (self.cp0gpr[Cop0_Status] & 0x200000) == 0 {  // check BEV==0
            0xFFFF_FFFF_8000_0000
        } else { 
            0xFFFF_FFFF_BFC0_0200
        } + if use_special && previous_exl == 0 { // check EXL==0 and tlb miss
            if self.kernel_64bit_addressing { 0x080 } else { 0x000 }
        } else {
            0x180
        };

        // we need to throw away next_instruction, so prefetch the new instruction now
        self.next_is_delay_slot = false;

        // when executing in JIT we can use prefetch_bridge() so we don't actually end up reading the bus,
        // since the code could already be compiled (and likely is since its the exception handler)
        if self.jit_executing {
            Cpu::prefetch_bridge(self);
        } else {
            self.prefetch()?;
        }

        // break out of all processing
        Err(InstructionFault::OtherException(exception_code))
    }

    fn address_exception(&mut self, virtual_address: u64, is_write: bool) -> Result<(), InstructionFault> {
        //println!("CPU: address exception!");

        // set EntryHi to the vpn that caused the fault, preserving ASID and R
        // the datasheet says the VPN value of EntryHi is undefined here, but the systemtests require it
        self.cp0gpr[Cop0_EntryHi] = (virtual_address & 0xFF_FFFF_E000) | (self.cp0gpr[Cop0_EntryHi] & 0xFF) | (virtual_address & 0xC000_0000_0000_0000);

        self.cp0gpr[Cop0_BadVAddr] = virtual_address;
        let bad_vpn2 = virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        let exception_code = if is_write { ExceptionCode_AdES } else { ExceptionCode_AdEL };
        self.exception(exception_code, false)
    }

    extern "sysv64" fn address_exception_bridge(cpu: *mut Cpu, virtual_address: u64, is_write: bool) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.address_exception(virtual_address, is_write) {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("address_exception_bridge error = {:?}", err);
            }
        }
    }

    fn tlb_miss(&mut self, address: Address, is_write: bool) -> Result<(), InstructionFault> {
        //info!(target: "CPU", "TLB miss exception virtual_address=${:016X} is_write={} 32-bits={}!", address.virtual_address, is_write, !self.kernel_64bit_addressing);

        // set EntryHi to the vpn that caused the fault, preserving ASID and R
        self.cp0gpr[Cop0_EntryHi] = (address.virtual_address & 0xFF_FFFF_E000) | (self.cp0gpr[Cop0_EntryHi] & 0xFF) | (address.virtual_address & 0xC000_0000_0000_0000);

        self.cp0gpr[Cop0_BadVAddr] = address.virtual_address;
        let bad_vpn2 = address.virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((address.virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        // TLB miss that doesn't hit an EntryHi gets vectored to a different exception handler
        // than a TLB miss that hits an EntryHi but has an invalid entry
        let use_special = if let Some(_) = address.tlb_index { false } else { true };

        let exception_code = if is_write { ExceptionCode_TLBS } else { ExceptionCode_TLBL };
        self.exception(exception_code, use_special)
    }

    fn tlb_mod(&mut self, virtual_address: u64) -> Result<(), InstructionFault> {
        //info!(target: "CPU", "TLB mod exception virtual_address=${:016X}", virtual_address);

        // set EntryHi to the page that caused the fault, preserving ASID
        self.cp0gpr[Cop0_EntryHi] = (virtual_address & 0xFF_FFFF_E000) | (self.cp0gpr[Cop0_EntryHi] & 0xFF);

        self.cp0gpr[Cop0_BadVAddr] = virtual_address;
        let bad_vpn2 = virtual_address >> 13;
        self.cp0gpr[Cop0_Context] = (self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) | ((bad_vpn2 & 0x7FFFF) << 4);

        // TODO the XContext needs the page table
        self.cp0gpr[Cop0_XContext] = ((self.cp0gpr[Cop0_Context] & 0xFFFF_FFFF_FF80_0000) << 10) 
                                     | ((virtual_address >> 31) & 0x1_8000_0000) 
                                     | ((bad_vpn2 & 0x7FF_FFFF) << 4);

        self.exception(ExceptionCode_Mod, false)
    }


    fn breakpoint_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Bp, false)
    }

    extern "sysv64" fn breakpoint_exception_bridge(cpu: *mut Cpu) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.breakpoint_exception() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("breakpoint_exception_bridge error = {:?}", err);
            }
        }
    }

    fn overflow_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Ov, false)
    }

    extern "sysv64" fn overflow_exception_bridge(cpu: *mut Cpu) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.overflow_exception() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("overflow_exception_bridge error = {:?}", err);
            }
        }
    }

    fn reserved_instruction_exception(&mut self, coprocessor_number: u64) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | (coprocessor_number << 28);
        self.exception(ExceptionCode_RI, false)
    }

    extern "sysv64" fn reserved_instruction_exception_bridge(cpu: *mut Cpu, coprocessor_number: u64) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.reserved_instruction_exception(coprocessor_number) {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("reserved_instruction_exception_bridge error = {:?}", err);
            }
        }
    }

    fn syscall_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Sys, false)
    }

    extern "sysv64" fn syscall_exception_bridge(cpu: *mut Cpu) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.syscall_exception() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("syscall_exception_bridge error = {:?}", err);
            }
        }
    }

    fn trap_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] &= !0x3000_0000; // clear coprocessor number
        self.exception(ExceptionCode_Tr, false)
    }

    extern "sysv64" fn trap_exception_bridge(cpu: *mut Cpu) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.trap_exception() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("trap_exception_bridge error = {:?}", err);
            }
        }
    }

    fn floating_point_exception(&mut self) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | ((self.next_cop_index as u64) << 28); // set coprocessor number
        self.exception(ExceptionCode_FPE, false)
    }

    fn coprocessor_unusable_exception(&mut self, coprocessor_number: u64) -> Result<(), InstructionFault> {
        //info!("CPU: coprocessor unusable exception (Cop0_Status = ${:08X})!", self.cp0gpr[Cop0_Status]);
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | (coprocessor_number << 28);
        self.exception(ExceptionCode_CpU, false)
    }

    extern "sysv64" fn coprocessor_unusable_exception_bridge(cpu: *mut Cpu, coprocessor_number: u64) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.coprocessor_unusable_exception(coprocessor_number) {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("coprocessor_unusable_exception_bridge error = {:?}", err);
            }
        }
    }
    
    extern "sysv64" fn cop1_unimplemented_instruction_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1) -> i64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cop1.unimplemented_instruction() {
            Ok(_) => 0,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as i64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle errors
                todo!("cop1_unimplemented_instruction_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_cfc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rd: u64) -> u32 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.cfc(rd as usize) {
            Ok(v) => v,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as u32
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("cfc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_ctc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rt_value: u64, rd: u64) -> i64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.ctc(rt_value, rd as usize) {
            Ok(_) => 0,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as i64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("ctc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_mfc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rd: u64) -> u32 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.mfc(rd as usize) {
            Ok(v) => v,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as u32
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("mfc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_mtc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rt_value: u32, rd: u64) -> i64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.mtc(rt_value, rd as usize) {
            Ok(_) => 0,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as i64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("mtc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_dmfc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rd: u64) -> u64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.dmfc(rd as usize) {
            Ok(v) => v,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as u64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("dmfc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_dmtc_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, rt_value: u64, rd: u64) -> i64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.dmtc(rt_value, rd as usize) {
            Ok(_) => 0,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as i64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("dmtc_bridge error = {:?}", err);
            }
        }
    }

    extern "sysv64" fn cop1_special_bridge(cpu: *mut Cpu, cop1: *mut cop1::Cop1, inst: u32) -> i64 {
        let cpu = unsafe { &mut *cpu };
        let cop1 = unsafe { &mut *cop1 };

        match cop1.special(inst) {
            Ok(_) => 0,

            Err(InstructionFault::FloatingPointException) => {
                match cpu.floating_point_exception() {
                    Err(InstructionFault::OtherException(exception_code)) => {
                        cpu.jit_other_exception = true;
                        exception_code as i64
                    },

                    _ => panic!("invalid"),
                }
            },

            Err(err) => {
                // TODO handle rrors
                todo!("special_bridge error = {:?}", err);
            }
        }
    }

    pub fn interrupt(&mut self, interrupt_signal: u64) -> Result<(), InstructionFault> {
        self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0xFF0) | (interrupt_signal << 8);

        // check if the interrupts are enabled
        if ((interrupt_signal as u64) & (self.cp0gpr[Cop0_Status] >> 8)) != 0 {
            if (self.cp0gpr[Cop0_Status] & 0x07) == 0x01 { // IE must be set, and EXL and ERL both clear
                // interrupts are executed outside of step(), which means self.current_instruction_pc isn't correct
                // and self.exception uses self.current_instruction_pc, so update it
                self.exception(ExceptionCode_Int, false)?;
            }
        }

        Ok(())
    }

    // int_pins is a 5 bit mask representing bits IP2-IP6 of the Cause register, with 1 being "request pending"
    #[inline(always)]
    pub fn rcp_interrupt(&mut self) -> Result<(), InstructionFault> {
        self.interrupt(InterruptCode_RPC)
    }

    #[inline(always)]
    fn timer_interrupt(&mut self) -> Result<(), InstructionFault> {
        self.interrupt(InterruptCode_Timer)
    }

    #[inline(always)]
    fn branch(&mut self, condition: bool) {
        if condition {
            // target is the sum of the address of the delay slot instruction
            // plus the sign extended and left-shifted immediate offset
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u64) << 2);
        }

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
    }

    // branch likely instructions discards the delay slot when the branch is not taken
    #[inline(always)]
    fn branch_likely(&mut self, condition: bool) -> Result<(), InstructionFault> {
        if condition {
            self.pc = (self.pc - 4).wrapping_add((self.inst.signed_imm as u64) << 2);

            // note that the next instruction to execute is a delay slot instruction
            self.next_is_delay_slot = true;
        } else {
            // we need to throw away next_instruction when branch is not taken
            self.prefetch()?;
        }
        Ok(())
    }

    #[inline(always)]
    fn set_tlb_entry(&mut self, index: u64) {
        let g = (self.cp0gpr[Cop0_EntryLo0] & self.cp0gpr[Cop0_EntryLo1]) & 0x1; // Global flag

        // the page mask needs to have the bits changed. the datasheet says they're undefined
        // when not certain values, but to mimic the N64 there is certain expected behavior.
        // 1. if the upper bit of the pair is not set then the lower bit of the pair is cleared
        let mut page_mask = (self.cp0gpr[Cop0_PageMask] >> 13) & 0xAAA;
        page_mask |= page_mask >> 1;

        self.tlb[(index & 0x1F) as usize] = TlbEntry {
            page_mask: page_mask << 13,
            entry_hi : self.cp0gpr[Cop0_EntryHi] | (g << 12),
            entry_lo1: self.cp0gpr[Cop0_EntryLo1] & 0x03FF_FFFE, // mask out G flag
            entry_lo0: self.cp0gpr[Cop0_EntryLo0] & 0x03FF_FFFE, // mask out G flag
        };
    }

    fn classify_address(&mut self, virtual_address: u64, generate_exceptions: bool, is_write: bool) -> Result<Option<Address>, InstructionFault> {
        let mut address = Address {
            virtual_address: virtual_address,
            physical_address: 0,
            cached: true,
            mapped: false,
            space: MemorySpace::User,
            tlb_index: None,
        };

        if self.kernel_64bit_addressing {
            let space_bits = virtual_address >> 62;
            address.space = match space_bits {
                0b00 => MemorySpace::User,
                0b01 => MemorySpace::Supervisor,
                0b10 => MemorySpace::XKPhys,
                0b11 => MemorySpace::Kernel,
                _ => panic!("not possible"),
            };

            match space_bits {
                0b00 => { // user
                    address.space = MemorySpace::User;
                    if virtual_address < 0x0000_0100_0000_0000 { // xkuseg, user segment, TLB mapped, cached
                        address.mapped = true;
                    } else {
                        self.address_exception(virtual_address, is_write)?;
                        return Ok(None);
                    }
                },

                0b01 => { // supervisor
                    address.space = MemorySpace::Supervisor;
                    if virtual_address < 0x4000_0100_0000_0000 { // xksseg, supervisor segment, TLB mapped, cached
                        address.mapped = true;
                    } else {
                        self.address_exception(virtual_address, is_write)?;
                        return Ok(None);
                    }
                },

                0b10 => { // xkphys
                    address.space = MemorySpace::XKPhys;
                    // addresses with bits 53:32 including 1 cause an address error
                    if ((virtual_address >> 32) & 0x3FFFFF) != 0 {
                        self.address_exception(virtual_address, is_write)?;
                        return Ok(None);
                    } else {
                        // all addresses are unmapped in this segment
                        // the physical_address is the lower 32 bits of the virtual_address
                        address.physical_address = virtual_address & 0xFFFF_FFFF;

                        // bits 61:59 indicate the use of cache. interestingly enough, only one
                        // region is uncached (0x9000_0000_0000_0000-0x9000_0000_FFFF_FFFF)
                        address.cached = ((virtual_address >> 59) & 0x07) != 2;
                    }
                },

                0b11 => { // kernel
                    address.space = MemorySpace::Kernel;
                    if virtual_address < 0xC000_00FF_8000_0000 { // xkseg, TLB mapped, cached
                        address.mapped = true;
                    } else if virtual_address < 0xFFFF_FFFF_8000_0000 { // invalid
                        self.address_exception(virtual_address, is_write)?;
                        return Ok(None);
                    // the address range from FFFF_FFFF_8000_0000-FFFF_FFFF_FFFF_FFFF is the 32-bit compatibility range
                    } else if virtual_address < 0xFFFF_FFFF_A000_0000 { // ckseg0, segment 0, directly mapped, cached
                        address.physical_address = virtual_address & 0x1FFF_FFFF;
                    } else if virtual_address < 0xFFFF_FFFF_C000_0000 { // ckseg1, segment 0, directly mapped, uncached
                        address.cached = false;
                        address.physical_address = virtual_address & 0x1FFF_FFFF;
                    } else if virtual_address < 0xFFFF_FFFF_E000_0000 { // cksseg, supervisor segment (unused), TLB mapped
                        address.mapped = true;
                    } else { // ckseg3, kernel segment 3, TLB mapped, cached
                        address.mapped = true;
                    }
                },

                _ => panic!("Not possible"),
            };
        } else {
            if (virtual_address & 0x8000_0000) != 0 && ((virtual_address >> 32) as i32) != -1 && generate_exceptions {
                self.address_exception(virtual_address, is_write)?;
                return Ok(None);
            }

            let word_address = virtual_address as u32;
            if word_address < 0x8000_0000 { // kuseg, TLB mapped, cached
                address.space = MemorySpace::User;
                address.mapped = true; 
            } else if word_address < 0xA000_0000 { // kseg0, unmapped, cached
                address.space = MemorySpace::Kernel;
                address.physical_address = virtual_address & 0x1FFF_FFFF;
            } else if word_address < 0xC000_0000 { // kseg1, unmapped, uncached
                address.space = MemorySpace::Kernel;
                address.cached = false;
                address.physical_address = virtual_address & 0x1FFF_FFFF;
            } else if word_address < 0xE000_0000 { // ksseg, mapped, cached
                address.space = MemorySpace::Supervisor;
                address.mapped = true;
            } else { // kseg3, mapped, cached
                address.space = MemorySpace::Kernel;
                address.mapped = true;
            }
        }

        return Ok(Some(address));
    }

    pub fn translate_address(&mut self, virtual_address: u64, generate_exceptions: bool, is_write: bool) -> Result<Option<Address>, InstructionFault> {
        // create the Address struct by first classifying the virtual address
        let mut address = match self.classify_address(virtual_address, generate_exceptions, is_write)? {
            Some(v) => v,
            None => {
                return Ok(None);
            }
        };

        // TODO check address.space and make sure current execution mode allows access to that space
        // i.e., if we're running in supervisor mode and try to access kernel address, we need to
        // generate an address_exception(). But as far as I can tell games are always running in
        // kernel mode.  We may need to support it in the future to support demos or other demo
        // programs, though.

        // if no further address translation is needed, return the Address
        if !address.mapped { return Ok(Some(address)); }

        // since the address is mapped there must be a TLB entry for it to be a valid
        // virtual_address, otherwise TLB exceptions occur

        //info!(target: "CPU", "TLB: translating virtual address ${:016X}!", virtual_address);

        // virtual addresses are 40 bits wide
        let virtual_address = virtual_address & 0xFF_FFFF_FFFF;

        // get the current ASID from EntryHi
        let asid = self.cp0gpr[Cop0_EntryHi] & 0xFF;

        for tlb_index in 0..32 {
            let tlb = &self.tlb[tlb_index];

            // if the G bit is not set, the asid must match
            if (tlb.entry_hi & 0x1000) == 0 && (tlb.entry_hi & 0xFF) != asid { continue; }

            // if the region (R) doesn't match, this entry isn't valid
            if ((tlb.entry_hi ^ address.virtual_address) >> 62) != 0 { continue; }

            // PageMask is (either 0, 3, 5, 7, F, 3F) at bit position 13
            // So offset_mask will be a mask that can mask out the offset into the page from
            // the virtual address
            let offset_mask = (tlb.page_mask >> 1) | 0xFFF; // page mask is a set of 1s at bit 13
                                                            // and if it isn't, it's undefined
                                                            // behavior

            // If the virtual address is referencing one of the two pages, it's a hit
            // notice that we keep PageMask in bit 13 rather shift it down
            let vpn_mask = !(tlb.page_mask | 0x1FFF) & 0xFF_FFFF_FFFF;

            // VPN2 is the virtual page number (divided by 2) this TLB entry covers
            let vpn2 = tlb.entry_hi & vpn_mask;

            //info!(target: "CPU", "TLB: virtual_address & vpn_mask = ${:016X}", virtual_address & vpn_mask);
            if (virtual_address & vpn_mask) != vpn2 { continue; }

            address.tlb_index = Some(tlb_index);

            // the odd bit is the 1 bit above the offset bits
            let odd_bit = virtual_address & (offset_mask + 1);

            // get the correct EntryLo field based on the odd page bit
            let entry_lo = if odd_bit != 0 { tlb.entry_lo1 } else { tlb.entry_lo0 };

            // if V (valid) bit is not set, a TLB miss occurs
            if (entry_lo & 0x02) == 0 { break; }

            // if D (dirty) is not set and this is a write, a Mod exception occurs
            if generate_exceptions && (entry_lo & 0x04) == 0 && is_write {
                self.tlb_mod(virtual_address)?;
                return Ok(None);
            }

            // get the page frame number of the corresponding EntryLo field
            let pfn = entry_lo & 0x03FF_FFC0;

            // the physical address is the pfn plus the offset (and the odd page bit)
            // pfn is at bit 6 in EntryLo, and needs to be at position 12 for the physical address
            address.physical_address = (pfn << 6) | ((virtual_address & !odd_bit) & (tlb.page_mask | 0x1FFF));

            //info!(target: "CPU", "TLB: translated address = ${:016X}", address.physical_address);

            // done
            return Ok(Some(address));
        }

        if generate_exceptions {
            self.tlb_miss(address, is_write)?;
        }

        Ok(None)
    }

    extern "sysv64" fn translate_address_bridge(cpu: *mut Cpu, virtual_address: u64, is_write: bool) -> u64 {
        let cpu = unsafe { &mut *cpu };

        // do the actual exception, setting self.pc, and Cop0 state, etc
        match cpu.translate_address(virtual_address, true, is_write) {
            Ok(Some(address)) => address.physical_address as u64,

            // address wasn't translatable?
            Ok(None) => todo!(),

            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as u64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("translate_address_bridge error = {:?}", err);
            }
        }
    }

    fn get_cached_block_map_index(&mut self, physical_address: u64) -> Option<usize> {
        // Using the physical_address, determine where the memory came from
        if physical_address < 0x0080_0000 { // RDRAM
            Some(CACHED_BLOCK_MAP_RDRAM_OFFSET + ((physical_address & 0x007F_FFFF) >> 2) as usize)
        } else if physical_address >= 0x0400_0000 && physical_address < 0x0400_2000 {
            Some(CACHED_BLOCK_MAP_RSP_MEM_OFFSET + ((physical_address & 0x1FFF) >> 2) as usize)
        } else if physical_address >= 0x1FC0_0000 && physical_address < 0x1FC0_07C0 {
            Some(CACHED_BLOCK_MAP_PIFROM_OFFSET + ((physical_address & 0x7FF) >> 2) as usize)
        } else {
            None
        }
    }

    fn lookup_cached_block(&mut self, physical_address: u64) -> Result<CachedBlockStatus, InstructionFault> {
        trace!(target: "JIT-RUN", "[looking up cached block @ physical_address=${:08X}]", physical_address as u32);
        //println!("[looking up cached block @ physical_address=${:08X}]", physical_address as u32);

        // determine where in the block map to look up the block
        let cached_block_map_index = match self.get_cached_block_map_index(physical_address) {
            Some(v) => v,
            None => {
                //println!("uncompilable: ${:08X}", physical_address);
                return Ok(CachedBlockStatus::Uncompilable);
            },
        };

        match self.cached_block_map[cached_block_map_index] {
            Some(CachedBlockReference::CachedBlock(block_id)) => {
                // block must exist (and block starts at the provided address)
                let cached_block = self.cached_blocks.get(&block_id).unwrap().clone();
                return Ok(CachedBlockStatus::Cached(cached_block));
            },

            Some(CachedBlockReference::AddressReference(index_reference, block_id_reference)) => {
                if let Some(CachedBlockReference::CachedBlock(referenced_block_id)) = self.cached_block_map[index_reference] {
                    if referenced_block_id == block_id_reference {
                        //debug!(target: "JIT-BUILD", "address ${:08X} has valid AddressReference but is being recompiled", physical_address);
                        // block id must exist because this is a CachedBlock
                        let cached_block = self.cached_blocks.get(&referenced_block_id).unwrap().clone();
                        return Ok(CachedBlockStatus::Cached(cached_block));
                    }
                }
                return Ok(CachedBlockStatus::NotCached);
            },

            // not found
            None => {
                return Ok(CachedBlockStatus::NotCached);
            }
        }
    }

    // run the JIT core, return the number of cycles actually ran
    // self.next_instruction_pc contains the address of code we start executing from
    pub fn run_for(&mut self, max_cycles: u64) -> Result<u64, InstructionFault> {
        // TODO: Catch PC address and TLB exceptions

        trace!(target: "JIT-RUN", "run_for started at ${:08X}, max_cycles={}", self.next_instruction_pc as u32, max_cycles);

        let steps_start = self.num_steps;
        // const MIN_STEPS_TO_BE_WORTH_IT: u64 = 8; // TODO does this "optimization" make sense?
        while self.num_steps < (steps_start + max_cycles) {
            // if the next step() instruction was a jump as well we should continue executing until it's not 
            // this happens if there's a jump within a jump, where we have exited the block and left next_is_delay_slot true
            if self.next_is_delay_slot {
                if self.next_instruction.is_none() {
                    self.next_instruction = Some(self.read_u32((self.next_instruction_pc) as usize).unwrap());
                    self.comms.increment_prefetch_counter();
                }
                self.step()?;
                continue;
            }

            // if the PC address is unaligned, just run Cpu::step() for the exception handler
            if (self.pc & 0x03) != 0 {
                // it doesn't matter that self.next_instruction is likely None right now, as step()
                // will see the address exception instead of trying to execute next_instruction
                self.pc -= 4; // the JIT code is one instruction ahead due to the call to prefetch().
                self.step()?;
                continue;
            }

            // Cop0_Count and Cop0_Random tracker start value
            self.cp0_count_tracker = self.num_steps & !1;
            self.cp0_random_tracker = self.num_steps;

            // determine how many cycles until Cop0_Compare would be hit, a value of 0 would mean
            // means there's actually 0x1_0000_0000 cycles until the exception triggers again, but
            // our block will never run that long
            self.cp0_compare_distance = (self.cp0gpr[Cop0_Compare] as u32).wrapping_sub(self.cp0gpr[Cop0_Count] as u32);
            if self.cp0_compare_distance == 0 { self.cp0_compare_distance = 0x7FFF_FFFF; }

            // for calcuating how many cycles have executed in the middle of a block
            let num_steps_before = self.num_steps;
            self.jit_run_limit = std::cmp::min(2 * self.cp0_compare_distance as u64, (steps_start + max_cycles) - self.num_steps);

            let physical_address = match self.translate_address(self.next_instruction_pc, true, false) {
                Ok(Some(address)) => address.physical_address,
                Ok(None) => panic!("can't translate next_instruction_pc=${:08X}, self.pc=${:08X}", self.next_instruction_pc, self.pc),
                Err(_) => {
                    // PC has changed due to an exception in translate_address. restart processing
                    break;
                }
            };

            match self.lookup_cached_block(physical_address)? {
                CachedBlockStatus::NotCached => {
                    debug!(target: "JIT-BUILD", "[pc=${:08X} not found in block cache, compiling]", self.next_instruction_pc as u32);

                    // build the block
                    let compiled_block_ref = self.build_block()?;
                    let compiled_block = compiled_block_ref.borrow();

                    // execute the block
                    trace!(target: "JIT-RUN", "[finished compiling. executing block id={} block_start=${:08X} start_offset=${:08X} cycles_ran={} limit={}]", compiled_block.id, compiled_block.start_address, physical_address-compiled_block.start_address, self.num_steps, self.jit_run_limit);
                    let starting_steps = self.num_steps;
                    self.num_steps += self.run_block(&*compiled_block, compiled_block.start_address)?; // start_address => run from the first instruction
                    trace!(target: "JIT-RUN", "[block executed {} cycles]", self.num_steps - starting_steps);
                },

                CachedBlockStatus::Cached(compiled_block) => {
                    let compiled_block = compiled_block.borrow();
                    trace!(target: "JIT-RUN", "[block found in cache, executing block id={} block_start=${:08X} start_offset=${:08X} cycles_ran={} limit={}]", compiled_block.id, compiled_block.start_address, physical_address-compiled_block.start_address, self.num_steps, self.jit_run_limit);
                    let starting_steps = self.num_steps;
                    self.num_steps += self.run_block(&*compiled_block, physical_address)?; // start execution at an offset into the block
                    trace!(target: "JIT-RUN", "[block executed {} cycles]", self.num_steps - starting_steps);
                }

                CachedBlockStatus::Uncompilable => { // interpret some instructions and try again
                    if self.next_instruction.is_none() {
                        self.next_instruction = Some(self.read_u32((self.next_instruction_pc) as usize).unwrap());
                        self.comms.increment_prefetch_counter();
                    }

                    for _ in 0..32 {
                        self.step()?;
                    }

                    continue;
                },
            }

            // if we broke out due to an idle loop, fast-forward time
            if self.jit_idle_loop_detected {
                // we could only have incremented by as many as jit_run_limit steps would allow, but we need to know how many steps were taken
                let steps_taken = self.num_steps - num_steps_before; 
                let steps_remaining = self.jit_run_limit.saturating_sub(steps_taken);

                trace!(target: "JIT-RUN", "skipping idle loop at ${:08X}, steps remaining={}", self.next_instruction_pc, steps_remaining);
                self.num_steps += steps_remaining;
                self.jit_idle_loop_detected = false; 
            }

            // PC and next_instruction_pc have changed, update current_instruction_pc for code that
            // relies on these values outside of this module (ie, debugger)
            self.current_instruction_pc = self.next_instruction_pc;
            self.is_delay_slot = self.next_is_delay_slot;

            // Cop0_Count increments at half the PClock, so we take half the cycles executed and add that to Count
            let cp0_count_increment = (self.num_steps - self.cp0_count_tracker) >> 1;
            if cp0_count_increment != 0 {
                self.cp0gpr[Cop0_Count] = ((self.cp0gpr[Cop0_Count] as u32) + (cp0_count_increment as u32)) as u64;

                // Trigger timer interrupt if enough cycles occurred
                if cp0_count_increment >= self.cp0_compare_distance as u64 {
                    debug!(target: "CPU", "CPU0: timer interrupt");
                    let _ = self.timer_interrupt();
                }
            }

            // update Cop0_Random
            let _ = Cpu::update_cop0_random(self, 0, false);
        }

        Ok(self.num_steps - steps_start)
    }

    // compile a block of code starting at self.next_instruction_pc
    fn build_block(&mut self) -> Result<Rc<RefCell<CompiledBlock>>, InstructionFault> {
        let mut assembler = dynasmrt::x64::Assembler::new().unwrap();
        let start_address = self.next_instruction_pc;

        let entry_point = assembler.offset();

        // get a copy of the rdram with instructions we're going to build
        let physical_address = self.translate_address(start_address, false, false)?.unwrap().physical_address;

        // read a block of memory instead of read_u32() on each instruction
        let source_buffer = if physical_address < 0x80_0000 {
            let access = self.comms.rdram.read().unwrap();
            let rdram: &[u32] = access.as_ref().unwrap();
            let start = (physical_address >> 2) as usize;
            let end   = std::cmp::min(start + JIT_BLOCK_MAX_INSTRUCTION_COUNT, 0x80_0000 >> 2);
            Some(rdram[start..end].to_owned())
        } else {
            None
        };

        // make sure next_instruction is valid
        if self.next_instruction.is_none() {
            self.next_instruction = Some(if source_buffer.is_some() {
                source_buffer.as_ref().unwrap()[0]
            } else { 
                self.comms.increment_prefetch_counter();
                self.read_u32((self.next_instruction_pc) as usize).unwrap()
            });
        }

        // set global block start address to the correct pc
        self.jit_block_start_pc = self.next_instruction_pc;

        // determine the start index into the cached block map
        // the physical address and index must be valid since we're in build_block()
        let cached_block_map_index = self.get_cached_block_map_index(physical_address).unwrap();

        // must first invalidate any block that might be referenced by an AddressReference at this location
        if let Some(CachedBlockReference::AddressReference(index_reference, block_id_reference)) = self.cached_block_map[cached_block_map_index] {
            if let Some(CachedBlockReference::CachedBlock(referenced_block_id)) = self.cached_block_map[index_reference] {
                if block_id_reference == referenced_block_id {
                    debug!(target: "JIT-BUILD", "[invalidating ${:08X} because ${:08X} contained a reference to it]", index_reference << 2, cached_block_map_index << 2);
                    self.cached_block_map[index_reference] = None;
                }
            }
        }

        // reset state that belongs to this block
        self.jit_jump = false;
        self.jit_jump_no_delay = false;
        self.jit_conditional_branch = None;

        let block_id = self.next_block_id;
        self.next_block_id += 1;

        // allocate space for the jump table and pin it in place
        // we can already use the pointer in the prologue
        let mut jump_table = Box::pin(Vec::<u64>::with_capacity(JIT_BLOCK_MAX_INSTRUCTION_COUNT + 8));

        // break before executing
        //letsbreak!(assembler);

        let mut instruction_start_offsets = HashMap::new();
        let mut jit_fixups = Vec::new();
        const MAX_NOPS: u32 = 5;
        let mut num_nops = 0;

        // loop over instructions, compiling one at a time
        let mut instruction_index = 0usize;

        let mut done_compiling = false;
        'build_loop: while self.next_is_delay_slot || (!done_compiling && instruction_index < JIT_BLOCK_MAX_INSTRUCTION_COUNT) {
            // tally up non-delay slot NOPs and break if we get too many (most of the time this
            // would be a non-code region)
            num_nops = if !self.next_is_delay_slot && self.next_instruction.unwrap() == 0 { num_nops + 1 } else { 0 };
            if num_nops == MAX_NOPS {
                // if there were other nops, next_is_delay_slot can't be set
                assert!(!self.next_is_delay_slot);
                break;
            }

            // if the current instruction address is unaligned, we can't decode or run anything and
            // just need to make an address exception block

            // setup self.inst
            self.decode_instruction(self.next_instruction.unwrap());

            // next instruction prefetch
            self.next_instruction = Some(if source_buffer.is_some() && (instruction_index + 1) < source_buffer.as_ref().unwrap().len() {
                source_buffer.as_ref().unwrap()[instruction_index + 1]
            } else { 
                self.read_u32((self.pc) as usize).unwrap()
            });

            // update and increment PC (perhaps this should also be part of next instruction prefetch)
            self.current_instruction_pc = self.next_instruction_pc;
            self.next_instruction_pc = self.pc;
            self.pc += 4;

            // in delay slot flag
            self.is_delay_slot = self.next_is_delay_slot;
            self.next_is_delay_slot = false;

            // update the block cache map. we don't have to check AddressReference because the
            // nearest CachedBlock has already been invalidated, and any other CachedBlocks along
            // the way will also get invalidated
            self.cached_block_map[cached_block_map_index + instruction_index] = Some(CachedBlockReference::AddressReference(cached_block_map_index, block_id));

            // save whether there's a jump that exits the block after this instruction (w/ delay slot)
            let jit_jump = std::mem::replace(&mut self.jit_jump, false);

            // save whether there's a branch after this instruction (w/ delay slot)
            let jit_conditional_branch = std::mem::replace(&mut self.jit_conditional_branch, None);

            // set pointer to instruction offset
            let instruction_offset = assembler.offset().0;
            instruction_start_offsets.insert(start_address + (instruction_index << 2) as u64, instruction_offset);
            self.jit_current_assembler_offset = instruction_offset;
            instruction_index += 1;

            // start each element in the jump table with the offset in the assembly
            jump_table.push(instruction_offset as u64);

            // TODO per-instruction prologue?
            //letsgo!(assembler
            //    ;    mov DWORD [rsp + s_inst], self.inst.v as _ // save current instruction opcode
            //);

            // In dev builds, check that r0 is still zero before every instruction
            cfg_if! {
                if #[cfg(feature="dev")] {
                    letsgo!(assembler
                        ;   cmp QWORD [r_gpr /*+ 0 as i32*/], BYTE 0u8 as _
                        ;   je >r0_ok
                        ;   int3
                        ;   mov rax, QWORD self.current_instruction_pc as _
                        ;r0_ok:
                    );
                }
            }

            // for branch likely instructions, skip the delay slot instruction if the branch is not taken
            if let Some((_, is_branch_likely, is_unconditional)) = jit_conditional_branch {
                assert!(self.is_delay_slot); // always the case when jit_conditional_branch is set
                if is_branch_likely && !is_unconditional { // don't check r_cond with is_unconditional, as we are taking the branch
                    trace!(target: "JIT-BUILD", "** previous instruction was a branch_likely, checking r_cond..");
                    // if the branch is not taken (r_cond == 0), we skip over the next compiled instruction
                    letsgo!(assembler
                        ;   dec r_cond     // test if r_cond starts at 1
                        ;   jne >no_branch // if it's non-zero, r_cond wasn't 1, and we can skip the next instruction,
                                           // but we can also skip the branch check below
                    );
                }
            }

            // compile the instruction
            match self.jit_instruction_table[self.inst.op as usize](self, &mut assembler) {
                CompileInstructionResult::Continue => {},

                CompileInstructionResult::Stop => {
                    done_compiling = true;
                },

                CompileInstructionResult::Cant => {
                    // at this point the instruction was never compiled, so we actually don't need
                    // to perform cycle epilog work, we just need to exit the block and set a flag
                    // that the current instruction needs to be run with the interpreter
                    // we don't need prefetch now, since we have the instruction and PC

                    // determine what PC needs to be after the current instruction is executed
                    if jit_jump { // use s_jump_target
                        letsgo!(assembler
                            // copy jump target directly to PC
                            ;   mov rax, QWORD [rsp+s_jump_target]
                            ;   mov QWORD [r_cpu + offset_of!(Cpu, pc) as i32], rax
                        );
                    } else if let Some((branch_offset, is_branch_likely, is_unconditional)) = jit_conditional_branch {
                        if is_branch_likely { 
                            letsgo!(assembler
                                // this no_branch label is incorrect; if the branch isn't taken, this
                                // "uncompilable" instruction isn't even executed, so we can skip it
                                // entirely
                                ;no_branch:
                                ;   int3
                                ;   mov rax, QWORD 0xFEDD_B0B0_0000_9999u64 as _
                            );
                        }

                        // this goes into PC
                        let branch_target = (self.current_instruction_pc as i64).wrapping_add(branch_offset);

                        // r_cond determines what we set PC to
                        if !is_unconditional {
                            letsgo!(assembler
                                ;   dec r_cond
                                ;   jz >okay
                                ;   int3
                                ;okay:
                            );
                        }

                        letsgo!(assembler
                            ;   mov rax, QWORD branch_target as u64 as _
                            ;   mov QWORD [r_cpu + offset_of!(Cpu, pc) as i32], rax
                        );
                    } else {
                        // would need to reinstate the jit_interpret_next_instruction value, but this code path isn't ever hit
                        panic!("unhandled Cant situation");
                    }

                    // the current instruction needs to be replayed in step()
                    letsgo!(assembler
                        // current_instruction_pc goes back into next_instruction_pc
                        ;   mov rax, QWORD self.current_instruction_pc as u64 as _
                        ;   mov QWORD [r_cpu + offset_of!(Cpu, next_instruction_pc) as i32], rax
                        // the current opcode is self.inst.v, place back in next_instruction via set_next_instruction
                        // v_arg1_32 is the second argument
                        ;   mov v_arg1_32, DWORD self.inst.v as u32 as _
                        // the current delay slot state back into next_is_delay_slot
                        ;   mov BYTE [r_cpu + offset_of!(Cpu, next_is_delay_slot) as i32], BYTE self.is_delay_slot as u8 as _
                    );

                    // set next_instruction
                    extern "sysv64" fn set_next_instruction(cpu: *mut Cpu, inst: u32) {
                        let cpu = unsafe { &mut *cpu };
                        cpu.next_instruction = Some(inst);
                    }

                    letscall!(assembler, set_next_instruction);

                    // exit the block without prefetch
                    letsgo!(assembler
                        ;   jmp >epilog
                    );

                    // break compilation, preventing the check count and jumps (which we know we don't have)
                    debug!(target: "JIT-BUILD", "** terminating block compilation due to Cant-compile situation");
                    break 'build_loop;
                },
            }

            // post-instruction prologue
            letsgo!(assembler
                ;   dec DWORD [rsp+s_cycle_count]
            );

            // jit_jump_no_delay is used in ERET
            if jit_jump || self.jit_jump_no_delay {
                // this assert should never happen (CompileInstructionResult::Cant will occur with a jump in the delay slot)
                assert!(!(self.jit_jump || self.jit_conditional_branch.is_some()), 
                        "jit_jump={:?} self.jit_conditional_branch={:?}", jit_jump, self.jit_conditional_branch);
                                      
                trace!(target: "JIT-BUILD", "** performing jump");

                self.jit_jump_no_delay = false;

                // the dynamic jump target is in s_jump_target..we can either jump directly to the code or
                // the code doesn't exist and needs to be compiled.  so we get out of this block now, but
                // we can continue compiling
                letsgo!(assembler
                    ;   mov v_arg1, QWORD [rsp+s_jump_target]                // copy jump target to self.pc
                    ;   test v_arg1_32, BYTE 3u8 as _                        // check valid execution address
                    ;   jz >check_cycle_count_or_block_link                  // and if valid, jump to it
                    ;   jmp >instruction_fetch_exception_handler             // generate an exception with a bad jump
                );
            } else if let Some((branch_offset, is_branch_likely, is_unconditional)) = jit_conditional_branch {
                assert!(!self.jit_jump && self.jit_conditional_branch.is_none()); // no branches within delay slots

                // TODO we can actually return from block execution here if branch_offset is
                // guaranteed to be outside of the block:
                // let will_exit_block = branch_offset < 0 || (branch_offset >> 2) >= instruction_start_offsets.len()
                // wouldn't be much of an optimization though since it just eliminates a single jump

                // the target is the offset plus the address of the instruction in delay slot,
                // which should be the /current/ instruction pc for jit_conditional_branch
                let dynamic_label = assembler.new_dynamic_label();
                let branch_target = (self.current_instruction_pc as i64).wrapping_add(branch_offset);

                // if we are executing code here, we're either not using branch_likely (and we have
                // to check r_cond), or we are using branch_likely and know that we're taking the
                // branch. alternatively, if the branch is uncondtional (always taken) we jump here
                if is_branch_likely || is_unconditional {
                    // try to detect basic idle loops, where we have an unconditional branch backwards 
                    // one instruction and the delay slot is a nop
                    if is_unconditional && branch_offset == -4 && self.inst.v == 0 {
                        trace!(target: "JIT-BUILD", "** idle loop detected at ${:08X}, exiting block", branch_target);
                        letsgo!(assembler
                            ;   inc BYTE [r_cpu + offset_of!(Cpu, jit_idle_loop_detected) as i32] // set jit_idle_loop_detected to nonzero
                            ;   jmp >break_out
                        );
                    }
                    trace!(target: "JIT-BUILD", "** in branch_likely to ${:08X}, checking cycle count break out", branch_target as u32);

                    // test if we need to break out of the block before the branch
                    // the use of the dynamic label lets us take branches within this same block
                    // otherwise, we can use the branch target and try for block linking
                    letsgo!(assembler
                        ;   cmp DWORD [rsp+s_cycle_count], BYTE 0i8 as _ // break out when run count hits 0 or less
                        ;   jle >break_out
                        ;   mov rax, QWORD self.comms.break_cpu_cycles.as_ptr() as _ // or when break_cpu_cycles is set
                        ;   cmp BYTE [rax], BYTE 0u8 as _
                        ;   je =>dynamic_label                           // otherwise, take the branch
                        ;break_out:
                        ;   mov v_arg1, QWORD branch_target as u64 as _  // PC address we need to jump to
                        ;   jmp >restore_pc_and_break_out                // we must exit the block, so no block linking
                        // this label is used above, before the instruction is compiled, to skip delay slots
                        ;no_branch:
                    );
                } else {
                    trace!(target: "JIT-BUILD", "** checking normal branch condition:");

                    // check s_cycle_count and break_cpu_cycles to see if we should exit the block
                    letsgo!(assembler
                        ;   cmp DWORD [rsp+s_cycle_count], BYTE 0i8 as _    // break out when run count hits 0 or less
                        ;   jle >break_out
                        ;   mov rax, QWORD self.comms.break_cpu_cycles.as_ptr() as _ // or when break_cpu_cycles is set
                        ;   cmp BYTE [rax], BYTE 0u8 as _
                        ;   je >no_break_out
                        // we have to exit the block, so set v_arg1 (for restore_pc_and_breakout) based on r_cond
                        ;break_out:
                        ;   mov v_arg1, QWORD self.next_instruction_pc as _ // default to fall through
                        ;   dec r_cond                                      // check if r_cond is set
                        ;   jne >skip_set                                   // .
                        ;   mov v_arg1, QWORD branch_target as u64 as _     // if set, use the branch target instead
                        ;skip_set:
                        ;   jmp >restore_pc_and_break_out                   // exit the block
                        // since we don't have to exit the block, jump to the dynamic label
                        ;no_break_out:
                        ;   dec r_cond          // test if r_cond is 1
                        ;   jz =>dynamic_label  // and branch if so
                    );
                }

                jit_fixups.push((branch_target, dynamic_label));
            }
            // // TODO this is temporary and isn't necessary beyond using the debugger
            // else if !self.next_is_delay_slot && !done_compiling {
            //     // test if we need to break out
            //     letsgo!(assembler
            //         ;   cmp DWORD [rsp+s_cycle_count], BYTE 0i8 as _ // break out when run count hits 0 or less
            //         ;   jg >no_break_out
            //         ;   mov v_arg1, QWORD self.next_instruction_pc as i64 as _
            //         ;   jmp >restore_pc_and_break_out
            //         ;no_break_out:
            //     );
            // }
        }

        if !done_compiling && instruction_index >= JIT_BLOCK_MAX_INSTRUCTION_COUNT {
            debug!(target: "JIT-BUILD", "[terminating block compilation after {} instructions]", instruction_index);
        } else if num_nops == MAX_NOPS {
            debug!(target: "JIT-BUILD", "[terminating block compilation after {} nops]", num_nops);
        }

        // If we fall through execution here instead of jumping to epilog, then we basically can
        // to continue execution of game code, but first check if there are cycles left.
        //
        // self.next_instruction_pc has been updating and currently points to the next instruction,
        // so we try linking to the next block and jumping to it directly.
        //
        // And we can't exit a block in a delay slot instruction, so it should never be true here
        assert!(!self.next_is_delay_slot);

        // block link expects the next instruction address v_arg1
        letsset!(assembler, v_arg1, v_arg1_32, self.next_instruction_pc);

        letsgo!(assembler
            ;check_cycle_count_or_block_link:
            ;   cmp DWORD [rsp+s_cycle_count], BYTE 0i8 as _  // check s_cycle_count for <= 0
            ;   jle >restore_pc_and_break_out
            ;   mov rax, QWORD self.comms.break_cpu_cycles.as_ptr() as _ // or when break_cpu_cycles is set
            ;   cmp BYTE [rax], BYTE 0u8 as _
            ;   je >try_block_link
            // fall through to restore_pc_and_break_out
        );

        // set next_instruction_pc (in v_arg1) into PC and call prefetch
        letsgo!(assembler
            ;restore_pc_and_break_out:
            // first test if our destination address is misaligned
            ;   test v_arg1_32, BYTE 3u8 as _
            ;   jnz >instruction_fetch_exception_handler
            ;   mov QWORD [r_cpu + offset_of!(Cpu, pc) as i32], v_arg1  // store branch dest in self.pc
            // fall through to prefetch_bridge
        );

        letscall!(assembler, Cpu::prefetch_bridge);                  // setup next_instruction_pc
        // fall through to epilog
        // ...
        // ...
        // Finally the "epilog", which due to the fact that we have a trapoline just returns
        letsgo!(assembler
            ;epilog:
            ;   ret
        );

        // instruction fetch exception (AdEL) occurs only on a branch, and we can't really use letsexcept!() because
        // it was built for handling exceptions during jit execution. No big deal, a few small things are duplicated here
        // v_arg1 contains the virtual_address of the invalid instruction fetch
        letsgo!(assembler
            ;instruction_fetch_exception_handler:
            ;   mov BYTE [r_cpu + offset_of!(Cpu, is_delay_slot) as i32], BYTE 0 as _    // TODO might need to set this correctly for certain test cases
            ;   mov QWORD [r_cpu + offset_of!(Cpu, current_instruction_pc) as i32], v_arg1 
            ;   xor v_arg2_32, v_arg2_32 // is_write = false
        );

        letscall!(assembler, Cpu::address_exception_bridge); // will call prefetch_bridge() and set up next_instruction_pc

        letsgo!(assembler
            ;   jmp <epilog // get out of block
        );

        // OK, well, I thought block linking the exception handler would be a nice performance improvement, but
        // I guess not.  Literally a huge overhead and I'm not sure why. It might have to do with the translate_address
        // call, but now I wonder if block linking is just going to be slower for all branches.
        //
        // // Handle exceptions -- we may or may not need to break out (depending on cycle count), but we can actually
        // // just do a block link to the exception handler most of the time. However, when an exception has occurred
        // // (jit_other_exception is true and letscheck!() branched here), the next instruction is in self.next_instruction_pc
        // // already (prefetch_bridge already called too). We need to get it back into v_arg1 for try_block_link
        // letsgo!(assembler
        //     ;exit_for_exception:
        //     // check cycle count first
        //     ;   cmp DWORD [rsp+s_cycle_count], BYTE 0i8 as _  // check s_cycle_count for <= 0
        //     ;   jle <epilog
        //     ;   mov rax, QWORD self.comms.break_cpu_cycles.as_ptr() as _ // or when break_cpu_cycles is set
        //     ;   cmp BYTE [rax], BYTE 0u8 as _
        //     ;   jne <epilog
        //     // ok, we need the PC in v_arg1 for try_block_link
        //     ;   mov v_arg1, QWORD [r_cpu + offset_of!(Cpu, next_instruction_pc) as i32]
        //     ;   jmp >try_block_link
        // );

        // fixup jumps
        for &(branch_target, dynamic_label) in &jit_fixups {
            if instruction_start_offsets.contains_key(&(branch_target as u64)) {
                // get the offset from the start of the block to the target instruction jump
                let target_instruction_offset = instruction_start_offsets.get(&(branch_target as u64)).unwrap();
                // update the dynamic label to use that offset
                let _ = assembler.labels_mut().define_dynamic(dynamic_label, dynasmrt::AssemblyOffset(*target_instruction_offset)).unwrap();
            } else {
                trace!(target: "JIT-BUILD", "** creating external block branch target handler for branch to ${:16X}", branch_target);

                // see if the target has a compiled block at that location. we need to check at
                // block-run time, since branch destinations can be invalidated or changed
                let code_start_offset = assembler.offset();

                // put the branch target in v_arg1
                letsset!(assembler, v_arg1, v_arg1_32, branch_target);

                // Branch instructions can't jump to misaligned addresses but build_inst_j and build_inst_jal make use of the relative
                // branch code, so we still have to check for misaligned jumps here
                if (branch_target & 3) != 0 {
                    letsgo!(assembler
                        ;   jmp <instruction_fetch_exception_handler
                    );

                    todo!("If anything ever hits this branch, let me know (branch to ${:08X})!", branch_target);
                } else {
                    letsgo!(assembler
                        ;   jmp >try_block_link
                    );
                }

                // set the dynamic_label to jump here
                let _ = assembler.labels_mut().define_dynamic(dynamic_label, code_start_offset).unwrap();

                // and any other jumps that might go to the same address
                instruction_start_offsets.insert(branch_target as u64, code_start_offset.0);
            }
        }

        // try linking to a block directly, destination pc in v_arg1
        // TODO: I'm not sure I'm actually getting any performance benefit out of block linking
        // the call to lookup_compiled_instruction_address is possibly way too much overhead.
        letsgo!(assembler
            ;try_block_link:
            // ;   jmp <restore_pc_and_break_out
            // save a copy of v_arg1
            // ;   int3
            ;   mov QWORD [rsp+s_tmp0], v_arg1
        );

        // call lookup_compiled_instruction_address with pc in v_arg1
        // if the return value is non-zero, we have a valid jump destination and we can just jump to it and be done
        letscall!(assembler, Cpu::lookup_compiled_instruction_address);
        letsgo!(assembler
            ;   test rax, rax
            ;   jz >notfound
            ;   jmp rax // weee
            ;notfound: // otherwise, put branch destination into self.pc, prefetch, and exit the block
            ;   mov v_arg1, QWORD [rsp+s_tmp0]
            ;   jmp <restore_pc_and_break_out
        );

        // finalize the block
        let executable_buffer = assembler.finalize().unwrap();

        // with the buffer finalized, we know where it is in memory, we can update the jump table offsets to be absolute addresses
        for entry in jump_table.as_mut().get_mut().iter_mut() {
            *entry = executable_buffer.ptr(dynasmrt::AssemblyOffset(*entry as usize)) as u64;
        }

        let compiled_block = CompiledBlock {
            id           : block_id,
            start_address: physical_address,
            _entry_point : entry_point,
            _code_buffer : executable_buffer,
            jump_table   : jump_table,
        };

        // insert into block cache
        self.cached_block_map[cached_block_map_index] = Some(CachedBlockReference::CachedBlock(block_id));

        self.cached_blocks.insert(block_id, Rc::new(RefCell::new(compiled_block)));

        Ok(self.cached_blocks.get(&block_id).unwrap().clone())
    }

    // return number of instructions executed
    fn run_block(&mut self, compiled_block: &CompiledBlock, execution_address: u64) -> Result<u64, InstructionFault> {
        let trampoline: extern "sysv64" fn(cpu: *mut Cpu, compiled_instruction_address: u64, run_limit: u32) -> i32 = unsafe {
            std::mem::transmute(self.jit_trampoline.as_ref().unwrap().ptr(self.jit_trampoline_entry_point))
        };

        // find the nth instruction in the block, which is the offset divided by sizeof(u32)
        let instruction_index = (execution_address - compiled_block.start_address) >> 2;
        let compiled_instruction_address = compiled_block.jump_table[instruction_index as usize];

        self.jit_executing = true;
        // blocks can run beyond their run limit so the return value of call_block could be negative
        let res = (self.jit_run_limit as i64) - (trampoline(self as *mut Cpu, compiled_instruction_address, self.jit_run_limit as u32) as i64);

        // TODO maybe, return error codes here
        //let res = match call_block(self.jit_run_limit as u32, instruction_index) {
        //    // blocks can run beyond their cycle limit, so we give 4B cycles of leeway, and anything else negative is an error
        //    // jit_run_limit can change during the course of the run (when we need to reduce the
        //    // length of the run due to Compare value changes)
        //    i if i >= -2147483648 => Ok((self.jit_run_limit as i64 - i) as u64), // 0xFFFF_FFFF_8000_0000

        //    i => { // i < 0xFFFF_FFFF_8000_0000 indicates error
        //        todo!("execution error code: {}", i as u32);
        //    },
        //};

        self.jit_executing = false;

        // TODO: return instruction fault?
        // if self.jit_other_exception { ... // if an exception occurred that caused the block to exit... 
        self.jit_other_exception = false;

        Ok(res as u64)
    }

    // since this function is used by assembly, a return value of 0 indicates "not present",
    // instead of using Option<>.
    #[inline(always)]
    extern "sysv64" fn lookup_compiled_instruction_address(cpu: *mut Cpu, virtual_address: u64) -> u64 {
        let cpu = unsafe { &mut *cpu };
        let physical_address = cpu.translate_address(virtual_address, false, false).unwrap().unwrap().physical_address;
        match cpu.lookup_cached_block(physical_address) {
            Ok(CachedBlockStatus::Cached(compiled_block)) => {
                let borrow = compiled_block.borrow();
                let instruction_index = (physical_address - borrow.start_address) >> 2;
                borrow.jump_table[instruction_index as usize]
            },
            // any result other than Cached() => block is not compiled
            _ => 0,
        }
    }

    #[inline]
    fn decode_instruction(&mut self, inst: u32) {
        self.inst.v = inst;

        // instruction decode
        self.inst.op         = inst >> 26;
        self.inst.rs         = ((inst >> 21) & 0x1F) as usize;
        self.inst.rt         = ((inst >> 16) & 0x1F) as usize;
        self.inst.rd         = ((inst >> 11) & 0x1F) as usize;
        self.inst.imm        = (inst & 0xFFFF) as u64;
        self.inst.signed_imm = (self.inst.imm as i16) as u64;
        self.inst.target     = inst & 0x3FFFFFF;
        self.inst.sa         = (inst >> 6) & 0x1F;
    }

    // interpret a single instruction
    pub fn step(&mut self) -> Result<(), InstructionFault> {
        self.num_steps += 1;

        // increment Cop0_Count at half PClock and trigger exception
        self.cp0gpr[Cop0_Count] = ((self.cp0gpr[Cop0_Count] as u32) + ((self.num_steps & 1) as u32)) as u64;
        //println!("Count=${:08X} Compare=${:08X}", self.cp0gpr[Cop0_Count], self.cp0gpr[Cop0_Compare]);
        if self.cp0gpr[Cop0_Compare] == self.cp0gpr[Cop0_Count] {
            debug!(target: "CPU", "COP0: timer interrupt");
            let _ = self.timer_interrupt();
        }

        // decrement Random every instruction, leaving bit 5 alone
        // mtc0 does have a delay before the register is is latched though
        self.cp0gpr_random_delay -= 1;
        if self.cp0gpr_random_delay <= 0 {
            if self.cp0gpr[Cop0_Random] == self.cp0gpr[Cop0_Wired] {
                self.cp0gpr[Cop0_Random] = 31;
            } else {
                self.cp0gpr[Cop0_Random] = self.cp0gpr[Cop0_Random].wrapping_sub(1) & 0x3F;
            }
        }

        // For address and TLB exceptions, the current_instruction_pc needs to point to
        // the address being fetched. This lets Cop0_EPC be set to the correct PC
        self.current_instruction_pc = self.pc; // For Cop0_EPC in case of a TLB miss in a delay
                                               // slot after a jump to an invalid TLB page
        self.is_delay_slot = false; // prefetch (self.pc) will only be invalid here after a jump
                                    // so even though the next instruction could be a delay slot,
                                    // the prefetch has to generate the exception
        // check for invalid PC addresses
        if (self.pc & 0x03) != 0 {
            if self.next_is_delay_slot && self.next_instruction.unwrap() != 0 {
                self.decode_instruction(self.next_instruction.unwrap()); // prepare self.inst
                match self.finish_delay_slot() {
                    Ok(_) => {},

                    Err(e) => {
                        unimplemented!("an exception was generated in the delay slot during an address exception: {:?}", e);
                    }
                }

                // fix values for address_exception()
                self.current_instruction_pc = self.pc;
                self.is_delay_slot = false;
            }

            return self.address_exception(self.pc, false);
        }

        // current instruction decode
        self.decode_instruction(self.next_instruction.unwrap());

        // if an exception occurs prefetching a jump target, we need to know if there was a pending delay slot
        let next_was_delay_slot = self.next_is_delay_slot;

        // next instruction prefetch. we need to catch TLB misses
        self.next_instruction = Some(match self.read_u32(self.pc as usize) {
            // most common situation
            Ok(x) => x,

            Err(InstructionFault::OtherException(ExceptionCode_TLBL)) => {
                // we need to know if the current self.next_instruction is a jump instruction and thus, if self.pc fetch is in a delay slot.
                // link instructions also need to correctly update RA to be after the delay slot
                let is_branch = match self.inst.op {
                    0b000_000 => { // special
                        match self.inst.v & 0x3F {
                            0b001_001 => { // JALR
                                self.gpr[self.inst.rd] = self.cp0gpr[Cop0_BadVAddr] + 4;
                                true
                            },

                            0b001_000 => true, // JR

                            _ => false,
                        }
                    },

                    0b000_001 => { // regimm
                        let regimm = (self.inst.v >> 16) & 0x1F;
                        match regimm {
                            0b00_000   // BLTZ
                            | 0b00_001 // BGEZ
                            | 0b00_010 // BLTZL
                            | 0b00_011 // BGEZL
                                => true,

                            // these two branches save the PC to r31
                            0b10_001 // BGEZAL
                            | 0b10_011 // BGEZALL
                                => {
                                self.gpr[31] = self.cp0gpr[Cop0_BadVAddr] + 4;
                                true
                            },

                            _ => false,
                        }
                    },

                    0b000_010   // J
                    | 0b000_100 // BEQ
                    | 0b000_101 // BNE
                    | 0b000_110 // BLEZ
                    | 0b000_111 // BGTZ
                    | 0b010_100 // BEQL
                    | 0b010_101 // BNEL
                    | 0b010_110 // BLEZL
                    | 0b010_111 // BGTZL
                        => true,

                    0b000_011 => { // JAL
                        self.gpr[31] = self.cp0gpr[Cop0_BadVAddr] + 4;
                        true
                    },

                    _ => false,
                };

                // exception in the delay slot sets EPC to the address of the jump, not the delay slot
                if is_branch {
                    // just curious if this actually ever happens...
                    // check if not jalr...
                    if (self.inst.v & 0xFC00_003F) != 0x0000_0009 { // JALR
                        warn!(target: "CPU", "let me know when this happens");
                        todo!();
                    }

                    self.cp0gpr[Cop0_EPC] -= 4;
                    // also set the BD flag indicating the exception was in a delay slot
                    self.cp0gpr[Cop0_Cause] |= 0x8000_0000;
                }

                // the previous branch caused a TLBL on prefetch but there's still a delay slot instruction that still
                // needs to be executed
                if next_was_delay_slot && self.next_instruction.unwrap() != 0 {
                    if is_branch {
                        unimplemented!("TLBL caused by the prefix but we have a jump in the delay slot, this is ugly..");
                    }

                    // self.inst is already setup from before
                    match self.finish_delay_slot() {
                        Ok(_) => {},
                        Err(e) => {
                            unimplemented!("an exception was generated in the delay slot during an address exception: {:?}", e);
                        }
                    }
                }

                return Err(InstructionFault::OtherException(ExceptionCode_TLBL));
            }

            Err(x) => { return Err(x); },
        });

        // determine next cop index based on the opcode 0b010_0xx are the copN instructions
        self.next_cop_index = if (self.next_instruction.unwrap() >> 29) == 0b010 { (self.next_instruction.unwrap() >> 26) & 0b011 } else { 0 };

        // update and increment PC
        // current_instruction_pc is set twice, once at the beginning in case an external factor
        // changes the PC (see fn interupt()), and once at the end to keep it valid outside of
        // step()
        self.current_instruction_pc = self.next_instruction_pc;
        self.next_instruction_pc = self.pc;
        self.pc += 4;

        // in delay slot flag
        self.is_delay_slot = self.next_is_delay_slot;
        self.next_is_delay_slot = false;

        // execute instruction and check result
        let result = match self.instruction_table[self.inst.op as usize](self) {
            // Most common situation
            result @ Ok(_) => result,

            // Other exceptions continue execution
            Err(InstructionFault::OtherException(_)) => {
                Ok(())
            }

            // Raise FPE on InstructionFault::FloatingPointException
            Err(InstructionFault::FloatingPointException) => { 
                let _ = self.floating_point_exception();
                Ok(())
            },

            Err(InstructionFault::CoprocessorUnusable) => {
                panic!("Am I using this?");
            }

            Err(InstructionFault::ReadWrite(fault)) => {
                error!(target: "CPU", "crash at PC=${:16X}: {:?}", self.current_instruction_pc, fault);
                info!(target: "CPU", "[$80000318] = ${:08X}", self.read_u32_phys(
                       Address {
                            virtual_address: 0,
                            physical_address: 0x00000318,
                            cached: false,
                            mapped: false,
                            space: MemorySpace::User,
                            tlb_index: None,
                        }).unwrap());
                panic!("cpu crash");
            }

            // Other faults like Break, Unimplemented actually stop processing
            result @ Err(_) => {
                // on error, restore the previous instruction since it didn't complete
                self.pc -= 4;
                self.next_instruction_pc = self.current_instruction_pc;
                self.next_instruction = Some(self.inst.v);
                result
            },
        };

        // r0 must always be zero
        self.gpr[0] = 0;

        // PC and next_instruction_pc have changed, update current_instruction_pc
        self.current_instruction_pc = self.next_instruction_pc;
        self.is_delay_slot = self.next_is_delay_slot;

        result
    }

    // self.inst should be set up via self.decode_instruction before calling this
    fn finish_delay_slot(&mut self) -> Result<(), InstructionFault> {
        //println!("flushing delay slot instruction ${:08X} at address ${:08X}", self.inst.v, self.next_instruction_pc);
        self.current_instruction_pc = self.next_instruction_pc;
        self.is_delay_slot = true;

        match self.instruction_table[self.inst.op as usize](self) {
            Ok(_) => Ok(()),

            e @ Err(_) => e
        }
    }

    fn inst_reserved(&mut self) -> Result<(), InstructionFault> {
        //warn!(target: "CPU", "reserved instruction ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
        self.reserved_instruction_exception(0)?;
        Ok(())
    }

    fn build_inst_reserved(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "reserved instruction ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);

        // pass 0 as coprocessor_number
        letsgo!(assembler
            ;   xor v_arg1_32, v_arg1_32
        );

        letsexcept!(self, assembler, Cpu::reserved_instruction_exception_bridge);

        // Stop decomp, at this point nothing makes sense any more
        CompileInstructionResult::Stop
    }

    fn inst_unknown(&mut self) -> Result<(), InstructionFault> {
        error!(target: "CPU", "unimplemented instruction ${:03b}_{:03b}", self.inst.op >> 3, self.inst.op & 0x07);
        Err(InstructionFault::Unimplemented)
    }

    fn build_inst_unknown(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        extern "sysv64" fn unknown_bridge(cpu: *mut Cpu, inst: u32) {
            let cpu = unsafe { &mut *cpu };
            cpu.decode_instruction(inst);
            if cpu.inst.op == 0 {
                error!(target: "CPU", "unimplemented instruction ${:03b}_{:03b} (special ${:03b}_{:03b})", cpu.inst.op >> 3, cpu.inst.op & 0x07, cpu.inst.special >> 3, cpu.inst.special & 0x07);
            } else if cpu.inst.op == 1 {
                error!(target: "CPU", "unimplemented instruction ${:03b}_{:03b} (regimm ${:02b}_{:03b})", cpu.inst.op >> 3, cpu.inst.op & 0x07, cpu.inst.regimm >> 3, cpu.inst.regimm & 0x07);
            } else {
                error!(target: "CPU", "unimplemented instruction ${:03b}_{:03b}", cpu.inst.op >> 3, cpu.inst.op & 0x07);
            }
            todo!();
        }

        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        letscall!(assembler, unknown_bridge);

        // we don't know what kind of data we're in now, but it probably isn't code
        CompileInstructionResult::Stop
    }

    fn inst_addi(&mut self) -> Result<(), InstructionFault> {
        // integer overflow exception occurs with ADDI, unlike ADDIU
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.inst.signed_imm as u32;
        let result = rs.wrapping_add(rt);
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            //println!("CPU: addi overflow detected: rs=${:08X} imm=${:04X} result=${:08X}", rs, rt, result);
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rt] = (result as i32) as u64;
        }

        Ok(())
    }

    fn build_inst_addi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: addi r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rs == 0 {
            if self.inst.rt != 0 { // don't overwrite r0
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], DWORD self.inst.signed_imm as u32 as _
                );
            }
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]
            );

            // skip add and checking overflow if signed_imm is 0
            if self.inst.signed_imm != 0 {
                letsgo!(assembler
                    ;   add v_tmp_32, DWORD self.inst.signed_imm as u32 as _
                    ;   jno >no_overflow
                );

                letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

                letsgo!(assembler
                    ;no_overflow:
                );
            }

            // don't overwrite r0
            if self.inst.rt != 0 {
                letsgo!(assembler
                    ;   movsxd v_tmp2, v_tmp_32
                    ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp2
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn inst_addiu(&mut self) -> Result<(), InstructionFault> {
        // no integer overflow exception occurs with ADDIU
        self.gpr[self.inst.rt] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.inst.signed_imm as u32) as i32) as u64;
        Ok(())
    }

    fn build_inst_addiu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: addiu r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rt != 0 { // if destination is zero, no-op
            if self.inst.rs == 0 { // We can skip the add and use a move
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], DWORD self.inst.signed_imm as u32 as _ // sign-extends
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs*8) as i32]      // put rs into v_tmp
                    ;   add v_tmp_32, DWORD self.inst.signed_imm as u32 as _       // 32-bit add signed_imm with overflow 
                    ;   movsxd v_tmp2, v_tmp_32                                    // sign-extend v_tmp into rt
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], v_tmp2        // .
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn inst_andi(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] & self.inst.imm;
        Ok(())
    }

    #[allow(unreachable_code)] // remove after all code paths have been tested
    #[allow(unused_variables)] // remove after all code paths have been tested
    fn build_inst_andi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: andi r{}, r{}, 0x{:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.imm);

        if self.inst.rt != 0 { // if destination register is 0, this is a nop
            if self.inst.rs == self.inst.rt { // can save an instruction when the operands are the same register
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                    ;   and QWORD [r_gpr + (self.inst.rs*8) as i32], v_tmp  // and rs with self.inst.imm, store result
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                    ;   and v_tmp, QWORD [r_gpr + (self.inst.rs*8) as i32]  // and rs with self.inst.imm
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], v_tmp  // store result in rt
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn inst_cop(&mut self) -> Result<(), InstructionFault> {
        let copno = ((self.inst.v >> 26) & 0x03) as u64;
        let cop = match copno {
            0 => panic!("TODO"),
            1 => &mut self.cop1,
            _ => {
                self.coprocessor_unusable_exception(copno)?;
                return Ok(());
            },
        };

        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & (0x1000_0000 << copno)) == 0 {
            self.coprocessor_unusable_exception(copno)?;
            return Ok(());
        }

        if (self.inst.v & (1 << 25)) != 0 {
            cop.special(self.inst.v)
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => { // MFC - move control word from coprocessor
                    self.gpr[self.inst.rt] = (cop.mfc(self.inst.rd)? as i32) as u64;
                    Ok(())
                },

                0b00_001 => { // DMFC
                    self.gpr[self.inst.rt] = cop.dmfc(self.inst.rd)?;
                    Ok(())
                },

                0b00_010 => { // CFC
                    self.gpr[self.inst.rt] = (cop.cfc(self.inst.rd)? as i32) as u64;
                    Ok(())
                },

                0b00_011 | 0b00_111 => { // dcfc1, dctc1 are unimplemented
                    cop.unimplemented_instruction()
                },

                0b00_100 => { // MTC
                    cop.mtc(self.gpr[self.inst.rt] as u32, self.inst.rd)
                },

                0b00_101 => { // DMTC
                    cop.dmtc(self.gpr[self.inst.rt], self.inst.rd)
                },

                0b00_110 => { // CTC - move control word to coprocessor
                    cop.ctc(self.gpr[self.inst.rt], self.inst.rd)
                },

                0b01_000 => {
                    let branch = (self.inst.v >> 16) & 0x1F;
                    match branch {
                        0b00_000 => { // BCzF
                            let coc = cop.condition_signal();
                            self.branch(!coc);
                            Ok(())
                        },

                        0b00_001 => { // BCzT
                            let coc = cop.condition_signal();
                            self.branch(coc);
                            Ok(())
                        },

                        0b00_010 => { // BCzFL 
                            let coc = cop.condition_signal();
                            self.branch_likely(!coc)
                        },

                        0b00_011 => { // BCzTL
                            let coc = cop.condition_signal();
                            self.branch_likely(coc)
                        },

                        _ => {
                            warn!(target: "COP1", "unhandled branch mode ${:05b}", branch);
                            Err(InstructionFault::Unimplemented)
                        }
                    }
                },

                _ => panic!("CPU: unknown cop function 0b{:02b}_{:03b} (called on cop{})", func >> 3, func & 7, copno),
            }
        }
    }

    // given a register number and value (to be written to that register), return a value that 
    // that masks out the proper bits or prevents writing, etc
    fn mask_cp0_register(&mut self, register_number: usize, value: u64) -> u64 {
        match register_number {
            Cop0_Index => {
                value & 0x0000_0000_8000_003F // 32-bit register
            },

            Cop0_Context => {
                (self.cp0gpr[Cop0_Context] & 0x7F_FFFF) | (value & 0xFFFF_FFFF_FF80_0000) // 64-bit register
            },

            Cop0_Wired => {
                // Cop0_Random is set to the upper bound of 31 when Cop0_Wired is written
                self.cp0gpr[Cop0_Random] = 0x1F;
                self.cp0gpr_random_delay = 3;
                value & 0x3F
            },

            Cop0_Random => {
                // read-only register, return the current contents 
                self.cp0gpr[Cop0_Random] & 0x3F
            },

            Cop0_Config => {
                let read_only_mask = 0xF0FF7FF0u64;
                ((value & 0xFFFF_FFFF) & !read_only_mask) | (self.cp0gpr[Cop0_Config] & read_only_mask)
            },

            Cop0_XContext => { // read-only register
                (self.cp0gpr[Cop0_XContext] & 0x1_FFFF_FFFF) | (value & 0xFFFF_FFFE_0000_0000)
            },

            Cop0_Compare => {
                // clear pending timer interrupt (see 6.3.4)
                self.cp0gpr[Cop0_Cause] &= !(InterruptCode_Timer << 8);
                // truncate to 32-bits
                value & 0x0000_0000_FFFF_FFFF
            },

            Cop0_Cause => {
                // handle software interrupts 
                if (value & 0x300) != 0 {
                    panic!("CPU: cop0 software interrupt");
                }

                // Only bits IP1 and IP0 are writable
                (self.cp0gpr[Cop0_Cause] & !0x300) | (value & 0x300)
            },

            Cop0_LLAddr => {
                value & 0x0000_0000_FFFF_FFFF // 32-bit register
            },

            Cop0_Status => {
                // TODO testing if little endian mode or 32-bit mode is ever used
                if (value & 0x0200_0000) != 0 {
                    panic!("CPU: unsupported little endian mode ${value:08X}");
                }

                if (value & 0x0000_0080) != 0x00 {
                    self.kernel_64bit_addressing = true;
                    //warn!(target: "CPU", "64-bit kernel addressing enabled ${value:08X}");
                } else {
                    self.kernel_64bit_addressing = false;
                    //debug!(target: "CPU", "32-bit kernel addressing enabled ${value:08X}");
                }

                if (value & 0x18) != 0 {
                    panic!("CPU: unsupported user or supervisor mode");
                }

                if (value & 0x800000) != 0 {
                    panic!("CPU: unsupported instruction trace mode");
                }

                if (value & 0x200000) != 0 {
                    panic!("CPU: exception vector change");
                }
                ///////////////////////////////////////////////////////////

                self.cop1.set_fr(((value >> 26) & 0x01) != 0); // notify COP1 on the FR bit change
                value & 0x0000_0000_FFF7_FFFF // 32-bit register, bit 19 always 0
            },

            Cop0_PErr => {
                value & 0xFF
            },

            // read-only registers
            Cop0_BadVAddr | Cop0_PRId  => {
                self.cp0gpr[register_number]
            },

            // TLB registers
            Cop0_EntryLo0 | Cop0_EntryLo1 => {
                // The datasheet says PFN is 20 bits wide but it seems like everywhere else,
                // including n64-systemtest, is saying it's 24 bits, so it's 24 bits.
                value & 0x0000_0000_3FFF_FFFF
            },

            Cop0_EntryHi => {
                value & 0xC000_00FF_FFFF_E0FF
            },

            Cop0_PageMask => {
                value & 0x0000_0000_01FF_E000
            },

            // unused registers
            7 | 21 | 22 | 23 | 24 | 25 | 31 | Cop0_CacheErr => {
                0
            },

            _ => { value },
        }
    }

    fn build_inst_cop(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        let copno = ((self.inst.v >> 26) & 0x03) as u64;
        let cop = match copno {
            0 => panic!("TODO"),
            1 => &mut self.cop1,
            _ => {
                // coprocessor_number in v_arg1_32
                letsgo!(assembler
                    ;   mov v_arg1_32, DWORD copno as _
                );

                letsexcept!(self, assembler, Cpu::coprocessor_unusable_exception_bridge);

                return CompileInstructionResult::Continue;
            },
        };

        // if the co-processor isn't enabled, generate an exception
        letsgo!(assembler
            ;   test DWORD [r_cp0gpr + (Cop0_Status*8) as i32], DWORD (0x1000_0000 << copno) as _
            ;   jnz >no_exception
        );

        // coprocessor_number in v_arg1_32
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD copno as _
        );

        letsexcept!(self, assembler, Cpu::coprocessor_unusable_exception_bridge);

        letsgo!(assembler
            ;no_exception:
        );

        if (self.inst.v & (1 << 25)) != 0 {
            trace!(target: "JIT-BUILD", "${:08X}[{:5}]: cop1 special ${:03b}_{:03b}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                     (self.inst.v & 0x3F) >> 3, self.inst.v & 0x07);

            // setup exception values for bridge
            letssetupexcept!(self, assembler, true);

            // opcode in 3nd argument
            letsgo!(assembler
                ;   mov v_arg2_32, DWORD self.inst.v as _
            );

            // check return value for errors
            letscop!(assembler, cop, Cpu::cop1_special_bridge);

            // check for exceptions
            letscheck!(assembler);
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => { // MFC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mfc1 r{}, fpr{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rt, self.inst.rd);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // register number rd in 3rd argument
                    letsgo!(assembler
                        ;   mov v_arg2_32, DWORD self.inst.rd as _
                    );

                    // result in eax
                    letscop!(assembler, cop, Cpu::cop1_mfc_bridge);

                    // check exceptions
                    letscheck!(assembler);

                    // store sign-extended result in rt
                    if self.inst.rt != 0 {
                        letsgo!(assembler
                            ;   movsxd v_tmp, eax
                            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
                        );
                    }
                },

                0b00_001 => { // DMFC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmfc1 r{}, fpr{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rt, self.inst.rd);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // register number rd in third argument
                    letsgo!(assembler
                        ;   mov v_arg2_32, DWORD self.inst.rd as _
                    );

                    // result in rax
                    letscop!(assembler, cop, Cpu::cop1_dmfc_bridge);

                    // check exceptions
                    letscheck!(assembler);

                    // store result in rt
                    if self.inst.rt != 0 {
                        letsgo!(assembler
                            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
                        );
                    }
                },

                0b00_010 => { // CFC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: cfc1 r{}, fpr{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rt, self.inst.rd);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // register number rd in third argument
                    letsgo!(assembler
                        ;   mov v_arg2_32, DWORD self.inst.rd as _
                    );

                    // result in eax
                    letscop!(assembler, cop, Cpu::cop1_cfc_bridge);

                    // check exceptions
                    letscheck!(assembler);

                    // store result in rt
                    if self.inst.rt != 0 {
                        letsgo!(assembler
                            ;   movsxd v_tmp2, eax
                            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp2
                        );
                    }
                },

                0b00_011 | 0b00_111 => { // dcfc1, dctc1 are unimplemented
                    // set cop pointer to v_arg1
                    letsgo!(assembler
                        ;   mov v_arg1, QWORD cop as *mut cop1::Cop1 as _
                    );

                    letsexcept!(self, assembler, Cpu::cop1_unimplemented_instruction_bridge);
                },

                0b00_100 => { // MTC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mtc1 fcr{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rd, self.inst.rt);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // 32-bits of rt in third arg, register number rd in fourth
                    if self.inst.rt == 0 {
                        letsgo!(assembler
                            ;   xor v_arg2_32, v_arg2_32
                        );
                    } else {
                        letsgo!(assembler
                            ;   mov v_arg2_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
                        );
                    }

                    letsgo!(assembler
                        ;   mov v_arg3_32, DWORD self.inst.rd as _
                    );

                    letscop!(assembler, cop, Cpu::cop1_mtc_bridge);

                    // check exceptions
                    letscheck!(assembler);
                },
  
                0b00_101 => { // DMTC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmtc1 fcr{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rd, self.inst.rt);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // contents of rt in third arg, register number rd in fourth
                    if self.inst.rt == 0 {
                        letsgo!(assembler
                            ;   xor v_arg2_32, v_arg2_32
                        );
                    } else {
                        letsgo!(assembler
                            ;   mov v_arg2, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                        );
                    }

                    letsgo!(assembler
                        ;   mov v_arg3_32, DWORD self.inst.rd as _
                    );

                    letscop!(assembler, cop, Cpu::cop1_dmtc_bridge);
                    
                    // check exceptions
                    letscheck!(assembler);
                },

                0b00_110 => { // CTC
                    trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ctc1 fcr{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                             self.inst.rd, self.inst.rt);

                    // setup exception values for bridge
                    letssetupexcept!(self, assembler, true);

                    // contents of rt in third arg, register number rd in fourth
                    if self.inst.rt == 0 {
                        letsgo!(assembler
                            ;   xor v_arg2_32, v_arg2_32
                        );
                    } else {
                        letsgo!(assembler
                            ;   mov v_arg2, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                        );
                    }

                    // fgpr index in 4th argument
                    letsgo!(assembler
                        ;   mov v_arg3_32, DWORD self.inst.rd as _
                    );

                    letscop!(assembler, cop, Cpu::cop1_ctc_bridge);

                    // check exceptions
                    letscheck!(assembler);
                },

                0b01_000 => {
                    // compute target jump and determine if the destination is within the current block
                    // (self.pc is pointing at the delay slot)
                    let dest = (self.pc - 4).wrapping_add((self.inst.signed_imm as u64) << 2);
                    trace!(target: "JIT-BUILD", "** cop1 conditional branch to ${:08X}", dest as u32);

                    self.next_is_delay_slot = true;

                    // put condition signal in register
                    cop.build_condition_signal_to_al(assembler);

                    let branch = (self.inst.v >> 16) & 0x1F;
                    match branch {
                        0b00_000 => { // BCzF
                            // branch if condition is false
                            letsgo!(assembler
                                ;   xor r_cond, r_cond    // clear condition register
                                ;   cmp al, BYTE 0i8 as _ // test al
                                ;   jne >skip_set         // if al was zero, skip
                                ;   inc r_cond            // set condition to true (take branch)
                                ;skip_set:
                            );

                            self.jit_conditional_branch = Some(((self.inst.signed_imm as i64) << 2, false, false)); // not a `likely` branch
                        },

                        0b00_001 => { // BCzT
                            // branch if condition is true
                            letsgo!(assembler
                                ;   xor r_cond, r_cond    // clear condition register
                                ;   cmp al, BYTE 0i8 as _ // test al
                                ;   je >skip_set          // if al was zero, skip
                                ;   inc r_cond            // set condition to true (take branch)
                                ;skip_set:
                            );

                            self.jit_conditional_branch = Some(((self.inst.signed_imm as i64) << 2, false, false)); // not a `likely` branch
                        },

                        0b00_010 => { // BCzFL
                            // branch if condition is false
                            letsgo!(assembler
                                ;   xor r_cond, r_cond    // clear condition register
                                ;   cmp al, BYTE 0i8 as _ // test al
                                ;   jne >skip_set         // if al was zero, skip
                                ;   inc r_cond            // set condition to true (take branch)
                                ;skip_set:
                            );

                            self.jit_conditional_branch = Some(((self.inst.signed_imm as i64) << 2, true, false)); // is a `likely` branch
                        },

                        0b00_011 => { // BCzTL
                            // branch if condition is true
                            letsgo!(assembler
                                ;   xor r_cond, r_cond    // clear condition register
                                ;   cmp al, BYTE 0i8 as _ // test al
                                ;   je >skip_set          // if al was zero, skip
                                ;   inc r_cond            // set condition to true (take branch)
                                ;skip_set:
                            );

                            self.jit_conditional_branch = Some(((self.inst.signed_imm as i64) << 2, true, false)); // is a `likely` branch
                        },

                        _ => panic!("JIT: unknown branch function 0b{:02b}_{:03b} (called on cop{})", branch >> 3, branch & 7, copno),
                    }
                },

                // unknown cop instructions go to build_inst_unknown
                _ => {
                    return self.build_inst_unknown(assembler);
                }
            }
        }

        CompileInstructionResult::Continue
    }

    // convert into inst_cop at some point
    // all the cop should implement a common cop trait (mfc/mtc/ctc/etc)
    fn inst_cop0(&mut self) -> Result<(), InstructionFault> {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_000 => { // MFC
                self.gpr[self.inst.rt] = (match self.inst.rd {
                    7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                        self.cp0gpr_latch
                    },

                    //Cop0_Random => self.cp0gpr[Cop0_Random] | self.cp0gpr_random_bit5,

                    _ => self.cp0gpr[self.inst.rd],
                } as i32) as u64;
            },

            0b00_001 => { // DMFC
                self.gpr[self.inst.rt] = match self.inst.rd {
                    7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                        self.cp0gpr_latch
                    },

                    //Cop0_Random => self.cp0gpr[Cop0_Random] | self.cp0gpr_random_bit5,

                    _ => self.cp0gpr[self.inst.rd],
                };
            },

            0b00_100 => { // MTC
                let val = (self.gpr[self.inst.rt] as i32) as u64;

                // latch value for unused register reads
                self.cp0gpr_latch = val;

                // fix bits of the various registers
                let old_val = self.cp0gpr[self.inst.rd];
                self.cp0gpr[self.inst.rd] = self.mask_cp0_register(self.inst.rd, val);

                // Setting IE, at this point if we're still executing an exception is pending
                if (self.inst.rd == Cop0_Status) && (old_val & 0x01) == 0 && (val & 0x01) == 0x01 {
                    if (self.cp0gpr[Cop0_Status] & 0x02) == 0 { // EXL must be clear to run a new exception
                        let pending_interrupts = (self.cp0gpr[Cop0_Cause] >> 8) as u8;
                        if (((self.cp0gpr[Cop0_Status] >> 8) as u8) & pending_interrupts) != 0 { // any enabled pending interrupts?
                            self.exception(ExceptionCode_Int, false)?;
                        }
                    }
                }
            },

            0b00_101 => { // DMTC
                let val = self.gpr[self.inst.rt];

                // save write value for unused register reads
                self.cp0gpr_latch = val;

                self.cp0gpr[self.inst.rd] = self.mask_cp0_register(self.inst.rd, val);
            },


            0b10_000..=0b11_111 => {
                let special = self.inst.v & 0x3F;
                match special {
                    0b000_001 => { // tlbr
                        let tlb = &self.tlb[(self.cp0gpr[Cop0_Index] & 0x1F) as usize];
                        let g = (tlb.entry_hi >> 12) & 0x01;
                        self.cp0gpr[Cop0_PageMask] = tlb.page_mask;
                        self.cp0gpr[Cop0_EntryHi]  = tlb.entry_hi & !(0x1000 | tlb.page_mask);
                        self.cp0gpr[Cop0_EntryLo1] = tlb.entry_lo1 | g;
                        self.cp0gpr[Cop0_EntryLo0] = tlb.entry_lo0 | g;
                        //info!(target: "CPU", "COP0: tlbr, index={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                        //            self.cp0gpr[Cop0_Index], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                    },

                    0b000_010 => { // tlbwi
                        debug!(target: "CPU", "COP0: tlbwi,  index={:2}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                                    self.cp0gpr[Cop0_Index], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                        self.set_tlb_entry(self.cp0gpr[Cop0_Index] & 0x1F);
                    },

                    0b000_110 => { // tlbwr
                        debug!(target: "CPU", "COP0: tlbwr, random={:2}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                                    self.cp0gpr[Cop0_Random], self.cp0gpr[Cop0_EntryHi], self.cp0gpr[Cop0_EntryLo0], self.cp0gpr[Cop0_EntryLo1], self.cp0gpr[Cop0_PageMask]);
                        self.set_tlb_entry(self.cp0gpr[Cop0_Random] & 0x1F);
                    },

                    0b001_000 => { // tlbp
                        //info!(target: "CPU", "COP0: tlbp, EntryHi=${:016X}", self.cp0gpr[Cop0_EntryHi]);
                        self.cp0gpr[Cop0_Index] = 0x8000_0000;
                        let entry_hi = self.cp0gpr[Cop0_EntryHi];
                        let asid = entry_hi & 0xFF;
                        for i in 0..32 {
                            let tlb = &self.tlb[i];

                            // if ASID matches or G is set, check this TLB entry
                            if (tlb.entry_hi & 0xFF) != asid && (tlb.entry_hi & 0x1000) == 0 { continue; }

                            // region must match
                            if ((tlb.entry_hi ^ entry_hi) & 0xC000_0000_0000_0000) != 0 { continue; }

                            // compare only VPN2 with the inverse of page mask
                            let tlb_vpn = (tlb.entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;
                            let ehi_vpn = (entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;

                            if tlb_vpn == ehi_vpn {
                                self.cp0gpr[Cop0_Index] = i as u64;
                                break;
                            }
                        }
                    },

                    0b011_000 => { // eret
                        assert!(!self.is_delay_slot); // ERET must not be in a delay slot TODO error gracefully

                        // If ERL bit is set, load the contents of ErrorEPC to the PC and clear the
                        // ERL bit. Otherwise, load the PC from EPC and clear the EXL bit.
                        if (self.cp0gpr[Cop0_Status] & 0x04) != 0 {
                            panic!("COP0: error bit set"); // TODO
                        } else {
                            self.pc = self.cp0gpr[Cop0_EPC];
                            //println!("ERET: returning from handler to ${:08X}", self.pc);

                            self.cp0gpr[Cop0_Status] &= !0x02;
                        }

                        // clear LLbit so that SC writes fail
                        self.llbit = false;

                        // eret can return to an invalid address, so we need to have
                        // current_instruction_pc correct just in case an exception occurs
                        let old_current_instruction_pc = self.current_instruction_pc;
                        self.current_instruction_pc = self.pc;
                        self.prefetch()?;
                        self.current_instruction_pc = old_current_instruction_pc;
                    },

                    _ => panic!("COP0: unknown cp0 function 0b{:03b}_{:03b}", special >> 3, special & 0x07),
                }
            },

            _ => panic!("CPU: unknown cop0 op: 0b{:02b}_{:03b} (0b{:032b})", cop0_op >> 3, cop0_op & 0x07, self.inst.v)
        }

        Ok(())
    }

    extern "sysv64" fn inst_cop0_bridge(cpu: *mut Cpu, inst: u32) -> i64 {
        let cpu = unsafe { &mut *cpu };

        // we're running inside a compiled block so self.inst isn't being used, we
        // can safely destroy it and call into rust code
        cpu.decode_instruction(inst);

        match cpu.inst_cop0() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_Int);
                cpu.jit_other_exception = true;
                exception_code as i64
            },

            Err(err) => {
                // TODO handle rrors
                todo!("inst_cop0 error = {:?}", err);
            }
        }
    }

    // convert into inst_cop at some point
    // all the cop should implement a common cop trait (mfc/mtc/ctc/etc)
    fn build_inst_cop0(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        let cop0_op = (self.inst.v >> 21) & 0x1F;
        match cop0_op {
            0b00_000 => { // MFC
                trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mfc0 c{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                         self.inst.rd, self.inst.rt);

                if self.inst.rt != 0 {
                    match self.inst.rd {
                        // undefined registers
                        7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                            letsgo!(assembler
                                ;   mov v_tmp, QWORD [r_cpu + offset_of!(Cpu, cp0gpr_latch) as i32]
                                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
                            );
                        },

                        // We need to catch reading from Count since it may not be updated yet,
                        // but reading Cop0_Compare doesn't change any state
                        Cop0_Count => {
                            // put the cycle count into v_arg1_32
                            letsgo!(assembler
                                ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]
                            );

                            // call update_count
                            letscall!(assembler, Cpu::update_cop0_count);

                            // return value in rax needs to go into rt
                            letsgo!(assembler
                                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
                            );
                        },

                        Cop0_Random => {
                            // we need to update Cop0_Random
                            letsgo!(assembler
                                ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]  // cycles_remaining
                                ;   mov v_arg2_32, BYTE 1 as _                // compute_num_steps
                            );

                            // return value is current Random value
                            letscall!(assembler, Cpu::update_cop0_random);

                            letsgo!(assembler
                                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
                            );
                        },

                        _ => {
                            letsgo!(assembler
                                ;   mov v_tmp_32, DWORD [r_cp0gpr + (self.inst.rd * 8) as i32]
                                ;   movsxd v_tmp2, v_tmp_32
                                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp2
                            );
                        },
                    }
                }

                CompileInstructionResult::Continue
            },

            0b00_001 => { // DMFC
                trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmfc0 c{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                         self.inst.rd, self.inst.rt);

                if self.inst.rt != 0 {
                    match self.inst.rd {
                        7 | 21 | 22 | 23 | 24 | 25 | 31 => {
                            letsgo!(assembler
                                ;   mov rax, QWORD [r_cpu + offset_of!(Cpu, cp0gpr_latch) as i32]
                            );
                        },

                        Cop0_Count => {
                            // TODO need to compute Count based on how many cycles have occurred,
                            // and then update our Count tracker.
                            letsgo!(assembler
                                ;   int3
                                ;   mov rax, DWORD 0x7EEE_3333u32 as _
                                ;   mov rax, QWORD [r_cp0gpr + (self.inst.rd * 8) as i32]
                            );
                        },

                        Cop0_Random => {
                            // we need to update Cop0_Random before reads
                            letsgo!(assembler
                                ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]  // cycles_remaining
                                ;   mov v_arg2_32, BYTE 1 as _                // compute_num_steps
                            );

                            letscall!(assembler, Cpu::update_cop0_random);

                            letsgo!(assembler
                                ;   mov rax, QWORD [r_cp0gpr + (self.inst.rd * 8) as i32]
                            );
                        },

                        _ => {
                            letsgo!(assembler
                                ;   mov rax, QWORD [r_cp0gpr + (self.inst.rd * 8) as i32]
                            );
                        },
                    }

                    letsgo!(assembler
                        ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
                    );
                }

                CompileInstructionResult::Continue
            },

            0b00_100 => { // MTC
                trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mtc0 c{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                         self.inst.rd, self.inst.rt);

                match self.inst.rd {
                    // Writing Count means we need to recompute cp0_compare_distance
                    // Writing Compare to a new value needs cp0_compare_distance updated, which requires Cop0_Count
                    // And both cases may affect how many cycles can be run in the rest of this block
                    Cop0_Count => {
                        letsgo!(assembler
                            // cycle count in v_arg1_32
                            ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]
                            // value in v_arg2_32
                            ;   mov v_arg2_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]    // zero high 32-bits of r8
                            // copy to cp0gpr_latch
                            ;   movsxd v_tmp, v_arg2_32
                            ;   mov QWORD [r_cpu + offset_of!(Cpu, cp0gpr_latch) as i32], v_tmp
                        );

                        extern "sysv64" fn write_count(cpu: *mut Cpu, mut cycles_remaining: i32, write_value: u32) -> i32 {
                            let cpu = unsafe { &mut *cpu };
                            // update Count
                            cpu.cp0gpr[Cop0_Count] = write_value as u64;
                            // update the count tracker, which requires the current step number
                            let cur_steps = cpu.num_steps + (cpu.jit_run_limit as i32 - cycles_remaining) as u64;
                            cpu.cp0_count_tracker = cur_steps & !1;
                            // update the compare distance
                            cpu.cp0_compare_distance = (cpu.cp0gpr[Cop0_Compare] as u32).wrapping_sub(write_value);
                            if cpu.cp0_compare_distance == 0 { cpu.cp0_compare_distance = 0x7FFF_FFFF; }
                            // compute the amount of Count that could increase in this run
                            let potential_count_increment = (cur_steps + cycles_remaining as u64 - cpu.cp0_count_tracker) >> 1;
                            // if Compare would trigger within this run, we need to reduce the
                            // length of the run, otherwise we can just keep executing
                            if (cpu.cp0_compare_distance as u64) < potential_count_increment {
                                let max_cycles_left = (2 * cpu.cp0_compare_distance) as u64;
                                assert!(cpu.jit_run_limit >= max_cycles_left);
                                // reduce self.jit_run_limit and cycles_remaining by delta
                                let delta = cpu.jit_run_limit - max_cycles_left;
                                cpu.jit_run_limit -= delta;
                                cycles_remaining -= delta as i32;
                            }
                            // new cycles remaining value
                            cycles_remaining
                        }

                        letscall!(assembler, write_count);

                        // copy eax to s_cycle_count
                        letsgo!(assembler
                            ;   mov DWORD [rsp+s_cycle_count], eax
                        );
                    },

                    // Writing Compare to a new value needs cp0_compare_distance updated, which requires Cop0_Count recomputed
                    Cop0_Compare => {
                        letsgo!(assembler
                            // cycle count in v_arg1_32
                            ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]
                            // value in v_arg2_32
                            ;   mov v_arg2_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]    // zero high 32-bits of r8
                            // copy to cp0gpr_latch
                            ;   movsxd v_tmp, v_arg2_32
                            ;   mov QWORD [r_cpu + offset_of!(Cpu, cp0gpr_latch) as i32], v_tmp
                        );

                        extern "sysv64" fn write_compare(cpu: *mut Cpu, mut cycles_remaining: i32, write_value: u32) -> i32 {
                            // recompute Count
                            let count = Cpu::update_cop0_count(cpu, cycles_remaining) as u32;
                            // update Compare
                            let cpu = unsafe { &mut *cpu };
                            cpu.cp0gpr[Cop0_Compare] = write_value as u64;
                            // update the compare distance
                            cpu.cp0_compare_distance = (cpu.cp0gpr[Cop0_Compare] as u32).wrapping_sub(count);
                            if cpu.cp0_compare_distance == 0 { cpu.cp0_compare_distance = 0x7FFF_FFFF; }
                            // compute the amount of Count that could increase in this run
                            let cur_steps = cpu.num_steps + (cpu.jit_run_limit as i32 - cycles_remaining) as u64;
                            let potential_count_increment = (cur_steps + cycles_remaining as u64 - cpu.cp0_count_tracker) >> 1;
                            // if Compare would trigger within this run, we need to reduce the
                            // length of the run, otherwise we can just keep executing
                            if (cpu.cp0_compare_distance as u64) < potential_count_increment {
                                let max_cycles_left = (2 * cpu.cp0_compare_distance) as u64;
                                assert!(cpu.jit_run_limit >= max_cycles_left);
                                // reduce self.jit_run_limit and cycles_remaining by delta
                                let delta = cpu.jit_run_limit - max_cycles_left;
                                cpu.jit_run_limit -= delta;
                                cycles_remaining -= delta as i32;
                            }
                            // clear pending timer interrupt (see 6.3.4)
                            cpu.cp0gpr[Cop0_Cause] &= !(InterruptCode_Timer << 8);
                            // new cycles remaining value
                            cycles_remaining
                        }

                        letscall!(assembler, write_compare);

                        // copy eax to s_cycle_count
                        letsgo!(assembler
                            ;   mov DWORD [rsp+s_cycle_count], eax
                        );
                    },

                    Cop0_Random | Cop0_Wired => {
                        // we need to update Cop0_Random when these registers change, since the
                        // starting value will change and so will the cp0_random_tracker value
                        // 'true' in v_arg2_8l, compute_num_steps
                        letsgo!(assembler
                            ;   mov v_arg1_32, DWORD [rsp+s_cycle_count]  // cycles_remaining
                            ;   mov v_arg2_32, BYTE 1 as _                // compute_num_steps
                        );

                        letscall!(assembler, Cpu::update_cop0_random);

                        letssetupexcept!(self, assembler, false);

                        // opcode in arg 2
                        letsgo!(assembler
                            ;   mov v_arg1_32, DWORD self.inst.v as _
                        );

                        // do the write
                        letscall!(assembler, Cpu::inst_cop0_bridge);
                        letscheck!(assembler);
                    },

                    _ => {
                        // setup exception values
                        letssetupexcept!(self, assembler, false);

                        // opcode in arg 2
                        letsgo!(assembler
                            ;   mov v_arg1_32, DWORD self.inst.v as _
                        );

                        // call self.inst_cop0_bridge(Cpu, inst)
                        letscall!(assembler, Cpu::inst_cop0_bridge);

                        // check for exceptions
                        letscheck!(assembler);
                    }
                }

                CompileInstructionResult::Continue
            },

            0b00_101 => { // DMTC
                trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmtc0 c{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                         self.inst.rd, self.inst.rt);

                match self.inst.rd {
                    Cop0_Count => todo!(),

                    Cop0_Random | Cop0_Wired => todo!(),

                    _ => {
                        // setup exception values
                        letssetupexcept!(self, assembler, false);

                        // opcode in second argument
                        letsgo!(assembler
                            ;   mov v_arg1_32, DWORD self.inst.v as _
                        );

                        // call self.inst_cop0_bridge(Cpu, inst)
                        letscall!(assembler, Cpu::inst_cop0_bridge);
                    },
                }

                CompileInstructionResult::Continue
            },

            0b10_000..=0b11_111 => {
                let special = self.inst.v & 0x3F;
                match special {
                    0b000_001 => { // tlbr
                        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: tlbr", self.current_instruction_pc as u32, self.jit_current_assembler_offset);

                        // TODO this could be done in assembly, but tlbr isn't a commonly executed instruction
                        extern "sysv64" fn tlbr_bridge(cpu: *mut Cpu) {
                            let cpu = unsafe { &mut *cpu };

                            let tlb = &cpu.tlb[(cpu.cp0gpr[Cop0_Index] & 0x1F) as usize];
                            let g = (tlb.entry_hi >> 12) & 0x01;
                            cpu.cp0gpr[Cop0_PageMask] = tlb.page_mask;
                            cpu.cp0gpr[Cop0_EntryHi]  = tlb.entry_hi & !(0x1000 | tlb.page_mask);
                            cpu.cp0gpr[Cop0_EntryLo1] = tlb.entry_lo1 | g;
                            cpu.cp0gpr[Cop0_EntryLo0] = tlb.entry_lo0 | g;
                            trace!(target: "CPU", "COP0: tlbr, index={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                                        cpu.cp0gpr[Cop0_Index], cpu.cp0gpr[Cop0_EntryHi], cpu.cp0gpr[Cop0_EntryLo0], cpu.cp0gpr[Cop0_EntryLo1], cpu.cp0gpr[Cop0_PageMask]);
                        }

                        letscall!(assembler, tlbr_bridge);

                        CompileInstructionResult::Continue
                    },

                    0b000_010 => { // tlbwi
                        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: tlbwi", self.current_instruction_pc as u32, self.jit_current_assembler_offset);

                        extern "sysv64" fn set_tlb_entry_bridge(cpu: *mut Cpu) {
                            let cpu = unsafe { &mut *cpu };
                            trace!(target: "CPU", "COP0: tlbwi, index={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                                        cpu.cp0gpr[Cop0_Index], cpu.cp0gpr[Cop0_EntryHi], cpu.cp0gpr[Cop0_EntryLo0], cpu.cp0gpr[Cop0_EntryLo1], cpu.cp0gpr[Cop0_PageMask]);
                            cpu.set_tlb_entry(cpu.cp0gpr[Cop0_Index] & 0x1F);
                        }

                        letscall!(assembler, set_tlb_entry_bridge);

                        CompileInstructionResult::Continue
                    },

                    0b000_110 => { // tlbwr
                        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: tlbwr", self.current_instruction_pc as u32, self.jit_current_assembler_offset);

                        extern "sysv64" fn set_tlb_entry_bridge(cpu: *mut Cpu) {
                            let cpu = unsafe { &mut *cpu };
                            info!(target: "CPU", "COP0: tlbwr, random={}, EntryHi=${:016X}, EntryLo0=${:016X}, EntryLo1=${:016X}, PageMask=${:016X}",
                                        cpu.cp0gpr[Cop0_Random], cpu.cp0gpr[Cop0_EntryHi], cpu.cp0gpr[Cop0_EntryLo0], cpu.cp0gpr[Cop0_EntryLo1], cpu.cp0gpr[Cop0_PageMask]);
                            cpu.set_tlb_entry(cpu.cp0gpr[Cop0_Random] & 0x1F);
                        }

                        letscall!(assembler, set_tlb_entry_bridge);

                        CompileInstructionResult::Continue
                    },

                    0b001_000 => { // tlbp
                        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: tlbp", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
                        //self.cp0gpr[Cop0_Index] = 0x8000_0000;
                        //let entry_hi = self.cp0gpr[Cop0_EntryHi];
                        //let asid = entry_hi & 0xFF;
                        //for i in 0..32 {
                        //    let tlb = &self.tlb[i];

                        //    // if ASID matches or G is set, check this TLB entry
                        //    if (tlb.entry_hi & 0xFF) != asid && (tlb.entry_hi & 0x1000) == 0 { continue; }

                        //    // region must match
                        //    if ((tlb.entry_hi ^ entry_hi) & 0xC000_0000_0000_0000) != 0 { continue; }

                        //    // compare only VPN2 with the inverse of page mask
                        //    let tlb_vpn = (tlb.entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;
                        //    let ehi_vpn = (entry_hi & 0xFF_FFFF_E000) & !tlb.page_mask;

                        //    if tlb_vpn == ehi_vpn {
                        //        self.cp0gpr[Cop0_Index] = i as u64;
                        //        break;
                        //    }
                        //}

                        // TODO this is just a wrapper so we can println!() the instruction
                        extern "sysv64" fn tlbp_bridge(cpu: *mut Cpu, inst: u32) -> i64 {
                            //let cpu = unsafe { &mut *cpu };
                            trace!(target: "CPU", "COP0: tlbp, EntryHi=${:016X}", unsafe { (*cpu).cp0gpr[Cop0_EntryHi] });
                            Cpu::inst_cop0_bridge(cpu, inst)
                        }

                        // opcode in second argument
                        letsgo!(assembler
                            ;   mov v_arg1_32, DWORD self.inst.v as _
                        );

                        letscall!(assembler, tlbp_bridge);

                        CompileInstructionResult::Continue
                    },

                    0b011_000 => { // eret
                        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: eret", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
                        assert!(!self.is_delay_slot); // ERET must not be in a delay slot TODO error gracefully

                        letsgo!(assembler
                            // If ERL bit is set, load the contents of ErrorEPC to the PC and clear the
                            // ERL bit. Otherwise, load the PC from EPC and clear the EXL bit.
                            ;   test BYTE [r_cp0gpr + (Cop0_Status * 8) as i32], BYTE 0x04 as _
                            ;   jz >no_erl
                            ;   mov rax, QWORD 0x1111_5414_0000_AAAAu64 as _
                            ;   int3
                            ;no_erl:
                            // copy Cop0_EPC to the jump target
                            ;   mov v_tmp, QWORD [r_cp0gpr + (Cop0_EPC * 8) as i32]
                            ;   mov QWORD [rsp+s_jump_target], v_tmp
                            // clear bit 0x02 of Cop0_Status
                            ;   and QWORD [r_cp0gpr + (Cop0_Status * 8) as i32], DWORD !0x0000_0002u32 as _ // clear bit (sign-extended DWORD)
                            // set llbit to false so SC instructions fail
                            ;   mov BYTE [r_cpu + offset_of!(Cpu, llbit) as i32], BYTE 0x00u8 as _
                        );

                        self.jit_jump_no_delay = true;

                        // there could be code after an eret but usually eret isn't inside a conditional block?
                        CompileInstructionResult::Stop
                    },

                    _ => {
                        self.build_inst_unknown(assembler)
                    }
                }
            },

            // unknown cop0 instructions go to build_inst_unknown
            _ => {
                self.build_inst_unknown(assembler)
            }
        }
    }

    // Count has not been updated within the currently executing block yet, so we need to update that
    // However, self.num_steps has also not been updated with the new cycles already ran in the block
    // So to figure that out first, we need the run limit and how many cycles have executed so far.
    // then we can update Count and cp0_count_tracker
    #[inline(always)]
    extern "sysv64" fn update_cop0_count(cpu: *mut Cpu, cycles_remaining: i32) -> u64 {
        let cpu = unsafe { &mut *cpu };
        // compute the current step number
        let cur_steps = cpu.num_steps + (cpu.jit_run_limit as i32 - cycles_remaining) as u64;
        // compute the increment to Count and add it
        let cp0_count_increment = (cur_steps - cpu.cp0_count_tracker) >> 1;
        cpu.cp0gpr[Cop0_Count] = ((cpu.cp0gpr[Cop0_Count] as u32) + (cp0_count_increment as u32)) as u64;
        // update the count tracker (so the next update is correct)
        let new_cp0_count_tracker = cur_steps & !1;
        let cp0_count_tracker_delta = new_cp0_count_tracker - cpu.cp0_count_tracker;
        cpu.cp0_count_tracker = new_cp0_count_tracker;
        // reduce the cp0_compare_distance by the distance added to cp0_count_tracker
        cpu.cp0_compare_distance = cpu.cp0_compare_distance.saturating_sub(cp0_count_tracker_delta as u32);
        // return the computed Count value in rax, sign-extended
        (cpu.cp0gpr[Cop0_Count] as i32) as u64
    }

    // Similar to Cop0_Count, Random has not been updated within the currently executing block
    #[inline(always)]
    extern "sysv64" fn update_cop0_random(cpu: *mut Cpu, cycles_remaining: i32, compute_num_steps: bool) -> u64 {
        let cpu = unsafe { &mut *cpu };
        // compute the current step number
        let cur_steps = if !compute_num_steps { cpu.num_steps } else { cpu.num_steps + (cpu.jit_run_limit as i32 - cycles_remaining) as u64 };
        // Cop0_Random decrements at 1 per instruction, but wraps between 31 and Wired
        let mut cp0_random_decrement = cur_steps - cpu.cp0_random_tracker;
        if cpu.cp0gpr_random_delay > 0 { 
            if (cp0_random_decrement as i64) < cpu.cp0gpr_random_delay {
                cpu.cp0gpr_random_delay -= cp0_random_decrement as i64;
                cp0_random_decrement = 0;
            } else {
                cp0_random_decrement -= cpu.cp0gpr_random_delay as u64;
                cpu.cp0gpr_random_delay = 0;
                // need to perform one decrement on the cycle that cp0gpr_random_delay becomes 0
                cp0_random_decrement += 1;
            }
        }

        // size of the Random/Wired cycle
        let cp0_random_distance = if cpu.cp0gpr[Cop0_Wired] <= 31 {
            32 - cpu.cp0gpr[Cop0_Wired]
        } else {
            32 + (64 - cpu.cp0gpr[Cop0_Wired])
        };

        // remove all full random cycles
        cp0_random_decrement = cp0_random_decrement % cp0_random_distance;

        // calculate the # steps until we would need to reset Random
        let cp0_random_remaining = if cpu.cp0gpr[Cop0_Random] < cpu.cp0gpr[Cop0_Wired] {
            // Random has to go to 0, and then from 63 down to wired, plus 1 cycle for reset
            cpu.cp0gpr[Cop0_Random] + 1 + (63 - cpu.cp0gpr[Cop0_Wired])
        } else {
            // Random just has to drop to Wired, plus 1 cycle for reset
            cpu.cp0gpr[Cop0_Random] - cpu.cp0gpr[Cop0_Wired]
        };

        // will the remaining decrement be enough to reset?
        if cp0_random_decrement >= (cp0_random_remaining + 1) {
            // reset it
            cpu.cp0gpr[Cop0_Random] = 31;
            cp0_random_decrement -= cp0_random_remaining + 1; // 1 cycle used for reset
        }

        // update cp0_random_tracker jic this function is called repeatedly
        cpu.cp0_random_tracker = cur_steps;

        // now we just need to subtract cp0_random_decrement from Random with underflow
        cpu.cp0gpr[Cop0_Random] = cpu.cp0gpr[Cop0_Random].wrapping_sub(cp0_random_decrement) & 0x3F;
        cpu.cp0gpr[Cop0_Random]
    }

    fn inst_cop2(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x4000_0000) == 0 {
            self.coprocessor_unusable_exception(2)?;
            return Ok(());
        }

        if (self.inst.v & (1 << 25)) != 0 {
            // no special functions on COP2
            self.coprocessor_unusable_exception(2)?;
            Ok(())
        } else {
            let func = (self.inst.v >> 21) & 0x0F;
            match func {
                0b00_000 => { // MFC - move control word from coprocessor
                    self.gpr[self.inst.rt] = (self.cp2gpr_latch as i32) as u64;
                    Ok(())
                },

                0b00_001 => { // DMFC
                    self.gpr[self.inst.rt] = self.cp2gpr_latch;
                    Ok(())
                },

                0b00_010 => { // CFC
                    self.gpr[self.inst.rt] = (self.cp2gpr_latch as i32) as u64;
                    Ok(())
                },

                0b00_011 | 0b00_111 => { // dcfc2, dctc2 are unimplemented
                    self.reserved_instruction_exception(2)?;
                    // set the cop in the Cause register
                    self.cp0gpr[Cop0_Cause] = (self.cp0gpr[Cop0_Cause] & !0x3000_0000) | 0x2000_0000;
                    Ok(())
                },

                0b00_100 => { // MTC
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b00_101 => { // DMTC
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b00_110 => { // CTC - move control word to coprocessor
                    self.cp2gpr_latch = self.gpr[self.inst.rt];
                    Ok(())
                },

                0b01_000 => { // No branches on COP2
                    warn!(target: "COP2", "no branches on COP2");
                    Err(InstructionFault::Unimplemented)
                },

                _ => panic!("CPU: unknown cop2 function 0b{:02b}_{:03b}", func >> 3, func & 7),
            }
        }
    }

    extern "sysv64" fn inst_cop2_bridge(cpu: *mut Cpu, inst: u32) -> u32 {
        let cpu = unsafe { &mut *cpu };
        // we're running inside a compiled block so self.inst isn't being used, we
        // can safely destroy it and call into rust code
        cpu.decode_instruction(inst);
        match cpu.inst_cop2() {
            Ok(_) => 0,

            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_CpU || exception_code == ExceptionCode_RI);
                cpu.jit_other_exception = true;
                exception_code as u32
            },

            Err(err) => {
                // TODO handle rrors
                todo!("inst_cop2 error = {:?}", err);
            }
        }
    }

    fn build_inst_cop2(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: cop2", self.current_instruction_pc as u32, self.jit_current_assembler_offset);

        // setup for exceptions
        letssetupexcept!(self, assembler, false);

        // opcode in arg 2
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        // call self.inst_cop2_bridge(Cpu, inst)
        letscall!(assembler, Cpu::inst_cop2_bridge);

        // check exceptions
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_beq(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn build_inst_beq(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsbranch!(self, assembler, "beq", jne, false, true);
        CompileInstructionResult::Continue
    }

    fn inst_beql(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] == self.gpr[self.inst.rt];
        self.branch_likely(condition)
    }

    fn build_inst_beql(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsbranch!(self, assembler, "beql", jne, true, true);
        CompileInstructionResult::Continue
    }

    fn inst_bgtz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) > 0;
        self.branch(condition);
        Ok(())
    }

    fn build_inst_bgtz(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgtz only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgtz", jle, false, false); // jump if less than or equal to 0 to not take the branch, 
                                                                 // branch not taken for bgtz r0 
        CompileInstructionResult::Continue
    }

    fn inst_blez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch(condition);
        Ok(())
    }

    fn build_inst_blez(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. blez only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "blez", jg, false, true); // jump if greater than 0 to not branch, 
                                                               // branch taken for blez r0 
        CompileInstructionResult::Continue
    }

    fn inst_blezl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) <= 0;
        self.branch_likely(condition)
    }

    fn build_inst_blezl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. blez only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "blezl", jg, true, true); // jump if greater than 0 to not branch, 
                                                               // branch taken for blez r0 
        CompileInstructionResult::Continue
    }

    fn inst_bgtzl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) > 0;
        self.branch_likely(condition)
    }

    fn build_inst_bgtzl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgtzl only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgtzl", jle, true, false); // jump if less than or equal to 0 to not take the branch, 
                                                                 // branch not taken for bgtz r0 
        CompileInstructionResult::Continue
    }

    fn inst_bne(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch(condition);
        Ok(())
    }

    fn build_inst_bne(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsbranch!(self, assembler, "bne", je, false, false);
        CompileInstructionResult::Continue
    }

    fn inst_bnel(&mut self) -> Result<(), InstructionFault> {
        let condition = self.gpr[self.inst.rs] != self.gpr[self.inst.rt];
        self.branch_likely(condition)
    }

    fn build_inst_bnel(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsbranch!(self, assembler, "bnel", je, true, false);
        CompileInstructionResult::Continue
    }

    fn inst_cache(&mut self) -> Result<(), InstructionFault> {
        Ok(())
    }

    fn build_inst_cache(&mut self, _: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: cache", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
        CompileInstructionResult::Continue
    }

    fn inst_daddi(&mut self) -> Result<(), InstructionFault> {
        // integer overflow exception occurs with DADDI, unlike DADDIU
        let rs = self.gpr[self.inst.rs] as u64;
        let rt = self.inst.signed_imm;
        let result = rs.wrapping_add(rt);

        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000) != 0 {
            //println!("CPU: daddi overflow detected: rs=${:16} imm=${:04X} result=${:16}", rs, rt, result);
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rt] = result;
        }
        Ok(())
    }

    fn build_inst_daddi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: addi r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rs == 0 { // source is zero, copy over the immediate
            if self.inst.rt != 0 { // don't overwrite r0
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], DWORD self.inst.signed_imm as u32 as _
                );
            }
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
            );

            // skip add and checking overflow if signed_imm is 0
            if self.inst.signed_imm != 0 {
                letsgo!(assembler
                    ;   add v_tmp, DWORD self.inst.signed_imm as u32 as _ // add sign-extended signed_imm to 64-bit v_tmp
                    ;   jno >no_overflow
                );

                letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

                letsgo!(assembler
                    ;no_overflow:
                );
            }

            // don't overwrite r0
            if self.inst.rt != 0 {
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


   fn inst_daddiu(&mut self) -> Result<(), InstructionFault> {
        // no integer overflow exception occurs with DADDIU
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        Ok(())
    }

    fn build_inst_daddiu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: daddiu r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rt != 0 { // if destination is zero, no-op
            if self.inst.rs == 0 { // We can skip the add and use a move
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], DWORD self.inst.signed_imm as u32 as _ // sign-extends
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs*8) as i32]      // put rs into v_tmp
                    ;   add v_tmp, DWORD self.inst.signed_imm as u32 as _       // add sign-extended signed_imm with overflow 
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], v_tmp      // .
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn inst_j(&mut self) -> Result<(), InstructionFault> {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);

        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn build_inst_j(&mut self, _assembler: &mut Assembler) -> CompileInstructionResult {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: j ${:08X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, dest);

        if self.is_delay_slot {
            return CompileInstructionResult::Cant;
        }

        // first value of jit_conditional_branch is relative to the delay slot instruction, so since
        // `J` is absolute, compute the relative target
        let relative_branch = (dest as i64).wrapping_sub(self.next_instruction_pc as i64);
        self.jit_conditional_branch = Some((relative_branch, false, true)); // not a `likely` branch

        self.next_is_delay_slot = true;

        CompileInstructionResult::Continue
    }


    fn inst_jal(&mut self) -> Result<(), InstructionFault> {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);

        self.gpr[31] = self.pc;
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;
        Ok(())
    }

    fn build_inst_jal(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        let dest = ((self.pc - 4) & 0xFFFF_FFFF_F000_0000) | ((self.inst.target << 2) as u64);
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: jal ${:08X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, dest as u32);

        if self.is_delay_slot {
            return CompileInstructionResult::Cant;
        }

        // always set r31 to self.pc
        if (self.pc & 0xFFFF_FFFF_8000_0000) == 0xFFFF_FFFF_8000_0000 {
            letsgo!(assembler
                ;   mov QWORD [r_gpr + (31 * 8) as i32], DWORD self.pc as i32 as _
            );
        } else {
            letsgo!(assembler
                ;   mov rax, QWORD self.pc as i64 as _
                ;   mov QWORD [r_gpr + (31 * 8) as i32], rax
            );
        }

        // first value of jit_conditional_branch is relative to the delay slot instruction, so since
        // `J` is absolute, compute the relative target
        let relative_branch = (dest as i64).wrapping_sub(self.next_instruction_pc as i64);
        self.jit_conditional_branch = Some((relative_branch, false, true)); // not a `likely` branch

        self.next_is_delay_slot = true;

        CompileInstructionResult::Continue
    }


    fn inst_lb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = (self.read_u8(address as usize)? as i8) as u64;
        Ok(())
    }

    fn build_inst_lb(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lb r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exceptions
        letssetupexcept!(self, assembler, false);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u8_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // call read_u8()
        letscall!(assembler, Cpu::read_u8_bridge);

        // check for exceptions
        letscheck!(assembler);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u8_bridge() is in al, and needs to be sign extended into rt
            letsgo!(assembler
                ;   movsx v_tmp, al   // sign-extend 8-bit result to 64-bits
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_lbu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.gpr[self.inst.rt] = self.read_u8(address as usize)? as u64;
        Ok(())
    }

    fn build_inst_lbu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lbu r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exceptions
        letssetupexcept!(self, assembler, false);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u8_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // call read_u8()
        letscall!(assembler, Cpu::read_u8_bridge);

        // check for exceptions
        letscheck!(assembler);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u8_bridge() is in al, needs to be truncated to 8-bit and zero-extended
            letsgo!(assembler
                ;   movzx v_tmp, al     // move with zero-extendion
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn inst_ld(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, false)?;
        }

        self.gpr[self.inst.rt] = self.read_u64(address as usize)?;
        Ok(())
    }

    fn build_inst_ld(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ld r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            ;   test v_arg1_8l, BYTE 0x07 as _   // check if address is valid (low three bits 000)            
            ;   jz >valid                        // .
            ;   mov rax, QWORD 0x0000_1234_0000_4444u64 as _ // search code
            ;   int3                       // TODO call self.address_exception
            ;valid:
        );

        // v_arg1 contains address
        letscall!(assembler, Cpu::read_u64_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u64_bridge() is in rax
            letsgo!(assembler
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_ldl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07)?;

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 0 {
            mem
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX >> (64 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = new; 
        Ok(())
    }

    fn build_inst_ldl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ldl r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x07u8 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            ;   and v_arg1, DWORD !0x07u32 as _    // sign-exnteded imm32
        );

        // call read_u32()
        letscall!(assembler, Cpu::read_u64_bridge);

        // check for exceptions
        letscheck!(assembler);

        // eax contains the result
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   je >done
            // otherwise, take the lower bits of rt and or with the low bits of rax shifted into position
            ;   mov cl, BYTE [rsp+s_tmp0]             // shift amount
            ;   shl rax, cl                           // shift mem into position
            ;   mov cl, BYTE 64 as _                  // compute 64-shift into cl
            ;   sub cl, BYTE [rsp+s_tmp0]             // .
            ;   mov v_arg1, DWORD 0xFFFF_FFFFu32 as _ // constant mask (sign extended) shifted by 32-shift
            ;   shr v_arg1, cl                        // .
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]  // lower bits of rt
            ;   and v_tmp, v_arg1                                     // .
            ;   or rax, v_tmp                        // result
            ;done:
            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
        );

        CompileInstructionResult::Continue
    }

    fn inst_ldr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u64 at the specified address
        // (perhaps an optimization would be only read one word if address & 0x4 is set
        let mem = self.read_u64(address & !0x07)?;

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 56 { // handle case where no shift occurs
            mem 
        } else {
            (self.gpr[self.inst.rt] & (u64::MAX << (8 + shift))) | (mem >> (56 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = new; 
        Ok(())
    }

    fn build_inst_ldr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ldr r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x07u8 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            ;   and v_arg1, DWORD !0x07u32 as _    // sign-exnteded imm32
        );

        // call read_u64()
        letscall!(assembler, Cpu::read_u64_bridge);

        // check for exceptions
        letscheck!(assembler);

        // eax contains the result
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 56u8 as _
            ;   je >done
            // otherwise, take the lower bits of rt and or with the low bits of rax shifted into position
            ;   mov cl, BYTE 56 as _                  // compute 56-shift
            ;   sub cl, BYTE [rsp+s_tmp0]             // .
            ;   shr rax, cl                           // shift mem into position
            ;   mov cl, BYTE 8 as _                   // compute 8+shift
            ;   add cl, BYTE [rsp+s_tmp0]             // .
            ;   mov v_arg1, DWORD 0xFFFF_FFFFu32 as _ // constant mask (sign extended) shifted by 8+shift
            ;   shl v_arg1, cl                        // .
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]  // lower bits of rt
            ;   and v_tmp, v_arg1                                     // .
            ;   or rax, v_tmp                        // result
            ;done:
            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
        );

        CompileInstructionResult::Continue
    }

    fn inst_lh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        if (address & 0x01) != 0 {
            self.address_exception(address, false)?;
        } else {
            self.gpr[self.inst.rt] = (self.read_u16(address as usize)? as i16) as u64;
        }

        Ok(())
    }

    fn build_inst_lh(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lh r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u16_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            ;   test v_arg1_8l, BYTE 0x01  // check for valid address
            ;   jz >valid
            ;   mov rax, QWORD 0xCCEC_BBEB_AAEA_0010u64 as _ // search code
            ;   int3 // call address_exception
            ;valid:
        );

        // v_arg1 contains address
        letscall!(assembler, Cpu::read_u16_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u16_bridge() is in eax, and needs to be sign extended into rt
            letsgo!(assembler
                ;   movsx v_tmp, ax   // sign-extend 16-bit ax to 64-bits
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_lhu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        if (address & 0x01) != 0 {
            self.address_exception(address, false)?;
        }

        self.gpr[self.inst.rt] = self.read_u16(address as usize)? as u64;
        Ok(())
    }
    
    fn build_inst_lhu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lhu r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u16_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            ;   test v_arg1_8l, BYTE 0x01 // check for valid address
            ;   jz >valid
            ;   mov rax, QWORD 0xAAEA_0010_DDED_BBCBu64 as _ // search code
            ;   int3 // call address_exception
            ;valid:
        );

        // v_arg1 contains address
        letscall!(assembler, Cpu::read_u16_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u16_bridge() is in eax, and needs to be sign extended into rt
            letsgo!(assembler
                ;   movzx v_tmp, ax   // zero-extend 16-bit ax to 64-bits
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_ll(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (virtual_address & 0x03) != 0 {
            self.address_exception(virtual_address, false)?;
        }

        // translate the address here
        let address = match self.translate_address(virtual_address, true, false)? {
            Some(address) => address,
            None => {
                // this shouldn't happen
                return self.address_exception(virtual_address, false);
            }
        };

        self.gpr[self.inst.rt] = (self.read_u32_phys(address)? as i32) as u64;

        // the "linked part" sets the LLAddr register in cop0 to the physical address
        // of the read, and the LLbit to 1
        // the LLAddr register only stores bits 31:4 of the address at bit 0 of the register
        self.cp0gpr[Cop0_LLAddr] = address.physical_address >> 4;
        self.llbit = true;

        Ok(())
    }

    fn build_inst_ll(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ll r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            // setup arguments to address_exception_bridge _and_ translate_address (v_arg2_32 = false)
            ;   xor v_arg2_32, v_arg2_32   // .
            ;   test v_arg1_8l, BYTE 0x03  // check if address is valid (low two bits 00)
            ;   jz >valid                  // .
        );

        // generate the exception
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        letsgo!(assembler
            ;   valid:
        );

        // setup call to translate_address() -- v_arg1 still contains virtual address and r8 is is_write=false
        letscall!(assembler, Cpu::translate_address_bridge);

        // check if exception occurred
        letscheck!(assembler);

        // rax now has the physical_address, store in Cop0_LLAddr
        letsgo!(assembler
            // put the physical address into v_arg1 for read_u32_phys_bridge
            ;   mov v_arg1, rax
            // shift physical_address right by 4 for Cop0_LLAddr
            ;   shr rax, BYTE 4 as _    
            ;   mov QWORD [r_cp0gpr + (Cop0_LLAddr * 8) as i32], rax
            // set llbit true
            ;   mov BYTE [r_cpu + offset_of!(Cpu, llbit) as i32], BYTE 1i8 as _
        );

        // physical_address in v_arg1
        letscall!(assembler, Cpu::read_u32_phys_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u32_phys_bridge() is in eax, and needs to be sign extended into rt
            letsgo!(assembler
                ;   movsxd v_tmp, eax   // sign-extend eax into v_tmp
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn inst_lld(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (virtual_address & 0x07) != 0 {
            self.address_exception(virtual_address, false)?;
        }

        // translate the address here
        let address = match self.translate_address(virtual_address, true, false)? {
            Some(address) => address,
            None => {
                // this shouldn't happen
                return self.address_exception(virtual_address, false);
            }
        };

        self.gpr[self.inst.rt] = self.read_u64_phys(address)?;

        // the "linked" part sets the LLAddr register in cop0 to the physical address
        // of the read, and the LLbit to 1
        // the LLAddr register only stores bits 31:4 of the address at bit 0 of the register
        self.cp0gpr[Cop0_LLAddr] = address.physical_address >> 4;
        self.llbit = true;

        Ok(())
    }

    fn build_inst_lld(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lld r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            // setup arguments to address_exception_bridge _and_ translate_address (v_arg2_32 = false)
            ;   xor v_arg2_32, v_arg2_32   // .
            ;   test v_arg1_8l, BYTE 0x07  // check if address is valid (low three bits 000)
            ;   jz >valid                  // .
        );

        // generate the exception
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        letsgo!(assembler
            ;   valid:
        );

        // setup call to translate_address() -- v_arg1 still contains virtual address and r8 is is_write=false
        letscall!(assembler, Cpu::translate_address_bridge);

        // check if exception occurred
        letscheck!(assembler);

        // rax now has the physical_address, store in Cop0_LLAddr
        letsgo!(assembler
            // put the physical address into v_arg1 for read_u32_phys_bridge
            ;   mov v_arg1, rax
            // shift physical_address right by 4 for Cop0_LLAddr
            ;   shr rax, BYTE 4 as _    
            ;   mov QWORD [r_cp0gpr + (Cop0_LLAddr * 8) as i32], rax
            // set llbit true
            ;   mov BYTE [r_cpu + offset_of!(Cpu, llbit) as i32], BYTE 1i8 as _
        );

        // physical_address in v_arg1
        letscall!(assembler, Cpu::read_u64_phys_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u64_phys_bridge() is in rax
            letsgo!(assembler
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
            );
        }

        CompileInstructionResult::Continue
    }

    fn inst_lui(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.inst.signed_imm << 16;
        Ok(())
    }

    fn build_inst_lui(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lui r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16);

        // if destination register is 0, this is a nop
        if self.inst.rt == 0 { return CompileInstructionResult::Continue; }

        // store immediate sign-extended into gpr
        letsgo!(assembler
            ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], DWORD (self.inst.signed_imm << 16) as u32 as _
        );

        CompileInstructionResult::Continue
    }

    fn inst_lw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        } else {
            self.gpr[self.inst.rt] = (self.read_u32(address as usize)? as i32) as u64;
            match self.current_instruction_pc & 0xFFFF_FFFF {
                0x8000_0228 => { info!(target: "CPU", "IPL3 computed CRC1: ${:016X}, PIF computed CRC1: ${:016X} (if these don't match, the game won't start)", self.gpr[self.inst.rt], self.gpr[7]); },
                0x8000_0234 => { info!(target: "CPU", "IPL3 computed CRC2: ${:016X}, PIF computed CRC2: ${:016X} (if these don't match, the game won't start)", self.gpr[self.inst.rt], self.gpr[16]); },
                _ => {},
            }
        }
        Ok(())
    }

    fn build_inst_lw(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lw r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            ;   test v_arg1_8l, BYTE 0x03  // check if address is valid (low two bits must be 0)
            ;   jz >valid                  // .
            // setup call to address_exception_bridge
            ;   xor v_arg2_32, v_arg2_32   // is_write argument is false
        );

        // an address_exception occurred. v_arg0 will get 'self', v_arg1 already has the virtual address
        // that caused the exception. we just set r8 to 1 for writes, 0 for loads (above).
        // this will call prefetch(), and jump to the epilog of the block
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        // v_arg1 contains address
        letsgo!(assembler
            ;valid:
        );

        // setup variables necessary for exceptions
        letssetupexcept!(self, assembler, false);

        // make the bridge call
        letscall!(assembler, Cpu::read_u32_bridge);

        // check the jit_other_exception flag after call to read_u32_bridge
        letscheck!(assembler);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u32_bridge() is in eax, and needs to be sign extended into rt
            letsgo!(assembler
                ;   movsxd v_tmp, eax   // sign-extend eax into v_tmp
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn inst_lwu(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        } else {
            self.gpr[self.inst.rt] = self.read_u32(address as usize)? as u64;
        }
        Ok(())
    }

    fn build_inst_lwu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lwu r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            ;   test v_arg1, BYTE 0x03 as _ // check if address is valid (low two bits 00)
            ;   jz >valid                   // .
            ;   int3           // call self.address_exception
            ;   int3           // end current run_block
            ;valid:
        );

        //  v_arg1 contains address, result in eax
        letscall!(assembler, Cpu::read_u32_bridge);

        // if destination register is 0, we still do the read (above) but don't store the result
        if self.inst.rt != 0 { 
            // result of call to self.read_u32_bridge() is in eax, needs to be zero-extended
            letsgo!(assembler
                ;   and rax, DWORD 0xFFFF_FFFFu32 as _  // TODO not terribly sure if this is
                                                        // necessary, since I don't know the state
                                                        // of the upper bits of rax after the call
                                                        // to read_u32_bridge
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_lwl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03)?;

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 0 {
            mem
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX >> (32 - shift))) | (mem << shift)
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
        Ok(())
    }

    fn build_inst_lwl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lwl r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x03u8 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            ;   and v_arg1, DWORD !0x03u32 as _    // sign-exnteded imm32
        );

        // call read_u32_bridge()
        letscall!(assembler, Cpu::read_u32_bridge);

        // check for exceptions
        letscheck!(assembler);

        // eax contains the result
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   je >done
            // otherwise, take the lower bits of rt and or with the low bits of eax shifted into position
            ;   mov cl, BYTE [rsp+s_tmp0]            // shift amount
            ;   shl eax, cl                          // shift mem into position
            ;   mov cl, BYTE 32 as _                 // compute 32-shift into cl
            ;   sub cl, BYTE [rsp+s_tmp0]            // .
            ;   mov v_arg2_32, DWORD 0xFFFF_FFFFu32 as _   // constant mask shifted by 32-shift
            ;   shr v_arg2_32, cl                          // .
            ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]        // lower bits of rt
            ;   and v_tmp_32, v_arg2_32                                        // .
            ;   or eax, v_tmp_32                     // result
            ;done:
            // sign-extend eax into rt
            ;   movsxd v_tmp, eax
            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
        );

        CompileInstructionResult::Continue
    }


    fn inst_lwr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm) as usize;

        // fetch the u32 at the specified address
        let mem = self.read_u32(address & !0x03)?;

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 24 { // handle case where no shift occurs
            mem 
        } else {
            ((self.gpr[self.inst.rt] as u32) & (u32::MAX << (8 + shift))) | (mem >> (24 - shift))
        };

        // set value
        self.gpr[self.inst.rt] = (new as i32) as u64; 
        Ok(())
    }

    fn build_inst_lwr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lwr r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x03u8 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            ;   and v_arg1, DWORD !0x03u32 as _    // sign-exnteded imm32
        );

        // call read_u32_bridge()
        letscall!(assembler, Cpu::read_u32_bridge);

        // check for exceptions
        letscheck!(assembler);

        // eax contains the result
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 24u8 as _
            ;   je >done
            // otherwise, take the upper bits of rt and or with the upper bits of eax shifted into position
            ;   mov cl, BYTE 24 as _                 // shift amount for mem, 24-shift
            ;   sub cl, BYTE [rsp+s_tmp0]            // .
            ;   shr eax, cl                          // shift mem into position
            ;   mov cl, BYTE 8 as _                  // compute 8+shift into cl
            ;   add cl, BYTE [rsp+s_tmp0]            // .
            ;   mov v_arg2_32, DWORD 0xFFFF_FFFFu32 as _   // constant mask shifted by 8+shift
            ;   shl v_arg2_32, cl                          // .
            ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]  // upper bits of rt
            ;   and v_tmp_32, v_arg2_32                                  // .
            ;   or eax, v_tmp_32                     // result
            ;done:
            // sign-extend eax into rt
            ;   movsxd v_tmp, eax
            ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], v_tmp
        );

        CompileInstructionResult::Continue
    }

    fn inst_ldc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x07) != 0 {
            self.address_exception(address, false)?;
        }

        let value = self.read_u64(address as usize)?;
        self.cop1.ldc(self.inst.rt, value)
    }

    extern "sysv64" fn inst_ldc1_bridge(cpu: *mut Cpu, inst: u32) -> u32 {
        let cpu = unsafe { &mut *cpu };
        cpu.decode_instruction(inst);
        match cpu.inst_ldc1() {
            Ok(_) => 0,
            
            Err(InstructionFault::OtherException(exception_code)) => {
                assert!(exception_code == ExceptionCode_CpU || exception_code == ExceptionCode_AdEL || exception_code == ExceptionCode_TLBL);
                cpu.jit_other_exception = true;
                exception_code as u32
            },

            Err(e) => {
                // TODO handle errors better
                panic!("error in inst_ldc1_bridge: {:?}", e);
            }
        }
    }

    fn build_inst_ldc1(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ldc1 fpr{}, ${:04X}(r{}) (STUB)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // opcode in v_arg1_32 (2nd arg)
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        // do the call
        letscall!(assembler, Cpu::inst_ldc1_bridge);

        // check for exceptions
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }


    fn inst_lwc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        
        if (address & 0x03) != 0 {
            self.address_exception(address, false)?;
        }

        let value = self.read_u32(address as usize)?;
        self.cop1.lwc(self.inst.rt, value)
    }

    extern "sysv64" fn inst_lwc1_bridge(cpu: *mut Cpu, inst: u32) -> u32 {
        let cpu = unsafe { &mut *cpu };
        cpu.decode_instruction(inst);
        match cpu.inst_lwc1() {
            Ok(_) => 0,

            // return exceptions as negative values
            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as u32
            }

            Err(e) => {
                // TODO handle errors better
                panic!("error in inst_lwc1_bridge: {:?}", e);
            }
        }
    }

    fn build_inst_lwc1(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: lwc1 fpr{}, ${:04X}(r{}) (STUB)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup values needed for exceptions in LWC1
        letssetupexcept!(self, assembler, false);

        // opcode in v_arg1_32 (2nd arg)
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        letscall!(assembler, Cpu::inst_lwc1_bridge);

        // check for exceptions
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }


    fn inst_ori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] | self.inst.imm;
        Ok(())
    }

    fn build_inst_ori(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ori r{}, r{}, 0x{:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.imm);

        // if destination register is 0, this is a nop
        if self.inst.rt == 0 { return CompileInstructionResult::Continue; }

        if self.inst.rs == self.inst.rt { // can save an instruction when the operands are the same register
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                ;   or QWORD [r_gpr + (self.inst.rs*8) as i32], v_tmp   // or rs with self.inst.imm, store result
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                ;   or v_tmp, QWORD [r_gpr + (self.inst.rs*8) as i32]   // or rs with self.inst.imm
                ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], v_tmp  // store result in rt
            );
        }

        CompileInstructionResult::Continue
    }

    fn inst_regimm(&mut self) -> Result<(), InstructionFault> {
        self.inst.regimm = (self.inst.v >> 16) & 0x1F;
        self.regimm_table[self.inst.regimm as usize](self)
    }

    fn build_inst_regimm(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        self.inst.regimm = (self.inst.v >> 16) & 0x1F;
        self.jit_regimm_table[self.inst.regimm as usize](self, assembler)
    }

    fn inst_sb(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.write_u8(self.gpr[self.inst.rt] as u32, address as usize)?;
        Ok(())
    }

    fn build_inst_sb(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sb r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // value to write in 2nd argument (v_arg1_32)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // rt in 2nd argument
            );
        }

        // place virtual address in the 3rd argument (v_arg2)
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);

        // v_arg1_32 has 32-bit value to write, v_arg2 contains address
        letscall!(assembler, Cpu::write_u8_bridge);

        // TODO check return value for error/exception
        CompileInstructionResult::Continue
    }

    
    fn inst_sh(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        self.write_u16(self.gpr[self.inst.rt] as u32, address as usize)?;
        Ok(())
    }

    fn build_inst_sh(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sh r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // value to write in 2nd argument (v_arg1_32)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32  // zeroes v_arg1
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // rt in 2nd argument
            );
        }

        // place virtual address in the 3rd argument (v_arg2)
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);

        // v_arg1_32 has 32-bit value to write, v_arg2 contains address
        letscall!(assembler, Cpu::write_u16_bridge);

        // TODO check return value for error/exception
        CompileInstructionResult::Continue
    }


    fn inst_sc(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (virtual_address & 0x03) != 0 {
            self.address_exception(virtual_address, true)?;
        }

        // translate the address here
        let address = match self.translate_address(virtual_address, true, true)? {
            Some(address) => address,
            None => {
                // this shouldn't happen
                return self.address_exception(virtual_address, true);
            }
        };

        if self.llbit {
            self.write_u32_phys(self.gpr[self.inst.rt] as u32, address)?;
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }
        Ok(())
    }

    fn build_inst_sc(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sc r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            // setup arguments to address_exception_bridge _and_ translate_address (v_arg2_32 = true)
            ;   mov v_arg2_32, DWORD 1 as _      // .
            ;   test v_arg1_8l, BYTE 0x03        // check if address is valid (low two bits 00)
            ;   jz >valid                        // .
        );

        // generate the exception
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        letsgo!(assembler
            ;   valid:
        );

        // setup call to translate_address() -- v_arg1 still contains virtual address and r8 is is_write=true
        letscall!(assembler, Cpu::translate_address_bridge);

        // check if exception occurred
        letscheck!(assembler);

        // do the write if the physical_address matches Cop0_LLAddr and llbit is set
        letsgo!(assembler
            // preserve the physical_address in v_arg2 for the call write_u32_phys_bridge
            ;   mov v_arg2, rax
            // check if llbit is set
            ;   test BYTE [r_cpu + offset_of!(Cpu, llbit) as i32], BYTE 0xFFu8 as _
            ;   jz >not_valid
        );

        // the value to be written to memory is in rt and needs to be in v_arg1_32
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
            );
        }

        // make the write call -- v_arg1_32 contains the value to write, v_arg2 the physical_address 
        letscall!(assembler, Cpu::write_u32_phys_bridge);

        // set rt to 0 or 1 based on whether the store was successful or not
        letsgo!(assembler
            ;   mov DWORD [r_gpr + (self.inst.rt * 8) as i32], BYTE 1 as _
            ;   jmp >done
            ;not_valid:
            ;   mov DWORD [r_gpr + (self.inst.rt * 8) as i32], BYTE 0 as _
            ;done:
        );

        CompileInstructionResult::Continue
    }

    fn inst_scd(&mut self) -> Result<(), InstructionFault> {
        let virtual_address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (virtual_address & 0x07) != 0 {
            self.address_exception(virtual_address, true)?;
        }

        // translate the address here
        let address = match self.translate_address(virtual_address, true, false)? {
            Some(address) => address,
            None => {
                // this shouldn't happen
                return self.address_exception(virtual_address, true);
            }
        };

        if self.llbit && ((address.physical_address >> 4) == self.cp0gpr[Cop0_LLAddr]) {
            self.write_u64_phys(self.gpr[self.inst.rt], address)?;
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }
        Ok(())
    }

    fn build_inst_scd(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: scd r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // v_arg1 is used for the 2nd parameter to Cpu::read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        letsgo!(assembler
            // setup arguments to address_exception_bridge _and_ translate_address (v_arg2_32 = true)
            ;   mov v_arg2_32, DWORD 1 as _ // .
            ;   test v_arg1_8l, BYTE 0x07   // check if address is valid (low three bits 000)
            ;   jz >valid                   // .
        );

        // generate the exception
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        letsgo!(assembler
            ;   valid:
        );

        // setup call to translate_address() -- v_arg1 still contains virtual address and r8 is is_write=true
        letscall!(assembler, Cpu::translate_address_bridge);

        // check if exception occurred
        letscheck!(assembler);

        // do the write if the physical_address matches Cop0_LLAddr and llbit is set
        letsgo!(assembler
            // preserve the physical_address in v_arg2 for the call write_u64_phys_bridge
            ;   mov v_arg2, rax
            // check if llbit is set
            ;   test BYTE [r_cpu + offset_of!(Cpu, llbit) as i32], BYTE 0xFFu8 as _
            ;   jz >not_valid
        );

        // the value to be written to memory is in rt and needs to be in v_arg1
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            );
        }

        // make the write call -- v_arg2_32 contains the value to write, r8 the physical_address 
        letscall!(assembler, Cpu::write_u64_phys_bridge);

        // set rt to 0 or 1 based on whether the store was successful or not
        letsgo!(assembler
            ;   mov DWORD [r_gpr + (self.inst.rt * 8) as i32], BYTE 1 as _
            ;   jmp >done
            ;not_valid:
            ;   mov DWORD [r_gpr + (self.inst.rt * 8) as i32], BYTE 0 as _
            ;done:
        );

        CompileInstructionResult::Continue
    }

    fn inst_sd(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, true)?;
        }

        self.write_u64(self.gpr[self.inst.rt], address as usize)?;
        Ok(())
    }

    fn build_inst_sd(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sd r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // value to write in 2nd argument (v_arg1)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32] // rt in 2nd argument
            );
        }

        // place virtual address in the 3rd argument (v_arg2)
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);

        letsgo!(assembler
            ;   test v_arg2_8l, BYTE 0x07 as _    // check if address is valid (low three bits 000)            
            ;   jz >valid                         // .
            ;   mov rax, QWORD 0xABCD_ABCD_ABCD_ABCDu64 as _ 
            ;   int3           // call self.address_exception
            ;valid:
        );

        // v_arg1 has 64-bit value to write, v_arg2 contains address
        letscall!(assembler, Cpu::write_u64_bridge);

        // TODO check return value for error/exception
        CompileInstructionResult::Continue
    }


    fn inst_sdc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x07) != 0 {
            self.address_exception(address, true)?;
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.sdc(self.inst.rt)?;
        self.write_u64(value, address as usize)?;
        Ok(())
    }

    extern "sysv64" fn inst_sdc1_bridge(cpu: *mut Cpu, inst: u32) -> u32 {
        let cpu = unsafe { &mut *cpu };
        cpu.decode_instruction(inst);
        match cpu.inst_sdc1() {
            Ok(_) => 0,

            // return exceptions as negative values
            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as u32
            }

            Err(e) => {
                // TODO handle errors better
                panic!("error in inst_sdc1_bridge: {:?}", e);
            }
        }
    }

    fn build_inst_sdc1(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sdc1 fpr{}, ${:04X}(r{}) (STUB)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup values needed for exceptions in LWC1
        letssetupexcept!(self, assembler, false);

        // opcode in v_arg1_32 (2nd arg)
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        letscall!(assembler, Cpu::inst_sdc1_bridge);

        // check for exceptions
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_swc1(&mut self) -> Result<(), InstructionFault> {
        // if the co-processor isn't enabled, generate an exception
        if (self.cp0gpr[Cop0_Status] & 0x2000_0000) == 0 {
            self.coprocessor_unusable_exception(1)?;
            return Ok(());
        }

        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true)?;
        }

        // TODO: need to catch invalid sitatuations (see datasheet)

        let value = self.cop1.swc(self.inst.rt)?;
        self.write_u32(value, address as usize)?;
        Ok(())
    }

    extern "sysv64" fn inst_swc1_bridge(cpu: *mut Cpu, inst: u32) -> u32 {
        let cpu = unsafe { &mut *cpu };
        cpu.decode_instruction(inst);
        match cpu.inst_swc1() {
            Ok(_) => 0,

            // return exceptions as negative values
            Err(InstructionFault::OtherException(exception_code)) => {
                cpu.jit_other_exception = true;
                exception_code as u32
            }

            Err(e) => {
                // TODO handle errors better
                panic!("error in inst_swc1_bridge: {:?}", e);
            }
        }
    }

    fn build_inst_swc1(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: swc1 fpr{}, ${:04X}(r{}) (STUB)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup values needed for exceptions in SWC1
        letssetupexcept!(self, assembler, false);
        
        // opcode in v_arg1_32 (2nd arg)
        letsgo!(assembler
            ;   mov v_arg1_32, DWORD self.inst.v as _
        );

        letscall!(assembler, Cpu::inst_swc1_bridge);

        // check for exceptions
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }


    fn inst_sdl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        // fetch the u64 at the specified address
        // we need the TLB exception to happen with is_write=true, so we translate here
        let mem = match self.translate_address(address, true, true)? {
            Some(mut address) => {
                address.physical_address &= !7;
                self.read_u64_phys(address)?
            },
            None => { // shouldn't happen
                return self.address_exception(address, true);
            }
        };

        // combine register and mem
        let shift = (address & 0x07) << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] >> shift) | (mem & (u64::MAX << (64 - shift)))
        };

        // write new value
        self.write_u64(new, (address as usize) & !0x07)?;
        Ok(())
    }

    fn build_inst_sdl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sdl r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // setup parameters for translate_address_bridge
        letsgo!(assembler
            // is_write = true
            ;   mov v_arg2_32, BYTE 1 as _
        );

        // translate the original address, which might cause a TLB miss
        letscall!(assembler, Cpu::translate_address_bridge);
        
        // check for exceptions
        letscheck!(assembler);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            // move physical_address into v_arg1
            ;   mov v_arg1, rax
            // compute 32-shift
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x07 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            // mask out low two bits of address
            ;   and v_arg1, DWORD !0x07u32 as _    // sign-exnteded imm32
        );

        // call read_u64_phys()
        letscall!(assembler, Cpu::read_u64_phys_bridge);

        // rax contains the result
        // we place the computation in v_arg1, as it's the second parameter to write_u64_bridge
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use rt as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   jne >combine
            ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   jmp >done
            // otherwise, take the lower bits of rt and or with the low bits of rax shifted into position
            ;combine:
            ;   mov cl, BYTE 64 as _                    // compute 64-shift
            ;   sub cl, BYTE [rsp+s_tmp0]               // .
            ;   mov v_arg1, DWORD 0xFFFF_FFFFu32 as _   // constant mask (sign extended) shifted by 64-shift
            ;   shl v_arg1, cl                          // .
            ;   and rax, v_arg1                         // mask memory
            ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32] // shift rt by the shift amount
            ;   mov cl, BYTE [rsp+s_tmp0]                             // .
            ;   shr v_arg1, cl                                        // .
            ;   or v_arg1, rax                          // result
            ;done:
        );

        // v_arg2 is the 3nd parameter to write_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);
        letsgo!(assembler
            ;   and v_arg2, DWORD !0x07u32 as _
        );

        // write v_arg1 to v_arg2
        letscall!(assembler, Cpu::write_u64_bridge);

        // check for exception
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_sdr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        // fetch the u64 at the specified address
        // we need the TLB exception to happen with is_write=true, so we translate here
        let mem = match self.translate_address(address, true, true)? {
            Some(mut address) => {
                address.physical_address &= !7;
                self.read_u64_phys(address)?
            },
            None => { // shouldn't happen
                return self.address_exception(address, true);
            }
        };

        // combine register and mem
        let shift = 56 - ((address & 0x07) << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt]
        } else {
            (self.gpr[self.inst.rt] << shift) | (mem & (u64::MAX >> (64 - shift)))
        };

        // write new value
        self.write_u64(new, (address as usize) & !0x07)?;
        Ok(())
    }

    fn build_inst_sdr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sdr r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to read_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // setup parameters for translate_address_bridge
        letsgo!(assembler
            // is_write = true
            ;   mov v_arg2_32, BYTE 1 as _
        );

        // translate the original address, which might cause a TLB miss
        letscall!(assembler, Cpu::translate_address_bridge);
        
        // check for exceptions
        letscheck!(assembler);

        // get the shift amount from the address, and mask low bits out of v_arg1
        letsgo!(assembler
            // move physical_address into v_arg1
            ;   mov v_arg1, rax
            // compute 32-shift
            ;   mov cl, v_arg1_8l
            ;   and cl, BYTE 0x07 as _
            ;   shl cl, BYTE 3 as _
            ;   mov BYTE [rsp+s_tmp0], BYTE 56 as _
            ;   sub BYTE [rsp+s_tmp0], cl
            // place mask out low two bits of address
            ;   and v_arg1, DWORD !0x07u32 as _    // sign-exnteded imm32
        );

        // call read_u64_phys()
        letscall!(assembler, Cpu::read_u64_phys_bridge);

        // rax contains the result
        // we place the computation in v_arg1, as it's the second parameter to write_u64_bridge
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   jne >combine
            ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   jmp >done
            // otherwise, take the lower bits of rt and or with the low bits of eax shifted into position
            ;combine:
            ;   mov cl, BYTE 64 as _                    // compute 64-shift
            ;   sub cl, BYTE [rsp+s_tmp0]               // .
            ;   mov v_arg1, DWORD 0xFFFF_FFFFu32 as _   // constant mask (sign extended) shifted by 64-shift
            ;   shr v_arg1, cl                             // .
            ;   and rax, v_arg1                            // mask memory
            ;   mov v_arg1, QWORD [r_gpr + (self.inst.rt * 8) as i32] // shift rt by the shift amount
            ;   mov cl, BYTE [rsp+s_tmp0]                             // .
            ;   shl v_arg1, cl                                        // .
            ;   or v_arg1, rax                          // result
            ;done:
        );

        // v_arg2 is the 3nd parameter to write_u64_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);
        letsgo!(assembler
            ;   and v_arg2, DWORD !0x07u32 as _
        );

        // write v_arg1 to v_arg2
        letscall!(assembler, Cpu::write_u64_bridge);

        // check for exception
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_slti(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.inst.signed_imm as i64) {
            self.gpr[self.inst.rt] = 1;
        } else {
            self.gpr[self.inst.rt] = 0;
        }

        Ok(())
    }

    fn build_inst_slti(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: slti r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rt != 0 {
            if self.inst.rs == 0 { // if rs is zero, set v_tmp to 0
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]  // load rs to v_tmp
                );
            }

            letsgo!(assembler
                ;   xor rax, rax    // default result value to 0
            );

            // perform the compare
            if self.inst.signed_imm == 0 { // compare against zero
                letsgo!(assembler
                    ;   cmp v_tmp, BYTE 0i8 as _
                );
            } else {
                letsgo!(assembler
                    ;   cmp v_tmp, DWORD self.inst.signed_imm as u32 as _ // sign-extends signed_imm to 64-bits
                );
            }

            letsgo!(assembler
                ;   jge >skip                                             // if rs >= rt (signed), branch
                ;   inc rax                                               // if rs < rt, set rax=1
                ;skip:
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax    // store flag in rd
            );
        }

        CompileInstructionResult::Continue
    }



    fn inst_sltiu(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = (self.gpr[self.inst.rs] < self.inst.signed_imm) as u64;

        Ok(())
    }

    fn build_inst_sltiu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sltiu r{}, r{}, ${:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.signed_imm as u16);

        if self.inst.rt != 0 {
            if self.inst.rs == 0 { // if rs is zero, set v_tmp to 0
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]  // load rs to v_tmp
                );
            }

            letsgo!(assembler
                ;   xor rax, rax    // default result value to 0
            );

            // perform the compare
            if self.inst.signed_imm == 0 { // compare against zero
                letsgo!(assembler
                    ;   cmp v_tmp, BYTE 0i8 as _
                );
            } else {
                letsgo!(assembler
                    ;   cmp v_tmp, DWORD self.inst.signed_imm as u32 as _ // sign-extends signed_imm to 64-bits
                );
            }

            letsgo!(assembler
                ;   jae >skip                                             // if rs >= rt (unsigned), branch
                ;   inc rax                                               // if rs < rt, set rax=1
                ;skip:
                ;   mov QWORD [r_gpr + (self.inst.rt * 8) as i32], rax    // store flag in rd
            );
        }

        CompileInstructionResult::Continue
    }


    fn inst_special(&mut self) -> Result<(), InstructionFault> {
        self.inst.special = self.inst.v & 0x3F;
        self.special_table[self.inst.special as usize](self)
    }

    fn build_inst_special(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        self.inst.special = self.inst.v & 0x3F;
        self.jit_special_table[self.inst.special as usize](self, assembler)
    }

    fn inst_sw(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);
        if (address & 0x03) != 0 {
            self.address_exception(address, true)?;
        } else {
            self.write_u32(self.gpr[self.inst.rt] as u32, address as usize)?;
        }
        Ok(())
    }

    fn build_inst_sw(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sw r{}, ${:04X}(r{})", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // value to write in 2nd argument (v_arg1_32)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor v_arg1_32, v_arg1_32
            );
        } else {
            letsgo!(assembler
                ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // rt in 2nd argument
            );
        }

        // place virtual address in the 3rd argument (v_arg2)
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);

        letsgo!(assembler
            ;   test v_arg2_8l, BYTE 0x03   // check if address is valid (low two bits 00)
            ;   jz >valid                   // .
            // setup call to address_exception_bridge
            ;   mov v_arg1, v_arg2             // move virtual address to v_arg1
            ;   mov v_arg2_32, BYTE 0x01 as _  // is_write = true
        );

        // an address_exception occurred. v_arg0 will get 'self', v_arg1 has the virtual address
        // that caused the exception. we just set v_arg2_32 to 1 for writes (above)
        // this will call prefetch(), and jump to the epilog of the block
        letsexcept!(self, assembler, Cpu::address_exception_bridge);

        // v_arg1_32 has 32-bit value to write, v_arg2 contains address
        letsgo!(assembler
            ;valid:
        );

        // setup for exceptions
        letssetupexcept!(self, assembler, false);

        // make the call
        letscall!(assembler, Cpu::write_u32_bridge);

        // check the jit_other_exception flag after call to write_u32_bridge
        letscheck!(assembler);

        // TODO check return value for error/exception
        CompileInstructionResult::Continue
    }

    fn inst_swl(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        // fetch the u32 at the specified address
        // we need the TLB exception to happen with is_write=true, so we translate here
        let mem = match self.translate_address(address, true, true)? {
            Some(mut address) => {
                address.physical_address &= !3;
                self.read_u32_phys(address)?
            },
            None => { // shouldn't happen
                return self.address_exception(address, true);
            }
        };

        // combine register and mem
        let shift = (address & 0x03) << 3;
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) >> shift) | (mem & (u32::MAX << (32 - shift)))
        };

        // write mem value
        self.write_u32(new, (address as usize) & !0x03)?;
        Ok(())
    }

    fn build_inst_swl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: swl r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to translate_address_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // setup parameters for translate_address_bridge
        letsgo!(assembler
            // is_write = true
            ;   mov v_arg2_32, BYTE 1 as _
        );

        // translate the original address, which might cause a TLB miss
        letscall!(assembler, Cpu::translate_address_bridge);
        
        // check for exceptions
        letscheck!(assembler);

        letsgo!(assembler
            // move physical_address to v_arg1
            ;   mov v_arg1, rax
            // get the shift amount from the address, and mask low bits out of v_arg1
            ;   mov BYTE [rsp+s_tmp0], v_arg1_8l
            ;   and BYTE [rsp+s_tmp0], BYTE 0x03u8 as _
            ;   shl BYTE [rsp+s_tmp0], BYTE 3 as _
            ;   and v_arg1, DWORD !0x03u32 as _    // sign-exnteded imm32
        );

        // call read_u32_phys()
        letscall!(assembler, Cpu::read_u32_phys_bridge);

        // eax contains the result
        // we place the computation in v_arg1_32, as it's the second parameter to write_u32_bridge
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   jne >combine
            ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   jmp >done
            // otherwise, take the lower bits of rt and or with the low bits of eax shifted into position
            ;combine:
            ;   mov cl, BYTE 32 as _                 // compute 32-shift
            ;   sub cl, BYTE [rsp+s_tmp0]            // .
            ;   mov v_arg1_32, DWORD 0xFFFF_FFFFu32 as _   // constant mask shifted by 32-shift
            ;   shl v_arg1_32, cl                          // .
            ;   and eax, v_arg1_32                         // mask memory
            ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // shift rt by the shift amount
            ;   mov cl, BYTE [rsp+s_tmp0]                                // .
            ;   shr v_arg1_32, cl                                        // .
            ;   or v_arg1_32, eax                          // result
            ;done:
        );

        // v_arg2 is the 3nd parameter to write_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);
        letsgo!(assembler
            ;   and v_arg2, DWORD !0x03u32 as _
        );

        // write v_arg1_32 to v_arg2
        letscall!(assembler, Cpu::write_u32_bridge);

        // check for exception
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_swr(&mut self) -> Result<(), InstructionFault> {
        let address = self.gpr[self.inst.rs].wrapping_add(self.inst.signed_imm);

        // fetch the u32 at the specified address
        // we need the TLB exception to happen with is_write=true, so we translate here
        let mem = match self.translate_address(address, true, true)? {
            Some(mut address) => {
                address.physical_address &= !3;
                self.read_u32_phys(address)?
            },
            None => { // shouldn't happen
                return self.address_exception(address, true);
            }
        };

        // combine register and mem
        let shift = 24 - ((address & 0x03) << 3);
        let new = if shift == 0 {
            self.gpr[self.inst.rt] as u32
        } else {
            ((self.gpr[self.inst.rt] as u32) << shift) | (mem & (u32::MAX >> (32 - shift)))
        };

        // write new value
        self.write_u32(new, (address as usize) & !0x03)?;
        Ok(())
    }

    fn build_inst_swr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: swr r{}, ${:04X}(r{}) (TODO)", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.signed_imm as u16, self.inst.rs);

        // setup for exception
        letssetupexcept!(self, assembler, false);

        // v_arg1 is the 2nd parameter to translate_address_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg1);

        // setup parameters for translate_address_bridge
        letsgo!(assembler
            // is_write = true
            ;   mov v_arg2_32, BYTE 1 as _
        );

        // call translate_address_bridge() on the original address
        letscall!(assembler, Cpu::translate_address_bridge);

        // check for exceptions
        letscheck!(assembler);

        // physical address now in rax
        // get the shift amount from the address, and mask low bits out of v_arg2
        letsgo!(assembler
            // move address into v_arg1 for the call read_u32_phys
            ;   mov v_arg1, rax
            // compute 32-shift
            ;   mov cl, v_arg1_8l
            ;   and cl, BYTE 0x03 as _
            ;   shl cl, BYTE 3 as _
            ;   mov BYTE [rsp+s_tmp0], BYTE 24 as _
            ;   sub BYTE [rsp+s_tmp0], cl
            // place mask out low two bits of address
            ;   and v_arg1, DWORD !0x03u32 as _    // sign-exnteded imm32
        );

        letscall!(assembler, Cpu::read_u32_phys_bridge);

        // eax contains the result
        // we place the computation in v_arg1_32, as it's the second parameter to write_u32_bridge
        // TODO reduce # of instructions if possible
        letsgo!(assembler
            // no shift - use eax as is
            ;   cmp BYTE [rsp+s_tmp0], BYTE 0u8 as _
            ;   jne >combine
            ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   jmp >done
            // otherwise, take the lower bits of rt and or with the low bits of eax shifted into position
            ;combine:
            ;   mov cl, BYTE 32 as _                 // compute 32-shift
            ;   sub cl, BYTE [rsp+s_tmp0]            // .
            ;   mov v_arg1_32, DWORD 0xFFFF_FFFFu32 as _   // constant mask shifted by 32-shift
            ;   shr v_arg1_32, cl                          // .
            ;   and eax, v_arg1_32                         // mask memory
            ;   mov v_arg1_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // shift rt by the shift amount
            ;   mov cl, BYTE [rsp+s_tmp0]                  // .
            ;   shl v_arg1_32, cl                          // .
            ;   or v_arg1_32, eax                          // result
            ;done:
        );

        // v_arg2 is the 3nd parameter to write_u32_bridge
        letsoffset!(assembler, self.inst.rs, self.inst.signed_imm, v_arg2);
        letsgo!(assembler
            ;   and v_arg2, DWORD !0x03u32 as _
        );

        // write v_arg1_32 to v_arg2
        letscall!(assembler, Cpu::write_u32_bridge);

        // check for exception
        letscheck!(assembler);

        CompileInstructionResult::Continue
    }

    fn inst_xori(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rt] = self.gpr[self.inst.rs] ^ self.inst.imm;
        Ok(())
    }

    fn build_inst_xori(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: xori r{}, r{}, 0x{:04X}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.imm);

        if self.inst.rt != 0 { // if destination register is 0, this is a nop
            if self.inst.rs == self.inst.rt { // can save an instruction when the operands are the same register
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                    ;   xor QWORD [r_gpr + (self.inst.rs*8) as i32], v_tmp  // or rs with self.inst.imm, store result
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD self.inst.imm as _              // zeroes out the upper dword of v_tmp
                    ;   xor v_tmp, QWORD [r_gpr + (self.inst.rs*8) as i32]  // or rs with self.inst.imm
                    ;   mov QWORD [r_gpr + (self.inst.rt*8) as i32], v_tmp  // store result in rt
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn regimm_unknown(&mut self) -> Result<(), InstructionFault> {
        panic!("CPU: unimplemented regimm op: 0b{:02b}_{:03b}", self.inst.regimm >> 3, self.inst.regimm & 0x07);
    }

    fn regimm_bgezal(&mut self) -> Result<(), InstructionFault> {
        // compute condition before changing gpr31 as rs can be 31...
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register
        self.branch(condition);
        Ok(())
    }

    fn build_regimm_bgezal(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgezal only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgezal", jl, false, true); // JL for signed comparison

        // the link register is always set to the address after the delay slot
        letslink!(self, assembler);

        CompileInstructionResult::Continue
    }

    fn regimm_bgezall(&mut self) -> Result<(), InstructionFault> {
        // addresses are sign extended
        self.gpr[31] = self.pc; // unconditionally, the address after the delay slot is stored in the link register

        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch_likely(condition)?;

        Ok(())
    }

    fn build_regimm_bgezall(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgezal only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgezall", jl, true, true); // JL for signed comparison

        // the link register is always set to the address after the delay slot
        letslink!(self, assembler);

        CompileInstructionResult::Continue
    }


    fn regimm_bgez(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch(condition);

        Ok(())
    }

    fn build_regimm_bgez(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgez only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgez", jl, false, true); // jump if less than

        CompileInstructionResult::Continue
    }

    fn regimm_bgezl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) >= 0;
        self.branch_likely(condition)
    }

    fn build_regimm_bgezl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bgez only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bgezl", jl, true, true); // jump if less than
        CompileInstructionResult::Continue
    }

    fn regimm_bltz(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) < 0;
        self.branch(condition);

        Ok(())
    }

    fn build_regimm_bltz(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bltz only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bltz", jge, false, false); // jump if greater than or equal

        CompileInstructionResult::Continue
    }


    fn regimm_bltzl(&mut self) -> Result<(), InstructionFault> {
        let condition = (self.gpr[self.inst.rs] as i64) < 0;
        self.branch_likely(condition)
    }

    fn build_regimm_bltzl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // letsbranch will compare rs to rt. bltz only uses rs, so we set rt to r0
        self.inst.rt = 0;
        letsbranch!(self, assembler, "bltzl", jge, true, false); // jump if greater than or equal
        CompileInstructionResult::Continue
    }

    fn special_add(&mut self) -> Result<(), InstructionFault> {
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_add(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }

        Ok(())
    }

    fn build_special_add(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: add r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        // start with v_tmp set to 0 if zero register is used
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // don't add anything if rt is zero (thus, no possible overflow)
        if self.inst.rt != 0 {
            letsgo!(assembler
                ;   add v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
                ;   jno >no_overflow
            );

            letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

            letsgo!(assembler
                ;no_overflow:
            );
        }

        // don't store if destination is zero register
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   movsxd v_tmp2, v_tmp_32
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_addu(&mut self) -> Result<(), InstructionFault> {
        // addu does not cause an overflow exception
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as u32).wrapping_add(self.gpr[self.inst.rt] as u32) as i32) as u64;

        Ok(())
    }

    fn build_special_addu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: addu r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if destination is zero, no-op
            if self.inst.rt == 0 && self.inst.rs == 0 { // we can skip the entire operation, set the result to zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // We can skip the add
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]    // move 32-bit value to v_tmp
                    ;   movsxd v_tmp2, v_tmp_32                                    // sign-extend into v_tmp2
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2      // store in rd
                );
            } else if self.inst.rs == 0 { // We can skip the add
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]    // move 32-bit value to v_tmp
                    ;   movsxd v_tmp2, v_tmp_32                                    // sign-extend into v_tmp2
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2      // store in rd
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]    // put rs into v_tmp
                    ;   add v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]    // 32-bit add rt
                    ;   movsxd v_tmp2, v_tmp_32                                    // sign-extend v_tmp into rd
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2      // .
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_and(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] & self.gpr[self.inst.rt];

        Ok(())
    }
    
    fn build_special_and(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: and r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if dest register is 0, this is a no-op
            if self.inst.rs == 0 || self.inst.rt == 0 { // if either source register is 0, set our destination register to 0
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], DWORD 0i32 as _
                );
            } else {
                // TODO: if destination rd is rs or rt, we can do the AND with one less mov
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   and v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_break(&mut self) -> Result<(), InstructionFault> {
        self.breakpoint_exception()?;
        Ok(())
    }

    fn build_special_break(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: break", self.current_instruction_pc as u32, self.jit_current_assembler_offset);

        letsexcept!(self, assembler, Cpu::breakpoint_exception_bridge);

        // TODO Kinda unsure whether we should keep compiling or not...
        CompileInstructionResult::Stop
    }

    fn special_dadd(&mut self) -> Result<(), InstructionFault> {
        // add does cause an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];
        let result = rs.wrapping_add(rt);

        // if addends had the same sign but the result differs, overflow occurred
        if ((!(rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            //println!("CPU: dadd overflow exception occurred");
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = result;
        }

        Ok(())
    }

    fn build_special_dadd(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dadd r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        // start with v_tmp set to 0 if zero register is used
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // don't add anything if rt is zero (thus, no possible overflow)
        if self.inst.rt != 0 {
            letsgo!(assembler
                ;   add v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                ;   jno >no_overflow
            );

            letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

            letsgo!(assembler
                ;no_overflow:
            );
        }

        // don't store if destination is zero register
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_daddu(&mut self) -> Result<(), InstructionFault> {
        // daddu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_add(self.gpr[self.inst.rt]);

        Ok(())
    }

    fn build_special_daddu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: daddu r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if destination is zero, no-op
            if self.inst.rt == 0 && self.inst.rs == 0 { // we can skip the entire operation, set the result to zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // We can skip the add
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]    // move 64-bit value to v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // store in rd
                );
            } else if self.inst.rs == 0 { // We can skip the add
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // move 64-bit value to v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // store in rd
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]    // put rs into v_tmp
                    ;   add v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // 64-bit add rt
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // .
                );
            }
        }

        CompileInstructionResult::Continue
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

    // build an signed divide of rax/rcx
    // (LO) is placed in rax (quotient)
    // (HI) is placed in rdx (remainder)
    fn build_divide(&mut self, assembler: &mut Assembler) {
        letsgo!(assembler
            // check divisor for 0 value
            ;   cmp rcx, BYTE 0i8 as _
            ;   jne >not_zero
            // on divide by zero, set default in rax:rdx
            ;   mov rdx, rax                       // (HI) dividend into rdx
            ;   cmp rax, BYTE 0                    // check if dividend is negative
            ;   jge >not_neg                       // .
            ;   mov rax, BYTE 1                    // set (LO) to to 1 if dividend is negative
            ;   jmp >skip_div
            ;not_neg:
            ;   mov rax, DWORD 0xFFFF_FFFFu32 as _ // otherwise set (LO) sign-extended -1
            ;   jmp >skip_div
            ;not_zero:
            ;   mov v_tmp, QWORD i64::MIN as _    // compare 64-bit to sign-extended i64 MIN
            ;   cmp rax, v_tmp
            ;   jne >do_div
            ;   cmp rcx, DWORD -1i32 as _          // compare 64-bit divisor to -1
            ;   jne >do_div
            // can't negate this value, so default to:
            ;   mov rax, v_tmp
            ;   xor rdx, rdx
            ;   jmp >skip_div
            ;do_div:
            ;   cqo       // sign extend rax into rdx:rax
            ;   idiv rcx  // perform (rdx:rax) / rcx, quotient in rax, remainder in rdx
            ;skip_div:
        );
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

    // build an unsigned divide of rax/rcx
    // (LO) is placed in rax (quotient)
    // (HI) is placed in rdx (remainder)
    fn build_divide_unsigned(&mut self, assembler: &mut Assembler) {
        letsgo!(assembler
            // check divisor for 0 value
            ;   cmp rcx, BYTE 0i8 as _
            ;   jne >not_zero
            // on divide by zero, set default in rax:rdx
            ;   mov rdx, rax                       // (HI) dividend into rdx
            ;   mov rax, DWORD 0xFFFF_FFFFu32 as _ // (LO) sign-extended -1
            ;   jmp >skip_div
            ;not_zero:
            ;   xor rdx, rdx
            ;   div rcx  // perform (rdx:rax) / rcx, quotient in rax, remainder in rdx
            ;skip_div:
        );
    }

    fn special_ddiv(&mut self) -> Result<(), InstructionFault> {
        let rs = self.gpr[self.inst.rs] as i64;
        let rt = self.gpr[self.inst.rt] as i64;
        self.divide(rs, rt);

        Ok(())
    }

    fn build_special_ddiv(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ddiv r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // dividend goes into rax
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor rax, rax
            );
        } else {
            letsgo!(assembler
                ;   mov rax, QWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // divisor goes into rcx
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor rcx, rcx
            );
        } else {
            letsgo!(assembler
                ;   mov rcx, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            );
        }

        // dividend in rax, divisor in rcx
        // quotient in rax, remainder in rdx
        self.build_divide(assembler);

        // store quotient in LO, remainder in HI
        letsgo!(assembler
            ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], rax
            ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], rdx
        );

        CompileInstructionResult::Continue
    }


    fn special_ddivu(&mut self) -> Result<(), InstructionFault> {
        // unsigned 64 bit values
        self.divide_unsigned(self.gpr[self.inst.rs], self.gpr[self.inst.rt]);

        Ok(())
    }

    fn build_special_ddivu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: ddivu r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // dividend goes into rax (64-bit divide but truncated to 32-bit values)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor rax, rax
            );
        } else {
            letsgo!(assembler
                ;   mov rax, QWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // divisor goes into rcx
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor rcx, rcx
            );
        } else {
            letsgo!(assembler
                ;   mov rcx, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            );
        }

        // result in rdx:rax
        self.build_divide_unsigned(assembler);

        // store quotient in LO, remainder in HI
        letsgo!(assembler
            ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], rax
            ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], rdx
        );

        CompileInstructionResult::Continue
    }


    fn special_div(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit sign extended values
        let rs = (self.gpr[self.inst.rs] as i32) as i64;
        let rt = (self.gpr[self.inst.rt] as i32) as i64;
        self.divide(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;

        Ok(())
    }

    fn build_special_div(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: div r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // dividend goes into rax
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor rax, rax
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32] // sign-extended into rax
                ;   movsxd rax, v_tmp_32
            );
        }

        // divisor goes into rcx
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor rcx, rcx
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // sign-extended
                ;   movsxd rcx, v_tmp_32
            );
        }

        // dividend in rax, divisor in rcx
        // quotient in rax, remainder in rdx
        self.build_divide(assembler);

        // store quotient in LO, remainder in HI
        letsgo!(assembler
            ;   movsxd v_tmp, eax                                     // sign-extend 32-bit
            ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp // .
            ;   movsxd v_tmp, edx                                     // sign-extend 32-bit
            ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp // .
        );

        CompileInstructionResult::Continue
    }


    fn special_divu(&mut self) -> Result<(), InstructionFault> {
        // unsigned 32 bit values
        let rs = self.gpr[self.inst.rs] & 0xFFFF_FFFF;
        let rt = self.gpr[self.inst.rt] & 0xFFFF_FFFF;
        self.divide_unsigned(rs, rt);

        // with the 32 bit divide the sign needs to come from bit 31
        self.lo = (self.lo as i32) as u64;
        self.hi = (self.hi as i32) as u64;

        Ok(())
    }

    fn build_special_divu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: divu r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // dividend goes into rax (64-bit divide but truncated to 32-bit values)
        if self.inst.rt == 0 {
            letsgo!(assembler
                ;   xor rax, rax
            );
        } else {
            letsgo!(assembler
                ;   mov eax, DWORD [r_gpr + (self.inst.rs * 8) as i32] // zero-extend
            );
        }

        // divisor goes into rcx
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor rcx, rcx
            );
        } else {
            letsgo!(assembler
                ;   mov ecx, DWORD [r_gpr + (self.inst.rt * 8) as i32] // zero-extend
            );
        }

        // result in rdx:rax
        self.build_divide_unsigned(assembler);

        // store quotient in LO, remainder in HI
        letsgo!(assembler
            ;   movsxd v_tmp, eax                                     // sign-extend 32-bit
            ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp // .
            ;   movsxd v_tmp, edx                                     // sign-extend 32-bit
            ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp // .
        );

        CompileInstructionResult::Continue
    }

    fn special_dmult(&mut self) -> Result<(), InstructionFault> {
        // use 64-bit signed integers and get 128-bit result
        let result = ((self.gpr[self.inst.rs] as i64) as i128) * ((self.gpr[self.inst.rt] as i64) as i128);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result as u64;
        self.hi = (result >> 64) as u64;

        Ok(())
    }

    fn build_special_dmult(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmult r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // must be 64-bit unsigned numbers
        if self.inst.rs == 0 || self.inst.rt == 0 { // set hi/lo to zero
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov rax, QWORD [r_gpr + (self.inst.rs * 8) as i32]  // first 64-bit operand sign-extended in rax
                ;   mov rdx, QWORD [r_gpr + (self.inst.rt * 8) as i32]  // second 64-bit operand sign-extended into rdx
                ;   imul rdx                // 64-bit signed multiply
                                            // result in rdx:rax (both are caller saved)
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], rax    // low 64-bits in LO
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], rdx    // high 64-bits in HI
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_dmultu(&mut self) -> Result<(), InstructionFault> {
        // must be 128-bit unsigned numbers in order to get the full multiply
        let result = (self.gpr[self.inst.rs] as u128) * (self.gpr[self.inst.rt] as u128);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result as u64;
        self.hi = (result >> 64) as u64;

        Ok(())
    }

     fn build_special_dmultu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dmultu r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        if self.inst.rs == 0 || self.inst.rt == 0 { // set hi/lo to zero
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov rax, QWORD [r_gpr + (self.inst.rs * 8) as i32]   // first operand in rax
                ;   mov rdx, QWORD [r_gpr + (self.inst.rt * 8) as i32]   // second operand in rdx
                ;   mul rdx            // 64-bit multiply
                                       // 128-bit result in rdx:rax (both are caller saved)
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], rax    // low 64-bits from rax
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], rdx    // high 64-bits from rdx
            );
        }

        CompileInstructionResult::Continue
    }

 
    fn special_dsll(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << self.inst.sa;
        Ok(())
    }

    fn build_special_dsll(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsll r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shl v_tmp, cl
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_dsllv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn build_special_dsllv(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsllv r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rs == 0 { // just copy
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else { // do shift
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]
                    ;   and cl, BYTE 0x3Fu8 as _
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shl v_tmp, cl
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_dsll32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] << (32 + self.inst.sa);
        Ok(())
    }

    fn build_special_dsll32(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsll32 r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   add cl, 32
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shl v_tmp, cl
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_dsra(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> self.inst.sa) as u64;
        Ok(())
    }

    fn build_special_dsra(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsra r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   sar v_tmp, cl
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_dsrav(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (self.gpr[self.inst.rs] & 0x3F)) as u64;
        Ok(())
    }

    fn build_special_dsrav(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsrav r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            if self.inst.rt == 0 { // if source is zero, zero out destination
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                // TODO test, but I'm pretty sure the sign extension is unnecessary, since it'll
                // always be a positive sign after the shift
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]        // put shift amount into cl
                    ;   and cl, BYTE 0x3F as _                                  // mod with 63
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // take value to shift into v_tmp
                    ;   sar v_tmp, cl                                           // 64-bit signed shift right
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // save in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }



    fn special_dsra32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] as i64) >> (32 + self.inst.sa)) as u64;
        Ok(())
    }

    fn build_special_dsra32(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsra32 r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   add cl, 32
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   sar v_tmp, cl  // signed shift right
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_dsrl32(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (32 + self.inst.sa);
        Ok(())
    }

    fn build_special_dsrl32(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsrl32 r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   add cl, 32
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shr v_tmp, cl  // unsigned shift right
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_dsrl(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> self.inst.sa;
        Ok(())
    }

    fn build_special_dsrl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsrl r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rt, self.inst.rs, self.inst.sa as u8);

        if self.inst.rd != 0 { // NOP on rd==r0
            if self.inst.rt == 0 { // zero out rd
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE self.inst.sa as _
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shr v_tmp, cl
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_dsrlv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x3F);
        Ok(())
    }

    fn build_special_dsrlv(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsrlv r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            if self.inst.rt == 0 { // if source is zero, zero out destination
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                // TODO test, but I'm pretty sure the sign extension is unnecessary, since it'll
                // always be a positive sign after the shift
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]        // put shift amount into cl
                    ;   and cl, BYTE 0x3F as _                                  // mod with 63
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // take value to shift into v_tmp
                    ;   shr v_tmp, cl                                           // 64-bit unsigned shift right
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // save in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_dsub(&mut self) -> Result<(), InstructionFault> {
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs];
        let rt = self.gpr[self.inst.rt];

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000_0000_0000u64) != 0 {
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = result;
        }

        Ok(())
    }

    fn build_special_dsub(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsub r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        // start with v_tmp set to 0 if zero register is used
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // don't sub anything if rt is zero (thus, no possible overflow)
        if self.inst.rt != 0 {
            letsgo!(assembler
                ;   sub v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                ;   jno >no_overflow
            );

            letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

            letsgo!(assembler
                ;no_overflow:
            );
        }

        // don't store if destination is zero register
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_dsubu(&mut self) -> Result<(), InstructionFault> {
        // dsubu does not cause an overflow exception
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]);

        Ok(())
    }

    fn build_special_dsubu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: dsubu r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if destination is zero, no-op
            if self.inst.rt == 0 && self.inst.rs == 0 { // we can skip the entire operation, set the result to zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // We can skip the sub
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]    // move 64-bit value to v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // store in rd
                );
            } else { // don't skip sub when rs is 0, since rt will get negated
                if self.inst.rs == 0 {
                    letsgo!(assembler
                        ;   xor v_tmp, v_tmp
                    );
                } else {
                    letsgo!(assembler
                        ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]    // put rs into v_tmp
                    );
                }
                letsgo!(assembler
                    ;   sub v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // 64-bit sub rt
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp    // .
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_jalr(&mut self) -> Result<(), InstructionFault> {
        let dest = self.gpr[self.inst.rs]; // get dest before changing RD, as RS could be == RD
        self.gpr[self.inst.rd] = self.pc; // pc pointing to after the delay slot already
        self.pc = dest;

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn build_special_jalr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: jalr r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rs, self.inst.rd);

        if self.is_delay_slot {
            return CompileInstructionResult::Cant;
        }

        // move dest to jump target before setting the link register
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   mov QWORD [rsp+s_jump_target], DWORD 0u32 as _
            );
            //panic!("does this ever happen?");
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                ;   mov QWORD [rsp+s_jump_target], v_tmp
            );
        }

        // always set rd to self.pc (if dest register is not 0)
        if self.inst.rd != 0 {
            if (self.pc & 0xFFFF_FFFF_8000_0000) == 0xFFFF_FFFF_8000_0000 {
                letsgo!(assembler
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], DWORD self.pc as i32 as _
                );
            } else {
                letsgo!(assembler
                    ;   mov rax, QWORD self.pc as i64 as _
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], rax
                );
            }
        }

        self.jit_jump = true;
        self.next_is_delay_slot = true;

        CompileInstructionResult::Continue
    }


    fn special_jr(&mut self) -> Result<(), InstructionFault> {
        self.pc = self.gpr[self.inst.rs];

        // note that the next instruction to execute is a delay slot instruction
        self.next_is_delay_slot = true;

        Ok(())
    }

    fn build_special_jr(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: jr r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rs);

        if self.is_delay_slot {
            return CompileInstructionResult::Cant;
        }

        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   mov QWORD [rsp+s_jump_target], DWORD 0u32 as _
            );
            //panic!("does this ever happen?");
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                ;   mov QWORD [rsp+s_jump_target], v_tmp
            );
        }

        self.jit_jump = true;
        self.next_is_delay_slot = true;

        CompileInstructionResult::Continue
    }

    fn special_mfhi(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.hi;
        Ok(())
    }

    fn build_special_mfhi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mfhi r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rd);
                 
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_cpu + offset_of!(Cpu, hi) as i32]
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_mflo(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.lo;
        Ok(())
    }

    fn build_special_mflo(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mflo r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rd);
                 
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_cpu + offset_of!(Cpu, lo) as i32]
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_mthi(&mut self) -> Result<(), InstructionFault> {
        self.hi = self.gpr[self.inst.rd];
        Ok(())
    }

    fn build_special_mthi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mthi r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rd);

        if self.inst.rd == 0 {
            letsgo!(assembler
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], DWORD 0u32 as _
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rd * 8) as i32]
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_mtlo(&mut self) -> Result<(), InstructionFault> {
        self.lo = self.gpr[self.inst.rd];
        Ok(())
    }

    fn build_special_mtlo(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mtlo r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, self.inst.rd);

        if self.inst.rd == 0 {
            letsgo!(assembler
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], DWORD 0u32 as _
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rd * 8) as i32]
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_mult(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit sign extended values
        let result = ((self.gpr[self.inst.rs] as i32) as u64).wrapping_mul((self.gpr[self.inst.rt] as i32) as u64);

        // mult results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;

        Ok(())
    }

    fn build_special_mult(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: mult r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // must be 32-bit unsigned numbers
        if self.inst.rs == 0 || self.inst.rt == 0 { // set hi/lo to zero
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]   // first 32-bit operand sign-extended in rax
                ;   movsxd rax, v_tmp_32                                      // .
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]   // second 32-bit operand sign-extended into rdx
                ;   movsxd rdx, v_tmp_32                                      // .
                ;   mul rdx                 // 64-bit unsigned multiply
                                            // result in rdx:rax (both are caller saved)
                ;   mov v_tmp_32, eax                                        // zero-extend lo/hi
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp    // .
                ;   shr rax, BYTE 32u8 as _                                  // high dword of rax
                ;   mov v_tmp_32, eax                                        // . (zero-extend v_tmp here)
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp    // .
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_multu(&mut self) -> Result<(), InstructionFault> {
        // must be 32-bit unsigned numbers
        let result = (self.gpr[self.inst.rs] & 0xFFFF_FFFF).wrapping_mul(self.gpr[self.inst.rt] & 0xFFFF_FFFF);

        // multu results are available in the next instruction since the multiply
        // was started earlier in the pipeline
        self.lo = result & 0xFFFF_FFFF;
        self.hi = result >> 32;

        Ok(())
    }

    fn build_special_multu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: multu r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rs, self.inst.rt);

        // must be 32-bit unsigned numbers
        if self.inst.rs == 0 || self.inst.rt == 0 { // set hi/lo to zero
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov eax, DWORD [r_gpr + (self.inst.rs * 8) as i32]   // first operand in eax (zero-extended into rax)
                ;   mov edx, DWORD [r_gpr + (self.inst.rt * 8) as i32]   // second operand in edx (zero-extend into rdx)
                ;   mul rdx            // 64-bit multiply
                                       // result in rdx:rax (both are caller saved)
                ;   mov v_tmp_32, eax                                        // zero-extend lo/hi
                ;   mov QWORD [r_cpu + offset_of!(Cpu, lo) as i32], v_tmp    // .
                ;   shr rax, BYTE 32u8 as _                                  // high dword of rax
                ;   mov v_tmp_32, eax                                        // . (zero-extend v_tmp here)
                ;   mov QWORD [r_cpu + offset_of!(Cpu, hi) as i32], v_tmp    // .
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_nor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = !(self.gpr[self.inst.rs] | self.gpr[self.inst.rt]);
        Ok(())
    }

    fn build_special_nor(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: nor r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if dest register is 0, this is a no-op
            if self.inst.rs == 0 { // if either source register is 0, set our destination register directly
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   not v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // if either source register is 0, set our destination register directly
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   not v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                // TODO: if destination rd is rs or rt, we can do the AND with one less mov
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   or  v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   not v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_or(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] | self.gpr[self.inst.rt];
        Ok(())
    }

    fn build_special_or(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: or r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // NOP on write to zero reg
            if self.inst.rs == 0 { // just copy rt over
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // just copy rs over
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else { // do the OR
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   or  v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_teq(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] == self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_teq(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jne >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }

    fn regimm_teqi(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] == (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_teqi(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, DWORD self.inst.signed_imm as _
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jne >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn special_tge(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) >= (self.gpr[self.inst.rt] as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_tge(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jl >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn regimm_tgei(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) >= (self.inst.signed_imm as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_tgei(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, DWORD self.inst.signed_imm as _
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jl >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn special_tgeu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] >= self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_tgeu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        // trap will always be happen if rt is r0, otherwise check the condition
        if self.inst.rt != 0 { 
            letsgo!(assembler
                ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
                ;   jb >no_trap
            );
        }

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }

    fn regimm_tgeiu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] >= (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_tgeiu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        if self.inst.signed_imm != 0 {
            letsgo!(assembler
                ;   mov v_tmp, DWORD self.inst.signed_imm as _
                ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
                ;   jb >no_trap
            );
        }

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }



    fn special_tlt(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.gpr[self.inst.rt] as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_tlt(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jge >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }

    fn regimm_tlti(&mut self) -> Result<(), InstructionFault> {
        if (self.gpr[self.inst.rs] as i64) < (self.inst.signed_imm as i64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_tlti(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, DWORD self.inst.signed_imm as _
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jge >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn special_tltu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] < self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_tltu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jae >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn regimm_tltiu(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] < (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_tltiu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, DWORD self.inst.signed_imm as _
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   jae >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn special_tne(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] != self.gpr[self.inst.rt] {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_special_tne(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   je >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }


    fn regimm_tnei(&mut self) -> Result<(), InstructionFault> {
        if self.gpr[self.inst.rs] != (self.inst.signed_imm as u64) {
            self.trap_exception()?;
        }
        Ok(())
    }

    fn build_regimm_tnei(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        letsgo!(assembler
            ;   mov v_tmp, DWORD self.inst.signed_imm as _
            ;   cmp QWORD [r_gpr + (self.inst.rs * 8) as i32], v_tmp
            ;   je >no_trap
        );

        // no arguments to trap_exception_bridge
        letsexcept!(self, assembler, Cpu::trap_exception_bridge);
        letsgo!(assembler
            ;no_trap:
        );

        CompileInstructionResult::Continue
    }

    fn special_sll(&mut self) -> Result<(), InstructionFault> {
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << self.inst.sa) as i32) as u64;
        Ok(())
    }

    fn build_special_sll(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sll r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                     self.inst.rd, self.inst.rt, self.inst.sa as u8);

            if self.inst.rt == 0 { // if source is zero, destination is zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]  // zeroes out upper dword of v_tmp
                    ;   shl v_tmp_32, BYTE self.inst.sa as _                     // 32-bit shift left
                    ;   movsxd v_tmp2, v_tmp_32                                  // sign-extended store in rd
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2    // .
                );
            }
        } else {
            trace!(target: "JIT-BUILD", "${:08X}[{:5}]: nop", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
        }

        CompileInstructionResult::Continue
    }

    fn special_sllv(&mut self) -> Result<(), InstructionFault> {
        // 32-bit shift and sign extended into 64 bits
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) << (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn build_special_sllv(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sllv r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            if self.inst.rt == 0 { // if source is zero, destination is zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]        // put shift amount into cl
                    ;   and cl, BYTE 0x1F as _                                  // mod with 31
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // take value to shift into v_tmp_32
                    ;   shl v_tmp_32, cl                                        // 32-bit shift left
                    ;   movsxd v_tmp2, v_tmp_32                                 // sign-extend to 64-bits
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2   // save in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_slt(&mut self) -> Result<(), InstructionFault> {
        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rs] as i64) < (self.gpr[self.inst.rt] as i64)) as u64;
        Ok(())
    }

    fn build_special_slt(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: slt r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 {
            if self.inst.rs == 0 { // if rs is zero, set v_tmp to 0
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]  // load rs to v_tmp
                );
            }

            letsgo!(assembler
                ;   xor rax, rax                                          // value to store at the end
            );

            if self.inst.rt == 0 { // compare against 0 or rt
                letsgo!(assembler
                    ;   cmp v_tmp, BYTE 0i8 as _
                );
            } else {
                letsgo!(assembler
                    ;   cmp v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]  // compare rs to rt
                );
            }

            letsgo!(assembler
                ;   jge >skip                                             // if rs >= rt (signed), branch
                ;   inc rax                                               // if rs < rt, set rax=1
                ;skip:
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], rax    // store flag in rd
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_sltu(&mut self) -> Result<(), InstructionFault> {
        // set rd to 1 if rs < rt, otherwise 0
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs] < self.gpr[self.inst.rt]) as u64;
        Ok(())
    }

    fn build_special_sltu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sltu r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 {
            if self.inst.rs == 0 { // if rs is zero, set v_tmp to 0
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]  // load rs to v_tmp
                );
            }

            letsgo!(assembler
                ;   xor rax, rax                                          // value to store at the end
            );

            if self.inst.rt == 0 { // compare against 0 or rt
                letsgo!(assembler
                    ;   cmp v_tmp, BYTE 0i8 as _
                );
            } else {
                letsgo!(assembler
                    ;   cmp v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]  // compare rs to rt
                );
            }

            letsgo!(assembler
                ;   jae >skip                                             // if rs >= rt (unsigned), branch
                ;   inc rax                                               // if rs < rt, set rax=1
                ;skip:
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], rax    // store flag in rd
            );
        }

        CompileInstructionResult::Continue
    }

    fn special_sra(&mut self) -> Result<(), InstructionFault> {
        // TODO I'm very confused here. The VR4300 datasheet is very clear that a 32-bit signed
        // integer is right shifted, and at the end 64-bit sign extended, but n64-systemtest
        // seems to check that it's just a 64 shift and then truncated to 32 bits and then sign
        // extended.  I've left the "working" one in place but the one I think is correct (at least
        // until someone explains to my why it isn't!) below.
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> self.inst.sa) as i32) as u64;

        // truncate to u32, convert to signed, shift (fills 1s in the upper bits) and sign extend to u64
        //self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) as i32) >> self.inst.sa) as u64;
        Ok(())
    }

    fn build_special_sra(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sra r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.sa);

        if self.inst.rd != 0 { // if dest register is 0, this is a no-op
            if self.inst.rt == 0 { // if source register is 0, clear our destination register
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   shr v_tmp, BYTE self.inst.sa as _                      // unsigned shift the entire 64-bit
                    ;   movsxd v_tmp2, v_tmp_32                                // sign-extend the resulting 32-bit value
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2  // store in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_srav(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = ((self.gpr[self.inst.rt] >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn build_special_srav(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: srav r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            if self.inst.rt == 0 { // if source is zero, zero out destination
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                // TODO test, but I'm pretty sure the sign extension is unnecessary, since it'll
                // always be a positive sign after the shift
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]        // put shift amount into cl
                    ;   and cl, BYTE 0x1F as _                                  // mod with 31
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // take 64-bit value to shift into v_tmp
                    ;   shr v_tmp, cl                                           // 64-bit unsigned shift right
                    ;   movsxd v_tmp2, v_tmp_32                                 // sign-extend 32-bit value
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2   // save in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_srl(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> self.inst.sa) as i32) as u64;
        Ok(())
    }

    fn build_special_srl(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: srl r{}, r{}, {}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.sa);

        if self.inst.rd != 0 { // if dest register is 0, this is a no-op
            if self.inst.rt == 0 { // if source register is 0, clear our destination register
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]  // zeroes out upper dword of v_tmp
                    ;   shr v_tmp_32, BYTE self.inst.sa as _                     // unsigned shift right
                    ;   movsxd v_tmp2, v_tmp_32                                  // sign-extend result
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2    // store in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }

    fn special_srlv(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = (((self.gpr[self.inst.rt] as u32) >> (self.gpr[self.inst.rs] & 0x1F)) as i32) as u64;
        Ok(())
    }

    fn build_special_srlv(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: srlv r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rt, self.inst.rs);

        if self.inst.rd != 0 { // if dest is zero, this is a no-op
            if self.inst.rt == 0 { // if source is zero, zero out destination
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else {
                // TODO test, but I'm pretty sure the sign extension is unnecessary, since it'll
                // always be a positive sign after the shift
                letsgo!(assembler
                    ;   mov cl, BYTE [r_gpr + (self.inst.rs * 8) as i32]        // put shift amount into cl
                    ;   and cl, BYTE 0x1F as _                                  // mod with 31
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32] // take value to shift into v_tmp_32
                    ;   shr v_tmp_32, cl                                        // 32-bit unsigned shift right
                    ;   movsxd v_tmp2, v_tmp_32                                 // sign-extend to 64-bits
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2   // save in rd
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_sub(&mut self) -> Result<(), InstructionFault> {
        // sub causes an overflow exception
        let rs = self.gpr[self.inst.rs] as u32;
        let rt = self.gpr[self.inst.rt] as u32;

        let result = rs.wrapping_sub(rt);
        // if addends had the same sign but the result differs, overflow occurred
        if (((rs ^ rt) & (rs ^ result)) & 0x8000_0000) != 0 {
            //println!("CPU: sub overflow exception occurred");
            self.overflow_exception()?;
        } else {
            self.gpr[self.inst.rd] = (result as i32) as u64;
        }
        Ok(())
    }

    fn build_special_sub(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sub r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        // start with v_tmp set to 0 if zero register is used
        if self.inst.rs == 0 {
            letsgo!(assembler
                ;   xor v_tmp, v_tmp
            );
        } else {
            letsgo!(assembler
                ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]
            );
        }

        // don't subtract anything if rt is zero (thus, no possible overflow)
        if self.inst.rt != 0 {
            letsgo!(assembler
                ;   sub v_tmp_32, DWORD [r_gpr + (self.inst.rt * 8) as i32]
                ;   jno >no_overflow
            );

            letsexcept!(self, assembler, Cpu::overflow_exception_bridge);

            letsgo!(assembler
                ;no_overflow:
            );
        }

        // don't store if destination is zero register
        if self.inst.rd != 0 {
            letsgo!(assembler
                ;   movsxd v_tmp2, v_tmp_32
                ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2
            );
        }

        CompileInstructionResult::Continue
    }


    fn special_subu(&mut self) -> Result<(), InstructionFault> {
        // subu does not cause an overflow exception
        self.gpr[self.inst.rd] = (self.gpr[self.inst.rs].wrapping_sub(self.gpr[self.inst.rt]) as i32) as u64;
        Ok(())
    }

    fn build_special_subu(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: subu r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if destination is zero, no-op
            if self.inst.rs == 0 && self.inst.rt == 0 { // zero out the result
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 { // We can skip the sub, we know rs is not zero
                letsgo!(assembler
                    ;   mov v_tmp_32, DWORD [r_gpr + (self.inst.rs * 8) as i32]    // move 32-bit value to v_tmp
                    ;   movsxd v_tmp2, v_tmp_32                                    // sign-extend into v_tmp2
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2      // store in rd
                );
            } else {
                if self.inst.rs == 0 { // zero out v_tmp, still perform the sub negating value in rt
                    letsgo!(assembler
                        ;   xor v_tmp, v_tmp
                    );
                } else {
                    letsgo!(assembler
                        ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]    // put rs into v_tmp
                    );
                }

                letsgo!(assembler
                    ;   sub v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]    // 64-bit sub rt
                    ;   movsxd v_tmp2, v_tmp_32                                 // sign-extend v_tmp_32 into rd
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp2   // .
                );
            }
        }

        CompileInstructionResult::Continue
    }


    fn special_syscall(&mut self) -> Result<(), InstructionFault> {
        self.syscall_exception()?;
        Ok(())
    }

    fn build_special_syscall(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: syscall", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
       
        letsexcept!(self, assembler, Cpu::syscall_exception_bridge);

        CompileInstructionResult::Continue
    }

    fn special_sync(&mut self) -> Result<(), InstructionFault> {
        // NOP on VR4300
        Ok(())
    }

    fn build_special_sync(&mut self, _: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: sync", self.current_instruction_pc as u32, self.jit_current_assembler_offset);
        CompileInstructionResult::Continue
    }

    fn special_xor(&mut self) -> Result<(), InstructionFault> {
        self.gpr[self.inst.rd] = self.gpr[self.inst.rs] ^ self.gpr[self.inst.rt];
        Ok(())
    }

    fn build_special_xor(&mut self, assembler: &mut Assembler) -> CompileInstructionResult {
        trace!(target: "JIT-BUILD", "${:08X}[{:5}]: xor r{}, r{}, r{}", self.current_instruction_pc as u32, self.jit_current_assembler_offset, 
                 self.inst.rd, self.inst.rs, self.inst.rt);

        if self.inst.rd != 0 { // if dest register is 0, this is a no-op
            if self.inst.rs == 0 && self.inst.rt == 0 { // if both are zero, result is zero
                letsgo!(assembler
                    ;   xor v_tmp, v_tmp
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rs == 0 { // if either source register is 0, set our destination register
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else if self.inst.rt == 0 {
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            } else { // otherwise perform the xor
                letsgo!(assembler
                    ;   mov v_tmp, QWORD [r_gpr + (self.inst.rs * 8) as i32]
                    ;   xor v_tmp, QWORD [r_gpr + (self.inst.rt * 8) as i32]
                    ;   mov QWORD [r_gpr + (self.inst.rd * 8) as i32], v_tmp
                );
            }
        }

        CompileInstructionResult::Continue
    }

    pub fn disassemble(address: u64, inst: u32, use_abi_names: bool) -> String {
        let op = inst >> 26;
        let rs = (inst >> 21) & 0x1F;
        let rt = (inst >> 16) & 0x1F;
        let rd = (inst >> 11) & 0x1F;
        let sa = (inst >> 6) & 0x1F;
        let func = inst & 0x3F;
        let imm = inst & 0xFFFF;
        let target = inst & 0x03FFFFFF;

        let rname = |r| {
            if use_abi_names {
                Cpu::abi_name(r as usize)
            } else {
                Cpu::register_name(r as usize)
            }
        };

        let no_type = |mn| {
            format!("{}", mn)
        };

        let j_type = |mn| {
            format!("{} ${:08X}", mn, (((address as u32) + 4) & 0xF000_0000) | (target << 2))
        };

        let i_type_rt = |mn| {
            format!("{} {}, ${:04X}", mn, rname(rt), imm)
        };

        let i_type_rt_base = |mn| {
            format!("{} {}, ${:04X}({})", mn, rname(rt), imm, rname(rs))
        };

        let i_type_ft_base = |mn| {
            format!("{} f{}, ${:04X}({})", mn, rt, imm, rname(rs))
        };

        let i_type1_rs = |mn| {
            format!("{} {}, ${:04X}", mn, rname(rs), imm)
        };

        let i_type_rs_rt = |mn| {
            format!("{} {}, {}, ${:04X}", mn, rname(rs), rname(rt), imm)
        };

        let i_type_rt_rs = |mn| {
            format!("{} {}, {}, ${:04X}", mn, rname(rt), rname(rs), imm)
        };

        let r_type_rd = |mn| {
            format!("{} {}", mn, rname(rd))
        };

        let r_type_rs = |mn| {
            format!("{} {}", mn, rname(rs))
        };

        let r_type_rd_rs = |mn| {
            format!("{} {}, {}", mn, rname(rd), rname(rs))
        };

        let r_type_rs_rt = |mn| {
            format!("{} {}, {}", mn, rname(rs), rname(rt))
        };

        let r_type_rt_rd = |mn| {
            format!("{} {}, {}", mn, rname(rt), rname(rd))
        };

        let r_type_rd_rt_rs = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rt), rname(rs))
        };

        let r_type_rd_rs_rt = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rs), rname(rt))
        };

        let r_type_rd_rt_sa = |mn| {
            format!("{} {}, {}, {}", mn, rname(rd), rname(rt), sa)
        };

        match op {
            0b000_000 => {
                match func {
                    0b000_000 => {
                        if inst == 0 {
                            format!("nop")
                        } else {
                            r_type_rd_rt_sa("sll")
                        }
                    },
                    0b000_010 => r_type_rd_rt_sa("srl"),
                    0b000_011 => r_type_rd_rt_sa("sra"),
                    0b000_100 => r_type_rd_rt_rs("sllv"),
                    0b000_110 => r_type_rd_rt_rs("srlv"),
                    0b000_111 => r_type_rd_rt_rs("srav"),

                    0b001_000 => r_type_rs("jr"),
                    0b001_001 => r_type_rd_rs("jalr"),
                    0b001_100 => no_type("syscall"),
                    0b001_101 => no_type("break"),
                    0b001_111 => no_type("sync"),

                    0b010_000 => r_type_rd("mfhi"),
                    0b010_001 => r_type_rs("mthi"),
                    0b010_010 => r_type_rd("mflo"),
                    0b010_011 => r_type_rs("mtlo"),
                    0b010_100 => r_type_rd_rt_rs("dsllv"),
                    0b010_110 => r_type_rd_rt_rs("dsrlv"),
                    0b010_111 => r_type_rd_rt_rs("dsrav"),

                    0b011_000 => r_type_rs_rt("mult"),
                    0b011_001 => r_type_rs_rt("multu"),
                    0b011_010 => r_type_rs_rt("div"),
                    0b011_011 => r_type_rs_rt("divu"),
                    0b011_100 => r_type_rs_rt("dmult"),
                    0b011_101 => r_type_rs_rt("dmultu"),
                    0b011_110 => r_type_rs_rt("ddiv"),
                    0b011_111 => r_type_rs_rt("ddivu"),

                    0b100_000 => r_type_rd_rs_rt("add"),
                    0b100_001 => r_type_rd_rs_rt("addu"),
                    0b100_010 => r_type_rd_rs_rt("sub"),
                    0b100_011 => r_type_rd_rs_rt("subu"),
                    0b100_100 => r_type_rd_rs_rt("and"),
                    0b100_101 => r_type_rd_rs_rt("or"),
                    0b100_110 => r_type_rd_rs_rt("xor"),
                    0b100_111 => r_type_rd_rs_rt("nor"),

                    0b101_010 => r_type_rd_rs_rt("slt"),
                    0b101_011 => r_type_rd_rs_rt("sltu"),
                    0b101_100 => r_type_rd_rs_rt("dadd"),
                    0b101_101 => r_type_rd_rs_rt("daddu"),
                    0b101_110 => r_type_rd_rs_rt("dsub"),
                    0b101_111 => r_type_rd_rs_rt("dsubu"),

                    0b110_000 => r_type_rs_rt("tge"),
                    0b110_001 => r_type_rs_rt("tgeu"),
                    0b110_010 => r_type_rs_rt("tlt"),
                    0b110_011 => r_type_rs_rt("tltu"),
                    0b110_100 => r_type_rs_rt("teq"),
                    0b110_110 => r_type_rs_rt("tne"),

                    0b111_000 => r_type_rd_rt_sa("dsll"),
                    0b111_010 => r_type_rd_rt_sa("dsrl"),
                    0b111_011 => r_type_rd_rt_sa("dsra"),
                    0b111_100 => r_type_rd_rt_sa("dsll32"),
                    0b111_110 => r_type_rd_rt_sa("dsrl32"),
                    0b111_111 => r_type_rd_rt_sa("dsra32"),

                    _ => panic!("unknown special rs=${:03b}_{:03b}", func >> 3, func & 0x07),
                }
            },

            0b000_001 => {
                // RT field contains the instruction code
                const REGIMM: [&str; 32] = [
                    "bltz", "bgez", "bltzl", "bgezl", "?", "?", "?", "?",
                    "tgei", "tgeiu", "tlti", "tltiu", "teqi", "?", "tnei", "?",
                    "bltzal", "bgezal", "bltzall", "bgezall", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                ];
                i_type1_rs(REGIMM[rt as usize])
            },

            0b000_010 => j_type("j"),
            0b000_011 => j_type("jal"),

            0b000_100 => {
                if rs == 0 && rt == 0 {
                    format!("bra ${:04X}", imm)
                } else if rt == 0 {
                    format!("beqz {}, ${:04X}", rname(rs), imm)
                } else {
                    i_type_rs_rt("beq")
                }
            },

            0b000_101 => i_type_rs_rt("bne"),
            0b000_110 => i_type_rs_rt("blez"),
            0b000_111 => i_type_rs_rt("bgtz"),

            0b001_000 => i_type_rt_rs("addi"),
            0b001_001 => i_type_rt_rs("addiu"),
            0b001_010 => i_type_rt_rs("slti"),
            0b001_011 => i_type_rt_rs("sltiu"),

            0b001_100 => i_type_rt_rs("andi"),
            0b001_101 => i_type_rt_rs("ori"),
            0b001_110 => i_type_rt_rs("xori"),

            0b001_111 => i_type_rt("lui"),

            0b010_000 | 0b010_001 | 0b010_010 => {
                // RS field contains the instruction code
                const COP_FN: [&str; 32] = [
                    "mfc", "dmfc", "cfc", "?", "mtc", "dmtc", "ctc", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?",
                    "?", "?", "?", "?", "?", "?", "?", "?"
                ];
                let copno = op & 0x03;
                if rs < 0b10_000 {
                    if COP_FN[rs as usize].starts_with("?") {
                        error!("unknown cop function rs=${:02b}_{:03b}", rs >> 3, rs & 0x07);
                    }
                    r_type_rt_rd(format!("{}{}", COP_FN[rs as usize], copno))
                } else {
                    match copno {
                        1 => {
                            const CP1_FN: [&str; 64] = [
                                "add", "sub", "mul", "div", "sqrt", "abs", "mov", "neg",
                                "round.l", "trunc.l", "ceil.l", "floor.l", "round.w", "trunc.w", "ceil.w", "floor.w",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "cvt.s", "cvt.d", "?", "?", "cvt.w", "cvt.l", "?", "?",
                                "?", "?", "?", "?", "?", "?", "?", "?",
                                "c.f", "c.un", "c.eq", "c.ueq", "c.olt", "c.ult", "c.ole", "c.ule",
                                "c.sf", "c.ngle", "c.seq", "c.ngl", "c.lt", "c.nge", "c.le", "c.ngt"
                            ];
                            if CP1_FN[func as usize].starts_with("?") {
                                error!("unknown cp1 function func=${:03b}_{:03b}", rs >> 3, rs & 0x07);
                            }
                            r_type_rt_rd(format!("{}", CP1_FN[func as usize]))
                        },
                        _ => {
                            format!("unhandled cop{} instruction ${:X}", copno, func)
                        },
                    }
                }
            },

            0b010_100 => i_type_rs_rt("beql"),
            0b010_101 => i_type_rs_rt("bnel"),
            0b010_110 => i_type_rs_rt("blezl"),
            0b010_111 => i_type_rs_rt("bgtzl"),

            0b011_000 => i_type_rt_rs("daddi"),
            0b011_001 => i_type_rt_rs("daddiu"),
            0b011_010 => i_type_rt_base("ldl"),
            0b011_011 => i_type_rt_base("ldr"),

            0b011_111 => no_type("<invalid>"),

            0b100_000 => i_type_rt_base("lb"),
            0b100_001 => i_type_rt_base("lh"),
            0b100_010 => i_type_rt_base("lwl"),
            0b100_011 => i_type_rt_base("lw"),
            0b100_100 => i_type_rt_base("lbu"),
            0b100_101 => i_type_rt_base("lhu"),
            0b100_110 => i_type_rt_base("lwr"),
            0b100_111 => i_type_rt_base("lwu"),
            0b101_000 => i_type_rt_base("sb"),
            0b101_001 => i_type_rt_base("sh"),
            0b101_010 => i_type_rt_base("swl"),
            0b101_011 => i_type_rt_base("sw"),
            0b101_100 => i_type_rt_base("sdl"),
            0b101_101 => i_type_rt_base("sdr"),
            0b101_110 => i_type_rt_base("swr"),
            0b101_111 => no_type("cache"),

            0b110_000 => i_type_rt("ll"),
            0b110_001 => i_type_ft_base("lwc1"),
            0b110_101 => i_type_ft_base("ldc1"),
            0b110_111 => i_type_rt("ld"),

            0b111_000 => i_type_rt("sc"),
            0b111_001 => i_type_rt("swc1"),
            0b111_101 => i_type_ft_base("sdc1"),
            0b111_111 => i_type_rt("sd"),

            _ => panic!("cannot disassemble 0b{:03b}_{:03b}", op >> 3, op & 0x07),
        }
    }
}

