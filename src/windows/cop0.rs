use crate::{*, gui::game::Utils};
use crossbeam::channel::{Receiver, Sender, self};
use n64::{cpu, debugger::{self}};
use gui::game::GameWindow;

pub struct Cop0State {
    comms: SystemCommunication,    

    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    /// Current set of Cop0 registers
    cp0regs: [u64; 32],
    
    /// Group related registers together in parent nodes
    group_registers: bool,
    
    // internal state
    requested_register_state: bool,
}

impl Cop0State {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            cp0regs: [0; 32],
            group_registers: true,
            requested_register_state: false,
        }    
    }

    fn update(&mut self, _delta_time: f32) {
        // request the cop1 regs
        if !self.requested_register_state {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetCop0State,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };
            if debugger::Debugger::send_command(&self.comms, command).is_ok() {
                self.requested_register_state = true;
            }
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::Cop0State(cp0regs) => {
                    self.cp0regs = cp0regs;
                    self.requested_register_state = false;
                },
                
                r @ _ => {
                    warn!(target: "Cop0 State", "Unhandled message from debugger: {:?}", r);
                }
            }
        }
    }
}

impl GameWindow for Cop0State {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        let mut opened = true;

        self.update(ui.io().delta_time);

        let window = ui.window("COP0");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .build(|| {
            Utils::flag_button(ui, Some(&mut self.group_registers), "G", Some("Group registers by common utility"));
            // TODO ?
            // ui.same_line();
            // Utils::flag_button(ui, None, "<", Some("Collapse all"));
            // ui.same_line();
            // Utils::flag_button(ui, None, ">", Some("Show all"));
            ui.separator();

            // Index
            let draw_index = || {
                let reg = cpu::Cop0_Index;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : Index   :: 0x{:08X}###cop0_index", reg, value))
                    .default_open(false)
                    .build(|| {
                        let mut probe = (value >> 31) != 1;
                        ui.checkbox("P", &mut probe);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 31) probe bit; tlbp success or failure")
                        }
                        ui.same_line();
                        let index = value & 0x3F;
                        ui.text(format!("Index: 0x{:02X} ({}d)", index as u8, index as u8));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 5-0) index used by tlbr and tlbw instructions")
                        }
                    });
            };

            // Random
            let draw_random = || {
                let reg = cpu::Cop0_Random;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : Random  :: 0x{:08X}###cop0_random", reg, value))
                    .default_open(false)
                    .build(|| {
                        let index = value & 0x3F;
                        ui.text(format!("Index: 0x{:02X} ({}d)", index as u8, index as u8));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 5-0) index used by the tlbwr instruction");
                        }
                    });
            };

            // EntryLo0
            let draw_entrylo0 = || {
                let reg = cpu::Cop0_EntryLo0;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : EntryLo0:: 0x{:08X}###cop0_entrylo0", reg, value))
                    .default_open(false)
                    .build(|| {
                        let pfn = (value >> 6) & 0xF_FFFF;
                        let attr = (value >> 3) & 0x07;
                        let mut dirty = (value & 0x04) != 0;
                        let mut valid = (value & 0x02) != 0;
                        let mut global = (value & 0x01) != 0;
                        ui.text(format!("PFN: 0x{:08}", pfn));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 25-6) Page Frame Number; maybe better visualized as 0x{:08X}", pfn << 6)); // TODO
                        }
                        ui.text(format!("Attr: 0b{:03b} ({})", attr, if attr == 2 { "cache is not used" } else { "cache is used" }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 5-3) TLB page attributes");
                        }
                        ui.checkbox("Dirty", &mut dirty);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 2) dirty bit; if 1, the page is writable");
                        }
                        ui.same_line();
                        ui.checkbox("Valid", &mut valid);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 1) valid bit; if 1, this entry is valid");
                        }
                        ui.same_line();
                        ui.checkbox("Global", &mut global);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 0) global bit; when set on both EntryLo0 and EntryLo1, the ASID is is unused");
                        }
                    });
            };
            
            // EntryLo1
            let draw_entrylo1 = || {
                let reg = cpu::Cop0_EntryLo1;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : EntryLo1:: 0x{:08X}###cop0_entrylo1", reg, value))
                    .default_open(false)
                    .build(|| {
                        let pfn = (value >> 6) & 0xF_FFFF;
                        let attr = (value >> 3) & 0x07;
                        let mut dirty = (value & 0x04) != 0;
                        let mut valid = (value & 0x02) != 0;
                        let mut global = (value & 0x01) != 0;
                        ui.text(format!("PFN: 0x{:08}", pfn));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 25-6) Page Frame Number; maybe better visualized as 0x{:08X}", pfn << 6)); // TODO
                        }
                        ui.text(format!("Attr: 0b{:03b} ({})", attr, if attr == 2 { "cache is not used" } else { "cache is used" }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 5-3) TLB page attributes");
                        }
                        ui.checkbox("Dirty", &mut dirty);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 2) dirty bit; if 1, the page is writable");
                        }
                        ui.same_line();
                        ui.checkbox("Valid", &mut valid);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 1) valid bit; if 1, this entry is valid");
                        }
                        ui.same_line();
                        ui.checkbox("Global", &mut global);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 0) global bit; when set on both EntryLo0 and EntryLo1, the ASID is is unused");
                        }
                    });
            };

            // Context
            let draw_context = || {
                let reg = cpu::Cop0_Context;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : Context :: 0x{:016X}###cop0_context", reg, value))
                    .default_open(false)
                    .build(|| {
                        let ptebase = value >> 23;
                        ui.text(format!("PTEBase: 0x{:X}", ptebase));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 63-23) Base address of page table entry. Maybe better visualized as 0x{:016X}", ptebase << 23));
                        }
                        let badvpn2 = (value >> 4) & 0x7FFFF;
                        ui.text(format!("BadVPN2: 0x{:X}", badvpn2));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 22-4) Page number of the virtual address divided by 2. For 4KB pages, this is the\neven and odd pages at 0x{:08X}", badvpn2 << 13));
                        }
                    });
            };

            // PageMask
            let draw_pagemask = || {
                let reg = cpu::Cop0_PageMask;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : PageMask:: 0x{:08X}###cop0_pagemask", reg, value))
                    .default_open(false)
                    .build(|| {
                        let page_mask = (value >> 13) & 0xFFF;
                        let fixed_mask = (page_mask & 0xAAA) | (page_mask >> 1);
                        let offset_mask = (fixed_mask << 12) | 0xFFF;
                        ui.text(format!("PageMask: 0x{:X} (offset mask 0x{:X} page size {})", page_mask, offset_mask, offset_mask+1));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 24-13) Page comparison mask; determines the virtual page size of the entry"))
                        }
                    });
            };

            // Wired
            let draw_wired = || {
                let reg = cpu::Cop0_Wired;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : Wired   :: 0x{:08X}###cop0_wired", reg, value))
                    .default_open(false)
                    .build(|| {
                        let wired = value & 0x3F;
                        ui.text(format!("Wired: 0x{:02X} ({}d)", wired as u8, wired as u8));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 5-0) TLB wired boundary");
                        }
                    });
            };

            // BadVAddr
            let draw_badvaddr = || {
                let reg = cpu::Cop0_BadVAddr;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : BadVAddr:: 0x{:016X}###cop0_badvaddr", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("BadVAddr: 0x{:016X}", value));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 63-0) virtual address at which an error occrred");
                        }
                    });
            };

            // Count
            let draw_count = || {
                let reg = cpu::Cop0_Count;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{} : Count   :: 0x{:08X}###cop0_count", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("Count: 0x{:08X} ({})", value as u32, value as u32));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 31-0) Count increments at half the system clock");
                        }
                    });
            };

            // EntryHi
            let draw_entryhi = || {
                let reg = cpu::Cop0_EntryHi;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: EntryHi :: 0x{:016X}###cop0_entryhi", reg, value))
                    .default_open(false)
                    .build(|| {
                        let region = (value >> 62) & 0x03;
                        let vpn2 = (value >> 13) & 0x7FF_FFFF;
                        let asid = value & 0xFF;
                        ui.text(format!("Region: 0b{:02b} ({})", region, match region {
                            0 => "user", 1 => "supervisor", 2 => "invalid", 3 => "kernel", _ => panic!(),
                        }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 63-62) Region used to match virtual addresses");
                        }
                        ui.text(format!("VPN2: 0x{:X}", vpn2));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 39-13) Virtual Page Number divided by 2; may be better visualized as 0x{:010X} (40-bit address)", vpn2 << 13));
                        }
                        ui.text(format!("ASID: 0x{:02X}", asid as u8));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 7-0) Address Space ID; used when the Global bit is not enabled");
                        }
                    });
            };

            // Compare
            let draw_compare = || {
                let reg = cpu::Cop0_Compare;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: Compare :: 0x{:08X}###cop0_compare", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("Compare: 0x${:08X} ({})", value as u32, value as u32));
                    });
            };

            // Status
            let draw_status = || {
                let reg = cpu::Cop0_Status;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: Status  :: 0x{:08X}###cop0_status", reg, value))
                    .default_open(self.group_registers) // if in group registers, default to open
                    .build(|| {
                        let mut cp1en = ((value >> 29) & 0x01) != 0;
                        let mut cp0en = ((value >> 28) & 0x01) != 0;
                        let mut fr    = ((value >> 26) & 0x01) != 0;
                        let mut re    = ((value >> 25) & 0x01) != 0;
                        let mut kx    = ((value >> 7) & 0x01) != 0;
                        let mut sx    = ((value >> 6) & 0x01) != 0;
                        let mut ux    = ((value >> 5) & 0x01) != 0;
                        let mut erl   = ((value >> 2) & 0x01) != 0;
                        let mut exl   = ((value >> 1) & 0x01) != 0;
                        let mut ie    = ((value >> 0) & 0x01) != 0;
                        let ds        = (value >> 16) & 0x1FF;
                        let im        = (value >>  8) & 0xFF;
                        let ksu       = (value >>  3) & 0x03;
                        ui.text(format!("Mode: 0b{:02b} ({})", ksu, match ksu {
                            0 => "kernel", 1 => "supervisor", 2 => "user", 3 => "<invalid>", _ => panic!(),
                        }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 4-3) Specifies the current operating mode/level [kernel/supervisor/user]");
                        }
                        ui.checkbox("COP0 Enabled", &mut cp0en);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 28) Generates Coprocessor Unusable Exceptions if the coprocessor isn't enabled")
                        }
                        ui.same_line();
                        ui.checkbox("COP1 Enabled", &mut cp1en);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 29) Generates Coprocessor Unusable Exceptions if the coprocessor isn't enabled")
                        }
                        ui.checkbox("FR", &mut fr);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 26) When set, enables additional floating-point registers in COP1")
                        }
                        ui.same_line();
                        ui.checkbox("RE", &mut re);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 25) When set, enables reverse-endian in system and user modes")
                        }
                        ui.same_line();
                        ui.checkbox("KX", &mut kx);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 7) When set, enables 64-bit addressing in kernel mode")
                        }
                        ui.same_line();
                        ui.checkbox("SX", &mut sx);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 6) When set, enables 64-bit addressing in supervisor mode")
                        }
                        ui.same_line();
                        ui.checkbox("UX", &mut ux);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 5) When set, enables 64-bit addressing in user mode")
                        }
                        ui.same_line();
                        ui.checkbox("ERL", &mut erl);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 2) Error level; when set, an error has occrred")
                        }
                        ui.same_line();
                        ui.checkbox("EXL", &mut exl);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 1) Exception level; when set, an exception has occrred")
                        }
                        ui.checkbox("IE", &mut ie);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 0) Global interrupt enable flag")
                        }
                        ui.same_line();
                        let mut timer = ((im >> 7) & 0x01) != 0;
                        let mut rcp   = ((im >> 4) & 0x01) != 0;
                        ui.checkbox("Timer IM", &mut timer);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 15) Enables the Timer interrupt");
                        }
                        ui.same_line();
                        ui.checkbox("RCP IM", &mut rcp);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 10) Enables the external interrupts received from the RCP/MIPS Interface");
                        }
                        ui.tree_node_config(format!("Self-Diagnostic Status Field :: 0x{:02X}###cop0_status_ds", ds))
                            .default_open(false)
                            .build(|| {
                                let mut its = ((ds >> 8) & 0x01) != 0;
                                let mut bev = ((ds >> 6) & 0x01) != 0;
                                let mut ts  = ((ds >> 5) & 0x01) != 0;
                                let mut sr  = ((ds >> 4) & 0x01) != 0;
                                let mut ch  = ((ds >> 2) & 0x01) != 0;
                                ui.checkbox("ITS", &mut its);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 24) When set, enables instruction trace support");
                                }
                                ui.same_line();
                                ui.checkbox("BEV", &mut bev);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 22) Selects the location of the TLB miss and exception vectors (0 = )");
                                }
                                ui.same_line();
                                ui.checkbox("TS", &mut ts);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 21) Readonly, indicates when a TLB shutdown has occurred");
                                }
                                ui.same_line();
                                ui.checkbox("SR", &mut sr);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 20) Indicates whether a set reset or NMI has occurred");
                                }
                                ui.same_line();
                                ui.checkbox("CH", &mut ch);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 18) COP0 condition bit, used by software only");
                                }
                            });
                    });
            };

            // Cause
            let draw_cause = || {
                let reg = cpu::Cop0_Cause;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: Cause   :: 0x{:08X}###cop0_cause", reg, value))
                    .default_open(self.group_registers)
                    .build(|| {
                        let mut bd = ((value >> 31) & 0x01) != 0;
                        let ce = (value >> 28) & 0x03;
                        let ip = (value >> 8) & 0xFF;
                        let exc_code = (value >> 2) & 0x1F;

                        ui.checkbox("BD", &mut bd);
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bit 31) Set when the last exception was in a delay slot");
                        }
                        ui.text(format!("CE: {}", ce));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 29-28) Coprocessor that caused a Coprocessor Unusable exception");
                        }

                        ui.tree_node_config(format!("Pending Interrupts: 0x{:02X}###cop0_cause_ip", ip))
                            .default_open(true)
                            .build(|| {
                                let mut timer = ((ip >> 7) & 0x01) != 0;
                                let mut rcp   = ((ip >> 3) & 0x01) != 0;
                                ui.checkbox("Timer", &mut timer);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 15) 1 when a Timer interrupt is pending");
                                }
                                ui.same_line();
                                ui.checkbox("RCP", &mut rcp);
                                if ui.is_item_hovered() {
                                    ui.tooltip_text("(bit 12) 1 when an external RCP/MIPS Interface interrupt is pending");
                                }
                            });

                        ui.text(format!("ExcCode: 0x{:02X} ({}) - {}", exc_code, exc_code, match exc_code {
                            cpu::ExceptionCode_Int  => "Interrupt", cpu::ExceptionCode_Mod  => "TLB mod",
                            cpu::ExceptionCode_TLBL => "TLBL"     , cpu::ExceptionCode_TLBS => "TLBS",
                            cpu::ExceptionCode_AdEL => "AdEL"     , cpu::ExceptionCode_AdES => "AdES",
                            cpu::ExceptionCode_Sys  => "Syscall"  , cpu::ExceptionCode_Bp   => "Breakpoint",
                            cpu::ExceptionCode_RI   => "Reserved" , cpu::ExceptionCode_CpU  => "Coprocessor unusable",
                            cpu::ExceptionCode_Ov   => "Overflow" , cpu::ExceptionCode_Tr   => "Trap",
                            cpu::ExceptionCode_FPE  => "Floating-point",
                            _ => panic!(),
                        }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 6-2) Exception code field of the last exception");
                        }

                    });
            };

            // EPC
            let draw_epc = || {
                let reg = cpu::Cop0_EPC;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: EPC     :: 0x{:016X}###cop0_epc", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("EPC: 0x{:016X}", value));
                        if ui.is_item_hovered() {
                            // Technically can be the branch before a delay slot if an exception occurred in the delay slot
                            ui.tooltip_text("Exception Program Counter contains the address* that was the cause of an exception");
                        }
                    });
            };

            // PRId
            let draw_prid = || {
                let reg = cpu::Cop0_PRId;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: PRId    :: 0x{:08X}###cop0_prid", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("PRId: 0x{:016X}", value));
                        if ui.is_item_hovered() {
                            // Technically can be the branch before a delay slot if an exception occurred in the delay slot
                            ui.tooltip_text("Processor Revision Identifier");
                        }
                    });
            };

            // Config
            let draw_config = || {
                let reg = cpu::Cop0_Config;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: Config  :: 0x{:08X}###cop0_config", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text("TODO");
                    });
            };

            // LLAddr
            let draw_lladdr = || {
                let reg = cpu::Cop0_LLAddr;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: LLAddr  :: 0x{:08X}###cop0_lladdr", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("PAddr: 0x{:08X}", value));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 27-0) Physical address used in the most recent LL instruction (0x{:08X})", value << 4));
                        }
                    });
            };

            // XContext
            let draw_xcontext = || {
                let reg = cpu::Cop0_XContext;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: XContext:: 0x{:016X}###cop0_xcontext", reg, value))
                    .default_open(false)
                    .build(|| {
                        let ptebase = value >> 33;
                        let r       = (value >> 31) & 0x03;
                        let badvpn2 = (value >> 4) & 0x7FFFFFF;

                        ui.text(format!("PTEBase: 0x{:08X}", ptebase as u32));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 63-33) Base address of the page table entry");
                        }
                        ui.text(format!("R: 0b{:02b} ({})", r, match r {
                            0 => "user", 1 => "supervisor", 2 => "<invalid>", 3 => "kernel", _ => panic!(),
                        }));
                        if ui.is_item_hovered() {
                            ui.tooltip_text("(bits 32-31) address region identifier");
                        }
                        ui.text(format!("BadVPN2: 0x{:04X}", badvpn2));
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bits 30-4) VPN2 address that cause an invalid translation (virtual address 0x{:010X})", badvpn2 << 13));
                        }
                    });
            };

            // PErr
            let draw_perr = || {
                let reg = cpu::Cop0_PErr;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: PErr    :: 0x{:08X}###cop0_perr", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text(format!("Diagnostic: 0x{:02X}", value as u8));
                        if ui.is_item_hovered() {
                            // Technically can be the branch before a delay slot if an exception occurred in the delay slot
                            ui.tooltip_text("(bits 7-0) Parity Error; not used by the hardware");
                        }
                    });
            };

            // CacheErr
            let draw_cacheerr = || {
                let reg = cpu::Cop0_CacheErr;
                let value = self.cp0regs[reg];
                ui.tree_node_config(format!("c{}: CacheErr:: 0x{:08X}###cop0_cacheerr", reg, value))
                    .default_open(false)
                    .build(|| {
                        ui.text("This register must be zero");
                    });
            };

            if self.group_registers {
                ui.tree_node_config("Control/State")
                    .default_open(false)
                    .build(|| {
                        draw_status();
                    });
                ui.tree_node_config(format!("Timer"))
                    .default_open(false)
                    .build(|| {
                        draw_count();
                        draw_compare();
                    });
                ui.tree_node_config(format!("TLB"))
                    .default_open(false)
                    .build(|| {
                        ui.tree_node_config("Index, Random, Wired")
                            .default_open(false)
                            .build(|| {
                                draw_index();
                                draw_random();
                                draw_wired();
                            });
                        ui.tree_node_config("PageMask, EntryLo0, EntryLo1, EntryHi")
                            .default_open(true)
                            .build(|| {
                                draw_pagemask();
                                draw_entrylo0();
                                draw_entrylo1();
                                draw_entryhi();
                            });
                    });
                ui.tree_node_config("Exceptions")
                    .default_open(false)
                    .build(|| {
                        draw_cause();
                        draw_epc();
                        ui.tree_node_config("TLB")
                            .default_open(true)
                            .build(|| {
                                draw_badvaddr();
                                draw_context();
                                draw_xcontext();
                            });
                    });
                ui.tree_node_config("Other")
                    .default_open(false)
                    .build(|| {
                        draw_prid();
                        draw_config();
                        draw_lladdr();
                        draw_perr();
                        draw_cacheerr();
                    });
            } else {
                draw_index();
                draw_random();
                draw_entrylo0();
                draw_entrylo1();
                draw_context();
                draw_pagemask();
                draw_wired();
                draw_badvaddr();
                draw_count();
                draw_entryhi();
                draw_compare();
                draw_status();
                draw_cause();
                draw_epc();
                draw_prid();
                draw_config();
                draw_lladdr();
                draw_xcontext();
                draw_perr();
                draw_cacheerr();
            }
        });

        opened
    }
}


