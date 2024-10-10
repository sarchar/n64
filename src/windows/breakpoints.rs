use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::{debugger, cpu, mips};
use gui::game::{GameWindow, Utils};

#[derive(Clone, Debug, Default)]
struct PopupState {
    read: bool,
    write: bool,
    execute: bool,
    address: String,
    enable: bool,
    dont_focus: bool,
    error_message: String,
}

pub struct Breakpoints {
    comms: SystemCommunication,    

    /// request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    /// list of current breakpoints
    breakpoints: Vec<debugger::BreakpointInfo>,
    
    /// Enabled exception breakpoints
    break_on_exception: u32,
    break_on_interrupt: u8,
    break_on_rcp: u8,
    break_on_dirty: bool,

    /// show table resizers
    show_resizers: bool,

    /// new breakpoint popup state
    popup_state: PopupState,
    
    // internal state
    requested_breakpoints: bool,
}

impl Breakpoints {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            break_on_exception: 0,
            break_on_interrupt: 0,
            break_on_rcp: 0,
            break_on_dirty: true, // clear break_on immediately
            show_resizers: false,
            requested_breakpoints: false,
            breakpoints: Vec::new(),
            popup_state: PopupState::default(),
        }    
    }

    fn update(&mut self, _delta_time: f32) {
        if !self.requested_breakpoints {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetBreakpoints,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };

            self.requested_breakpoints = debugger::Debugger::send_command(&self.comms, command).is_ok();
        }

        if self.break_on_dirty {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::SetBreakOnException(self.break_on_exception, self.break_on_interrupt, self.break_on_rcp),
                response_channel: None,
            };

            self.break_on_dirty = !debugger::Debugger::send_command(&self.comms, command).is_ok(); 
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::Breakpoints(breakpoints) => {
                    self.breakpoints = breakpoints.into_values().collect::<Vec<debugger::BreakpointInfo>>();
                    self.breakpoints.sort_by(|a, b| a.address.cmp(&b.address)); // sort list by address
                    self.requested_breakpoints = false;                    
                },

                _ => {},
            }
        }
    }

    fn render_popups(&mut self, ui: &imgui::Ui) {
        let result = ui.modal_popup("Set Breakpoint", || -> Option<String> {
            let mut accept_dialog = false;
            
            ui.checkbox("Read", &mut self.popup_state.read);
            ui.same_line();

            ui.checkbox("Write", &mut self.popup_state.write);
            ui.same_line();

            ui.checkbox("Execute", &mut self.popup_state.execute);

            if !self.popup_state.dont_focus {
                ui.set_keyboard_focus_here();
                self.popup_state.dont_focus = true;
            }

            if ui.input_text("Address", &mut self.popup_state.address).enter_returns_true(true).build() {
                accept_dialog = true;
            }

            if ui.is_item_hovered() {
                ui.tooltip_text("Virtual (for execute) or Physical (for R/W). 8 digit hex numbers will be sign extended")
            }

            ui.same_line();
            ui.checkbox("Enable", &mut self.popup_state.enable);

            if ui.button("Cancel") {
                ui.close_current_popup();
            }

            ui.same_line();
            if ui.button("OK") {
                accept_dialog = true;
            }

            if accept_dialog {
                ui.close_current_popup();
                if let Some(address) = Utils::parse_u64_se32(&self.popup_state.address) {
                    // create a breakpoint
                    let breakpoint_info = debugger::BreakpointInfo {
                        address,
                        mode: if self.popup_state.read    { debugger::BP_READ  } else { 0 }
                            | if self.popup_state.write   { debugger::BP_WRITE } else { 0 }
                            | if self.popup_state.execute { debugger::BP_EXEC  } else { 0 },
                        enable: self.popup_state.enable,
                        ..Default::default()
                    };

                    let command = debugger::DebuggerCommand {
                        command_request: debugger::DebuggerCommandRequest::SetBreakpoint(breakpoint_info.clone()),
                        response_channel: None,
                    };

                    let _ = debugger::Debugger::send_command(&self.comms, command);

                    self.breakpoints.push(breakpoint_info);
                    None
                } else {
                    // Another popup!
                    Some(format!("Could not parse '{}'", self.popup_state.address))
                }
            } else {
                None
            }
        });

        if let Some(Some(error_message)) = result {
            self.popup_state.error_message = error_message;
            Utils::show_messagebox(ui);
        }

        // always draw even though it may not be open
        Utils::messagebox(ui, "Error", &self.popup_state.error_message);
    }
}

impl GameWindow for Breakpoints {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        self.update(ui.io().delta_time);
        
        let mut opened = true;
        let window = ui.window("Breakpoints");     
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .build(|| {

            ui.columns(2, "column string", true);

            Utils::flag_button(ui, Some(&mut self.show_resizers), "R", Some("Show column resizers"));

            ui.same_line();
            if Utils::flag_button(ui, None, "+", Some("Set a new breakpoint")) {
                self.popup_state = PopupState {
                    execute: true,
                    enable : true,
                    ..Default::default()
                };
                ui.open_popup("Set Breakpoint");
            }
            self.render_popups(ui);

            Utils::good_separator(ui);
            
            let columns =  [
                imgui::TableColumnSetup { name: "Enable"  , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(0) },
                imgui::TableColumnSetup { name: "RWX"     , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(1) },
                imgui::TableColumnSetup { name: "Location", flags: imgui::TableColumnFlags::WIDTH_STRETCH, init_width_or_weight: 10.0, user_id: ui.new_id(2) },
                imgui::TableColumnSetup { name: "Del"     , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(3) },
            ];

            let table_flags = if self.show_resizers {
                imgui::TableFlags::BORDERS_INNER_V
            } else {
                imgui::TableFlags::NO_BORDERS_IN_BODY
            } | imgui::TableFlags::RESIZABLE | imgui::TableFlags::NO_PAD_OUTER_X;

            // use begin_table_with_sizing so we can skip the ui.table_headers_row call, as we want to
            // set up and use columns but we don't want to display the headers row
            if let Some(_) = ui.begin_table_with_sizing("Breakpoints", columns.len(), table_flags, [0.0, 0.0], 0.0) {
                // create the columns
                for column in columns {
                    ui.table_setup_column_with(column);    
                }

                ui.table_headers_row();

                for breakpoint_info in &mut self.breakpoints {
                    let _id_stack_token = ui.push_id(format!("{:016X}", breakpoint_info.address));
                    
                    ui.table_next_column();
                    if ui.checkbox("##enabled", &mut breakpoint_info.enable) {
                        let command = debugger::DebuggerCommand {
                            command_request: debugger::DebuggerCommandRequest::EnableBreakpoint(breakpoint_info.address, breakpoint_info.enable),
                            response_channel: None,
                        };

                        let _ = debugger::Debugger::send_command(&self.comms, command);
                    }

                    ui.table_next_column();
                    let mut read = (breakpoint_info.mode & debugger::BP_READ) != 0;
                    if ui.checkbox("##read", &mut read) {
                        breakpoint_info.mode = if read {
                            breakpoint_info.mode | debugger::BP_READ
                        } else {
                            breakpoint_info.mode & !debugger::BP_READ
                        };

                        let command = debugger::DebuggerCommand {
                            command_request: debugger::DebuggerCommandRequest::ChangeBreakpointMode(breakpoint_info.address, breakpoint_info.mode),
                            response_channel: None,
                        };

                        let _ = debugger::Debugger::send_command(&self.comms, command);
                    }

                    ui.same_line_with_spacing(0.0, 0.0);
                    let mut write = (breakpoint_info.mode & debugger::BP_WRITE) != 0;
                    if ui.checkbox("##write", &mut write) {
                        breakpoint_info.mode = if write {
                            breakpoint_info.mode | debugger::BP_WRITE
                        } else {
                            breakpoint_info.mode & !debugger::BP_WRITE
                        };

                        let command = debugger::DebuggerCommand {
                            command_request: debugger::DebuggerCommandRequest::ChangeBreakpointMode(breakpoint_info.address, breakpoint_info.mode),
                            response_channel: None,
                        };

                        let _ = debugger::Debugger::send_command(&self.comms, command);
                    }

                    ui.same_line_with_spacing(0.0, 0.0);
                    let mut execute = (breakpoint_info.mode & debugger::BP_EXEC) != 0;
                    if ui.checkbox("##execute", &mut execute) {
                        breakpoint_info.mode = if execute {
                            breakpoint_info.mode | debugger::BP_EXEC
                        } else {
                            breakpoint_info.mode & !debugger::BP_EXEC
                        };

                        let command = debugger::DebuggerCommand {
                            command_request: debugger::DebuggerCommandRequest::ChangeBreakpointMode(breakpoint_info.address, breakpoint_info.mode),
                            response_channel: None,
                        };

                        let _ = debugger::Debugger::send_command(&self.comms, command);
                    }

                    ui.table_next_column();

                    // expand text box to the full length of the cell
                    let available_size = ui.content_region_avail();
                    let size_token = ui.push_item_width(available_size[0]);
                    ui.text(format!("0x{:08X}", breakpoint_info.address));

                    ui.table_next_column();
                    if Utils::flag_button(ui, None, "X", Some("Remove this breakpoint")) {
                        let command = debugger::DebuggerCommand {
                            command_request: debugger::DebuggerCommandRequest::RemoveBreakpoint(breakpoint_info.address),
                            response_channel: None,
                        };

                        let _ = debugger::Debugger::send_command(&self.comms, command);
                    }
                    size_token.end();
                }
            }

            ui.next_column();

            ////////////////////////////////////////////////////////////////
            let widget_width = ui.current_column_width() / 5.0;
 
            ui.set_next_item_width(widget_width);
            let mut tlb_load = (self.break_on_exception & (1 << cpu::ExceptionCode_TLBL)) != 0;
            if ui.checkbox("TLB Load", &mut tlb_load) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_TLBL;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("TLB Miss exception (load or instruction fetch)"); }

            ui.same_line_with_pos(1.0*widget_width);
            let mut tlb_store = (self.break_on_exception & (1 << cpu::ExceptionCode_TLBS)) != 0;
            if ui.checkbox("TLB Store", &mut tlb_store) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_TLBS;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("TLB Miss exception (store)"); }

            ui.same_line_with_pos(2.0*widget_width);
            let mut tlb_mod = (self.break_on_exception & (1 << cpu::ExceptionCode_Mod)) != 0;
            if ui.checkbox("TLB Mod", &mut tlb_mod) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Mod;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("TLB modification"); }

            ui.same_line_with_pos(3.0*widget_width);
            let mut adel = (self.break_on_exception & (1 << cpu::ExceptionCode_AdEL)) != 0;
            if ui.checkbox("Addr Load", &mut adel) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_AdEL;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Address Error exception (load or instruction fetch)"); }

            ui.same_line_with_pos(4.0*widget_width);
            let mut ades = (self.break_on_exception & (1 << cpu::ExceptionCode_AdES)) != 0;
            if ui.checkbox("Addr Store", &mut ades) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_AdES;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Address Error exception (store)"); }

            ////////////////////////////////////////////////////////////////
            let mut overflow = (self.break_on_exception & (1 << cpu::ExceptionCode_Ov)) != 0;
            if ui.checkbox("Overflow", &mut overflow) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Ov;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Arithmetic Overflow exception"); }

            ui.same_line_with_pos(1.0*widget_width);
            let mut fpe = (self.break_on_exception & (1 << cpu::ExceptionCode_FPE)) != 0;
            if ui.checkbox("FPE", &mut fpe) { 
                self.break_on_exception ^= 1 << cpu::ExceptionCode_FPE;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Floating-point exception"); }

            ui.same_line_with_pos(2.0*widget_width);
            let mut coprocessor = (self.break_on_exception & (1 << cpu::ExceptionCode_CpU)) != 0;
            if ui.checkbox("Coprocessor", &mut coprocessor) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_CpU;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Coprocessor Unusable exception"); }

            ui.same_line_with_pos(3.0*widget_width);
            let mut reserved = (self.break_on_exception & (1 << cpu::ExceptionCode_RI)) != 0;
            if ui.checkbox("Reserved", &mut reserved) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_RI;
                self.break_on_dirty = true;
            }
            if ui.is_item_hovered() { ui.tooltip_text("Reserved Instruction exception"); }

            ////////////////////////////////////////////////////////////////
            let mut trap = (self.break_on_exception & (1 << cpu::ExceptionCode_Tr)) != 0;
            if ui.checkbox("Trap", &mut trap) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Tr;
                self.break_on_dirty = true;
            } 
            if ui.is_item_hovered() { ui.tooltip_text("Trap instruction exception"); }

            ui.same_line_with_pos(1.0*widget_width);
            let mut syscall = (self.break_on_exception & (1 << cpu::ExceptionCode_Sys)) != 0;
            if ui.checkbox("Syscall", &mut syscall) { 
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Sys;
                self.break_on_dirty = true;
            } 
            if ui.is_item_hovered() { ui.tooltip_text("Syscall exception"); }

            ui.same_line_with_pos(2.0*widget_width);
            let mut breakpoint = (self.break_on_exception & (1 << cpu::ExceptionCode_Bp)) != 0;
            if ui.checkbox("Breakpoint", &mut breakpoint) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Bp;
                self.break_on_dirty = true;
            } 
            if ui.is_item_hovered() { ui.tooltip_text("Breakpoint exception"); }

            ui.same_line_with_pos(3.0*widget_width);
            let mut interrupt = (self.break_on_exception & (1 << cpu::ExceptionCode_Int)) != 0;
            if ui.checkbox("Interrupt", &mut interrupt) {
                self.break_on_exception ^= 1 << cpu::ExceptionCode_Int;
                self.break_on_dirty = true;
                // if we toggled on and no interrupts are enabled, enable them all
                if interrupt && self.break_on_interrupt == 0 {
                    self.break_on_interrupt = (cpu::InterruptCode_RCP | cpu::InterruptCode_Timer) as u8;
                    if self.break_on_rcp == 0 {
                        self.break_on_rcp = 0x3F;
                    }
                }
            }
            if ui.is_item_hovered() { ui.tooltip_text("Interrupt (select which interrupts below)"); }

            Utils::good_separator(ui);

            ////////////////////////////////////////////////////////////////
            ui.tree_node_config("Interrupts").default_open(true).build(|| {
                let old_break_on_interrupt = self.break_on_interrupt;
 
                let mut int_timer = (self.break_on_interrupt & (cpu::InterruptCode_Timer as u8)) != 0;
                if ui.checkbox("Timer", &mut int_timer) {
                    self.break_on_interrupt ^= cpu::InterruptCode_Timer as u8;
                    self.break_on_dirty = true;
                }
                if ui.is_item_hovered() { ui.tooltip_text("Timer interrupt"); }

                ui.same_line();
                let mut int_rcp = (self.break_on_interrupt & (cpu::InterruptCode_RCP as u8)) != 0;
                if ui.checkbox("RCP", &mut int_rcp) {
                    self.break_on_interrupt ^= cpu::InterruptCode_RCP as u8;
                    self.break_on_dirty = true;
                    // if toggled ON and no RCP interrupts are enabled, enable them all
                    if int_rcp && self.break_on_rcp == 0 {
                        self.break_on_rcp = 0x3F;
                    }
                }
                if ui.is_item_hovered() { ui.tooltip_text("External Interrupt caused by the RCP"); }

                // if we enabled an Interrupt from nothing enabled, make sure ExceptionCode_Int is set
                if old_break_on_interrupt == 0 && self.break_on_interrupt != 0 {
                    self.break_on_exception |= 1 << cpu::ExceptionCode_Int;
                    self.break_on_dirty = true;
                }

                ui.tree_node_config("RCP interrupts").default_open(true).build(|| {
                    let old_break_on_rcp = self.break_on_rcp;

                    let mut sp = (self.break_on_rcp & (1 << mips::IMask_SP)) != 0;
                    if ui.checkbox("SP", &mut sp) {
                        self.break_on_rcp ^= 1 << mips::IMask_SP;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("RSP break") }

                    ui.same_line();
                    let mut si = (self.break_on_rcp & (1 << mips::IMask_SI)) != 0;
                    if ui.checkbox("SI", &mut si) {
                        self.break_on_rcp ^= 1 << mips::IMask_SI;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("Serial Interface DMA to/from PIF RAM finished") }

                    ui.same_line();
                    let mut ai = (self.break_on_rcp & (1 << mips::IMask_AI)) != 0;
                    if ui.checkbox("AI", &mut ai) {
                        self.break_on_rcp ^= 1 << mips::IMask_AI;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("Audio Interface (audio DMA start)"); }

                    ui.same_line();
                    let mut vi = (self.break_on_rcp & (1 << mips::IMask_VI)) != 0;
                    if ui.checkbox("VI", &mut vi) {
                        self.break_on_rcp ^= 1 << mips::IMask_VI;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("Video Interface hit VI_V_INTR"); }

                    ui.same_line();
                    let mut pi = (self.break_on_rcp & (1 << mips::IMask_PI)) != 0;
                    if ui.checkbox("PI", &mut pi) {
                        self.break_on_rcp ^= 1 << mips::IMask_PI;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("Peripheral Inteface DMA transfer finished"); }

                    ui.same_line();
                    let mut dp = (self.break_on_rcp & (1 << mips::IMask_DP)) != 0;
                    if ui.checkbox("DP", &mut dp) {
                        self.break_on_rcp ^= 1 << mips::IMask_DP;
                        self.break_on_dirty = true;
                    }
                    if ui.is_item_hovered() { ui.tooltip_text("RDP FULL_SYNC interrupt"); }

                    // if all interrupts were disabled and we enable any one of them, make sure RCP interrupt is enabled
                    if old_break_on_rcp == 0 && self.break_on_rcp != 0 {
                        self.break_on_interrupt |= cpu::InterruptCode_RCP as u8;
                        self.break_on_dirty = true;
                        // if break_on_interrupt went from 0 to something, then make sure ExceptionCode_Int is enabled
                        if (self.break_on_interrupt & !(cpu::InterruptCode_RCP as u8)) == 0 {
                            self.break_on_exception |= 1 << cpu::ExceptionCode_Int;
                        }
                    }

                });
            });
        });

        opened
    }
}

impl Drop for Breakpoints {
    fn drop(&mut self) {
        self.comms.decrement_debugger_windows();
    }
}
