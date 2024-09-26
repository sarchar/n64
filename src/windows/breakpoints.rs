use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use imgui::DrawList;
use n64::debugger;
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

            ui.separator();

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
            
        });

        if !opened {
            self.comms.decrement_debugger_windows();
        }

        opened
    }
}
