use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::debugger;
use gui::game::{GameWindow, Utils};

pub struct Breakpoints {
    comms: SystemCommunication,    

    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // list of current breakpoints
    breakpoints: Vec<debugger::BreakpointInfo>,
    
    // show table resizers
    show_resizers: bool,
    
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
            // ui.same_line();
            // Utils::flag_button(ui, Some(&mut self.use_abi_names), "A", Some("Display registers using the ABI names"));
            // ui.same_line();

            // Utils::flag_button(ui, Some(&mut self.show_64bit_regs), "64", Some("Toggle between 64-bit and 32-bit-truncated"));
            // ui.same_line();

            // if Utils::flag_button(ui, None, format!("{}", self.num_columns), Some("Layout registers in this many columns")) {
            //     match self.num_columns {
            //         8 => { self.num_columns = 1; },
            //         _ => { self.num_columns *= 2; },
            //     }                                
            // }
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
                        
                    }

                    ui.same_line_with_spacing(0.0, 0.0);
                    let mut write = (breakpoint_info.mode & debugger::BP_WRITE) != 0;
                    if ui.checkbox("##write", &mut write) {
                        
                    }

                    ui.same_line_with_spacing(0.0, 0.0);
                    let mut execute = (breakpoint_info.mode & debugger::BP_EXEC) != 0;
                    if ui.checkbox("##execute", &mut execute) {
                        
                    }

                    ui.table_next_column();

                    // expand text box to the full length of the cell
                    let available_size = ui.content_region_avail();
                    let size_token = ui.push_item_width(available_size[0]);
                    let mut address = format!("0x{:08X}", breakpoint_info.address);
                    ui.input_text("##location", &mut address).enter_returns_true(true).build();

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
