
use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::{cpu::{Cpu, DisassembledInstruction}, debugger::{self}};
use gui::game::{GameWindow, Utils};

pub struct Registers {
    comms: SystemCommunication,    

    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // register values
    register_values: [u64; 32],
    register_value_strings: Vec<String>,

    // highlight individual registers
    highlighted_registers: [Option<HighlightInfo>; 32],
    
    // display registers using ABI names
    use_abi_names: bool,

    // display full 64-bit values or truncate to 32-bit
    show_64bit_regs: bool,
    
    // display registers in 1, 2, 4, or 8 columns
    num_columns: usize,

    // internal state
    requested_register_state: bool,
    requested_cpu_state: bool,
}

const HIGHLIGHT_COLORS: [[f32; 4]; 3] = [
    [0.6, 0.0, 0.0, 1.0],
    [0.0, 0.6, 0.0, 1.0],
    [0.0, 0.0, 0.6, 1.0],
];

#[derive(Copy, Clone, Debug, Default)]
struct HighlightInfo {
    color   : [f32; 4],
    duration: Option<f32>,
    // start   : f32,
}

impl Registers {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        let mut register_value_strings = Vec::new();
        for _ in 0..32 {
            register_value_strings.push(String::new());
        } 
        
        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            register_values: [0; 32],
            register_value_strings,
            highlighted_registers: [None; 32],
            use_abi_names: true,
            show_64bit_regs: true,
            num_columns: 1,
            requested_register_state: false,
            requested_cpu_state: false,
        }    
    }
}

impl GameWindow for Registers {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        let mut opened = true;
        
        // request and process the register data
        if !self.requested_register_state {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetCpuRegisters,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };
            if debugger::Debugger::send_command(&self.comms, command).is_ok() {
                self.requested_register_state = true;
            }
        }
        
        // also request the cpu state with 1 instruction under the pc
        if !self.requested_cpu_state {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetCpuState(Some((0, 1))),
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };
            if debugger::Debugger::send_command(&self.comms, command).is_ok() {
                self.requested_cpu_state = true;
            }
        }
        
        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::CpuRegisters(regs) => {
                    self.register_values = regs;

                    for i in 0..32 {
                        if self.show_64bit_regs {
                            self.register_value_strings[i] = format!("0x{:016X}", self.register_values[i]);
                        } else {
                            self.register_value_strings[i] = format!("0x{:08X}", self.register_values[i] as u32);
                        }
                    }

                    self.requested_register_state = false;
                },

                debugger::DebuggerCommandResponse::CpuState(cpu_state) => {
                    // clear out all non-timed highlighted registers
                    for i in 0..32 {
                        if self.highlighted_registers[i].is_some() {
                            if self.highlighted_registers[i].as_ref().unwrap().duration.is_none() {
                                self.highlighted_registers[i] = None;
                            }
                        }
                    }

                    if let Some(memory) = cpu_state.instruction_memory {
                        let disassembly = Cpu::disassemble(cpu_state.next_instruction_pc, memory[0]);
                        let mut hl = 0;
                        for (_, ref operand) in disassembly.iter().enumerate().skip(1) {
                            match operand {
                                DisassembledInstruction::Register(rnum) => {
                                    self.highlighted_registers[*rnum as usize] = Some(HighlightInfo {
                                        color: HIGHLIGHT_COLORS[hl],
                                        ..Default::default()
                                    });
                                    hl += 1;
                                },

                                DisassembledInstruction::OffsetRegister(_, rnum) => {
                                    self.highlighted_registers[*rnum as usize] = Some(HighlightInfo {
                                        color: HIGHLIGHT_COLORS[hl],
                                        ..Default::default()
                                    });
                                    hl += 1;
                                },

                                _ => {},
                            }

                            // give up if there's any more (there should never be)
                            if hl == 3 { break; }
                        }
                    }

                    self.requested_cpu_state = false;
                },

                _ => {},
            }
        }

        let window = ui.window("Registers");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .build(|| {
            Utils::flag_button(ui, Some(&mut self.use_abi_names), "A", Some("Display registers using the ABI names"));
            ui.same_line();

            Utils::flag_button(ui, Some(&mut self.show_64bit_regs), "64", Some("Toggle between 64-bit and 32-bit-truncated"));
            ui.same_line();

            if Utils::flag_button(ui, None, format!("{}", self.num_columns), Some("Layout registers in this many columns")) {
                match self.num_columns {
                    8 => { self.num_columns = 1; },
                    _ => { self.num_columns *= 2; },
                }
            }
            ui.separator();

            // calculate the width of a label (always 3/4 chars)
            let label_size = ui.calc_text_size(if self.use_abi_names { "xy:" } else { "xyz:" });

            // along with internal padding, divide up the content width into 4 sections to compute the text box width
            let column_padding = 18.0; //unsafe { ui.style().frame_padding[0] };
            let region_size = ui.content_region_avail();
            let text_input_size = (region_size[0] - ((self.num_columns as f32) - 1.0) * column_padding) / (self.num_columns as f32) - label_size[0];
            let rows = 32 / self.num_columns;

            for row in 0..rows {
                for col in 0..self.num_columns {
                    let rnum = row * self.num_columns + col;
                
                    if self.use_abi_names {
                        ui.text(format!("{:2}:", Cpu::abi_name(rnum)));
                    } else {
                        ui.text(format!("{:-3}:", Cpu::register_name(rnum)));
                    }
                    ui.same_line_with_spacing(0.0, 0.0);

                    // determine bg color for the text input
                    let mut style_tokens = Vec::new();
                    if let Some(ref mut hl) = self.highlighted_registers[rnum] {
                        style_tokens.push(ui.push_style_color(imgui::StyleColor::FrameBg, hl.color));
                        // hl.duration -= ui.io().delta_time;
                        // if hl.duration < 0.0 {
                        //     hl.duration = None;
                        // }
                    }

                    let token = ui.push_item_width(text_input_size);

                    // id for input_text needs to be unique to allow editing to work
                    if ui.input_text(format!("##r{}", rnum), &mut self.register_value_strings[rnum as usize])
                           .chars_hexadecimal(true)
                           .enter_returns_true(true)
                           .build() {
                        println!("here");               
                    }
                    token.end();

                    if ui.is_item_hovered() {
                        let details = format!("Dec: {} (64-bit)\nDec: {} (32-bit)", self.register_values[rnum], self.register_values[rnum] as u32);
                        ui.tooltip_text(details);
                    }

                    for st in style_tokens {
                        st.end();
                    }

                    if col != (self.num_columns - 1) {
                        // ui.same_line();
                        ui.same_line_with_spacing(0.0, column_padding);
                    }
                }
            }
        });

        if !opened {
            self.comms.decrement_debugger_windows();
        }

        opened
    }
}
