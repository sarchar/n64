
use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::{cpu::{Cpu, DisassembledInstruction}, debugger::{self}};
use gui::game::{GameWindow, Utils};

pub struct Cop1State {
    comms: SystemCommunication,    

    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // cpu running state
    cpu_running: bool,
    
    /// floatpoing point control/status register
    fcr_control_status: u32,
    
    /// FR bit state
    fr_bit: bool,
    
    // register values
    fgr_values: [n64::cop1::Fgr; 32],
    fgr_value_strings: Vec<String>,

    // highlight individual registers
    highlighted_registers: [HighlightInfo; 32],
    
    // display full 64-bit values or truncate to 32-bit
    use_f64: bool,

    // display values in hex
    show_hex: bool,
    
    // display fgr in 1, 2, 4, or 8 columns
    num_columns: usize,

    // internal state
    requested_register_state: bool,
    requested_cpu_state: bool,
}

// const HIGHLIGHT_COLORS: [[f32; 4]; 3] = [
//     [0.4, 0.1, 0.1, 1.0],
//     [0.1, 0.4, 0.1, 1.0],
//     [0.1, 0.1, 0.4, 1.0],
// ];

#[derive(Copy, Clone, Debug, Default)]
struct HighlightInfo {
    color   : [f32; 4],
    duration: Option<f32>,
}

impl Cop1State {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        let mut fgr_value_strings = Vec::new();
        for _ in 0..32 {
            fgr_value_strings.push(String::new());
        } 
        
        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            cpu_running: true,
            fcr_control_status: 0,
            fr_bit: false,
            fgr_values: [n64::cop1::Fgr { as_u64: 0 }; 32],
            fgr_value_strings,
            highlighted_registers: [HighlightInfo::default(); 32],
            use_f64: false,
            show_hex: false,
            num_columns: 2,
            requested_register_state: false,
            requested_cpu_state: false,
        }    
    }

    fn update(&mut self, delta_time: f32, frame_bg: [f32; 4], disabled_bg: [f32; 4]) {
        // request and process the register data
        if !self.requested_register_state {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetCop1State,
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

        // animate durations
        for i in 0..32 {
            if let Some(ref mut duration) = self.highlighted_registers[i].duration {
                *duration -= delta_time;
                if *duration <= 0.0 {
                    self.highlighted_registers[i].duration = None;
                }
            }
        }
        
        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::Cop1State { fgr, fcr, fr_bit } => {
                    // look over which registers have changed and start a fade duration
                    for i in 0..32 {
                        if self.use_f64 {
                            if unsafe { fgr[i].as_u64 != self.fgr_values[i].as_u64 } {
                                self.highlighted_registers[i].duration = Some(Utils::DATA_CHANGED_FADE_TIME);
                            }
                        } else {
                            if unsafe { fgr[i].as_u32 != self.fgr_values[i].as_u32 } {
                                self.highlighted_registers[i].duration = Some(Utils::DATA_CHANGED_FADE_TIME);
                            }
                        }
                    }

                    self.fcr_control_status = fcr;
                    self.fr_bit = fr_bit;
                    self.fgr_values = fgr;

                    for i in 0..32 {
                        if self.show_hex {
                            if self.use_f64 {
                                self.fgr_value_strings[i] = format!("0x{:016X}", unsafe { self.fgr_values[i].as_u64 });
                            } else {
                                self.fgr_value_strings[i] = format!("0x{:08X}", unsafe { self.fgr_values[i].as_u32 });
                            }
                        } else {
                            if self.use_f64 {
                                self.fgr_value_strings[i] = format!("{:.16}", unsafe { self.fgr_values[i].as_f64 });
                            } else {
                                self.fgr_value_strings[i] = format!("{:.8}", unsafe { self.fgr_values[i].as_f32 });
                            }
                        }
                    }

                    self.requested_register_state = false;
                },

                debugger::DebuggerCommandResponse::CpuState(cpu_state) => {
                    self.cpu_running = cpu_state.running;
                    
                    // clear out all non-timed highlighted registers
                    for i in 0..32 {
                        self.highlighted_registers[i].color = if (i % 2) == 0 || self.fr_bit { frame_bg } else { disabled_bg };
                    }

                    if let Some(memory) = cpu_state.instruction_memory {
                        let disassembly = Cpu::disassemble(cpu_state.next_instruction_pc, memory[0]);
                    //     let mut hl = 0;
                    //     for (_, ref operand) in disassembly.iter().enumerate().skip(1) {
                    //         match operand {
                    //             DisassembledInstruction::Register(rnum) => {
                    //                 self.highlighted_registers[*rnum as usize].color = HIGHLIGHT_COLORS[hl];
                    //                 hl += 1;
                    //             },

                    //             DisassembledInstruction::OffsetRegister(_, rnum) => {
                    //                 self.highlighted_registers[*rnum as usize].color = HIGHLIGHT_COLORS[hl];
                    //                 hl += 1;
                    //             },

                    //             _ => {},
                    //         }

                    //         // give up if there's any more (there should never be)
                    //         if hl == 3 { break; }
                    //     }
                    }

                    self.requested_cpu_state = false;
                },

                _ => {},
            }
        }
    }
}

impl GameWindow for Cop1State {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        let mut opened = true;

        let frame_bg = ui.style_color(imgui::StyleColor::FrameBg);
        let disabled_bg = [frame_bg[0] * 0.4, frame_bg[1] * 0.4, frame_bg[2] * 0.4, 1.0];

        self.update(ui.io().delta_time, frame_bg, disabled_bg);

        let window = ui.window("COP1");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .build(|| {
            let use_f64_label = if self.use_f64 { "64" } else { "32" };
            Utils::flag_button(ui, Some(&mut self.use_f64), use_f64_label, Some("Toggle between 32-bit and 64-bit values"));

            ui.same_line();
            Utils::flag_button(ui, Some(&mut self.show_hex), "H", Some("Toggle hexadecimal display"));

            ui.same_line();
            if Utils::flag_button(ui, None, format!("{}", self.num_columns), Some("Layout registers in this many columns")) {
                match self.num_columns {
                    8 => { self.num_columns = 1; },
                    _ => { self.num_columns *= 2; },
                }
            }

            ui.separator();

            let mut fr_bit = self.fr_bit;
            ui.checkbox("FR bit", &mut fr_bit);
            if ui.is_item_hovered() {
                ui.tooltip_text("From bit 26 of COP0_STATUS");
            }

            // calculate the width of a label (always 6 chars)
            let label_size = ui.calc_text_size("xyz00:");

            // along with internal padding, divide up the content width into 4 sections to compute the text box width
            let column_padding = 18.0; //unsafe { ui.style().frame_padding[0] };
            let region_size = ui.content_region_avail();
            let text_input_size = (region_size[0] - ((self.num_columns as f32) - 1.0) * column_padding) / (self.num_columns as f32) - label_size[0];
            let rows = 32 / self.num_columns;

            for row in 0..rows {
                for col in 0..self.num_columns {
                    let rnum = row * self.num_columns + col;
                
                    ui.text(format!("{:-6}", format!("fgr{}:", rnum)));
                    ui.same_line_with_spacing(0.0, 0.0);

                    // determine bg color for the text input
                    let bg_color = if let Some(ref duration) = self.highlighted_registers[rnum].duration {
                        if self.cpu_running {
                            self.highlighted_registers[rnum].color
                        } else {
                            let perc = *duration / Utils::DATA_CHANGED_FADE_TIME;
                            Utils::interpolate_colors(self.highlighted_registers[rnum].color, Utils::average_colors(self.highlighted_registers[rnum].color, Utils::DATA_CHANGED_FADE_COLOR), perc)
                        }
                    } else {
                        self.highlighted_registers[rnum].color
                    };

                    let mut _color_tokens = Vec::new();
                    _color_tokens.push(ui.push_style_color(imgui::StyleColor::FrameBg, bg_color));

                    if (rnum % 2) != 0 && !self.fr_bit {
                        _color_tokens.push(ui.push_style_color(imgui::StyleColor::Text, ui.style_color(imgui::StyleColor::TextDisabled)));
                    }

                    let _token = ui.push_item_width(text_input_size);

                    // id for input_text needs to be unique to allow editing to work
                    if ui.input_text(format!("##fgr{}", rnum), &mut self.fgr_value_strings[rnum as usize])
                           .enter_returns_true(true)
                           .build() {
                        if self.show_hex {
                            if self.use_f64 {
                               if let Some(value) = Utils::parse_u64(&self.fgr_value_strings[rnum as usize]) {
                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::SetFgrInt64(rnum as usize, value),
                                        response_channel: None,
                                    };
                                    let _ = debugger::Debugger::send_command(&self.comms, command);

                                    // update the value
                                    self.fgr_values[rnum as usize].as_u64 = value; // weird. why do I not need unsafe here?
                                    self.fgr_value_strings[rnum as usize] = format!("0x{:016X}", value);
                               }
                            } else {
                               if let Some(value) = Utils::parse_u64_se32(&self.fgr_value_strings[rnum as usize]) {
                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::SetFgrInt32(rnum as usize, value as u32),
                                        response_channel: None,
                                    };
                                    let _ = debugger::Debugger::send_command(&self.comms, command);

                                    // update the value
                                    self.fgr_values[rnum as usize].as_u32 = value as u32; // weird. why do I not need unsafe here?
                                    self.fgr_value_strings[rnum as usize] = format!("0x{:08X}", value);
                               }
                            }
                        } else {
                            if self.use_f64 {
                               if let Some(value) = Utils::parse_u64_float::<f64, String>(&self.fgr_value_strings[rnum as usize]) {
                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::SetFgrFloat64(rnum as usize, value),
                                        response_channel: None,
                                    };
                                    let _ = debugger::Debugger::send_command(&self.comms, command);

                                    // update the value
                                    self.fgr_values[rnum as usize].as_f64 = value; // weird. why do I not need unsafe here?
                                    self.fgr_value_strings[rnum as usize] = format!("{:.16}", value);
                               }
                            } else {
                               if let Some(value) = Utils::parse_u64_float::<f32, String>(&self.fgr_value_strings[rnum as usize]) {
                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::SetFgrFloat32(rnum as usize, value),
                                        response_channel: None,
                                    };
                                    let _ = debugger::Debugger::send_command(&self.comms, command);

                                    // update the value
                                    self.fgr_values[rnum as usize].as_f32 = value;
                                    self.fgr_value_strings[rnum as usize] = format!("{:.8}", value);
                               }
                            }
                        }
                    }

                    if ui.is_item_hovered() {
                        let details = format!("u32: 0x{:08X}\nu64: 0x{:016X}\nf32: {:.8}\nf64: {:.16}", 
                            unsafe { self.fgr_values[rnum as usize].as_u32 },
                            unsafe { self.fgr_values[rnum as usize].as_u64 },
                            unsafe { self.fgr_values[rnum as usize].as_f32 },
                            unsafe { self.fgr_values[rnum as usize].as_f64 }
                        );
                        ui.tooltip_text(details);
                    }

                    if col != (self.num_columns - 1) {
                        // ui.same_line();
                        ui.same_line_with_spacing(0.0, column_padding);
                    }
                }
            }

            ui.separator();

            let fcr_control_status = self.fcr_control_status;
            let label = format!("FCR31 Control/Status Register: 0x{:08X}###fcr31", fcr_control_status);
            ui.tree_node_config(label)
                .default_open(true)
                .build(|| {
                    let mut fs = ((fcr_control_status & 0x01000000) >> 24) != 0;
                    ui.checkbox("FS##flush", &mut fs);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("(bit 24) flush denormalized numbers");
                    }

                    ui.same_line();
                    let mut cond = ((fcr_control_status & 0x00800000) >> 23) != 0;
                    ui.checkbox("C##cond", &mut cond);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("(bit 23) condition bit");
                    }

                    ui.same_line();
                    let (rounding_mode, tooltip) = match fcr_control_status & 0x03 {
                        0b00 => ("Nearest", "Round result to nearest representable value"),
                        0b01 => ("Zero", "Round toward 0"),
                        0b10 => ("+Inf", "Round towards +infinity"),
                        0b11 => ("-Inf", "Round towards -infinity"),
                        _ => todo!(),
                    };

                    ui.text(format!("RM: {}", rounding_mode));
                    if ui.is_item_hovered() {
                        ui.tooltip_text(tooltip);
                    }

                    let cause_bits = (fcr_control_status >> 12) & 0x3F;
                    let enable_bits = (fcr_control_status >> 7) & 0x1F;
                    let flag_bits = (fcr_control_status >> 2) & 0x1F;

                    let e_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Unimplemented Operation", bit));
                        }
                    };

                    let v_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Invalid Operation", bit));
                        }
                    };

                    let z_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Division by Zero", bit));
                        }
                    };

                    let o_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Overflow", bit));
                        }
                    };

                    let u_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Underflow", bit));
                        }
                    };

                    let i_tooltip = |ui: &imgui::Ui, bit: u32| {
                        if ui.is_item_hovered() {
                            ui.tooltip_text(format!("(bit {}) Inexact", bit));
                        }
                    };

                    let columns =  [
                        imgui::TableColumnSetup { name: "##category", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(0) },
                        imgui::TableColumnSetup { name: "E##e_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(1) },
                        imgui::TableColumnSetup { name: "V##v_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(2) },
                        imgui::TableColumnSetup { name: "Z##z_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(3) },
                        imgui::TableColumnSetup { name: "O##o_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(4) },
                        imgui::TableColumnSetup { name: "U##u_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(5) },
                        imgui::TableColumnSetup { name: "I##i_field", flags: imgui::TableColumnFlags::WIDTH_FIXED, init_width_or_weight: 16.0, user_id: ui.new_id(6) },
                    ];

                    let table_flags = imgui::TableFlags::RESIZABLE  
                                      | imgui::TableFlags::BORDERS_OUTER
                                      | imgui::TableFlags::SIZING_FIXED_FIT
                                      | imgui::TableFlags::NO_BORDERS_IN_BODY_UNTIL_RESIZE
                                      | imgui::TableFlags::NO_HOST_EXTEND_X
                                      ;
                    if let Some(_table_token) = ui.begin_table_with_sizing("FCR Cause Bits###fcr_cause_bits", columns.len(), table_flags, [0.0, 0.0], 0.0) {
                        // create the columns
                        for column in columns {
                            ui.table_setup_column_with(column);
                        }

                        ui.table_headers_row();
                        ui.table_next_row();

                        ui.table_next_column();
                        ui.text("Cause");
                        ui.table_next_column();
                        let mut e0 = ((cause_bits >> 5) & 0x01) != 0; ui.checkbox("##e0", &mut e0); e_tooltip(ui, 17); ui.table_next_column();
                        let mut v0 = ((cause_bits >> 4) & 0x01) != 0; ui.checkbox("##v0", &mut v0); v_tooltip(ui, 16); ui.table_next_column();
                        let mut z0 = ((cause_bits >> 3) & 0x01) != 0; ui.checkbox("##z0", &mut z0); z_tooltip(ui, 15); ui.table_next_column();
                        let mut o0 = ((cause_bits >> 2) & 0x01) != 0; ui.checkbox("##o0", &mut o0); o_tooltip(ui, 14); ui.table_next_column();
                        let mut u0 = ((cause_bits >> 1) & 0x01) != 0; ui.checkbox("##u0", &mut u0); u_tooltip(ui, 13); ui.table_next_column();
                        let mut i0 = ((cause_bits >> 0) & 0x01) != 0; ui.checkbox("##i0", &mut i0); i_tooltip(ui, 12); 
                        ui.table_next_row();

                        ui.table_next_column();
                        ui.text("Enable");
                        ui.table_next_column();
                        /* no E */                                                                                      ui.table_next_column();
                        let mut v1 = ((enable_bits >> 4) & 0x01) != 0; ui.checkbox("##v1", &mut v1); v_tooltip(ui, 11); ui.table_next_column();
                        let mut z1 = ((enable_bits >> 3) & 0x01) != 0; ui.checkbox("##z1", &mut z1); z_tooltip(ui, 10); ui.table_next_column();
                        let mut o1 = ((enable_bits >> 2) & 0x01) != 0; ui.checkbox("##o1", &mut o1); o_tooltip(ui,  9); ui.table_next_column();
                        let mut u1 = ((enable_bits >> 1) & 0x01) != 0; ui.checkbox("##u1", &mut u1); u_tooltip(ui,  8); ui.table_next_column();
                        let mut i1 = ((enable_bits >> 0) & 0x01) != 0; ui.checkbox("##i1", &mut i1); i_tooltip(ui,  7); 
                        ui.table_next_row();

                        ui.table_next_column();
                        ui.text("Flag");
                        ui.table_next_column();
                        /* no E */                                                                                   ui.table_next_column();
                        let mut v2 = ((flag_bits >> 4) & 0x01) != 0; ui.checkbox("##v2", &mut v2); v_tooltip(ui, 6); ui.table_next_column();
                        let mut z2 = ((flag_bits >> 3) & 0x01) != 0; ui.checkbox("##z2", &mut z2); z_tooltip(ui, 5); ui.table_next_column();
                        let mut o2 = ((flag_bits >> 2) & 0x01) != 0; ui.checkbox("##o2", &mut o2); o_tooltip(ui, 4); ui.table_next_column();
                        let mut u2 = ((flag_bits >> 1) & 0x01) != 0; ui.checkbox("##u2", &mut u2); u_tooltip(ui, 3); ui.table_next_column();
                        let mut i2 = ((flag_bits >> 0) & 0x01) != 0; ui.checkbox("##i2", &mut i2); i_tooltip(ui, 2); 
                    }
                }
            );
                             
        });

        if !opened {
            self.comms.decrement_debugger_windows();
        }

        opened
    }
}
