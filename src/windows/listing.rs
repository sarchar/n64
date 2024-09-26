use std::collections::HashMap;

use crossbeam::channel::{self, Receiver, Sender};

use crate::*;
use n64::cpu::{self, DisassembledInstruction};
use n64::debugger;
use gui::game::{GameWindow, Utils};

const BREAKPOINT_COLOR   : [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const CURSOR_COLOR       : [f32; 4] = [0x59 as f32 / 255.0, 0x57 as f32 / 255.0, 0x51 as f32 / 255.0, 1.0]; // #595751
const PC_COLOR           : [f32; 4] = [0x2B as f32 / 255.0, 0x8F as f32 / 255.0, 0xAD as f32 / 255.0, 1.0]; // #2b8fad
const TEXT_COLOR         : [f32; 4] = [0.8, 0.8, 0.8, 1.0];
const ADDRESS_COLOR      : [f32; 4] = TEXT_COLOR;  // this is the address in the lefthand bar, while addresses in operands are different
const OPCODE_COLOR       : [f32; 4] = [0.7, 0.4, 0.4, 1.0];
const MNEMONIC_COLOR     : [f32; 4] = [0x30 as f32 / 255.0, 0xB5 as f32 / 255.0, 0xB8 as f32 / 255.0, 1.0];
const R0_COLOR           : [f32; 4] = [0x8B as f32 / 255.0, 0x69 as f32 / 255.0, 0x9B as f32 / 255.0, 1.0]; // r0: #8b699b
const REGISTERS_COLOR    : [f32; 4] = [0x98 as f32 / 255.0, 0x72 as f32 / 255.0, 0xE3 as f32 / 255.0, 1.0]; // registers: #9872ab
const FPU_REGISTERS_COLOR: [f32; 4] = [0x80 as f32 / 255.0, 0x53 as f32 / 255.0, 0x2F as f32 / 255.0, 1.0]; // fpu registers: #80532f
const CONSTANTS_COLOR    : [f32; 4] = TEXT_COLOR; // constants grey
const SHIFT_AMOUNT_COLOR : [f32; 4] = TEXT_COLOR; // constants grey
const ADDRESSES_COLOR    : [f32; 4] = [0x38 as f32 / 255.0, 0x79 as f32 / 255.0, 0xAB as f32 / 255.0, 1.0]; // addresses: #3879ab
const OFFSETS_COLOR      : [f32; 4] = TEXT_COLOR; // constants grey
const COMMENT_COLOR      : [f32; 4] = [0.4, 0.4, 0.4, 1.0];

pub struct Listing {
    comms: SystemCommunication,
    
    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // true if GetCpuState has been requested and we're waiting
    requested_cpu_state: bool,
    requested_breakpoints: bool,

    // when listing_address is Some(), we're looking at a specific address
    // when None, following PC
    listing_address: Option<u64>,
    listing_memory: Vec<u32>,

    // cursor address and length of selection
    cursor_address: Option<u64>,
    cursor_length: usize,
    
    // the number of instructions requested is delayed by 1 request due to the calculation of the window size
    num_instructions_displayed: u32,
    instruction_offset: i64,

    // returned CPU state
    cpu_state: debugger::CpuStateInfo,

    // breakpoints
    breakpoints: HashMap<u64, debugger::BreakpointInfo>, // virtual address to breakpoint id map

    // use ABI register names
    use_abi_names: bool,
    
    // show table resizers
    show_resizers: bool,
}

impl Listing {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();
        
        comms.increment_debugger_windows();
        
        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            requested_cpu_state: false,
            requested_breakpoints: false,
            listing_address: None,
            listing_memory: Vec::new(),
            cursor_address: None,
            cursor_length: 0,
            num_instructions_displayed: 0,
            instruction_offset: 0,
            cpu_state: debugger::CpuStateInfo::default(),
            breakpoints: HashMap::new(),
            use_abi_names: true,
            show_resizers: false,
        }
    }

    fn listing_start_address(&self) -> u64 {
        if self.listing_address.is_none() {
            ((self.cpu_state.next_instruction_pc as i64) + (self.instruction_offset * 4)) as u64
        } else {
            ((self.listing_address.unwrap() as i64) + (self.instruction_offset * 4)) as u64
        }
    }

    fn update(&mut self, _delta_time: f32) {
        self.instruction_offset = -(self.num_instructions_displayed as i64) / 2 + 1;

        // get cpu state
        if !self.requested_cpu_state {
            let request = debugger::DebuggerCommand {
                // the number of instructions displayed is one frame behind a resize--oh well
                command_request: if self.listing_address.is_none() {
                    debugger::DebuggerCommandRequest::GetCpuState(Some((self.instruction_offset, self.num_instructions_displayed as usize)))
                } else {
                    debugger::DebuggerCommandRequest::GetCpuState(None)
                },
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };

            self.requested_cpu_state = debugger::Debugger::send_command(&self.comms, request).is_ok();
        } 

        if !self.requested_breakpoints {
            let request = debugger::DebuggerCommand {
                // the number of instructions displayed is one frame behind a resize--oh well
                command_request:  debugger::DebuggerCommandRequest::GetBreakpoints,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };

            self.requested_breakpoints = debugger::Debugger::send_command(&self.comms, request).is_ok();
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::CpuState(cpu_state) => {
                    self.cpu_state = cpu_state;
                    self.requested_cpu_state = false;
                },

                debugger::DebuggerCommandResponse::ReadBlock(id, memory) => {
                    if id == 0 {
                        if let Some(memory) = memory {
                            self.listing_memory = memory;
                        } else {
                            self.listing_memory.clear();
                        }
                    }
                },

                debugger::DebuggerCommandResponse::Breakpoints(breakpoints) => {
                    self.breakpoints = breakpoints;
                    self.requested_breakpoints = false;
                },

                _ => {},
            }
        } 
    }
}

impl GameWindow for Listing {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        self.update(ui.io().delta_time);

        let mut opened = true;
        let window = ui.window("Listing");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              // because I want to fill up enough lines to fill up the screen, that causes the actual table borders to
              // run outside of the content area of the window, which means a scrollbar will appear...so disable it
              .flags(imgui::WindowFlags::NO_SCROLLBAR | imgui::WindowFlags::NO_SCROLL_WITH_MOUSE)
              .build(|| {

            let mouse_pos = ui.io().mouse_pos;

            if ui.is_window_focused() && ui.io().mouse_wheel != 0.0 {
                // if switching from track-pc to not...
                if self.listing_address.is_none() {
                    self.listing_address = Some(self.cpu_state.next_instruction_pc);
                }

                // increment address address by mouse_wheel count
                self.listing_address = Some((self.listing_address.unwrap() as i64 + (-ui.io().mouse_wheel as i64) * 4) as u64);

                // send the request-memory command
                let address = (self.listing_address.unwrap() as i64) + (self.instruction_offset << 2);
                let command = debugger::DebuggerCommand {
                    command_request: debugger::DebuggerCommandRequest::ReadBlock(0, address as u64, self.num_instructions_displayed as usize),
                    response_channel: Some(self.debugging_request_response_tx.clone()),
                };

                let _ = debugger::Debugger::send_command(&self.comms, command);
            }

            if self.cpu_state.running && ui.button("Stop") {
                let command = debugger::DebuggerCommand {
                    command_request: debugger::DebuggerCommandRequest::StopCpu,
                    response_channel: Some(self.debugging_request_response_tx.clone()),
                };
                let _ = debugger::Debugger::send_command(&self.comms, command);
            } else if !self.cpu_state.running && ui.button("Run") {
                let command = debugger::DebuggerCommand {
                    command_request: debugger::DebuggerCommandRequest::RunCpu,
                    response_channel: Some(self.debugging_request_response_tx.clone()),
                };
                let _ = debugger::Debugger::send_command(&self.comms, command);
            }
            ui.same_line();

            // ui.input_int("", &mut self.step_cycle_count)
            if ui.button("Step") {
                let command = debugger::DebuggerCommand {
                    command_request: debugger::DebuggerCommandRequest::StepCpu(1),
                    response_channel: Some(self.debugging_request_response_tx.clone()),
                };
                let _ = debugger::Debugger::send_command(&self.comms, command);

                // Also enable follow PC
                self.listing_address = None;
                self.listing_memory.clear();
            }

            if self.listing_address.is_none() {
                ui.text(format!("Address: ${:08X}", self.cpu_state.next_instruction_pc as u32));
            } else {
                ui.text(format!("Address: ${:08X}", self.listing_address.unwrap() as u32));
            }

            ui.same_line();
            if ui.button("Follow PC") {
                self.listing_address = None;
                self.listing_memory.clear();
            }
            
            // draw the `A` button
            Utils::flag_button(ui, Some(&mut self.use_abi_names), "A", Some("Display registers using the ABI names"));
            ui.same_line();

            // draw the `R` button
            Utils::flag_button(ui, Some(&mut self.show_resizers), "R", Some("Show column resizers"));
            ui.separator();
                
            ui.group(|| {
                let draw_list = ui.get_window_draw_list();
                // println!("line_count = {}", line_count);
                // let cursor_pos = ui.cursor_screen_pos();
                // let pos = [0.0+cursor_pos[0], 0.0+cursor_pos[1]];
                // let end = [available_area[0] - 1.0 + pos[0], available_area[1] - 1.0 + pos[1]];
                // draw_list.add_rect(pos, end, 0xff0000ff).build();

                // ABGR32
                let columns =  [
                    imgui::TableColumnSetup { name: "Address"    , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(0) },
                    imgui::TableColumnSetup { name: "Opcode"     , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(1) },
                    imgui::TableColumnSetup { name: "Instruction", flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(2) },
                    imgui::TableColumnSetup { name: "Operands"   , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(3) },
                    imgui::TableColumnSetup { name: "Comments"   , flags: imgui::TableColumnFlags::WIDTH_STRETCH, init_width_or_weight: 10.0, user_id: ui.new_id(4) },
                ];

                let table_flags = if self.show_resizers {
                    imgui::TableFlags::BORDERS_INNER_V
                } else {
                    imgui::TableFlags::NO_BORDERS_IN_BODY
                } | imgui::TableFlags::RESIZABLE | imgui::TableFlags::NO_PAD_OUTER_X;

                // use begin_table_with_sizing so we can skip the ui.table_headers_row call, as we want to
                // set up and use columns but we don't want to display the headers row
                if let Some(_) = ui.begin_table_with_sizing("table test", columns.len(), table_flags, [0.0, 0.0], 0.0) {
                    // create the columns
                    for column in columns {
                        ui.table_setup_column_with(column);    
                    }

                    // compute the number of instructions to display after we've started building the table
                    let available_area = ui.content_region_avail();
                    let line_height = ui.text_line_height_with_spacing();
                    let line_count = (available_area[1] / line_height) as u32;
                    self.num_instructions_displayed = line_count + 1;

                    if ui.is_mouse_clicked(imgui::MouseButton::Left) && ui.is_window_hovered() {
                        let current_cursor = ui.cursor_screen_pos();
                        // clicking in the icons column shouldn't move the cursor
                        if (mouse_pos[0] >= current_cursor[0] + line_height) && (mouse_pos[1] >= current_cursor[1]) {
                            let row = (mouse_pos[1] - current_cursor[1]) / line_height;
                            // println!("selected row {}", row);

                            let start = self.listing_start_address();
                            let selected_address = start + (row as u64) * 4;
                            // println!("selected address ${:08X}", selected_address);

                            if ui.io().key_shift && self.cursor_address.is_some() {
                                let current_cursor = self.cursor_address.unwrap();
                                if selected_address < current_cursor {
                                    self.cursor_address = Some(selected_address);
                                    self.cursor_length = (current_cursor - selected_address) as usize;
                                } else {
                                    self.cursor_length = (selected_address - current_cursor + 1) as usize;
                                }
                            } else {
                                if self.cursor_address.is_some_and(|addr| addr == selected_address) {
                                    self.cursor_address = None;
                                } else {
                                    self.cursor_address = Some(selected_address);
                                    self.cursor_length = 1;
                                }
                            }
                        }
                    }

                    // get the starting address of the listing
                    let mut virtual_address = self.listing_start_address();

                    // now display the instructions if the memory is valid
                    let instruction_memory: Option<&Vec<u32>> = if self.listing_address.is_none() {
                        self.cpu_state.instruction_memory.as_ref()
                    } else {
                        Some(&self.listing_memory)
                    };
                    
                    if let Some(memory) = instruction_memory {
                        for inst in memory.iter() {
                            // start the first column so that cursor is in the right place
                            ui.table_next_column();
                            let mut cursor_pos = ui.cursor_screen_pos();
                            let mut width = available_area[0];

                            // let window_alpha = ui.style_color(StyleColor::WindowBg)[3];
                            let mut line_fill_color = None;

                            let mut comment = String::new();

                            // draw icons back to front
                            // make the icons column equal to a line height so that it looks square
                            // but take of two pixels for padding on each side

                            // first, the cursor
                            if let Some(cursor_address) = self.cursor_address {
                                if virtual_address >= cursor_address && virtual_address < (cursor_address + self.cursor_length as u64) {
                                    line_fill_color = Some([CURSOR_COLOR[0], CURSOR_COLOR[1], CURSOR_COLOR[2], 2.0]);
                                }
                            }

                            let mouse_in_icons_cell = (mouse_pos[0] >= cursor_pos[0] && mouse_pos[0] < (cursor_pos[0] + line_height))
                                                      && (mouse_pos[1] >= cursor_pos[1] && mouse_pos[1] < (cursor_pos[1] + line_height));

                            // if the user clicked in the icons cell, set/toggle/unset a breakpoint
                            if ui.is_mouse_clicked(imgui::MouseButton::Left) && mouse_in_icons_cell {
                                if let Some(breakpoint_info) = self.breakpoints.get_mut(&virtual_address) {
                                    // toggle the breakpoint
                                    breakpoint_info.enable = !breakpoint_info.enable;
                                    
                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::EnableBreakpoint(breakpoint_info.address, breakpoint_info.enable),
                                        response_channel: None,
                                    };

                                    let _ = debugger::Debugger::send_command(&self.comms, command);
                                } else {
                                    // create the breakpoint
                                    let breakpoint_info = debugger::BreakpointInfo {
                                        address: virtual_address,
                                        mode   : debugger::BP_EXEC,
                                        enable : true,
                                        ..Default::default()
                                    };

                                    let command = debugger::DebuggerCommand {
                                        command_request: debugger::DebuggerCommandRequest::SetBreakpoint(breakpoint_info.clone()),
                                        response_channel: None,
                                    };

                                    let _ = debugger::Debugger::send_command(&self.comms, command);

                                    self.breakpoints.insert(virtual_address, breakpoint_info);
                                }
                            }

                            // then breakpoints
                            let is_enabled = self.breakpoints.get(&virtual_address).map_or_else(|| None, |info| Some(info.enable));

                            if let Some(is_enabled) = is_enabled {
                                let center = [cursor_pos[0] + line_height / 2.0, cursor_pos[1] + line_height / 2.0];
                                let radius = (line_height - 4.0) / 2.0;
                                // line_fill_color = Some([BREAKPOINT_COLOR[0], BREAKPOINT_COLOR[1], BREAKPOINT_COLOR[2], 0.2]);
                                draw_list.add_circle(center, radius, BREAKPOINT_COLOR).filled(is_enabled).build();
                            }

                            // then PC arrow
                            if virtual_address == self.cpu_state.next_instruction_pc {
                                let midy = line_height / 2.0;
                                let points = vec![
                                    [cursor_pos[0] + 2.0              , cursor_pos[1] + midy - 2.0],
                                    [cursor_pos[0] + line_height - 2.0, cursor_pos[1] + midy - 2.0],
                                    [cursor_pos[0] + line_height - 2.0, cursor_pos[1] + midy + 2.0],
                                    [cursor_pos[0] + 2.0              , cursor_pos[1] + midy + 2.0],
                                ];
                                line_fill_color = Some([PC_COLOR[0], PC_COLOR[1], PC_COLOR[2], 0.2]);
                                draw_list.add_polyline(points, PC_COLOR).filled(true).build();
                            }

                            // fill the row with a background color if set
                            if let Some(color) = line_fill_color {
                                ui.table_set_bg_color(imgui::TableBgTarget::ROW_BG0, color);
                            } else {
                                ui.table_set_bg_color(imgui::TableBgTarget::ROW_BG0, [0.0, 0.0, 0.0, 0.0]);
                            }

                            // shift X over
                            cursor_pos[0] += line_height;
                            width -= line_height;
                            ui.set_cursor_screen_pos(cursor_pos);

                            // let height = ui.calc_text_size("X")[1] + ui.style().frame_padding[1];

                            // display address
                            // ui.text_colored([0.8, 0.8, 0.8, 1.0], format!("{:08X}: ${:08X}", virtual_address as u32, inst));
                            ui.text_colored(ADDRESS_COLOR, format!("{:08X}", virtual_address as u32));

                            // display opcode
                            ui.table_next_column();
                            ui.text_colored(OPCODE_COLOR, format!("{:08X}", *inst));

                            // disassembly instruction
                            let disassembly = cpu::Cpu::disassemble(virtual_address, *inst);

                            // display instruction mnemonic
                            ui.table_next_column();
                            // 30b5b8
                            if let DisassembledInstruction::Mnemonic(ref mnemonic) = disassembly[0] {
                                ui.text_colored(MNEMONIC_COLOR, mnemonic);
                            }

                            // display operands
                            // constants: #abaa67
                            ui.table_next_column();

                            for (index, ref operand) in disassembly.iter().enumerate().skip(1) {
                                match operand {
                                    DisassembledInstruction::Register(rnum) => {
                                        let reg_name = if self.use_abi_names {
                                            n64::cpu::Cpu::abi_name(*rnum)
                                        } else {
                                            n64::cpu::Cpu::register_name(*rnum)
                                        };

                                        if *rnum == 0 { // make r0 look different
                                            ui.text_colored(R0_COLOR, reg_name);
                                        } else {
                                            ui.text_colored(REGISTERS_COLOR, reg_name);
                                        }
                                    },

                                    DisassembledInstruction::ConstantS16(imm) => {
                                        ui.text_colored(CONSTANTS_COLOR, format!("0x{:04X}", *imm));
                                        comment.push_str(&format!("imm={}", *imm));
                                    }

                                    DisassembledInstruction::ShiftAmount(sa) => {
                                        ui.text_colored(SHIFT_AMOUNT_COLOR, format!("{}", *sa));
                                    }

                                    DisassembledInstruction::FpuRegister(fnum) => {
                                        ui.text_colored(FPU_REGISTERS_COLOR, format!("f{}", *fnum));
                                    },

                                    DisassembledInstruction::Address(address) => {
                                        if (*address as i32) as u64 == *address {
                                            ui.text_colored(ADDRESSES_COLOR, format!("0x{:08X}", *address as u32));
                                        } else {
                                            ui.text_colored(ADDRESSES_COLOR, format!("0x{:016X}", *address));
                                        }
                                    },

                                    DisassembledInstruction::OffsetRegister(offset, rnum) => {
                                        ui.text_colored(OFFSETS_COLOR, format!("0x{:04X}", *offset));
                                        ui.same_line_with_spacing(0.0, 0.0);
                                        ui.text_colored(TEXT_COLOR, format!("("));
                                        ui.same_line_with_spacing(0.0, 0.0);

                                        let reg_name = if self.use_abi_names {
                                            n64::cpu::Cpu::abi_name(*rnum)
                                        } else {
                                            n64::cpu::Cpu::register_name(*rnum)
                                        };

                                        if *rnum == 0 { // make r0 look different
                                            ui.text_colored(R0_COLOR, reg_name);
                                        } else {
                                            ui.text_colored(REGISTERS_COLOR, reg_name);
                                        }

                                        ui.same_line_with_spacing(0.0, 0.0);
                                        ui.text_colored(TEXT_COLOR, format!(")"));

                                        comment.push_str(&format!("imm={}, address=?", *offset));
                                    }

                                    _ => {},
                                }

                                if index != disassembly.len() - 1 {
                                    ui.same_line_with_spacing(0.0, 0.0);
                                    ui.text_colored(TEXT_COLOR, format!(", "));
                                    ui.same_line_with_spacing(0.0, 0.0);
                                }

                                // ui.text_colored([0.8, 0.8, 0.8, 1.0], format!(", "));
                                // ui.same_line_with_spacing(0.0, 0.0);
                                // ui.text_colored([0x98 as f32 / 255.0, 0x72 as f32 / 255.0, 0xE3 as f32 / 255.0, 1.0], format!("r1"));
                                // ui.same_line_with_spacing(0.0, 0.0);
                                // ui.text_colored([0.8, 0.8, 0.8, 1.0], format!(", "));
                                // ui.same_line_with_spacing(0.0, 0.0);
                                // ui.text_colored([0.8, 0.8, 0.8, 1.0], format!("0x1234"));
                            }

                            // display comment
                            ui.table_next_column();
                            if comment.len() != 0 {
                                ui.text_colored(COMMENT_COLOR, format!("; {}", comment));
                            }

                            // next row
                            virtual_address += 4;
                            // ui.new_line();
                        }
                    }
                }
            });
        });

        if !opened {
            self.comms.decrement_debugger_windows();
        }
        opened
    }
}
