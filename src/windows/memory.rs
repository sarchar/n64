use crossbeam::channel::{self, Receiver, Sender};

use crate::*;
use n64::debugger;
use gui::game::{GameWindow, Utils};

#[derive(PartialOrd, PartialEq, Copy, Clone, Debug)]
enum DataSize {
    Byte,
    Short,
    Word,
    DoubleWord,
}

impl DataSize {
    fn size(&self) -> usize {
        match self {
            DataSize::Byte       => 1,
            DataSize::Short      => 2,
            DataSize::Word       => 4,
            DataSize::DoubleWord => 8,
        }
    }
}

#[derive(PartialEq, Copy, Clone, Debug)]
enum DataFormat {
    Dec,
    Hex,
    Float,
    Hexii,
}

const MEMORY_REGIONS: [(&'static str, u32, u32); 6] = [
    ("RDRAM"             , 0x00000000, 0x007F0000),
    ("RSP MEM"           , 0x04000000, 0x04001FFF),
    ("Cart SRAM/FlashRAM", 0x08000000, 0x0FFFFFFF),
    ("Cartridge ROM"     , 0x10000000, 0x1FBFFFFF),
    ("PIF ROM"           , 0x1FC00000, 0x1FC007BF),
    ("PIF RAM"           , 0x1FC007C0, 0x1FC007FF),
];

fn format_region(region_index: usize) -> String {
    let r = &MEMORY_REGIONS[region_index];
    format!("{} [0x{:08X}-0x{:08X}]", r.0, r.1, r.2)
}

pub struct Memory {
    comms: SystemCommunication,
    
    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // currently selected memory region
    current_region: usize,

    // number of columns of DataType to display
    column_count: usize,

    // current data format and size
    data_format: DataFormat,
    data_size: DataSize,
    data_signed: bool,
    
    // display ascii representation of data
    show_ascii: bool,

    // memory state
    requested_memory: bool,
    memory_display_min: Option<u32>,
    memory_display_max: Option<u32>,
    memory: Option<(u64, Vec<u32>)>,
}

impl Memory {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();
        
        comms.increment_debugger_windows();
        
        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            current_region: 0,
            column_count: 4,
            data_format: DataFormat::Hex,
            data_size: DataSize::Word,
            data_signed: false,
            show_ascii: true,

            requested_memory: false,
            memory_display_min: None,
            memory_display_max: None,
            memory: None,
        }
    }

    // Certain keys work no matter what, as long as the Listing window is open.  Others only work when the Listing window is in focus.
    fn update_inputs(&mut self, ui: &imgui::Ui) {
        // Start/Stop execution

        // Any following keys require window focus
        if !ui.is_window_focused() { return; }
    }

    // must be called from within a window()
    fn update(&mut self, ui: &imgui::Ui) {
        self.update_inputs(ui);

        if !self.requested_memory {
            // send the request-memory command
            if let Some(start_addr) = self.memory_display_min {
                // align to next word
                let end_addr = (self.memory_display_max.unwrap() + 3) & !3;

                // pass the start address as the ID, we'll use it later
                let command = debugger::DebuggerCommand {
                    command_request: debugger::DebuggerCommandRequest::ReadBlock(start_addr as u64, 0xA0000000 | (start_addr as u64), ((end_addr - start_addr) >> 2) as usize),
                    response_channel: Some(self.debugging_request_response_tx.clone()),
                };

                self.requested_memory = debugger::Debugger::send_command(&self.comms, command).is_ok();
            }
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::ReadBlock(id, memory) => {
                    let memory = memory.iter().map(|element| match element {
                            debugger::MemoryChunk::Valid(_, memory) => memory.clone(),
                            debugger::MemoryChunk::Invalid(_, count) => vec![0xEAEAEAEA; *count],
                        })
                        .collect::<Vec<_>>()
                        .concat();
                    self.memory = Some((id, memory));
                    self.requested_memory = false;
                },

                // Ignore all other messages
                _ => {},
            }
        }
    }

    fn draw_contents(&mut self, ui: &imgui::Ui) {
        self.draw_toolbar(ui);
        self.draw_memory(ui);
    }

    fn draw_toolbar(&mut self, ui: &imgui::Ui) {
        let (format_label, next_value) = match self.data_format {
            DataFormat::Dec   => ("Dec", DataFormat::Hex),
            DataFormat::Hex   => ("Hex", DataFormat::Float),
            DataFormat::Float => ("Flo", DataFormat::Hexii),
            DataFormat::Hexii => ("Xii", DataFormat::Dec),
        };

        if Utils::flag_button(ui, None, format!("{}##data_format", format_label), Some("Cycle though Dec/Hex/Float/HexII")) {
            self.data_format = next_value;
        }

        if self.data_format == DataFormat::Float {
            self.data_signed = true;
            if self.data_size < DataSize::Word {
                self.data_size = DataSize::Word;
            }
        } else if self.data_format == DataFormat::Hexii {
            self.data_size = DataSize::Byte;
            self.data_signed = false;
        }

        let (size_label, next_value) = match self.data_format {
            // Floats are either 32-bit or 64-bit
            DataFormat::Float => {
                match self.data_size {
                    DataSize::Word       => ("W", DataSize::DoubleWord),
                    DataSize::DoubleWord => ("D", DataSize::Word),
                    _ => panic!(),
                }
            },

            // only Byte size for HexII
            DataFormat::Hexii => ("B", DataSize::Byte),

            // Dec and Hex can be any size
            _ => {
                match self.data_size {
                    DataSize::Byte       => ("B", DataSize::Short),
                    DataSize::Short      => ("H", DataSize::Word),
                    DataSize::Word       => ("W", DataSize::DoubleWord),
                    DataSize::DoubleWord => ("D", DataSize::Byte),
                }
            },
        };

        ui.same_line();
        if Utils::flag_button(ui, None, format!("{}##data_size", size_label), Some("Data Size (dec/hex values can be Byte, Short, Word or Double Words. Floats can be single or double. Hexii values are always Bytes)")) {
            self.data_size = next_value;
        }

        ui.same_line();
        let data_signed = self.data_signed;
        if Utils::flag_button(ui, Some(&mut self.data_signed), format!("{}##signed", if data_signed { "S" } else { "U" }), Some("Only Dec/Hex numbers can be (S)igned or (U)nsigned data")) {
            if self.data_format == DataFormat::Float {
                self.data_signed = true;
            } else if self.data_format == DataFormat::Hexii {
                self.data_signed = false;
            }
        }

        ui.same_line();
        Utils::flag_button(ui, Some(&mut self.show_ascii), "A", Some("Show Ascii"));

        ui.same_line();
        Utils::flag_button(ui, None, "Save", Some("Save memory region to file"));

        ui.same_line();
        Utils::flag_button(ui, None, "Load", Some("Load memory region from file"));

        ui.separator();

        let mut address = format!("${:08X}", self.memory_display_min.unwrap_or(0));
        ui.set_next_item_width(78.0);  // TODO calculate font width and provide ~9-10 digits worth of space?
        ui.input_text("Address", &mut address).build();

        ui.same_line();
        ui.set_next_item_width(350.0); // TODO how to better calculate this width? it doesn't seem to count the label width
        if let Some(_combobox) = ui.begin_combo("Region", format_region(self.current_region)) {
            for i in 0..MEMORY_REGIONS.len() {
                if i == self.current_region {
                    ui.set_item_default_focus();
                }

                if ui.selectable_config(format_region(i)).selected(i == self.current_region).build() {
                    // change current address to base of the region?
                    self.current_region = i;
                }
            }
        }

        ui.same_line();
        ui.set_next_item_width(100.0);
        let mut cols = self.column_count as i32;
        if ui.input_int("Cols", &mut cols).step(1).build() {
            // TODO max column count will depend on data type
            self.column_count = std::cmp::min(std::cmp::max(cols, 1), 32) as usize;
        }    
    }

    fn draw_memory(&mut self, ui: &imgui::Ui) {
        let memory_region = &MEMORY_REGIONS[self.current_region];

        if let Some(_memory_window) = ui.child_window("##scrolling").begin() {
            // calculate the width of a character glyph, assuming a monospaced font for rendering
            let char_width = ui.calc_text_size("T")[0];
            let space_width = ui.calc_text_size(" ")[0];

            // calculate characters per data element
            let max_chars_per_element = match self.data_format {
                DataFormat::Dec => {
                    match self.data_size {
                        DataSize::Byte       => if self.data_signed {  4.0 } else {  3.0 },      // -128-127 / 0-255
                        DataSize::Short      => if self.data_signed {  6.0 } else {  5.0 },      // -32768-32767 / 0-65535
                        DataSize::Word       => if self.data_signed { 11.0 } else { 10.0 },      // -2147483647-2147483646 / 0-4294967295
                        DataSize::DoubleWord => 20.0,                                            // -9223372036854775807-922337203685477579 / 0-18446744073709551615
                    }
                },
                DataFormat::Hex => (if self.data_signed { 1 } else { 0 } + 2 * self.data_size.size()) as f32,
                DataFormat::Float => {
                    // No idea on these sizes
                    match self.data_size {
                        DataSize::Word       => 13.0,
                        DataSize::DoubleWord => 19.0,
                        _ => panic!(),
                    }
                },
                // Hexii is always 2
                DataFormat::Hexii => 2.0,
            };

            // column width depends on the data format, but always add 1 for the trailing space
            let column_width = (1.0 * space_width) + max_chars_per_element * char_width;

            // column "groups" are 8 bytes/4 shorts/2 words/1 dword, minus the final space at the end of the last item
            let elements_per_column_group = 8 / self.data_size.size();
            let column_group_width = column_width * (elements_per_column_group as f32) - char_width;
            
            // spacing between columns is constant
            let spacing_between_column_groups = 3.0 * char_width;

            // address field is "$00000000:  " -- 10 normal chars and 2 spaces
            let address_width = 10.0 * char_width + 2.0 * space_width;

            // determine location of the ascii display divider and draw it
            if self.show_ascii {
                let num_group_dividers = (((self.column_count as f32) / (elements_per_column_group as f32)).ceil() as u32).saturating_sub(1); // if there are two groups (even incomplete), there's one divider
                                    // ascii divider placed after the address indicator
                let ascii_divider = address_width 
                                    // plus the size of all full columns
                                    + ((self.column_count / elements_per_column_group) as f32) * column_group_width 
                                    // plus the size of all the dividers
                                    + (num_group_dividers as f32) * spacing_between_column_groups
                                    // plus plus the width of the leftover elements in the final column group,
                                    // minus the final space on the final element, truncate to 0
                                    + (((self.column_count % elements_per_column_group) as f32) * column_width - space_width).max(0.0)
                                    // plus one and a half characters width for the center of the line
                                    + 1.5 * char_width
                                    ;

                let draw_list = ui.get_window_draw_list();
                let window_pos = ui.window_pos();
                draw_list.add_line([window_pos[0] + ascii_divider, window_pos[1]], 
                                   [window_pos[0] + ascii_divider, window_pos[1] + 10000.0], 
                                   ui.style_color(imgui::StyleColor::Border)).build();
            }

            // for the edit box, clear all padding to make things much more compact
            let _frame_padding_token = ui.push_style_var(imgui::StyleVar::FramePadding([0.0, 0.0]));
            // reduce spacing between line items
            let _item_spacing_token =  ui.push_style_var(imgui::StyleVar::ItemSpacing([0.0, 0.0]));
            
            // reset memory ranges
            self.memory_display_min = None;
            self.memory_display_max = None;

            // memory bytes per row will be useful
            let bytes_per_row = self.column_count * self.data_size.size();
            
            // start a Clipper within the region
            let line_height = ui.text_line_height_with_spacing();
            let list_clipper = imgui::ListClipper::new(((((memory_region.2 + 1) - memory_region.1) as f32) / (bytes_per_row as f32)).ceil() as i32).items_height(line_height).begin(ui);
            for i in list_clipper.iter() {
                let mut x_pos = 0.;
                
                let row_address = memory_region.1.wrapping_add((i as u32) * (bytes_per_row as u32));
                ui.text(format!("${:08X}:", row_address));
                x_pos += address_width;

                // determine min/max of the memory actually visible on screen
                self.memory_display_min = Some(self.memory_display_min.unwrap_or(u32::MAX).min(row_address));
                self.memory_display_max = Some(self.memory_display_max.unwrap_or(0).max(row_address + (self.column_count * self.data_size.size()) as u32));

                // render each column
                for col in 0..self.column_count {
                    let address = row_address + (col * self.data_size.size()) as u32;

                    // format the data
                    let (is_zero, mut value_str) = self.format_data_column(address, max_chars_per_element as usize).unwrap_or_else(|| {
                        (true, String::from_utf8(vec![b'?'; max_chars_per_element as usize]).unwrap())
                    });

                    ui.same_line_with_spacing(x_pos, 0.0);
                    if address == 0x24 {
                        ui.set_next_item_width(max_chars_per_element * char_width);
                        let _id_token = ui.push_id_int(address as i32);
                        if ui.input_text("##data", &mut value_str)
                                .chars_hexadecimal(true)
                                .enter_returns_true(true)
                                .auto_select_all(true)
                                .no_horizontal_scroll(true)
                                .build() {
                            println!("got value: {}", value_str);       
                        }
                    } else {
                        if is_zero {
                            ui.text_disabled(value_str);
                        } else {
                            ui.text(value_str);
                        }
                    }

                    // add column width, and subtract 1 space on the last element in a group, or the last element on the line
                    x_pos += column_width;
                    if ((col % elements_per_column_group) == (elements_per_column_group - 1)) || (col == (self.column_count - 1)) {
                        x_pos -= space_width;

                        // if we're drawing more columns, insert spacing between groups
                        if col != (self.column_count - 1) {
                            x_pos += spacing_between_column_groups; 
                        }
                    }
                }

                if self.show_ascii {
                    x_pos += spacing_between_column_groups; // spacing between the final group and ascii is the same as column groups
                    // ASCII is always in groups of 8 bytes
                    for col in 0..bytes_per_row {
                        let address = row_address + col as u32;

                        ui.same_line_with_spacing(x_pos, 0.0);
                        if let Some(value) = self.get_memory_u8(address) {
                            if value >= 32 && value < 128 {
                                ui.text(format!("{}", String::from_utf8(vec![value]).unwrap()));
                            } else {
                                ui.text_disabled(".");
                            }
                        } else {
                            ui.text_disabled(".");
                        }

                        x_pos += char_width;
                        if ((col % 8) == 7) && (col != (bytes_per_row - 1)) { // extra space between groups of 8
                            x_pos += space_width;
                        }
                    }
                }

                // Terminates the line and move the cursor to the next row
                ui.set_cursor_pos([x_pos, ui.cursor_pos()[1]]);
                ui.dummy([0.0, 0.0]);
            }
        }
    }

    fn get_memory_u8(&self, address: u32) -> Option<u8> {
        if let Some((base_address, memory)) = &self.memory {
            if address >= (*base_address as u32) {
                if let Some(value) = memory.get((((address & !3) - (*base_address as u32)) >> 2) as usize) {
                    let shift = 24 - ((address & 3) << 3);
                    let value = (value >> shift) as u8;
                    return Some(value);
                }
            }
        }
        None
    }

    fn get_memory_u16(&self, address: u32) -> Option<u16> {
        if let Some((base_address, memory)) = &self.memory {
            if address >= (*base_address as u32) {
                if let Some(value) = memory.get((((address & !3) - (*base_address as u32)) >> 2) as usize) {
                    let shift = 16 - ((address & 2) << 4);
                    let value = (value >> shift) as u16;
                    return Some(value);
                }
            }
        }
        None
    }

    fn get_memory_u32(&self, address: u32) -> Option<u32> {
        if let Some((base_address, memory)) = &self.memory {
            if address >= (*base_address as u32) {
                if let Some(value) = memory.get((((address & !3) - (*base_address as u32)) >> 2) as usize) {
                    return Some(*value);
                }
            }
        }
        None
    }

    fn get_memory_u64(&self, address: u32) -> Option<u64> {
        if let Some((base_address, memory)) = &self.memory {
            if address >= (*base_address as u32) {
                if let Some(value1) = memory.get((((address & !3) - (*base_address as u32)) >> 2) as usize) {
                    if let Some(value2) = memory.get(((((address & !3) + 1) - (*base_address as u32)) >> 2) as usize) {
                        return Some((*value1 as u64) << 32 | (*value2 as u64));
                    }
                }
            }
        }
        None
    }

    fn format_data_column(&self, address: u32, max_chars_per_element: usize) -> Option<(bool, String)> {
        match self.data_format {
            DataFormat::Dec => {
                match self.data_size {
                    DataSize::Byte => {
                        if let Some(value) = self.get_memory_u8(address) {
                            if self.data_signed {
                                Some((value == 0, format!("{:>width$}", value as i8, width = max_chars_per_element)))
                            } else {
                                Some((value == 0, format!("{:>width$}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::Short => {
                        if let Some(value) = self.get_memory_u16(address) {
                            if self.data_signed {
                                Some((value == 0, format!("{:>width$}", value as i16, width = max_chars_per_element)))
                            } else {
                                Some((value == 0, format!("{:>width$}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::Word  => {
                        if let Some(value) = self.get_memory_u32(address) {
                            if self.data_signed {
                                Some((value == 0, format!("{:>width$}", value as i32, width = max_chars_per_element)))
                            } else {
                                Some((value == 0, format!("{:>width$}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::DoubleWord  => {
                        if let Some(value) = self.get_memory_u64(address) {
                            if self.data_signed {
                                Some((value == 0, format!("{:>width$}", value as i64, width = max_chars_per_element)))
                            } else {
                                Some((value == 0, format!("{:>width$}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },
                }
            },
            
            DataFormat::Hex => {
                match self.data_size {
                    DataSize::Byte => {
                        if let Some(value) = self.get_memory_u8(address) {
                            // not a fan of this formatting...
                            if self.data_signed {
                                if (value as i8) < 0 {
                                    Some((value == 0, format!("-{:0width$X}", (!value).wrapping_add(1), width = max_chars_per_element - 1)))
                                } else {
                                    Some((value == 0, format!(" {:0width$X}", value, width = max_chars_per_element - 1)))
                                }
                            } else {
                                Some((value == 0, format!("{:0width$X}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::Short => {
                        if let Some(value) = self.get_memory_u16(address) {
                            if self.data_signed {
                                if (value as i16) < 0 {
                                    Some((value == 0, format!("-{:0width$X}", (!value).wrapping_add(1), width = max_chars_per_element - 1)))
                                } else {
                                    Some((value == 0, format!(" {:0width$X}", value, width = max_chars_per_element - 1)))
                                }
                            } else {
                                Some((value == 0, format!("{:0width$X}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::Word => {
                        if let Some(value) = self.get_memory_u32(address) {
                            if self.data_signed {
                                if (value as i32) < 0 {
                                    Some((value == 0, format!("-{:0width$X}", (!value).wrapping_add(1), width = max_chars_per_element - 1)))
                                } else {
                                    Some((value == 0, format!(" {:0width$X}", value, width = max_chars_per_element - 1)))
                                }
                            } else {
                                Some((value == 0, format!("{:0width$X}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },

                    DataSize::DoubleWord => {
                        if let Some(value) = self.get_memory_u64(address) {
                            if self.data_signed {
                                if (value as i64) < 0 {
                                    Some((value == 0, format!("-{:0width$X}", (!value).wrapping_add(1), width = max_chars_per_element - 1)))
                                } else {
                                    Some((value == 0, format!(" {:0width$X}", value, width = max_chars_per_element - 1)))
                                }
                            } else {
                                Some((value == 0, format!("{:0width$X}", value, width = max_chars_per_element)))
                            }
                        } else {
                            None
                        }
                    },
                }
            },

            DataFormat::Float => {
                match self.data_size {
                    DataSize::Word => {
                        if let Some(value) = self.get_memory_u32(address) {
                            let float = f32::from_bits(value);
                            let config = pretty_dtoa::FmtFloatConfig::default()
                                            .add_point_zero(true)
                                            .max_significant_digits(6)
                                            .round();
                            let float_str = String::from(pretty_dtoa::dtoa(float as f64, config));
                            Some((float == 0.0, format!("{:>width$}", float_str, width = max_chars_per_element)))
                        } else {
                            None
                        }
                    },

                    DataSize::DoubleWord => {
                        if let Some(value) = self.get_memory_u64(address) {
                            let float = f64::from_bits(value);
                            let config = pretty_dtoa::FmtFloatConfig::default()
                                            .add_point_zero(true)
                                            .max_significant_digits(12)
                                            .round();
                            let float_str = String::from(pretty_dtoa::dtoa(float, config));
                            Some((float == 0.0, format!("{:>width$}", float_str, width = max_chars_per_element)))
                        } else {
                            None
                        }
                    },

                    _ => panic!("invalid"),
                }
            },

            DataFormat::Hexii => {
                assert!(self.data_size == DataSize::Byte);
                if let Some(value) = self.get_memory_u8(address) {
                    if value >= 32 && value < 128 {
                        Some((false, format!(".{}", String::from_utf8(vec![value]).unwrap())))
                    } else if value == 0xFF {
                        Some((true, format!("##")))
                    } else if value == 0 {
                        Some((true, format!("  ")))
                    } else {
                        Some((false, format!("{:02X}", value)))
                    }
                } else {
                    None
                }
            }
        }
    }
}

impl GameWindow for Memory {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        let mut opened = true;
        let window = ui.window("Memory");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .flags(imgui::WindowFlags::NO_SCROLLBAR | imgui::WindowFlags::NO_SCROLL_WITH_MOUSE)
              .build(|| {

            // update and process inputs
            self.update(ui);
            self.draw_contents(ui);
        });

        opened
    }
}    

impl Drop for Memory {
    fn drop(&mut self) {
        self.comms.decrement_debugger_windows();
    }
}

