use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::{cpu, debugger::{self}};
use gui::game::GameWindow;

pub struct Tlb {
    comms: SystemCommunication,    

    // request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    // current tlb
    tlb: [cpu::TlbEntry; 32],
    
    // requested state
    requested_tlb: bool,
}

impl Tlb {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            tlb: [cpu::TlbEntry::default(); 32],
            // group_registers: true,
            requested_tlb: false,
        }    
    }

    fn update(&mut self, _delta_time: f32) {
        // request the current TLB
        if !self.requested_tlb {
            let command = debugger::DebuggerCommand {
                command_request: debugger::DebuggerCommandRequest::GetTlb,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };
            if debugger::Debugger::send_command(&self.comms, command).is_ok() {
                self.requested_tlb = true;
            }
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::Tlb(tlb) => {
                    self.tlb = tlb;
                    self.requested_tlb = false;
                },
                
                r @ _ => {
                    warn!(target: "Cop0 State", "Unhandled message from debugger: {:?}", r);
                }
            }
        }
    }
}


impl GameWindow for Tlb {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        let mut opened = true;

        self.update(ui.io().delta_time);

        let window = ui.window("TLB");
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .build(|| {

            let columns =  [
                imgui::TableColumnSetup { name: "##index"    , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(0) },
                imgui::TableColumnSetup { name: "G"          , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(1) },
                imgui::TableColumnSetup { name: "ASID"       , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(2) },
                imgui::TableColumnSetup { name: "R"          , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(3) },
                imgui::TableColumnSetup { name: "VPN"        , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(4) },
                imgui::TableColumnSetup { name: "Page Mask"  , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(5) },
                imgui::TableColumnSetup { name: "PFN0"       , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(6) },
                imgui::TableColumnSetup { name: "C0"         , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(7) },
                imgui::TableColumnSetup { name: "D0"         , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(8) },
                imgui::TableColumnSetup { name: "PFN1"       , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(9) },
                imgui::TableColumnSetup { name: "C1"         , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(10) },
                imgui::TableColumnSetup { name: "D1"         , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(11) },
            ];

            let table_flags = if true /*self.show_resizers*/ {
                imgui::TableFlags::BORDERS_INNER_V
            } else {
                imgui::TableFlags::NO_BORDERS_IN_BODY
            } | imgui::TableFlags::RESIZABLE | imgui::TableFlags::NO_PAD_OUTER_X | imgui::TableFlags::ROW_BG;
            
            if let Some(_table_token) = ui.begin_table_with_sizing("TLB Table", columns.len(), table_flags, [0.0, 0.0], 0.0) {
                // create the columns
                for column in columns {
                    ui.table_setup_column_with(column);
                }

                ui.table_headers_row();
                
                // Loop over each entry in the tlb and display information
                for (i, tlb_entry) in (&self.tlb).iter().enumerate() {
                    ui.table_next_column();
                    ui.text(format!("{}", i));
                    if ui.is_item_hovered() {
                        ui.tooltip_text(format!("TLB entry {}", i));
                    }

                    let mut g = (tlb_entry.entry_hi & 0x1000) != 0;
                    ui.table_next_column();
                    ui.checkbox(format!("##g{}", i), &mut g);
                    if ui.is_item_hovered() {
                        ui.tooltip_text(format!("Global bit"));
                    }

                    let asid = tlb_entry.entry_hi & 0xFF;
                    ui.table_next_column();
                    ui.text(format!("0x{:02X}", asid));
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Address Space ID");
                    }

                    let r = (tlb_entry.entry_hi >> 62) & 0x03;
                    ui.table_next_column();
                    ui.text(match r {
                        0 => "U", 1 => "S", 2 => "?", 3 => "K", _ => panic!(),
                    });
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Memory Region (U=user, S=Supervisor, K=kernel)")
                    }

                    ui.table_next_column();
                    let vpn2 = tlb_entry.entry_hi & 0xFF_FFFF_E000;
                    ui.text(format!("0x{:010X}", vpn2));
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Virtual Page Number");
                    }

                    ui.table_next_column();
                    let page_mask = (tlb_entry.page_mask >> 1) | 0xFFF;
                    ui.text(format!("0x{:04X}", page_mask));
                    if ui.is_item_hovered() {
                        ui.tooltip_text(format!("Page Mask. For this entry page size is {} bytes", page_mask + 1));
                    }

                    ui.table_next_column();
                    let pfn0 = (tlb_entry.entry_lo0 >> 6) << 12;
                    let v0 = (tlb_entry.entry_lo0 & 0x02) != 0;
                    if v0 {
                        ui.text(format!("0x{:08X}", pfn0));
                    } else {
                        ui.text_disabled(format!("0x{:08X}", pfn0));
                    }
                    if ui.is_item_hovered() {
                        ui.tooltip_text(format!("Page Frame Number of EntryLo0 (physical address).{}", if v0 { "" } else { " This entry is disabled." }));
                    }

                    ui.table_next_column();
                    let c0 = (tlb_entry.entry_lo0 >> 3) & 0x07;
                    let mut c0_bool = c0 != 2;
                    ui.checkbox(format!("##c0{}", i), &mut c0_bool);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Cache coherency. If clear, access will be uncached.");
                    }

                    ui.table_next_column();
                    let mut d0 = ((tlb_entry.entry_lo0 >> 2) & 0x01) != 0;
                    ui.checkbox(format!("##d0{}", i), &mut d0);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Dirty bit. If set, this page is writable");
                    }

                    ui.table_next_column();
                    let pfn1 = (tlb_entry.entry_lo1 >> 6) << 12;
                    let v1 = (tlb_entry.entry_lo1 & 0x02) != 0;
                    if v1 {
                        ui.text(format!("0x{:08X}", pfn1));
                    } else {
                        ui.text_disabled(format!("0x{:08X}", pfn1));
                    }
                    if ui.is_item_hovered() {
                        ui.tooltip_text(format!("Page Frame Number of EntryLo1 (physical address).{}", if v0 { "" } else { " This entry is disabled." }));
                    }

                    ui.table_next_column();
                    let c1 = (tlb_entry.entry_lo1 >> 3) & 0x07;
                    let mut c1_bool = c1 != 2;
                    ui.checkbox(format!("##c1{}", i), &mut c1_bool);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Cache coherency. If clear, access will be uncached.");
                    }

                    ui.table_next_column();
                    let mut d1 = ((tlb_entry.entry_lo1 >> 2) & 0x01) != 0;
                    ui.checkbox(format!("##d1{}", i), &mut d1);
                    if ui.is_item_hovered() {
                        ui.tooltip_text("Dirty bit. If set, this page is writable");
                    }

                    if i != self.tlb.len() - 1 {
                        ui.table_next_row();
                    }
                }
            }
        });

        opened
    }
}

