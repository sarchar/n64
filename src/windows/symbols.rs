use crate::*;
use crossbeam::channel::{Receiver, Sender, self};
use n64::debugger;
use gui::game::GameWindow;

#[derive(Clone, Debug)]
pub enum SymbolsMessage {
    GotoAddress(u64),
}

pub struct Symbols {
    comms: SystemCommunication,    

    /// request-response channel to Debugger
    debugging_request_response_rx: Receiver<debugger::DebuggerCommandResponse>,
    debugging_request_response_tx: Sender<debugger::DebuggerCommandResponse>,

    /// we need an entire copy of the symbol database, but maybe that's a pretty bad waste of memory
    requested_symbols: bool,
    all_symbols: Vec<(u64, String)>,
    filtered_symbols: Vec<(u64, String)>,
    filter_string: String,

    /// publishing channel
    publish_channel: Sender<SymbolsMessage>,    
}

impl Symbols {
    pub fn new(mut comms: SystemCommunication) -> Self {
        let (debugging_request_response_tx, debugging_request_response_rx) = channel::unbounded();

        comms.increment_debugger_windows();

        let publish_channel = comms.pubsub.create_channel();
        
        Self {
            comms,
            debugging_request_response_rx,
            debugging_request_response_tx,
            requested_symbols: false,
            all_symbols: Vec::new(),
            filtered_symbols: Vec::new(),
            filter_string: String::new(),
            publish_channel,
        }
    }

    fn update(&mut self, _delta_time: f32) {
        if !self.requested_symbols {
            let request = debugger::DebuggerCommand {
                command_request:  debugger::DebuggerCommandRequest::GetAllSymbols,
                response_channel: Some(self.debugging_request_response_tx.clone()),
            };

            self.requested_symbols = debugger::Debugger::send_command(&self.comms, request).is_ok();            
        }

        while let Ok(response) = self.debugging_request_response_rx.try_recv() {
            match response {
                debugger::DebuggerCommandResponse::AllSymbols(symbols) => {
                    self.all_symbols.clear();
                    for (address, name) in symbols.into_iter() {
                        self.all_symbols.push((address, name));
                    }
                },

                _ => {},
            }
        }

    }
}

impl GameWindow for Symbols {
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool {
        self.update(ui.io().delta_time);
        
        let mut opened = true;
        let window = ui.window("Symbols");     
        window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .opened(&mut opened)
              .flags(imgui::WindowFlags::NO_SCROLLBAR | imgui::WindowFlags::NO_SCROLL_WITH_MOUSE)
              .build(|| {

            let mut filter = self.filter_string.clone();
            ui.input_text("Filter", &mut filter).callback(imgui::InputTextCallback::EDIT, SymbolFilter::new(self)).build();
            self.filter_string = filter;

            ui.separator();
            
            let columns =  [
                imgui::TableColumnSetup { name: "Address"    , flags: imgui::TableColumnFlags::WIDTH_FIXED  , init_width_or_weight: 10.0, user_id: ui.new_id(0) },
                imgui::TableColumnSetup { name: "Symbol"     , flags: imgui::TableColumnFlags::WIDTH_STRETCH, init_width_or_weight: 10.0, user_id: ui.new_id(1) },
            ];

            // let table_flags = if self.show_resizers {
            //     imgui::TableFlags::BORDERS_INNER_V
            // } else {
            //     imgui::TableFlags::NO_BORDERS_IN_BODY
            // } | imgui::TableFlags::RESIZABLE | imgui::TableFlags::NO_PAD_OUTER_X;
            let table_flags = imgui::TableFlags::RESIZABLE | imgui::TableFlags::BORDERS_INNER_V | imgui::TableFlags::ROW_BG 
                              | imgui::TableFlags::SORTABLE | imgui::TableFlags::SCROLL_Y;

            // use begin_table_with_sizing so we can skip the ui.table_headers_row call, as we want to
            // set up and use columns but we don't want to display the headers row
            if let Some(_) = ui.begin_table_with_sizing("##symbols", columns.len(), table_flags, [0.0, 0.0], 0.0) {

                ui.table_setup_scroll_freeze(columns.len(), 1);

                // create the columns
                for column in columns {
                    ui.table_setup_column_with(column);    
                }

                // show the headers row
                ui.table_headers_row();

                if let Some(sort_specs) = ui.table_sort_specs_mut() {
                    sort_specs.conditional_sort(|specs| {
                        for spec in specs.iter() {
                            let index = spec.column_idx();
                            let direction = spec.sort_direction();
                            if index == 0 {
                                self.all_symbols.sort_by_key(|f| f.0);
                            } else if index == 1 {
                                self.all_symbols.sort_by_cached_key(|f| f.1.clone());
                            }
                            if direction.unwrap() == imgui::TableSortDirection::Descending {
                                self.all_symbols.reverse();
                            }

                            // refilter after sort
                            let s = self.filter_string.clone();
                            SymbolFilter::new(self).filter(s);

                            break;
                        }
                    });
                }

                let list_clipper = imgui::ListClipper::new(self.filtered_symbols.len() as i32).begin(ui);
                for i in list_clipper.iter() {
                    let (address, symbol) = self.filtered_symbols.get(i as usize).unwrap();

                    ui.table_next_column();
                    if ui.selectable_config(format!("##{}", i)).flags(imgui::SelectableFlags::SPAN_ALL_COLUMNS).build() {
                        let _ = self.publish_channel.try_send(SymbolsMessage::GotoAddress(*address));
                    }
                    ui.same_line_with_spacing(0.0, 0.0);
                    
                    if ((*address as i32) as u64) == *address {
                        ui.text(format!("${:08X}", *address as u32));
                    } else {
                        ui.text(format!("${:016X}", *address));
                    }

                    ui.table_next_column();
                    ui.text(format!("{}", symbol));
                }
            }
        });

        opened
    }
}

impl Drop for Symbols {
    fn drop(&mut self) {
        self.comms.decrement_debugger_windows();
    }
}

struct SymbolFilter<'a> {
    symbols: &'a mut Symbols,
}

impl<'a> SymbolFilter<'a> {
    fn new(symbols: &'a mut Symbols) -> Self {
        Self {
            symbols,
        }
    }

    fn filter<T: AsRef<str>>(&mut self, s: T) {
        let str = s.as_ref().trim();
        self.symbols.filtered_symbols = self.symbols.all_symbols.iter().filter_map(|e| {
            if e.1.to_lowercase().contains(str.to_lowercase().as_str()) {
                Some((e.0, e.1.clone()))
            } else {
                None
            }
        }).collect();
    }
}

impl<'a> imgui::InputTextCallbackHandler for SymbolFilter<'a> {
    fn on_edit(&mut self, data: imgui::TextCallbackData) {
        self.filter(data.str());
    }
}
