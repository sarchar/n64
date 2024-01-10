use std::env;
use cfg_if::cfg_if;

#[cfg(all(feature="gui", feature="headless"))]
compile_error!{"features \"gui\" and \"headless\" cannot be enabled together"}

#[cfg(not(feature="headless"))]
mod gui;

#[allow(unused_imports)]
use tracing::{debug, error, warn, info};
use tracing_core::Level;
use tracing_subscriber::{filter, prelude::*};

//.use gilrs::{Gilrs, Button};
//.use gilrs::ev::filter::Filter;

use n64::{System, SystemCommunication};
use n64::debugger::Debugger;

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() < 2 {
        println!("Usage: {} file.z64 [-D]", args[0]);
        return;
    }

    // default global subscriber to stdout
    let subscriber = tracing_subscriber::fmt::layer()
                        .compact()
                        .with_ansi(true)
                        .with_file(false)
                        .with_line_number(false)
                        .with_thread_ids(false)
                        .with_level(true)
                        .with_target(true);

    // the starting filter disables all of rustyline and the rest of the program to INFO
    let default_filter = filter::Targets::new()
                            .with_target("rustyline", tracing_core::Level::ERROR)
                            //.with_target("RCP", tracing_core::Level::TRACE)
                            .with_target("RDP", tracing_core::Level::DEBUG)
                            //.with_target("VI", tracing_core::Level::DEBUG)
                            //.with_target("RSP", tracing_core::Level::DEBUG)
                            //.with_target("JOY", tracing_core::Level::DEBUG)
                            //.with_target("MI", tracing_core::Level::DEBUG)
                            //.with_target("DMA", tracing_core::Level::TRACE)
                            .with_target("HLE", tracing_core::Level::TRACE)
                            .with_target("wgpu_hal", tracing_core::Level::WARN)
                            .with_target("wgpu_core", tracing_core::Level::WARN)
                            .with_target("GUI", tracing_core::Level::DEBUG)
                            .with_default(Level::INFO);

    // create a reload layer so logging levels can be modified
    let (default_filter, reload_handle) = tracing_subscriber::reload::Layer::new(default_filter);

    // register the subscriber/filter
    tracing_subscriber::registry()
        .with(subscriber
                .with_filter(default_filter))
        .init();

    // create a way to change logging levels at runtime
    // move is required to move reload_handle into the closure
    // box is used to transfer the closure to the debugger
    // and in the future, the UI
    let change_logging = Box::new(move |filter_fmt: &str, default_level: Level| {
        info!("changing logging format to \"{}\" (default = {:?})", filter_fmt, default_level);
        reload_handle.modify(|filter| {
            if let Ok(new_filter) = filter_fmt.parse::<filter::Targets>() {
                *filter = new_filter
                            .with_target("rustyline", Level::ERROR)
                            .with_default(default_level);
            } else {
                eprintln!("invalid filter format: {}", filter_fmt);
            }
        }).unwrap();
    });

    // The following gilrs code does work on windows, but enumerating gamepads() does not work
    // without a GUI window
    // 
    //. let mut gilrs = match gilrs::GilrsBuilder::new().set_update_state(false).build() {
    //.     Ok(g) => g,
    //.     Err(gilrs::Error::NotImplemented(g)) => {
    //.         error!("gilrs doesn't support current platform");
    //.         g
    //.     },
    //.     Err(e) => {
    //.         error!("gilrs failed to create context: {}", e);
    //.         std::process::exit(-1);
    //.     },
    //. };

    //. let repeat_filter = gilrs::ev::filter::Repeat::new();
    //. loop {
    //.     while let Some(ev) = gilrs.next_event_blocking(None).filter_ev(&repeat_filter, &mut gilrs) {
    //.         gilrs.update(&ev);
    //.         println!("{:?}", ev);
    //.     }
    //. }

    let program_rom = String::from(args[1].as_str());
    let make_system = move |comms: SystemCommunication| {
        System::new("bios/pifrom.v64", &program_rom, comms)
    };

    // either run or debug
    if args.len() == 3 && args[2] == "-D" {
        let comms = SystemCommunication::new(None);
        let mut debugger = Debugger::new(make_system(comms), change_logging);
        println!("Entering debugger...");
        debugger.run().expect("Debugger failed");
    } else {
        cfg_if! {
            if #[cfg(feature="headless")] {
                let comms = SystemCommunication::new(None);
                make_system(comms).run();
            } else {
                pollster::block_on(gui::run::<gui::game::Game>(Box::new(make_system)));
            }
        }
    }
}

