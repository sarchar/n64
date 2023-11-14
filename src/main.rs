use std::env;

use tracing::info;
use tracing_core::Level;
use tracing_subscriber::{filter, prelude::*};

use n64::System;
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
                //println!("{:?}", filter);
            } else {
                eprintln!("invalid filter format: {}", filter_fmt);
            }
        }).unwrap();
    });

    // create the n64 system
    let mut system = System::new("boot.rom", args[1].as_str());

    // either run or debug
    if args.len() == 3 && args[2] == "-D" {
        let mut debugger = Debugger::new(system, change_logging);
        println!("Entering debugger...");
        debugger.run().expect("Debugger failed");
    } else {
        system.run();
    }
}
