use std::env;

use n64::System;
use n64::debugger::Debugger;

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() < 2 {
        println!("Usage: {} file.z64 [-D]", args[0]);
        return;
    }

    let mut system = System::new("boot.rom", args[1].as_str());

    if args.len() == 3 && args[2] == "-D" {
        let mut debugger = Debugger::new(system);
        println!("Entering debugger...");
        debugger.run().expect("Debugger failed");
    } else {
        system.run();
    }
}
