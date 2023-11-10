use std::env;

use n64::pifrom::PifRom;
use n64::rcp::Rcp;
use n64::peripheral::PeripheralInterface;
use n64::cpu::Cpu;
use n64::debugger::Debugger;

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() < 2 {
        println!("Usage: {} file.z64 [-D]", args[0]);
        return;
    }

    let pif = PifRom::new("boot.rom");
    let pi = PeripheralInterface::new(args[1].as_str());
    let rcp = Rcp::new(pif, pi);
    let mut cpu = Cpu::new(rcp);

    if args.len() == 3 && args[2] == "-D" {
        let mut debugger = Debugger::new(cpu);
        println!("Entering debugger...");
        debugger.run().expect("Debugger failed");
    } else {
        loop {
            cpu.step();
        }
    }
}
