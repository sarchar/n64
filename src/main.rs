use std::env;

use n64::pifrom::PifRom;
use n64::rcp::Rcp;
use n64::peripheral::PeripheralInterface;
use n64::cpu::Cpu;

fn main() {
    let args = env::args().collect::<Vec<String>>();
    if args.len() != 2 {
        assert!(args.len() == 1);
        println!("Usage: {} file.z64", args[0]);
        return;
    }

    let pif = PifRom::new("boot.rom");
    let pi = PeripheralInterface::new(args[1].as_str());
    let rcp = Rcp::new(pif, pi);
    let mut cpu = Cpu::new(rcp);

    loop {
        cpu.step();
    }
}
