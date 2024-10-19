use cfg_if::cfg_if;

#[cfg(all(feature="gui", feature="headless"))]
compile_error!{"features \"gui\" and \"headless\" cannot be enabled together"}

cfg_if! {
    if #[cfg(not(feature="headless"))] {
        mod gui;
        mod windows;
    }
}

#[allow(unused_imports)]
use tracing::{debug, error, warn, info};
use tracing_core::Level;
use tracing_subscriber::{filter, prelude::*};

use clap::Parser;

use n64::{System, SystemCommunication};

#[derive(Parser, Debug, Clone)]
#[command(author, version, about, long_about = "long about string")]
struct Args {
    /// Game to run. Only .Z64 (big-endian) files are currently supported.
    game_file: String,

    /// Increase logging verbosity. Can be specified multiple times. Default is WARN, then INFO -> DEBUG -> TRACE.
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,

    /// Specify logging string. Comma-separated list of MODULE=LEVEL, e.g., "HLE=trace,GUI=debug"
    #[arg(short('L'), long, value_name("STR"))]
    logging_format: Option<String>,

    /// Force use of OpenGL backend. Default is Vulkan.
    #[arg(short('g'), long("opengl"))]
    force_opengl: bool,

    /// Specify window scaling ratio. (1 = 320x240, 2 = 640x480, 3 = 960x720 [default], etc.)
    #[arg(short('s'), long("scale"), value_name("S"), default_value_t = 3)]
    window_scale: u8,

    /// Synchronize the UI with the Game render (useful for RenderDoc captures)
    #[arg(short('S'), long("syncui"))]
    sync_ui_to_game: bool,

    /// Use the interpreter core only (no JIT)
    #[arg(short('I'), long("interpreter"))]
    use_interpreter_cpu_core: bool,

    /// Use Docking features of imgui (useful for debugger). Places the game render into a imgui window
    #[arg(short('D'), long("docking"))]
    enable_docking: bool,

    /// Specify ELF file for DWARF debugging information
    #[arg(short('e'), long("elf"))]
    elf_file: Option<String>,
}

fn main() {
    let args = Args::parse();

    // default global subscriber to stdout
    let subscriber = tracing_subscriber::fmt::layer()
                        .compact()
                        .with_ansi(true)
                        .with_file(false)
                        .with_line_number(false)
                        .with_thread_ids(false)
                        .with_level(true)
                        .with_target(true);

    let default_level = match args.verbose {
        0 => Level::WARN,
        1 => Level::INFO,
        2 => Level::DEBUG,
        _ => Level::TRACE,
    };

    // the starting filter disables all of rustyline and the rest of the program to INFO
    let default_filter = filter::Targets::new()
                            .with_target("rustyline", tracing_core::Level::ERROR)
                            .with_target("wgpu_hal", tracing_core::Level::WARN)
                            .with_target("wgpu_core", tracing_core::Level::WARN)
                            //.with_target("RCP", tracing_core::Level::TRACE)
                            //.with_target("RDP", tracing_core::Level::TRACE)
                            //.with_target("VI", tracing_core::Level::TRACE)
                            //.with_target("RSP", tracing_core::Level::DEBUG)
                            //.with_target("JOY", tracing_core::Level::TRACE)
                            //.with_target("PIF", tracing_core::Level::TRACE)
                            //.with_target("MI", tracing_core::Level::DEBUG)
                            //.with_target("DMA", tracing_core::Level::TRACE)
                            //.with_target("HLE", tracing_core::Level::TRACE)
                            //.with_target("SI", tracing_core::Level::TRACE)
                            //.with_target("PIF-ROM", tracing_core::Level::TRACE)
                            //.with_target("GUI", tracing_core::Level::DEBUG)
                            //.with_target("AI", tracing_core::Level::TRACE)
                            .with_default(default_level)
                            ;

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
                            .with_target("wgpu_hal", Level::WARN)
                            .with_target("wgpu_core", Level::WARN)
                            .with_default(default_level);
            } else {
                eprintln!("invalid filter format: {}", filter_fmt);
            }
        }).unwrap();
    });

    if let Some(fmt) = args.logging_format.as_deref() {
        change_logging(fmt, default_level);
    }

    let program_rom = args.game_file.clone();
    let use_interpreter_cpu_core = args.use_interpreter_cpu_core;
    let make_system = move |comms: SystemCommunication| {
        comms.settings.write().unwrap().cpu_interpreter_only = use_interpreter_cpu_core;
        System::new(comms, "bios/pifrom.z64", &program_rom)
    };

    cfg_if! {
        if #[cfg(feature="headless")] {
            let comms = SystemCommunication::new(None);
            Debugger::new(make_system(comms)).run().unwrap();
        } else {
            let gilrs = match gilrs::GilrsBuilder::new().set_update_state(false).build() {
                Ok(g) => {
                    info!(target: "GUI", "Gilrs initialized");
                    g
                },

                Err(gilrs::Error::NotImplemented(g)) => {
                    error!(target: "GUI", "gilrs doesn't support current platform");
                    g
                },

                Err(e) => {
                    error!(target: "GUI", "gilrs failed to create context: {}", e);
                    std::process::exit(-1);
                },
            };

            pollster::block_on(gui::run::<gui::game::Game>(args, Box::new(make_system), change_logging, gilrs));
        }
    }
}

