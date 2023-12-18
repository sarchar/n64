use std::env;
use cfg_if::cfg_if;

cfg_if! {
    if #[cfg(feature="gui")] {
        use pixels::{
            Pixels, SurfaceTexture
        };
        
        use winit::{
            dpi::LogicalSize,
            event::{Event, WindowEvent, VirtualKeyCode},
            event_loop::{ControlFlow, EventLoop},
            window::WindowBuilder
        };
        
        use winit_input_helper::WinitInputHelper;

        use tracing::error;

        // switch constants for 320x240x16/640x480x32
        const WINDOW_WIDTH: u32 = 320;
        const WINDOW_HEIGHT: u32 = 240;
        const SCALE: f64 = 2.0;
        //const WINDOW_WIDTH: u32 = 640;
        //const WINDOW_HEIGHT: u32 = 480;
        //const SCALE: f64 = 1.0;
    }
}

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
                            .with_target("RCP", tracing_core::Level::DEBUG)
                            //.with_target("RDP", tracing_core::Level::DEBUG)
                            //.with_target("RSP", tracing_core::Level::DEBUG)
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

    // create the n64 system
    #[allow(unused_mut)]
    let mut system = System::new("bios/pifrom.v64", args[1].as_str());

    // either run or debug
    if args.len() == 3 && args[2] == "-D" {
        let mut debugger = Debugger::new(system, change_logging);
        println!("Entering debugger...");
        debugger.run().expect("Debugger failed");
    } else {
        cfg_if! {
            if #[cfg(feature="gui")] {
                run_gui(system);
            } else {
                system.run();
            }
        }
    }
}

#[cfg(feature="gui")]
fn run_gui(system: System) {
    let mut system = system;
    let event_loop = EventLoop::new();
    let mut input = WinitInputHelper::new();
    let window = {
        let size = LogicalSize::new(WINDOW_WIDTH as f64, WINDOW_HEIGHT as f64);
        let scaled_size = LogicalSize::new(WINDOW_WIDTH as f64 * SCALE, WINDOW_HEIGHT as f64 * SCALE);
        WindowBuilder::new()
            .with_title("N64 Emulator")
            .with_inner_size(scaled_size)
            .with_min_inner_size(size)
            .build(&event_loop)
            .expect("Error creating window")
    };

    let mut pixels = {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(WINDOW_WIDTH, WINDOW_HEIGHT, surface_texture).expect("Error creating pixels surface")
    };

    let mut closing = false;
    const CYCLES_PER_FRAME: i64 = 3125000;

    event_loop.run(move |event, _, control_flow| {
        if closing { 
            *control_flow = ControlFlow::Exit;
            return; 
        }

        *control_flow = ControlFlow::Poll; // continually call the update function even if
                                           // there are no events pending

        // Check for input
        if input.update(&event) {
            if input.key_pressed(VirtualKeyCode::Escape) || input.close_requested() {
                closing = true;
                *control_flow = ControlFlow::Exit;
                return;
            }
        }

        match event {
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                window_id,
            } if window_id == window.id() => {
                closing = true;
                *control_flow = ControlFlow::Exit;
            },

            Event::MainEventsCleared => {
                window.request_redraw();
            },

            Event::RedrawRequested(_) => {
                // run system for 93.75MHz/30fps = 3.125M cycles/frame
                // in reality, the video interface is rendering from the frame buffer
                // at a fixed clock rate not caring what the cpu is doing
                for _ in 0..CYCLES_PER_FRAME {
                    let _ = system.step();
                }

                // render from the frame buffer. the n64-systemtests rom sets the frame buffer to
                // 5:5:5:1 rgba, and our rdram is an array of words, so we dissect that and push it
                // to a 2d 32bpp pixel. TODO maybe pixels supports 5:5:5:1 formats?
                let rcp = system.rcp.borrow();
                let framebuffer_base = (rcp.vi.origin() >> 2) as usize;
                if framebuffer_base != 0 {
                    // need access to rdram
                    let rdram = rcp.ri.rdram();
                    let frame = pixels.frame_mut();

                    // uncomment for 640x480x32
                    //for y in 0..480 {
                    //    for x in 0..640 {
                    //        let offs = (y * 640) + x;
                    //        let word = rdram[framebuffer_base+(offs as usize)];
                    //        let r = word >> 24;
                    //        let g = (word >> 16) & 0xFF;
                    //        let b = (word >> 8) & 0xFF;
                    //        let offs = offs * 4;
                    //        frame[offs + 0] = r as u8;
                    //        frame[offs + 1] = g as u8;
                    //        frame[offs + 2] = b as u8;
                    //        frame[offs + 3] = 0xff;
                    //    }
                    //}

                    // uncomment for 320x240x16
                    for y in 0..240 {
                        for x in 0..320 {
                            let offs = (y * 320) + x; // want this half word from rdram
                            let word = rdram[framebuffer_base+((offs >> 1) as usize)];
                            let shift = 16 - ((offs & 1) << 4);
                            let color = ((word >> shift) & 0xffff) as u16;
                            let r = color >> 11;
                            let g = (color >> 6) & 0x1F;
                            let b = (color >> 1) & 0x1F;
                            let offs = (4 * ((y * WINDOW_WIDTH) + x)) as usize;
                            frame[offs + 0] = (r << 3) as u8;
                            frame[offs + 1] = (g << 3) as u8;
                            frame[offs + 2] = (b << 3) as u8;
                            frame[offs + 3] = 0xff;
                        }
                    }
                }

                if let Err(err) = pixels.render() {
                    error!(target: "pixels", "{err}");
                    closing = true;
                    *control_flow = ControlFlow::Exit;
                    return;
                }

                // request another redraw
                window.request_redraw();
            },

            _ => {},
        };
    });
}
