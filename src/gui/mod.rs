use std::sync::{Arc, mpsc};
use std::time::Instant;

use tracing_core::Level;

use winit::{
    dpi::{LogicalSize, PhysicalSize},
    event::{Event, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use winit_input_helper::WinitInputHelper;

use imgui::*;
use imgui_wgpu::{Renderer, RendererConfig};

use gilrs::ev::filter::Filter;

use crate::*;
use n64::SystemCommunication;
use n64::hle::HleCommandBuffer;

pub mod game;

struct WgpuInfo<'a> {
    //instance      : wgpu::Instance,
    surface       : wgpu::Surface<'a>,
    surface_config: wgpu::SurfaceConfiguration,
    device        : wgpu::Device,
    queue         : wgpu::Queue,

    needs_reconfigure: bool,
}

pub struct AppWindow<'a> {
    event_loop  : Option<EventLoop<()>>,
    size        : LogicalSize<u32>,
    window      : Arc<Window>,
    input_helper: WinitInputHelper,
    event_log   : Vec<WindowEvent>,
    wgpu        : WgpuInfo<'a>,
    gilrs       : gilrs::Gilrs,
    connected_gamepads: [Option<gilrs::GamepadId>; 4],
    change_logging_func: Box<dyn Fn(&str, Level) -> ()>,
}

impl<'a> AppWindow<'a> {
    async fn new(title: &str, width: u32, height: u32, allow_vulkan: bool, change_logging: Box<dyn Fn(&str, Level) -> ()>, gilrs: gilrs::Gilrs) -> Self {
        let event_loop = EventLoop::new().expect("error creating event loop");

        // Create the platform window with winit
        let window_size = LogicalSize::new(width, height);
        let window = Arc::new({
            WindowBuilder::new()
                .with_title(title)
                .with_inner_size(window_size)
                .with_min_inner_size(LogicalSize::new(50, 50))
                .with_resizable(allow_vulkan) // my current environment crashes when using the Gl
                                              // backend, so I'm disabling window resize in OpenGL
                .build(&event_loop)
                .expect("Error creating window")
        });

        let backends = if allow_vulkan { wgpu::util::backend_bits_from_env().unwrap_or_default() } else { wgpu::Backends::GL };
        let dx12_shader_compiler = wgpu::util::dx12_shader_compiler_from_env().unwrap_or_default();
        let gles_minor_version = wgpu::util::gles_minor_version_from_env().unwrap_or_default();

        // Create a wgpu instance
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: backends,
            flags: wgpu::InstanceFlags::from_build_config().with_env(),
            dx12_shader_compiler,
            gles_minor_version,
            ..Default::default()
        });

        // Create a wgpu surface
        let surface = instance.create_surface(window.clone()).expect("error creating window surface");

        let adapter = wgpu::util::initialize_adapter_from_env_or_default(&instance, Some(&surface))
                                    .await
                                    .expect("could not get adapter");
        let adapter_info = adapter.get_info();
        info!(target: "GUI", "Using adapter {} ({:?})", adapter_info.name, adapter_info.backend);

        // Create the wgpu device and device queue
        let (device, queue) = adapter.request_device(
            &wgpu::DeviceDescriptor {
                label   : None,
                required_features: wgpu::Features::empty(),
                required_limits  : wgpu::Limits::default(),
            },
            None,
        ).await.expect("Unable to initialize GPU device");

        // Configure the swap chain
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps.formats
                                .iter()
                                .copied()
                                .filter(|f| f.is_srgb())
                                .next()
                                .unwrap_or(surface_caps.formats[0]);
        info!(target: "GUI", "Surface format = {:?}", surface_format);
    
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: window_size.width as u32,
            height: window_size.height as u32,

            desired_maximum_frame_latency: 2,
            present_mode: wgpu::PresentMode::AutoNoVsync, //surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![surface_format],
        };

        Self {
            event_loop  : Some(event_loop),
            size        : window_size,
            window      : window,
            input_helper: WinitInputHelper::new(),
            event_log   : vec![],
            gilrs       : gilrs,
            connected_gamepads: [None; 4],
            change_logging_func: change_logging,

            wgpu        : WgpuInfo {
                //instance      : instance,
                surface       : surface,
                surface_config: config,
                device        : device,
                queue         : queue,

                needs_reconfigure: true,
            },
        }
    }

    fn resize_surface(&mut self, size: PhysicalSize<u32>) {
        self.size = LogicalSize::new(size.width, size.height);

        // reconfigure the swap chain
        self.wgpu.surface_config.width = size.width;
        self.wgpu.surface_config.height = size.height;
        //self.wgpu.surface.configure(&self.wgpu.device, &self.wgpu.surface_config);
        self.wgpu.needs_reconfigure = true;
    }

    fn device(&self) -> &wgpu::Device {
        &self.wgpu.device
    }

    fn input(&self) -> &WinitInputHelper {
        &self.input_helper
    }

    fn process_input(&mut self) {
        self.input_helper.step_with_window_events(&self.event_log);
        self.event_log.clear();
    }

    fn queue(&self) -> &wgpu::Queue {
        &self.wgpu.queue
    }

    fn surface(&self) -> &wgpu::Surface {
        &self.wgpu.surface
    }

    fn surface_config(&self) -> &wgpu::SurfaceConfiguration {
        &self.wgpu.surface_config
    }

    fn window(&self) -> &Window {
        &self.window
    }

    fn window_scale_factor(&self) -> f64 {
        self.window.scale_factor()
    }

    fn change_logging(&self, logging_str: &str, default_level: Level) {
        (self.change_logging_func)(logging_str, default_level);
    }

    fn gamepad_connected(&mut self, gamepad_id: gilrs::GamepadId) {
        for i in 0..self.connected_gamepads.len() {
            if self.connected_gamepads[i].is_none() {
                let gp = self.gilrs.gamepad(gamepad_id);
                println!("found device [{}] -- attached to game port {}", gp.name(), i+1);
                self.connected_gamepads[i] = Some(gamepad_id);
                return;
            }
        }
    }

    fn gamepad_disconnected(&mut self, gamepad_id: gilrs::GamepadId) {
        todo!("gamepad {:?} disconnected", gamepad_id);
    }

    fn gamepad_ispressed(&self, port: u8, button: gilrs::ev::Button) -> bool {
        match self.connected_gamepads[port as usize] {
            None => false,
            Some(gpid) => {
                let gp = self.gilrs.gamepad(gpid);
                if !gp.is_connected() { return false; }
                let res = gp.is_pressed(button);
                //println!("checking {button:?}, res = {}", res);
                res
            }
        }
    }

    fn gamepad_getaxis(&self, port: u8, axis: gilrs::ev::Axis) -> f32 {
        match self.connected_gamepads[port as usize] {
            None => 0.0,
            Some(gpid) => {
                let gp = self.gilrs.gamepad(gpid);
                if !gp.is_connected() { return 0.0 }
                if let Some(axis) = gp.axis_data(axis) {
                    axis.value()
                } else {
                    0.0
                }
            }
        }
    }

    fn run<F>(mut appwnd: AppWindow, mut user_update: F)
    where 
        F: FnMut(&mut AppWindow, &Event<()>) -> () + 'static
    {
        // take ownership of the event loop so that we can pass AppWindow around freely
        let event_loop = std::mem::replace(&mut appwnd.event_loop, None).unwrap();
        event_loop.set_control_flow(ControlFlow::Poll);
        let _ = event_loop.run(move |event, elwt| {
            // check if input helper has requested close
            if appwnd.input_helper.close_requested() || appwnd.input_helper.destroyed() {
                elwt.exit();
                return;
            }

            let repeat_filter = gilrs::ev::filter::Repeat::new();
            while let Some(ev) = appwnd.gilrs.next_event().filter_ev(&repeat_filter, &mut appwnd.gilrs) {
                appwnd.gilrs.update(&ev); // update controller states inside gilrs

                match ev.event {
                    gilrs::EventType::Connected => {
                        appwnd.gamepad_connected(ev.id);
                    },
                    gilrs::EventType::Disconnected => {
                        appwnd.gamepad_disconnected(ev.id);
                    },
                    _ => {},

                    //gilrs::EventType::ButtonPressed(btn, code) => {
                    //    println!("button pressed: btn={btn:?} code=${code:?}");
                    //},
                    //gilrs::EventType::ButtonReleased(btn, code) => {
                    //    println!("button release: btn={btn:?} code=${code:?}");
                    //},
                    //_ => {
                    //    println!("unhandled event {:?}", ev);
                    //}
                }
            }

            if let Event::WindowEvent { event, .. } = &event {
                appwnd.event_log.push(event.clone());
            }

            match event {
                Event::WindowEvent {
                    event: WindowEvent::Resized(size),
                    ..
                } => {
                    appwnd.resize_surface(size);
                },

                Event::AboutToWait => {
                    appwnd.window.request_redraw();
                },

                Event::WindowEvent {
                    event:
                        WindowEvent::RedrawRequested,
                        ..
                } => {
                    if appwnd.wgpu.needs_reconfigure {
                        appwnd.wgpu.surface.configure(&appwnd.wgpu.device, &appwnd.wgpu.surface_config);
                        appwnd.wgpu.needs_reconfigure = false;
                    }
                },

                // ignore other events
                _ => (),
            };

            user_update(&mut appwnd, &event);
        });
    }
}

pub trait App {
    /// Called to create the app
    /// Create your pipeline here
    fn create(appwnd: &AppWindow, comms: SystemCommunication, args: crate::Args) -> Self;

    /// Called once per frame to update the internal state of the app
    /// Check key presses and controllers that may affect render() here
    fn update(&mut self, appwnd: &AppWindow, delta_time: f32);

    /// Called once per frame to render the app
    fn render(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView);

    /// Called once per frame to render the UI
    /// Check key presses that may affect UI events such as shortcuts like Ctrl+Q
    fn render_ui(&mut self, appwnd: &AppWindow, ui: &imgui::Ui);
}

pub async fn run<T: App + 'static>(args: crate::Args,
                                   create_system: Box<dyn (FnOnce(SystemCommunication) -> System) + Send>, 
                                   change_logging: Box<dyn Fn(&str, Level) -> ()>,
                                   gilrs: gilrs::Gilrs) {

    let width = 320 * args.window_scale as u32;
    let height = 240 * args.window_scale as u32;

    let appwnd = AppWindow::new("Sarchar's N64 Emulator", width, height, !args.force_opengl, change_logging, gilrs).await;

    let mut imgui = imgui::Context::create();
    let mut platform = imgui_winit_support::WinitPlatform::init(&mut imgui);
    platform.attach_window(imgui.io_mut(), appwnd.window(), imgui_winit_support::HiDpiMode::Default);
    imgui.set_ini_filename(std::path::PathBuf::from("gui.ini"));

    // create the imgui font atlas and default font
    let scale_factor = appwnd.window_scale_factor();
    imgui.io_mut().font_global_scale = (1.0 / scale_factor) as f32;

    let font_size = (13.0 * scale_factor) as f32;
    imgui.fonts().add_font(&[FontSource::DefaultFontData {
        config: Some(imgui::FontConfig {
            oversample_h: 1,
            pixel_snap_h: true,
            size_pixels : font_size,
            ..Default::default()
        }),
    }]);

    // create the communication channels
    let hle_command_buffer = HleCommandBuffer::with_capacity(1024 * 16);
    let mut comms = SystemCommunication::new(Some(hle_command_buffer));

    // we need the mi_interrupts_tx channel out of the system, but the system is created in another thread
    let (tx, rx) = mpsc::channel();

    // start the emulation
    let thread_comms = comms.clone();
    std::thread::spawn(move || {
        let mut system = create_system(thread_comms);
        tx.send(system.rcp.borrow_mut().mi.get_update_channel()).unwrap();
        system.run();
    });

    // wait for the system to start and get the interrupt channel
    comms.mi_interrupts_tx = Some(rx.recv().unwrap());

    // start the frontend
    let mut app = T::create(&appwnd, comms.clone(), args.clone());

    // configure imgui to use wgpu 
    let renderer_config = RendererConfig {
        texture_format: appwnd.surface_config().format,
        ..Default::default()
    };

    let mut renderer = Renderer::new(&mut imgui, appwnd.device(), appwnd.queue(), renderer_config);
    let mut last_frame = Instant::now();
    AppWindow::run(appwnd, move |appwnd: &mut AppWindow, event| {
        match event {
            Event::WindowEvent {
                event:
                    WindowEvent::RedrawRequested,
                    ..
            } => {
                let now = Instant::now();
                imgui.io_mut().update_delta_time(now - last_frame);
                last_frame = now;

                let frame = match appwnd.surface().get_current_texture() {
                    Ok(frame) => frame,
                    Err(wgpu::SurfaceError::Timeout) => appwnd.surface().get_current_texture().expect("failed to acquire the next surface texture"),
                    e @ Err(
                        // Outdated and Lost means we need to reconfigure the surface
                        wgpu::SurfaceError::Outdated | wgpu::SurfaceError::Lost | wgpu::SurfaceError::OutOfMemory,
                    ) => {
                        error!(target: "GUI", "dropped frame: {e:?}");
                        appwnd.wgpu.surface.configure(&appwnd.wgpu.device, &appwnd.wgpu.surface_config);
                        return;
                    }
                };

                platform.prepare_frame(imgui.io_mut(), appwnd.window()).expect("failed to prepare frame");

                let ui = imgui.frame();
                let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());

                // must process input at some point
                appwnd.process_input();

                // render bg
                app.update(appwnd, ui.io().delta_time);
                app.render(appwnd, &view);

                // render ui
                app.render_ui(appwnd, &ui);

                // render imgui to wgpu
                {
                    let mut encoder: wgpu::CommandEncoder =
                        appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Main Encoder") });

                    platform.prepare_render(ui, appwnd.window());

                    {
                        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("imgui Render Pass"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load, // Do not clear
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: None,
                            occlusion_query_set: None,
                            timestamp_writes: None,
                        });

                        renderer
                            .render(imgui.render(), appwnd.queue(), appwnd.device(), &mut render_pass)
                            .expect("rendering failed");
                    }

                    appwnd.queue().submit(Some(encoder.finish()));
                }

                // finish frame
                frame.present();
            },

            _ => (),
        };

        // pass event onto imgui
        platform.handle_event(imgui.io_mut(), appwnd.window(), &event);
    });
}
