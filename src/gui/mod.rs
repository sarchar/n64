use std::sync::Arc;
use std::time::Instant;

use winit::{
    dpi::{LogicalSize, PhysicalSize},
    event::{Event, WindowEvent, VirtualKeyCode}, // VirtualKeyCode
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use winit_input_helper::WinitInputHelper;

use imgui::*;
use imgui_wgpu::{Renderer, RendererConfig};

use crate::*;
use n64::hle::HleCommandBuffer;

pub mod game;

struct WgpuInfo {
    //instance      : wgpu::Instance,
    surface       : wgpu::Surface,
    surface_config: wgpu::SurfaceConfiguration,
    device        : wgpu::Device,
    queue         : wgpu::Queue,

    needs_reconfigure: bool,
}

pub struct AppWindow {
    event_loop  : Option<EventLoop<()>>,
    size        : LogicalSize<u32>,
    window      : Window,
    input_helper: WinitInputHelper,
    wgpu        : WgpuInfo,
}

impl AppWindow {
    async fn new(title: &str, width: u32, height: u32, allow_vulkan: bool) -> Self {
        let event_loop = EventLoop::new();

        // Create the platform window with winit
        let window_size = LogicalSize::new(width, height);
        let window = {
            WindowBuilder::new()
                .with_title(title)
                .with_inner_size(window_size)
                .with_min_inner_size(LogicalSize::new(50, 50))
                .with_resizable(allow_vulkan) // my current environment crashes when using the Gl
                                              // backend, so I'm disabling window resize for now
                .build(&event_loop)
                .expect("Error creating window")
        };

        // Create a wgpu instance
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Create a wgpu surface
        let surface = unsafe { instance.create_surface(&window) }.expect("error creating window surface");

        for adapter in instance.enumerate_adapters(wgpu::Backends::all()).into_iter() {
            debug!(target: "GUI", "detected {:?}", adapter.get_info());
        }

        // Find an adapter to use
        let adapter = instance
            .enumerate_adapters(wgpu::Backends::all())
            .filter(|adapter| {
                adapter.is_surface_supported(&surface) && 
                    ((allow_vulkan && adapter.get_info().backend == wgpu::Backend::Vulkan)
                     || (!allow_vulkan && adapter.get_info().backend == wgpu::Backend::Gl))
            }).next().expect("could not find a working wgpu adapter");

        // Create the wgpu device and device queue
        let (device, queue) = adapter.request_device(
            &wgpu::DeviceDescriptor {
                features: wgpu::Features::empty(),
                limits  : wgpu::Limits::default(),
                label   : None,
            },
            None,
        ).await.unwrap();

        // Configure the swap chain
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps.formats.iter().copied().filter(|f| f.is_srgb()).next().unwrap_or(surface_caps.formats[0]);
    
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: window_size.width as u32,
            height: window_size.height as u32,
            present_mode: surface_caps.present_modes[0],
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
        };

        Self {
            event_loop  : Some(event_loop),
            size        : window_size,
            window      : window,
            input_helper: WinitInputHelper::new(),

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

    fn run<F>(mut appwnd: AppWindow, mut user_update: F)
    where 
        F: FnMut(&mut AppWindow, Event<()>) -> () + 'static
    {
        // take ownership of the event loop so that we can pass AppWindow around freely
        let event_loop = std::mem::replace(&mut appwnd.event_loop, None).unwrap();
        event_loop.run(move |event, _, control_flow| {
            *control_flow = ControlFlow::Poll; // continously update

            // check for input events
            if appwnd.input_helper.update(&event) {
                if appwnd.input_helper.quit() {
                    *control_flow = ControlFlow::Exit;
                    return;
                }
            }

            match event {
                Event::WindowEvent {
                    event: WindowEvent::Resized(size),
                    ..
                } => {
                    appwnd.resize_surface(size);
                },

                Event::MainEventsCleared => {
                    appwnd.window.request_redraw();
                },

                Event::RedrawEventsCleared => {
                    if appwnd.wgpu.needs_reconfigure {
                        appwnd.wgpu.surface.configure(&appwnd.wgpu.device, &appwnd.wgpu.surface_config);
                        appwnd.wgpu.needs_reconfigure = false;
                    }
                },

                // ignore other events
                _ => (),
            };

            user_update(&mut appwnd, event);
        });
    }
}

pub trait App {
    /// Called to create the app
    /// Create your pipeline here
    fn create(appwnd: &AppWindow, hle_command_buffer: Arc<atomicring::AtomicRingBuffer<n64::hle::HleRenderCommand>>) -> Self;

    /// Called once per frame to update the internal state of the app
    /// Check key presses and controllers that may affect render() here
    fn update(&mut self, appwnd: &AppWindow, delta_time: f32);

    /// Called once per frame to render the app
    fn render(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView);

    /// Called once per frame to render the UI
    /// Check key presses that may affect UI events such as shortcuts like Ctrl+Q
    fn render_ui(&mut self, appwnd: &AppWindow, ui: &imgui::Ui);
}

pub async fn run<T: App + 'static>(create_system: Box<dyn (FnOnce(Option<Arc<HleCommandBuffer>>) -> System) + Send>) {
    let appwnd = AppWindow::new("Sarchar's N64 Emulator", 1024, 768, true).await;

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

    // configure imgui to use wgpu 
    let renderer_config = RendererConfig {
        texture_format: appwnd.surface_config().format,
        ..Default::default()
    };

    let mut renderer = Renderer::new(&mut imgui, appwnd.device(), appwnd.queue(), renderer_config);

    let hle_command_buffer = Arc::new(atomicring::AtomicRingBuffer::with_capacity(4096));
    let mut app = T::create(&appwnd, hle_command_buffer.clone());

    let mut last_frame = Instant::now();
    let mut demo_open = false;

    std::thread::spawn(move || {
        let mut system = create_system(Some(hle_command_buffer));
        system.run();
    });

    AppWindow::run(appwnd, move |appwnd: &mut AppWindow, event| {
        if appwnd.input().key_pressed(VirtualKeyCode::F12) {
            demo_open = true;
        }

        match event {
            Event::RedrawEventsCleared => {
                let now = Instant::now();
                imgui.io_mut().update_delta_time(now - last_frame);
                last_frame = now;

                let frame = match appwnd.surface().get_current_texture() {
                    Ok(frame) => frame,
                    Err(e) => {
                        error!(target: "GUI", "dropped frame: {e:?}");
                        return;
                    }
                };

                platform.prepare_frame(imgui.io_mut(), appwnd.window()).expect("failed to prepare frame");

                let ui = imgui.frame();
                let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());

                // render bg
                app.update(appwnd, ui.io().delta_time);
                app.render(appwnd, &view);

                // render ui
                app.render_ui(appwnd, &ui);
                if demo_open {
                    ui.show_demo_window(&mut demo_open);
                }

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
                                    store: true, //. wgpu::StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: None,
                            //.occlusion_query_set: None,
                            //.timestamp_writes: None,
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
