use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

//use winit::event::VirtualKeyCode;
use image::GenericImageView;
use wgpu::util::DeviceExt;
use cgmath::prelude::*;

use crate::*;
use gui::{App, AppWindow};

use n64::SystemCommunication;
use n64::hle::{HleRenderCommand, HleCommandBuffer};
use n64::mips::{InterruptUpdate, InterruptUpdateMode, IMask_DP};

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position  : [f32; 3],
    tex_coords: [f32; 2],
    color     : [f32; 4],
}

// Y texture coordinate is inverted to flip the resulting image
const GAME_TEXTURE_VERTICES: &[Vertex] = &[
    Vertex { position: [-1.0,  1.0, 0.0], tex_coords: [0.0, 0.0], color: [0.0, 0.0, 0.0, 0.0], }, // TL
    Vertex { position: [ 1.0,  1.0, 0.0], tex_coords: [1.0, 0.0], color: [0.0, 0.0, 0.0, 0.0], }, // TR
    Vertex { position: [-1.0, -1.0, 0.0], tex_coords: [0.0, 1.0], color: [0.0, 0.0, 0.0, 0.0], }, // BL
    Vertex { position: [ 1.0, -1.0, 0.0], tex_coords: [1.0, 1.0], color: [0.0, 0.0, 0.0, 0.0], }, // BR
];

const GAME_TEXTURE_INDICES: &[u16] = &[
    2, 1, 0,
    1, 3, 2,
];

const GAME_INDICES: &[u16] = &[
    0, 1, 2,
    0, 2, 3,
];

impl Vertex {
    fn new() -> Self {
        Vertex { position: [0.0, 0.0, 0.0], tex_coords: [0.0, 0.0], color: [0.0, 0.0, 0.0, 1.0], }
    }

    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute { // position
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute { // tex_coords
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute { // color
                    offset: std::mem::size_of::<[f32; 5]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ]
        }
    }

    fn offset_of(index: usize) -> wgpu::BufferAddress {
        (index * std::mem::size_of::<[f32; 9]>()) as wgpu::BufferAddress
    }
}

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.5,
    0.0, 0.0, 0.0, 1.0,
);

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct MvpPacked {
    mvp_matrix: [[f32; 4]; 4],
}

impl MvpPacked {
    fn new() -> Self {
        Self {
           mvp_matrix: cgmath::Matrix4::identity().into(),
        }
    }
}

pub struct Game {
    comms: SystemCommunication,
    hle_command_buffer: Arc<HleCommandBuffer>,

    game_render_textures: HashMap<u32, wgpu::Texture>,
    game_render_texture_bind_group_layout: wgpu::BindGroupLayout,
    game_render_texture_pipeline: wgpu::RenderPipeline,
    game_render_texture_vertex_buffer: wgpu::Buffer,
    game_render_texture_index_buffer: wgpu::Buffer,
    game_render_texture_bind_groups: HashMap<u32, wgpu::BindGroup>,
    current_render_texture_view: Option<wgpu::TextureView>,

    raw_render_texture: Option<wgpu::Texture>,
    raw_render_texture_bind_group: Option<wgpu::BindGroup>,

    game_pipeline: wgpu::RenderPipeline,

    game_viewport: HleRenderCommand,
    game_modelview: cgmath::Matrix4<f32>,
    game_projection: cgmath::Matrix4<f32>,

    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    diffuse_bind_group: wgpu::BindGroup,

    mvp_buffer: wgpu::Buffer,
    mvp_bind_group: wgpu::BindGroup,

    //speed: f32,
    //is_forward_pressed: bool,
    //is_backward_pressed: bool,
    //is_left_pressed: bool,
    //is_right_pressed: bool,

    ui_frame_count: u64,
    ui_last_fps_time: Instant,
    ui_fps: f64,

    game_frame_count: u64,
    game_last_fps_time: Instant,
    game_fps: f64,
}

impl App for Game {
    fn create(appwnd: &AppWindow, mut comms: SystemCommunication) -> Self {
        let device: &wgpu::Device = appwnd.device();

        // create the main texture render shader
        let game_render_texture_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Render Texture Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("gametexture.wgsl").into()),
        });

        // create the texture bind group for the game texture
        let game_render_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Game Render Texture Bind Group"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        let game_render_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Render Texture Pipeline Layout"),
            bind_group_layouts: &[&game_render_texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let game_render_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Render Texture Pipeline"),
            layout: Some(&game_render_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &game_render_texture_shader,
                entry_point: "vs_main",
                buffers: &[
                    Vertex::desc(),
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &game_render_texture_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: appwnd.surface_config().format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        let game_render_texture_vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Render Texture Vertex Buffer"),
                contents: bytemuck::cast_slice(GAME_TEXTURE_VERTICES),
                usage: wgpu::BufferUsages::VERTEX,
            }
        );

        let game_render_texture_index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Render Texture Index Buffer"),
                contents: bytemuck::cast_slice(GAME_TEXTURE_INDICES),
                usage: wgpu::BufferUsages::INDEX,
            }
        );

        let diffuse_bytes = include_bytes!("happy-tree.png");
        let diffuse_image = image::load_from_memory(diffuse_bytes).unwrap();
        let diffuse_rgba  = diffuse_image.to_rgba8();
        let diffuse_dim   = diffuse_image.dimensions();

        let texture_size = wgpu::Extent3d {
            width: diffuse_dim.0,
            height: diffuse_dim.1,
            depth_or_array_layers: 1,
        };

        let diffuse_texture = device.create_texture(
            &wgpu::TextureDescriptor {
                size: texture_size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                label: Some("Game Diffuse Texture"),
                view_formats: &[],
            }
        );

        appwnd.queue().write_texture(
            wgpu::ImageCopyTexture {
                texture: &diffuse_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &diffuse_rgba,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * diffuse_dim.0),
                rows_per_image: Some(diffuse_dim.1),
            },
            texture_size,
        );

        let texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Game Texture Bind Group"),
            entries: &[
                wgpu::BindGroupLayoutEntry { // TextureView
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry { // Sampler
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        let diffuse_texture_view = diffuse_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let diffuse_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let diffuse_bind_group = device.create_bind_group( &wgpu::BindGroupDescriptor {
            label: Some("Game Diffuse Bind Group"),
            layout: &texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&diffuse_texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&diffuse_sampler),
                },
            ],
        });

        let game_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("game.wgsl").into()),
        });

        let mvp_data = MvpPacked::new();
        let mvp_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game MVP Matrix Buffer"),
                contents: bytemuck::cast_slice(&[mvp_data]),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST
            }
        );

        let mvp_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Game MVP Matrix Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry { // Uniform buffer (mvp_matrix)
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    }
                ],
            }
        );

        let mvp_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Game MVP Matrix Bind Group"),
            layout: &mvp_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: mvp_buffer.as_entire_binding(),
                }
            ],
        });

        let game_render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Pipeline Layout"),
            bind_group_layouts: &[
                &texture_bind_group_layout,
                &mvp_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        let game_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Pipeline"),
            layout: Some(&game_render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &game_shader,
                entry_point: "vs_main",
                buffers: &[
                    Vertex::desc(),
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &game_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: appwnd.surface_config().format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None, //Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        // reserve space for 32 vertices
        let vertices = &[Vertex::new(); 32];
        let vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Vertex Buffer"),
                contents: bytemuck::cast_slice(vertices),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            }
        );

        let index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Index Buffer"),
                contents: bytemuck::cast_slice(GAME_INDICES),
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
            }
        );

        let hle_command_buffer = std::mem::replace(&mut comms.hle_command_buffer, None).unwrap();
        Self {
            comms: comms,
            hle_command_buffer: hle_command_buffer,

            game_render_textures: HashMap::new(),
            game_render_texture_bind_group_layout: game_render_texture_bind_group_layout,
            game_render_texture_pipeline: game_render_texture_pipeline,
            game_render_texture_vertex_buffer: game_render_texture_vertex_buffer,
            game_render_texture_index_buffer: game_render_texture_index_buffer,
            game_render_texture_bind_groups: HashMap::new(),
            current_render_texture_view: None,
            raw_render_texture: None,
            raw_render_texture_bind_group: None,

            game_pipeline: game_pipeline,

            game_viewport: HleRenderCommand::Noop,
            game_modelview: cgmath::Matrix4::identity(),
            game_projection: cgmath::Matrix4::identity(),

            vertex_buffer: vertex_buffer,
            index_buffer: index_buffer,
            diffuse_bind_group: diffuse_bind_group,

            mvp_buffer: mvp_buffer,
            mvp_bind_group: mvp_bind_group,

            //speed: 0.2,
            //is_forward_pressed: false,
            //is_backward_pressed: false,
            //is_left_pressed: false,
            //is_right_pressed: false,

            ui_frame_count: 0,
            ui_last_fps_time: Instant::now(),
            ui_fps: 0.0,
            game_frame_count: 0,
            game_last_fps_time: Instant::now(),
            game_fps: 0.0,
        }
    }

    fn update(&mut self, _appwnd: &AppWindow, _delta_time: f32) {
        self.ui_frame_count += 1;
        if (self.ui_frame_count % 10) == 0 {
            self.ui_fps = 10.0 / self.ui_last_fps_time.elapsed().as_secs_f64();
            self.ui_last_fps_time = Instant::now();
        }

        //let input = appwnd.input();
        //self.is_forward_pressed  = input.key_held(VirtualKeyCode::W) || input.key_held(VirtualKeyCode::Up);
        //self.is_backward_pressed = input.key_held(VirtualKeyCode::S) || input.key_held(VirtualKeyCode::Down);
        //self.is_left_pressed     = input.key_held(VirtualKeyCode::A) || input.key_held(VirtualKeyCode::Left);
        //self.is_right_pressed    = input.key_held(VirtualKeyCode::D) || input.key_held(VirtualKeyCode::Right);

        //let forward = self.camera.target - self.camera.eye;
        //let forward_norm = forward.normalize();
        //let forward_mag = forward.magnitude();

        //if self.is_forward_pressed && forward_mag > self.speed {
        //    self.camera.eye += forward_norm * self.speed;
        //}
        //if self.is_backward_pressed {
        //    self.camera.eye -= forward_norm * self.speed;
        //}

        //let right = forward_norm.cross(self.camera.up);
        //let forward = self.camera.target - self.camera.eye;
        //let forward_mag = forward.magnitude();

        //if self.is_right_pressed {
        //    self.camera.eye = self.camera.target - (forward + right * self.speed).normalize() * forward_mag;
        //}
        //if self.is_left_pressed {
        //    self.camera.eye = self.camera.target - (forward - right * self.speed).normalize() * forward_mag;
        //}

        //self.camera_uniform.update_view_proj(&self.camera);
        //appwnd.queue().write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[self.camera_uniform]));

        //for instance in self.instances.iter_mut() {
        //    instance.rotation = instance.rotation * 
        //        cgmath::Quaternion::from_axis_angle(cgmath::Vector3::unit_y(), cgmath::Deg(360.0 / 2.0 * delta_time));
        //}

        //let instance_data = self.instances.iter().map(Instance::to_raw).collect::<Vec<_>>();
        //appwnd.queue().write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(&instance_data));

    }

    fn render(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView) {
        self.render_game(appwnd);

        // TODO we need the VI_ORIGIN value to know what to render..
        let video_buffer = self.comms.vi_origin.load(Ordering::SeqCst);
        if video_buffer == 0 { return; }

        // look for the texture associated with the color image address
        let bind_group = if let Some(bind_group) = self.game_render_texture_bind_groups.get(&video_buffer) {
            bind_group
        } else {
            if let Some(bind_group) = self.game_render_texture_bind_groups.get(&(video_buffer - 640)) { // video_buffer is + 640 on NTSC?
                bind_group
            } else {
                let width = self.comms.vi_width.load(Ordering::SeqCst) as usize;
                let height = if width == 320 { 240 } else if width == 640 { 480 } else { return; } as usize;
                let format = self.comms.vi_format.load(Ordering::SeqCst);

                // no game render texture found, if video_buffer is valid, render directly from RDRAM if possible
                if let None = self.raw_render_texture {
                    let (texture, bind_group) = self.create_framebuffer_texture(appwnd, format!("${:08X}", video_buffer).as_str(), width as u32, height as u32, true, false);
                    self.raw_render_texture = Some(texture);
                    self.raw_render_texture_bind_group = Some(bind_group);
                }

                // access RDRAM directly
                // would be nice if I could copy RGB555 into a texture, but this copy seems acceptable for now
                if let Some(rdram) = self.comms.rdram.read().as_deref().unwrap() { // rdram = &[u32]
                    let start = (video_buffer >> 2) as usize;
                    let mut image_data = vec![0u8; width*height*4];
                    for i in 0..(width*height) {
                        match format {
                            2 => {
                                let shift = 16 - ((i & 1) << 4);
                                let pix = (rdram[start + (i >> 1)] >> shift) as u16;
                                let r = ((pix >> 11) & 0x1F) as u8;
                                let g = ((pix >>  6) & 0x1F) as u8;
                                let b = ((pix >>  1) & 0x1F) as u8;
                                let a = (pix & 0x01) as u8;
                                image_data[i*4..][..4].copy_from_slice(&[r << 3, g << 3, b << 3, if a == 1 { 0 } else { 255 }]);
                            },
                            3 => { 
                                let pix = rdram[start+i] | 0xff;
                                image_data[i*4..][..4].copy_from_slice(&pix.to_be_bytes());
                            },
                            _ => break,
                        }
                    }

                    appwnd.queue().write_texture(
                        wgpu::ImageCopyTexture {
                            texture: self.raw_render_texture.as_ref().unwrap(),
                            mip_level: 0,
                            origin: wgpu::Origin3d::ZERO,
                            aspect: wgpu::TextureAspect::All,
                        },
                        bytemuck::cast_slice(&image_data),
                        wgpu::ImageDataLayout {
                            offset: 0,
                            bytes_per_row: Some(1 * 4 * width as u32), // 320 pix, rgba*f32,
                            rows_per_image: Some(height as u32),
                        },
                        wgpu::Extent3d {
                            width: width as u32,
                            height: height as u32,
                            depth_or_array_layers: 1,
                        },
                    );
                }

                self.raw_render_texture_bind_group.as_ref().unwrap()
            }
        };

        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Game Render Texture Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Game Render Texture Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        // the old color doesn't matter, so LoadOp::Load is more efficient
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 }),
                        store: true, //. wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                //.occlusion_query_set: None,
                //.timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.game_render_texture_pipeline);
            render_pass.set_bind_group(0, bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.game_render_texture_vertex_buffer.slice(..));
            render_pass.set_index_buffer(self.game_render_texture_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..GAME_TEXTURE_INDICES.len() as _, 0, 0..1);
        }
        appwnd.queue().submit(Some(encoder.finish()));
    }

    fn render_ui(&mut self, _appwnd: &AppWindow, ui: &imgui::Ui) {
        let window = ui.window("Stats");
        window.size([300.0, 100.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::Once)
              .build(|| {
                  ui.text(format!("UI   FPS: {}", self.ui_fps));
                  ui.text(format!("GAME FPS: {}", self.game_fps));
              });
    }
}

impl Game {
    fn create_framebuffer_texture(&mut self, appwnd: &AppWindow, name: &str, width: u32, height: u32, is_copy_dst: bool, is_filtered: bool) -> (wgpu::Texture, wgpu::BindGroup) {
        let device = appwnd.device();

        // create an offscreen render target for the actual game render
        // we double buffer so we don't get flickering when the n64/hle code is drawing too slowly
        // TODO need to resize texture with the window resize
        // OR maybe the render texture should be mapped to the n64 viewport?
        let texture = device.create_texture(
            &wgpu::TextureDescriptor {
                label: Some(format!("Game Render Texture: {name}").as_str()),
                size: wgpu::Extent3d {
                    width: width,
                    height: height,
                    ..Default::default()
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: appwnd.surface_config().format,
                // TODO at some point probably need COPY_SRC to copy the framebuffer into RDRAM
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING 
                    | if is_copy_dst { wgpu::TextureUsages::COPY_DST } else { wgpu::TextureUsages::empty() },
                view_formats: &[],
            }
        );

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter    : if is_filtered { wgpu::FilterMode::Linear } else { wgpu::FilterMode::Nearest },
            min_filter    : wgpu::FilterMode::Nearest,
            mipmap_filter : wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let bind_group = device.create_bind_group( &wgpu::BindGroupDescriptor {
            label: Some(format!("Game Render Texture Bind Group: {name}").as_str()),
            layout: &self.game_render_texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler),
                },
            ],
        });

        (texture, bind_group)
    }

    fn render_game(&mut self, appwnd: &AppWindow) {
        'cmd_loop: while let Some(cmd) = self.hle_command_buffer.try_pop() {
            match cmd {
                HleRenderCommand::SetColorImage {
                    framebuffer_address: addr,
                    ..
                } => {
                    if !self.game_render_textures.contains_key(&addr) {
                        let width = appwnd.surface_config().width;
                        let height = appwnd.surface_config().height;
                        let (texture, bind_group) = self.create_framebuffer_texture(appwnd, format!("${:08X}", addr).as_str(), width, height, false, false);
                        self.game_render_textures.insert(addr, texture);
                        self.game_render_texture_bind_groups.insert(addr, bind_group);
                        info!(target: "RENDER", "created color render target for address ${:08X} (width={})", addr, width);
                    }

                    let texture: &wgpu::Texture = self.game_render_textures.get(&addr).unwrap();
                    self.current_render_texture_view = Some(texture.create_view(&wgpu::TextureViewDescriptor::default()));
                },

                HleRenderCommand::Viewport { .. } => {
                    //println!("Viewport: {:?}", cmd);
                    self.game_viewport = cmd;
                    //render_pass.set_viewport(x, y, w, h, 0.0, 1.0);
                    //render_pass.set_viewport(0.0, 0.0, 1024.0, 768.0, 0.0, 1.0);
                },

                HleRenderCommand::SetModelViewMatrix(m) => {
                    //println!("got view: {:?}", m);
                    // On N64, the Z component is fixed point with 10 bits of precision, so that
                    // is scaled first to 1..-1 and then scaled to wgpu's 0..1
                    self.game_modelview = OPENGL_TO_WGPU_MATRIX * cgmath::Matrix4::from_nonuniform_scale(1.0, 1.0, 1.0 / 1023.0) * m.transpose();
                },

                HleRenderCommand::SetProjectionMatrix(m) => {
                    //println!("got proj: {:?}", m);

                    self.game_projection = m;
                    //self.game_projection = cgmath::Matrix4::from_nonuniform_scale(1.0, 1.0, 1.0/1023.0).transpose() * m;
                },

                HleRenderCommand::FillRectangle { x: rx, y: ry, w: rw, h: rh, c: rc } => {
                    if let None = self.current_render_texture_view { continue; }
                    let view = self.current_render_texture_view.as_ref().unwrap();

                    if let HleRenderCommand::Viewport { x: vx, y: vy, w: vw, h: vh } = self.game_viewport {
                        //let mvp_matrix = cgmath::ortho(0.0, vx, 0.0, vy, 0.1, 100.0);
                        let mvp_matrix = cgmath::ortho(-1.0, 1.0, -1.0, 1.0, 0.1, 100.0);
                        let mvp_packed = MvpPacked { mvp_matrix: mvp_matrix.transpose().into() };
                        appwnd.queue().write_buffer(&self.mvp_buffer, 0, bytemuck::cast_slice(&[mvp_packed]));

                        //println!("render rect: {rx},{ry},{rw},{rh} into vp {vx},{vy},{vw},{vh}, color {rc:?}");
                        let is_fullview = rx == vx && ry == vy && rw == vw && rh == vh;

                        // upload the rect vertices and color
                        // map (0,vx) -> (-1,1)
                        // so (rx/vx * 2) - 1
                        let scale = |s, maxs| ((s / maxs) * 2.0) - 1.0;
                        let vertices = &[
                            Vertex { position: [ scale(rx   , vw), scale(ry+rh, vh), 0.0], tex_coords: [0.0, 0.0], color: rc, }, // TL
                            Vertex { position: [ scale(rx+rw, vw), scale(ry+rh, vh), 0.0], tex_coords: [0.0, 0.0], color: rc, }, // TR
                            Vertex { position: [ scale(rx+rw, vw), scale(ry   , vh), 0.0], tex_coords: [0.0, 0.0], color: rc, }, // BR
                            Vertex { position: [ scale(rx   , vw), scale(ry   , vh), 0.0], tex_coords: [0.0, 0.0], color: rc, }, // BL
                        ];
                        appwnd.queue().write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(vertices));

                        // restore indices changed by other draw calls
                        let indices: &[u16] = &[0, 1, 2, 0, 2, 3];
                        appwnd.queue().write_buffer(&self.index_buffer, 0, bytemuck::cast_slice(indices));

                        let mut encoder: wgpu::CommandEncoder =
                            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Fill Rect Encoder") });
                        {
                            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                                label: Some("Game Render Pass"),
                                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                    view: view,
                                    resolve_target: None,
                                    ops: wgpu::Operations {
                                        load: if is_fullview {
                                            wgpu::LoadOp::Clear(wgpu::Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 })
                                        } else {
                                            wgpu::LoadOp::Load
                                        },
                                        store: true, //. wgpu::StoreOp::Store,
                                    },
                                })],
                                depth_stencil_attachment: None,
                                //.occlusion_query_set: None,
                                //.timestamp_writes: None,
                            });

                            render_pass.set_pipeline(&self.game_pipeline);
                            render_pass.set_bind_group(0, &self.diffuse_bind_group, &[]);
                            render_pass.set_bind_group(1, &self.mvp_bind_group, &[]);
                            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
                            render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
                            render_pass.draw_indexed(0..6 as _, 0, 0..1);
                        }
                        appwnd.queue().submit(Some(encoder.finish()));
                    }
                },

                HleRenderCommand::VertexData(v, start_index) => {
                    let mut vcopy = Vec::new();
                    for vdata in v.iter() {
                        let vnew = Vertex {
                            position: [
                                vdata.position[0] as f32, 
                                vdata.position[1] as f32, 
                                vdata.position[2] as f32
                            ],
                            tex_coords: [0.0, 0.0],
                            color: [
                                vdata.color_or_normal[0] as f32 / 255.0,
                                vdata.color_or_normal[1] as f32 / 255.0,
                                vdata.color_or_normal[2] as f32 / 255.0,
                                vdata.color_or_normal[3] as f32 / 255.0
                            ]
                        };
                        vcopy.push(vnew);
                    }

                    let vertices = &vcopy;
                    appwnd.queue().write_buffer(&self.vertex_buffer, Vertex::offset_of(start_index), bytemuck::cast_slice(vertices));
                },

                HleRenderCommand::DrawTriangle(v0, v1, v2) => {
                    if let None = self.current_render_texture_view { continue; }
                    let view = self.current_render_texture_view.as_ref().unwrap();

                    // first improvement would be to buffer this draw call until more triangle calls come in and render them all at once
                    // maybe it's naive of me to think that a new render pass for every triangle is inefficient?
                    let mvp_matrix = self.game_projection * self.game_modelview;
                    let mvp_packed = MvpPacked { mvp_matrix: mvp_matrix.into() };
                    appwnd.queue().write_buffer(&self.mvp_buffer, 0, bytemuck::cast_slice(&[mvp_packed]));

                    // upload indices to be rendered
                    let indices: &[u16] = &[v0, v1, v2, 0, 0, 0];
                    appwnd.queue().write_buffer(&self.index_buffer, 0, bytemuck::cast_slice(indices));

                    let mut encoder: wgpu::CommandEncoder =
                        appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Fill Rect Encoder") });
                    {
                        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Game Render Triangle Pass"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: wgpu::LoadOp::Load,
                                    store: true,
                                },
                            })],
                            depth_stencil_attachment: None,
                            //.occlusion_query_set: None,
                            //.timestamp_writes: None,
                        });

                        render_pass.set_pipeline(&self.game_pipeline);
                        //render_pass.set_viewport(0.0, 0.0, 1024.0, 768.0, 0.0, 1.0);
                        render_pass.set_bind_group(0, &self.diffuse_bind_group, &[]);
                        render_pass.set_bind_group(1, &self.mvp_bind_group, &[]);
                        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
                        render_pass.draw_indexed(0..3 as _, 0, 0..1);
                    }
                    appwnd.queue().submit(Some(encoder.finish()));
                },

                HleRenderCommand::Sync => {
                    self.game_frame_count += 1;
                    if (self.game_frame_count % 10) == 0 {
                        self.game_fps = 10.0 / self.game_last_fps_time.elapsed().as_secs_f64();
                        self.game_last_fps_time = Instant::now();
                    }

                    self.reset_render_state();

                    // trigger RDP interrupt to signal render is done
                    if let Some(mi) = &self.comms.mi_interrupts_tx {
                        mi.send(InterruptUpdate(IMask_DP, InterruptUpdateMode::SetInterrupt)).unwrap();
                    }
                    
                    break 'cmd_loop;
                },
    
                z => unimplemented!("unhandled HLE render comand {:?}", z),
            };
        }
    }

    fn reset_render_state(&mut self) {
        self.game_viewport   = HleRenderCommand::Noop;
        self.game_modelview  = cgmath::Matrix4::identity();
        self.game_projection = cgmath::Matrix4::identity();
    }
}

