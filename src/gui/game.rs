use std::sync::Arc;
use std::time::Instant;

use winit::event::VirtualKeyCode;
use image::GenericImageView;
use wgpu::util::DeviceExt;
use cgmath::prelude::*;

use crate::*;
use gui::{App, AppWindow};

use n64::hle::{HleRenderCommand, HleCommandBuffer};

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position  : [f32; 3],
    tex_coords: [f32; 2],
}

// Y texture coordinate is inverted to flip the resulting image
const GAME_TEXTURE_VERTICES: &[Vertex] = &[
    Vertex { position: [-1.0,  1.0, 0.0], tex_coords: [0.0, 0.0], }, // TL
    Vertex { position: [ 1.0,  1.0, 0.0], tex_coords: [1.0, 0.0], }, // TR
    Vertex { position: [-1.0, -1.0, 0.0], tex_coords: [0.0, 1.0], }, // BL
    Vertex { position: [ 1.0, -1.0, 0.0], tex_coords: [1.0, 1.0], }, // BR
];

const GAME_TEXTURE_INDICES: &[u16] = &[
    2, 1, 0,
    1, 3, 2,
];

const GAME_VERTICES: &[Vertex] = &[
    //Vertex { position: [-100.0868241, 100.49240386, 0.0], tex_coords: [0.4131759, 0.99240386], }, // A
    //Vertex { position: [-100.49513406, 100.06958647, 0.0], tex_coords: [0.0048659444, 0.56958647], }, // B
    //Vertex { position: [-100.21918549, -100.44939706, 0.0], tex_coords: [0.28081453, 0.05060294], }, // C
    //Vertex { position: [100.35966998, -100.3473291, 0.0], tex_coords: [0.85967, 0.1526709], }, // D
    //Vertex { position: [100.44147372, 100.2347359, 0.0], tex_coords: [0.9414737, 0.7347359], }, // E
//    Vertex { position: [-0.5, -0.5,  0.0], tex_coords: [0.0, 0.0], },
//    Vertex { position: [ 0.5, -0.5,  0.0], tex_coords: [0.0, 1.0], },
//    Vertex { position: [ 0.0,  0.5,  0.0], tex_coords: [1.0, 1.0], },
//    Vertex { position: [-1.0,  1.0, -2.0], tex_coords: [0.0, 0.0], },
//    Vertex { position: [-0.8,  1.0, -2.0], tex_coords: [0.0, 1.0], },
//    Vertex { position: [-0.9,  0.6, -2.0], tex_coords: [1.0, 1.0], },
    Vertex { position: [-64.0,  64.0, -5.0], tex_coords: [0.0, 1.0], },
    Vertex { position: [ 64.0,  64.0, -5.0], tex_coords: [1.0, 0.0], },
    Vertex { position: [ 64.0, -64.0, -5.0], tex_coords: [1.0, 1.0], },
    Vertex { position: [-64.0, -64.0, -5.0], tex_coords: [0.0, 1.0], },
];

const GAME_INDICES: &[u16] = &[
    0, 1, 2,
    //3, 4, 5
    0, 2, 3,
];

impl Vertex {
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
            ]
        }
    }
}

const NUM_INSTANCES_PER_ROW: u32 = 1;
const INSTANCE_DISPLACEMENT: cgmath::Vector3<f32> = cgmath::Vector3::new(NUM_INSTANCES_PER_ROW as f32 * 0.5, 0.0, NUM_INSTANCES_PER_ROW as f32 * 0.5);

struct Instance {
    position: cgmath::Vector3<f32>,
    rotation: cgmath::Quaternion<f32>,
}

#[repr(C)]
#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct InstanceRaw {
    model: [[f32; 4]; 4],
}

impl Instance {
    fn to_raw(&self) -> InstanceRaw {
        InstanceRaw {
            model: (cgmath::Matrix4::from_translation(self.position) * cgmath::Matrix4::from(self.rotation)).into(),
        }
    }
}

impl InstanceRaw {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        use std::mem;
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<InstanceRaw>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                wgpu::VertexAttribute { // model matrix row 0
                    offset: 0,
                    shader_location: 5,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // model matrix row 1
                    offset: mem::size_of::<[f32; 4]>() as wgpu::BufferAddress,
                    shader_location: 6,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // model matrix row 2
                    offset: mem::size_of::<[f32; 8]>() as wgpu::BufferAddress,
                    shader_location: 7,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // model matrix row 3
                    offset: mem::size_of::<[f32; 12]>() as wgpu::BufferAddress,
                    shader_location: 8,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
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
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
    fn new() -> Self {
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
        }
    }
}

pub struct Game {
    hle_command_buffer: Arc<HleCommandBuffer>,

    game_render_texture: wgpu::Texture,
    game_render_texture_pipeline: wgpu::RenderPipeline,
    game_render_texture_vertex_buffer: wgpu::Buffer,
    game_render_texture_index_buffer: wgpu::Buffer,
    game_render_texture_bind_group: wgpu::BindGroup,

    game_pipeline: wgpu::RenderPipeline,

    game_viewport: HleRenderCommand,
    game_modelview: cgmath::Matrix4<f32>,
    game_projection: cgmath::Matrix4<f32>,

    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    diffuse_bind_group: wgpu::BindGroup,

    instances: Vec<Instance>,
    instance_buffer: wgpu::Buffer,

    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,

    speed: f32,
    is_forward_pressed: bool,
    is_backward_pressed: bool,
    is_left_pressed: bool,
    is_right_pressed: bool,

    do_render: bool,

    ui_frame_count: u64,
    ui_last_fps_time: Instant,
    ui_fps: f64,

    game_frame_count: u64,
    game_last_fps_time: Instant,
    game_fps: f64,
}

impl App for Game {
    fn create(appwnd: &AppWindow, hle_command_buffer: Arc<HleCommandBuffer>) -> Self {
        let device: &wgpu::Device = appwnd.device();

        // create an offscreen render target for the actual game render
        // TODO need to resize texture with the window resize
        let game_render_texture = device.create_texture(
            &wgpu::TextureDescriptor {
                label: Some("Game Render Texture"),
                size: wgpu::Extent3d {
                    width: appwnd.surface_config().width,
                    height: appwnd.surface_config().height,
                    ..Default::default()
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: appwnd.surface_config().format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            }
        );

        // create the main texture render
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

        let game_render_texture_view = game_render_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let game_render_texture_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter    : wgpu::FilterMode::Linear,
            min_filter    : wgpu::FilterMode::Nearest,
            mipmap_filter : wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let game_render_texture_bind_group = device.create_bind_group( &wgpu::BindGroupDescriptor {
            label: Some("Game Render Texture Bind Group"),
            layout: &game_render_texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&game_render_texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&game_render_texture_sampler),
                },
            ],
        });

        let instances = (0..NUM_INSTANCES_PER_ROW).flat_map(|z| {
            (0..NUM_INSTANCES_PER_ROW).map(move |x| {
                let position = cgmath::Vector3 { x: x as f32, y: 0.0, z: z as f32 } - INSTANCE_DISPLACEMENT;
                let rotation = cgmath::Quaternion::from_axis_angle(position.normalize(), cgmath::Deg(0.0));

                Instance {
                    position, rotation
                }
            })
        }).collect::<Vec<_>>();

        let instance_data = instances.iter().map(Instance::to_raw).collect::<Vec<_>>();
        let instance_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Instance Buffer"),
                contents: bytemuck::cast_slice(&instance_data),
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
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

        let camera_uniform = CameraUniform::new();
        let camera_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Camera Buffer"),
                contents: bytemuck::cast_slice(&[camera_uniform]),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST
            }
        );

        let camera_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Game Camera Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry { // Uniform buffer (view_proj)
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

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Game Camera Bind Group"),
            layout: &camera_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: camera_buffer.as_entire_binding(),
                }
            ],
        });

        let game_render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Pipeline Layout"),
            bind_group_layouts: &[
                &texture_bind_group_layout,
                &camera_bind_group_layout,
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
                    InstanceRaw::desc(),
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

        let vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Vertex Buffer"),
                contents: bytemuck::cast_slice(GAME_VERTICES),
                usage: wgpu::BufferUsages::VERTEX,
            }
        );

        let index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Game Index Buffer"),
                contents: bytemuck::cast_slice(GAME_INDICES),
                usage: wgpu::BufferUsages::INDEX,
            }
        );

        Self {
            hle_command_buffer: hle_command_buffer,

            game_render_texture: game_render_texture,
            game_render_texture_pipeline: game_render_texture_pipeline,
            game_render_texture_vertex_buffer: game_render_texture_vertex_buffer,
            game_render_texture_index_buffer: game_render_texture_index_buffer,
            game_render_texture_bind_group: game_render_texture_bind_group,

            game_pipeline: game_pipeline,

            game_viewport: HleRenderCommand::Noop,
            game_modelview: cgmath::Matrix4::identity(),
            game_projection: cgmath::Matrix4::identity(),

            vertex_buffer: vertex_buffer,
            index_buffer: index_buffer,
            diffuse_bind_group: diffuse_bind_group,

            instances: instances,
            instance_buffer: instance_buffer,

            //camera: camera,
            camera_uniform: camera_uniform,
            camera_buffer: camera_buffer,
            camera_bind_group: camera_bind_group,

            speed: 0.2,
            is_forward_pressed: false,
            is_backward_pressed: false,
            is_left_pressed: false,
            is_right_pressed: false,

            do_render: false,

            ui_frame_count: 0,
            ui_last_fps_time: Instant::now(),
            ui_fps: 0.0,
            game_frame_count: 0,
            game_last_fps_time: Instant::now(),
            game_fps: 0.0,
        }
    }

    fn update(&mut self, appwnd: &AppWindow, delta_time: f32) {
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

        'cmd_loop: while let Some(cmd) = self.hle_command_buffer.try_pop() {
            match cmd {
                HleRenderCommand::Viewport { .. } => {
                    self.game_viewport = cmd;
                },

                HleRenderCommand::SetModelViewMatrix(m) => {
                    println!("got view: {:?}", m);
                    self.game_modelview = m.into();
                },

                HleRenderCommand::SetProjectionMatrix(m) => {
                    println!("got proj: {:?}", m);
                    self.game_projection = m.into();
                },

                HleRenderCommand::Sync => {
                    println!("sync!");
                    self.game_frame_count += 1;
                    if (self.game_frame_count % 10) == 0 {
                        self.game_fps = 10.0 / self.game_last_fps_time.elapsed().as_secs_f64();
                        self.game_last_fps_time = Instant::now();
                    }

                    self.do_render = true;
                    break 'cmd_loop;
                },
    
                _ => (),
            };
        }
    }

    fn render(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView) {
        if self.do_render { 
            self.do_render = false;
            let game_render_texture_view = self.game_render_texture.create_view(&wgpu::TextureViewDescriptor::default());
            self.render_game(appwnd, &game_render_texture_view);
            self.reset_render_state();
        } 

        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Game Render Texture Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Game Render Texture Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Load, // no clear since we do a fullscreen quad
                        store: true, //. wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                //.occlusion_query_set: None,
                //.timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.game_render_texture_pipeline);
            render_pass.set_bind_group(0, &self.game_render_texture_bind_group, &[]);
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
    fn render_game(&mut self, appwnd: &AppWindow, view: &wgpu::TextureView) {
        const CLEAR_COLOR: wgpu::Color = wgpu::Color { r: 0.1, g: 0.2, b: 0.3, a: 1.0 };

        //self.game_projection = cgmath::perspective(cgmath::Deg(45.0), 4.0/3.0, 0.1, 100.0);
        //self.game_projection = cgmath::ortho(-160.0, 160.0, -120.0, 120.0, 0.1, 100.0);
        //self.game_projection = cgmath::ortho(-2.0, 2.0, -2.0, 2.0, 0.1, 100.0).transpose(); // WGPU uses row-major
        //self.game_projection = cgmath::Matrix4::from_cols([2.0/4.0, 0.0, 0.0, 0.0].into(), 
        //                                                  [0.0, 2.0/4.0, 0.0, 0.0].into(), 
        //                                                  [0.0, 0.0, -2.0/(100.0-0.1), 0.0].into(), 
        //                                                  [0.0, 0.0, -(100.0+0.1)/(100.0-0.1), 1.0].into()).transpose();
        //self.game_modelview = cgmath::Matrix4::look_at_rh([0.0, 0.0, 2.0].into(), [0.0, 0.0, 0.0].into(), cgmath::Vector3::unit_y());
        //self.game_modelview = self.game_modelview * cgmath::Matrix4::from_axis_angle([0.0, 1.0, 0.0].into(), cgmath::Deg(self.speed));
        let mvp = if false /*(self.game_frame_count % 2) == 0*/ { 
            OPENGL_TO_WGPU_MATRIX * self.game_projection.transpose() * self.game_modelview
        } else {
            self.game_projection.transpose() *  self.game_modelview
        };

        self.camera_uniform.view_proj = mvp.into();
        appwnd.queue().write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[self.camera_uniform]));

        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Game Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Game Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(CLEAR_COLOR),
                        store: true, //. wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                //.occlusion_query_set: None,
                //.timestamp_writes: None,
            });

            render_pass.set_pipeline(&self.game_pipeline);
            if let HleRenderCommand::Viewport { x, y, w, h } = self.game_viewport {
                println!("Viewport: {:?}", self.game_viewport);
                //render_pass.set_viewport(x, y, w, h, 0.0, 1.0);
                //render_pass.set_viewport(0.0, 0.0, 1024.0, 768.0, 0.0, 1.0);
            }
            render_pass.set_bind_group(0, &self.diffuse_bind_group, &[]);
            render_pass.set_bind_group(1, &self.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..GAME_INDICES.len() as _, 0, 0..self.instances.len() as _);
        }
        appwnd.queue().submit(Some(encoder.finish()));
    }

    fn reset_render_state(&mut self) {
        self.game_viewport   = HleRenderCommand::Noop;
        self.game_modelview  = cgmath::Matrix4::identity();
        self.game_projection = cgmath::Matrix4::identity();
    }
}

