use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};
use tracing_core::Level;

use winit::event::VirtualKeyCode;
use wgpu::util::DeviceExt;
use cgmath::prelude::*;

use image::GenericImageView;

use crate::*;
use gui::{App, AppWindow};

use n64::{SystemCommunication, ButtonState};
use n64::hle::{
    self,
    HleRenderCommand, 
    HleCommandBuffer, 
    MatrixState, Vertex, VertexFlags,
    ColorCombinerState, LightState,
};

use n64::mips::{InterruptUpdate, InterruptUpdateMode, IMask_DP};

trait ShaderData {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<ColorCombinerState>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[],
        }
    }

    fn size() -> usize;

    fn offset_of(index: usize) -> wgpu::BufferAddress {
        (index * Self::size()) as wgpu::BufferAddress
    }
}

impl ShaderData for MatrixState {
    fn size() -> usize {
        (std::mem::size_of::<[f32; 16]>()    // projection
          + std::mem::size_of::<[f32; 16]>() // modelview
          + std::mem::size_of::<[f32; 16]>() // mv_inverse
          + std::mem::size_of::<[u64; 8]>() // alignment
        ) as usize
    }
}

impl ShaderData for Vertex {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute { // position
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // color
                    offset: std::mem::size_of::<[f32; 4]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // normal
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute { // tex_coords
                    offset: std::mem::size_of::<[f32; 11]>() as wgpu::BufferAddress,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute { // tex_params
                    offset: std::mem::size_of::<[f32; 13]>() as wgpu::BufferAddress,
                    shader_location: 4,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // flags
                    offset: std::mem::size_of::<[f32; 17]>() as wgpu::BufferAddress,
                    shader_location: 5,
                    format: wgpu::VertexFormat::Uint32,
                },
            ]
        }
    }

    fn size() -> usize {
        (std::mem::size_of::<[f32; 17]>() 
         + std::mem::size_of::<u32>()) as usize
    }
}

// The implementation here needs to match the layout in hle
impl ShaderData for ColorCombinerState {
    fn size() -> usize {
        (std::mem::size_of::<[u8; 16]>() + std::mem::size_of::<[f32; 8]>() + std::mem::size_of::<[u64; 26]>()) as usize
    }
}

impl ShaderData for LightState {
    fn size() -> usize {
        std::mem::size_of::<[f32; 4*LightState::NUM_LIGHTS]>()        // lights[]
            + std::mem::size_of::<[f32; 4*LightState::NUM_LIGHTS]>()  // color[]
            + std::mem::size_of::<[u64; 4]>()                         // _alignment
    }
}

// Y texture coordinate is inverted to flip the resulting image
// default sampler is Nearest, so we just need the textured flag 
const GAME_TEXTURE_VERTICES: &[Vertex] = &[
    Vertex { position: [-1.0,  1.0, 0.0, 1.0], tex_coords: [0.0, 0.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // TL
    Vertex { position: [ 1.0,  1.0, 0.0, 1.0], tex_coords: [1.0, 0.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // TR
    Vertex { position: [-1.0, -1.0, 0.0, 1.0], tex_coords: [0.0, 1.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // BL
    Vertex { position: [ 1.0, -1.0, 0.0, 1.0], tex_coords: [1.0, 1.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // BR
];

const GAME_TEXTURE_INDICES: &[u16] = &[2, 1, 0, 1, 3, 2];

#[derive(Debug,Copy,Clone,PartialEq)]
enum ViewMode {
    Game,
    Color(usize),
    Depth(usize),
    ViOrigin,
}

pub struct Game {
    args: crate::Args,
    comms: SystemCommunication,

    check_inputs: bool,

    hle_command_buffer: Arc<HleCommandBuffer>,

    view_mode: ViewMode,

    game_render_textures: HashMap<u32, wgpu::Texture>,
    game_render_color_texture_bind_group_layout: wgpu::BindGroupLayout,
    game_render_depth_texture_bind_group_layout: wgpu::BindGroupLayout,
    game_render_color_texture_pipeline: wgpu::RenderPipeline,
    game_render_depth_texture_pipeline: wgpu::RenderPipeline,
    game_render_texture_vertex_buffer: wgpu::Buffer,
    game_render_texture_index_buffer: wgpu::Buffer,
    game_render_texture_bind_groups: HashMap<u32, wgpu::BindGroup>,

    game_depth_textures: HashMap<u32, wgpu::Texture>,
    game_depth_texture_bind_groups: HashMap<u32, wgpu::BindGroup>,
    game_depth_texture_views: HashMap<u32, wgpu::TextureView>,

    raw_render_texture: Option<wgpu::Texture>,
    raw_render_texture_bind_group: Option<wgpu::BindGroup>,

    game_pipelines: Vec<wgpu::RenderPipeline>,
    game_pipeline_no_depth_attachment: wgpu::RenderPipeline,

    game_modelview: cgmath::Matrix4<f32>,
    game_projection: cgmath::Matrix4<f32>,

    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,

    game_texture_bind_group_layout: wgpu::BindGroupLayout,
    game_textures: Vec<wgpu::Texture>,
    game_texture_bind_groups: Vec<wgpu::BindGroup>,
    game_texture_map: HashMap<u32, usize>,

    mvp_buffer: wgpu::Buffer,
    mvp_bind_group: wgpu::BindGroup,
    color_combiner_state_buffer: wgpu::Buffer,
    light_state_buffer: wgpu::Buffer,

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

    capture_display_list: u32,
    capturing_display_list: bool,
}

impl App for Game {
    fn create(appwnd: &AppWindow, mut comms: SystemCommunication, args: crate::Args) -> Self {
        let device: &wgpu::Device = appwnd.device();

        // create the main color texture render shader
        let game_render_color_texture_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Render Color Texture Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("gametexture.wgsl").into()),
        });

        // create the depth texture render shader
        let game_render_depth_texture_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Render Depth Texture Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("gamedepth.wgsl").into()),
        });

        // create the texture bind group for the game textures
        let game_render_color_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Game Render Color Texture Bind Group"),
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

        let game_render_depth_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Game Render Depth Texture Bind Group"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Depth,
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

        let game_render_color_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Render Color Texture Pipeline Layout"),
            bind_group_layouts: &[&game_render_color_texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let game_render_depth_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Render Depth Texture Pipeline Layout"),
            bind_group_layouts: &[&game_render_depth_texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let game_render_color_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Render Color Texture Pipeline"),
            layout: Some(&game_render_color_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &game_render_color_texture_shader,
                entry_point: "vs_main",
                buffers: &[
                    Vertex::desc(),
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &game_render_color_texture_shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: appwnd.surface_config().format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
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

        let game_render_depth_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Render Depth Texture Pipeline"),
            layout: Some(&game_render_depth_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &game_render_depth_texture_shader,
                entry_point: "vs_main",
                buffers: &[
                    Vertex::desc(),
                ],
            },
            fragment: Some(wgpu::FragmentState {
                module: &game_render_depth_texture_shader,
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
                label: Some("Render Game Texture Vertex Buffer"),
                contents: bytemuck::cast_slice(GAME_TEXTURE_VERTICES),
                usage: wgpu::BufferUsages::VERTEX,
            }
        );

        let game_render_texture_index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Render Game Texture Index Buffer"),
                contents: bytemuck::cast_slice(GAME_TEXTURE_INDICES),
                usage: wgpu::BufferUsages::INDEX,
            }
        );

        let game_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("game.wgsl").into()),
        });

        let mvp_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Game MVP Matrix Buffer"),
                size : (MatrixState::size() * 1024) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        let color_combiner_state_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Color Combiner State Buffer"),
                size : (ColorCombinerState::size() * 1024) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        let light_state_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Light State Buffer"),
                size : (LightState::size() * 1024) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // Need both filtering and nonfiltered samplers for this texture
        let game_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Game Texture Bind Group Layout"),
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
                wgpu::BindGroupLayoutEntry { // TextureView
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        multisampled: false,
                        view_dimension: wgpu::TextureViewDimension::D2,
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry { // Sampler
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                    count: None,
                },
            ],
        });

    let mvp_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Game MVP Matrix Bind Group Layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry { // Uniform buffer (mvp_matrix)
                        binding: 0,
                        visibility: wgpu::ShaderStages::VERTEX,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: true,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry { // color combiner state
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT, // TODO both vs and fs need CC ?
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: true,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry { // light state
                        binding: 2,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Uniform,
                            has_dynamic_offset: true,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            }
        );

        let mvp_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Game MVP Matrix Bind Group"),
            layout: &mvp_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(
                        wgpu::BufferBinding {
                            buffer: &mvp_buffer,
                            offset: 0,
                            size: core::num::NonZeroU64::new(MatrixState::size() as u64),
                        }
                    ),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Buffer(
                        wgpu::BufferBinding {
                            buffer: &color_combiner_state_buffer,
                            offset: 0,
                            size  : core::num::NonZeroU64::new(ColorCombinerState::size() as u64),
                        }
                    ),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::Buffer(
                        wgpu::BufferBinding {
                            buffer: &light_state_buffer,
                            offset: 0,
                            size  : core::num::NonZeroU64::new(LightState::size() as u64),
                        }
                    ),
                },
            ],
        });

        let game_render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Pipeline Layout"),
            bind_group_layouts: &[
                &game_texture_bind_group_layout,
                &mvp_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        let game_pipeline_vertex_state = wgpu::VertexState {
            module: &game_shader,
            entry_point: "vs_main",
            buffers: &[Vertex::desc()],
        };

        let game_pipeline_fragment_state = wgpu::FragmentState {
            module: &game_shader,
            entry_point: "fs_main",
            targets: &[Some(wgpu::ColorTargetState {
                format: appwnd.surface_config().format,
                blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        };

        let game_pipeline_primitive_state = wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None, //Some(wgpu::Face::Back),
            polygon_mode: wgpu::PolygonMode::Fill,
            unclipped_depth: false,
            conservative: false,
        };

        let game_pipeline_multisample_state = wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        };

        // I need four different game pipelines for the various depth buffer states:
        //         no compare +    no write (commonish)     00
        //         no compare + depth write (rare)          01
        //      depth compare +    no write (common)        10
        //      depth compare + depth write (commonest)     11
        let game_pipelines = vec![
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ No DT No WE"),
                layout       : Some(&game_render_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: false,
                    depth_compare      : wgpu::CompareFunction::Always,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),

            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ WE No DT"),
                layout       : Some(&game_render_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: true,
                    depth_compare      : wgpu::CompareFunction::Always,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),

            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ DT No WE"),
                layout       : Some(&game_render_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: false,
                    depth_compare      : wgpu::CompareFunction::Less,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),

            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ DT+WE"),
                layout       : Some(&game_render_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: true,
                    depth_compare      : wgpu::CompareFunction::Less,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),
        ];

        let game_pipeline_no_depth_attachment = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Pipeline w/ No Depth Attachment"),
            layout: Some(&game_render_pipeline_layout),
            vertex: game_pipeline_vertex_state.clone(),
            fragment: Some(game_pipeline_fragment_state.clone()),
            primitive: game_pipeline_primitive_state,
            depth_stencil: None,
            multisample: game_pipeline_multisample_state,
            multiview: None,
        });

        // reserve space for 64k vertices
        let vertex_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Game Vertex Buffer"),
                size : (Vertex::size() * 64 * 1024) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // and 10k indices
        let index_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Game Index Buffer"),
                size : (std::mem::size_of::<u16>() * 10 * 1024) as u64,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        let hle_command_buffer = std::mem::replace(&mut comms.hle_command_buffer, None).unwrap();
        let mut ret = Self {
            args: args,
            comms: comms,
            check_inputs: true, 

            hle_command_buffer: hle_command_buffer,

            view_mode: ViewMode::Game,

            game_render_textures: HashMap::new(),
            game_render_color_texture_bind_group_layout: game_render_color_texture_bind_group_layout,
            game_render_depth_texture_bind_group_layout: game_render_depth_texture_bind_group_layout,
            game_render_color_texture_pipeline: game_render_color_texture_pipeline,
            game_render_depth_texture_pipeline: game_render_depth_texture_pipeline,
            game_render_texture_vertex_buffer: game_render_texture_vertex_buffer,
            game_render_texture_index_buffer: game_render_texture_index_buffer,
            game_render_texture_bind_groups: HashMap::new(),

            game_depth_textures: HashMap::new(),
            game_depth_texture_bind_groups: HashMap::new(),
            game_depth_texture_views: HashMap::new(),

            raw_render_texture: None,
            raw_render_texture_bind_group: None,

            game_pipelines: game_pipelines,
            game_pipeline_no_depth_attachment: game_pipeline_no_depth_attachment,

            game_modelview: cgmath::Matrix4::identity(),
            game_projection: cgmath::Matrix4::identity(),

            vertex_buffer: vertex_buffer,
            index_buffer: index_buffer,

            game_texture_bind_group_layout: game_texture_bind_group_layout,
            game_textures: vec![],
            game_texture_bind_groups: vec![],
            game_texture_map: HashMap::new(),

            mvp_buffer: mvp_buffer,
            mvp_bind_group: mvp_bind_group,

            color_combiner_state_buffer: color_combiner_state_buffer,
            light_state_buffer: light_state_buffer,

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

            capture_display_list: 0,
            capturing_display_list: false,
        };

        let null_texture_bytes = include_bytes!("nulltexture.png");
        let null_texture_image = image::load_from_memory(null_texture_bytes).unwrap();
        let null_texture_rgba  = null_texture_image.to_rgba8();
        let null_texture_dim   = null_texture_image.dimensions();

        let null_texture_size = wgpu::Extent3d {
            width: null_texture_dim.0,
            height: null_texture_dim.1,
            depth_or_array_layers: 1,
        };

        let _ = ret.new_game_texture(appwnd, &hle::MappedTexture {
            id: u32::MAX,
            width: null_texture_size.width as usize,
            height: null_texture_size.height as usize,
            ..Default::default()
        });

        let null_texture = ret.game_textures.get(0).unwrap();
        appwnd.queue().write_texture(
            wgpu::ImageCopyTexture {
                texture: &null_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &null_texture_rgba,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(4 * null_texture_size.width),
                rows_per_image: Some(null_texture_size.height),
            },
            null_texture_size,
        );

        ret
    }

    fn update(&mut self, appwnd: &AppWindow, _delta_time: f32) {
        self.ui_frame_count += 1;
        if (self.ui_frame_count % 10) == 0 {
            self.ui_fps = 10.0 / self.ui_last_fps_time.elapsed().as_secs_f64();
            self.ui_last_fps_time = Instant::now();
        }

        // CTRL+F5+n to generate interrupt signal n
        if appwnd.input().key_held(VirtualKeyCode::LControl) {
            const KEYS: &[VirtualKeyCode] = &[
                VirtualKeyCode::F5, VirtualKeyCode::F6, VirtualKeyCode::F7,
                VirtualKeyCode::F8, VirtualKeyCode::F9, VirtualKeyCode::F10,
            ];
            if let Some(mi) = &self.comms.mi_interrupts_tx {
                for i in 0..6 {
                    if appwnd.input().key_pressed(KEYS[i]) {
                        println!("generating interrupt {}", i);
                        mi.send(InterruptUpdate(i as u32, InterruptUpdateMode::SetInterrupt)).unwrap();
                    }
                }
            }

            // CTLR+V to change the view mode
            if appwnd.input().key_pressed(VirtualKeyCode::V) {
                self.view_mode = match self.view_mode {
                    ViewMode::Game => {
                        if self.game_render_texture_bind_groups.len() > 0 {
                            ViewMode::Color(0)
                        } else if self.game_depth_texture_bind_groups.len() > 0 {
                            ViewMode::Depth(0)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::Color(i) => {
                        if self.game_render_texture_bind_groups.len() > (i + 1) {
                            ViewMode::Color(i + 1)
                        } else if self.game_depth_texture_bind_groups.len() > 0 {
                            ViewMode::Depth(0)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::Depth(i) => {
                        if self.game_depth_texture_bind_groups.len() > (i + 1) {
                            ViewMode::Depth(i + 1)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::ViOrigin => {
                        ViewMode::Game
                    }
                };
            }

            // CTRL+T to toggle disable textures
            if appwnd.input().key_pressed(VirtualKeyCode::T) {
                let mut ef = self.comms.emulation_flags.write().unwrap();
                ef.disable_textures = !ef.disable_textures;
            }

            // CTRL+U to view the texture map 
            if appwnd.input().key_pressed(VirtualKeyCode::U) {
                let mut ef = self.comms.emulation_flags.write().unwrap();
                ef.view_texture_map = (ef.view_texture_map + 1) % 5;
            }

            // CTLR+L to disable lighting (and view normals as if they're the model color)
            if appwnd.input().key_pressed(VirtualKeyCode::L) {
                let mut ef = self.comms.emulation_flags.write().unwrap();
                ef.disable_lighting = !ef.disable_lighting;
            }

            // CTRL+P to print one frame of displaylists
            if appwnd.input().key_pressed(VirtualKeyCode::P) {
                self.capture_display_list = 1;
            }

            // CTRL+S to toggle sync_ui_to_game
            if appwnd.input().key_pressed(VirtualKeyCode::S) {
                self.args.sync_ui_to_game = !self.args.sync_ui_to_game;
            }
        }

        // Reset
        if appwnd.input().key_pressed(VirtualKeyCode::F11) {
            let soft_reset = appwnd.input().key_held(VirtualKeyCode::LControl);

            // send reset 1 for hard reset, 2 for soft reset
            // then break out any cpu step cycle
            self.comms.reset_signal.store(if soft_reset { 2 } else { 1 }, Ordering::SeqCst);
            self.comms.break_cpu();

            // TODO reset rendering states
        }

        if self.check_inputs {
            self.update_game_inputs(appwnd);
            self.check_inputs = false;
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
        // run once or sync
        while !self.render_game(appwnd) && self.args.sync_ui_to_game {}

        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Render Game Texture Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Game Texture Pass"),
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

            // look for the texture associated with the color image address
            match self.view_mode {
                ViewMode::Game | ViewMode::ViOrigin => {
                    // we need the VI_ORIGIN value to know what to render..
                    let mut video_buffer = self.comms.vi_origin.load(Ordering::SeqCst);
                    if video_buffer == 0 { 
                        // Throw away the render pass and encoder, no biggie
                        return; 
                    }

                    // force video_buffer to not exist in the bind groups so we fall through to rendering ram
                    if self.view_mode == ViewMode::ViOrigin {
                        video_buffer = 1280;
                    }
                    
                    // The video buffer pointer is either exact or off by 640, or it doesn't exist at all
                    let bind_group = if self.game_render_texture_bind_groups.contains_key(&video_buffer) {
                        self.game_render_texture_bind_groups.get(&video_buffer).unwrap()
                    } else if self.game_render_texture_bind_groups.contains_key(&(video_buffer - 640)) { // video_buffer is + 640 on NTSC?
                        self.game_render_texture_bind_groups.get(&(video_buffer - 640)).unwrap()
                    } else if self.game_render_texture_bind_groups.contains_key(&(video_buffer - 1280)) { // hmmm?
                        self.game_render_texture_bind_groups.get(&(video_buffer - 1280)).unwrap()
                    } else {
                        // restore video_buffer address for render
                        let video_buffer = self.comms.vi_origin.load(Ordering::SeqCst);

                        // no game render texture found, if video_buffer is valid, render directly from RDRAM if possible
                        let width = self.comms.vi_width.load(Ordering::SeqCst) as usize;
                        let height = if width == 320 { 240 } else if width == 640 { 480 } else { warn!(target: "RENDER", "unknown render size {}", width); return; } as usize;
                        let format = self.comms.vi_format.load(Ordering::SeqCst);


                        if self.raw_render_texture.is_none() {
                            let (texture, bind_group) = self.create_color_texture(appwnd, format!("${:08X}", video_buffer).as_str(), width as u32, height as u32, true, false);
                            self.raw_render_texture = Some(texture);
                            self.raw_render_texture_bind_group = Some(bind_group);
                        }

                        // access RDRAM directly
                        // would be nice if I could copy RGB555 into a texture, but this copy seems acceptable for now
                        if let Some(rdram) = self.comms.rdram.read().as_deref().unwrap() { // rdram = &[u32]
                            let start = (video_buffer >> 2) as usize;

                            let bpp = if format == 2 { 2 } else if format == 3 { 4 } else { todo!() };
                            let size = ((width * height * bpp) >> 2) as usize;
                            if start + size >= rdram.len() { return; }

                            let mut image_data = vec![0u8; width*height*4];
                            for y in 0..height {
                                for x in 0..width {
                                    let i = (y * width) + x;
                                    let iy = (y * width) + x;

                                    match format {
                                        2 => {
                                            let shift = 16 - ((i & 1) << 4);
                                            let pix = (rdram[start + (i >> 1)] >> shift) as u16;
                                            let r = ((pix >> 11) & 0x1F) as u8;
                                            let g = ((pix >>  6) & 0x1F) as u8;
                                            let b = ((pix >>  1) & 0x1F) as u8;
                                            let a = (pix & 0x01) as u8;
                                            image_data[iy*4..][..4].copy_from_slice(&[r << 3, g << 3, b << 3, if a == 1 { 0 } else { 255 }]);
                                        },
                                        3 => { 
                                            let pix = rdram[start+i] | 0xff;
                                            image_data[iy*4..][..4].copy_from_slice(&pix.to_be_bytes());
                                        },
                                        _ => break,
                                    }
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
                    };

                    render_pass.set_pipeline(&self.game_render_color_texture_pipeline);
                    render_pass.set_bind_group(0, bind_group, &[]);
                },

                ViewMode::Color(color_buffer) => {
                    let buffers: Vec<_> = self.game_render_texture_bind_groups.iter().collect();
                    if color_buffer >= buffers.len() {
                        return;
                    }

                    render_pass.set_pipeline(&self.game_render_color_texture_pipeline);
                    render_pass.set_bind_group(0, buffers[color_buffer].1, &[]);
                },

                ViewMode::Depth(depth_buffer) => {
                    let buffers: Vec<_> = self.game_depth_texture_bind_groups.iter().collect();
                    if depth_buffer >= buffers.len() {
                        return;
                    }

                    render_pass.set_pipeline(&self.game_render_depth_texture_pipeline);
                    render_pass.set_bind_group(0, buffers[depth_buffer].1, &[]);
                },
            };

            render_pass.set_vertex_buffer(0, self.game_render_texture_vertex_buffer.slice(..));
            render_pass.set_index_buffer(self.game_render_texture_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..GAME_TEXTURE_INDICES.len() as _, 0, 0..1);
        }
        appwnd.queue().submit(Some(encoder.finish()));
    }

    fn render_ui(&mut self, _appwnd: &AppWindow, ui: &imgui::Ui) {
        let window = ui.window("Stats");
        window.size([300.0, 100.0], imgui::Condition::FirstUseEver)
              .position([0.0, 0.0], imgui::Condition::FirstUseEver)
              .build(|| {
                  ui.text(format!("UI   FPS: {}", self.ui_fps));
                  ui.text(format!("GAME FPS: {}", self.game_fps));
                  ui.text(format!("VIEW    : {:?} (Ctrl+V)", self.view_mode));
              });
    }
}

impl Game {
    fn update_game_inputs(&mut self, appwnd: &AppWindow) {
        let set_button_state = |state: &mut ButtonState, is_pressed: bool| {
            let pressed  = (!state.pressed && !state.held) &&  is_pressed;
            let held     = ( state.pressed ||  state.held) &&  is_pressed;
            let released = ( state.pressed ||  state.held) && !is_pressed;
            *state = ButtonState {
                held    : held,
                pressed : pressed,
                released: released
            };
        };

        let controllers = &mut self.comms.controllers.write().unwrap();
        set_button_state(&mut controllers[0].a    , appwnd.gamepad_ispressed(0, gilrs::Button::South)         || appwnd.input().key_held(VirtualKeyCode::Slash));
        set_button_state(&mut controllers[0].b    , appwnd.gamepad_ispressed(0, gilrs::Button::West)          || appwnd.input().key_held(VirtualKeyCode::Period));
        set_button_state(&mut controllers[0].z    , appwnd.gamepad_ispressed(0, gilrs::Button::LeftTrigger2)  || appwnd.input().key_held(VirtualKeyCode::Space));
        set_button_state(&mut controllers[0].start, appwnd.gamepad_ispressed(0, gilrs::Button::Start)         || appwnd.input().key_held(VirtualKeyCode::Return));

        set_button_state(&mut controllers[0].l_trigger, appwnd.gamepad_ispressed(0, gilrs::Button::LeftTrigger)  || appwnd.input().key_held(VirtualKeyCode::Semicolon));
        set_button_state(&mut controllers[0].r_trigger, appwnd.gamepad_ispressed(0, gilrs::Button::RightTrigger) || appwnd.input().key_held(VirtualKeyCode::Apostrophe));

        // C buttons - IJKL
        set_button_state(&mut controllers[0].c_up   , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickY)>0.2) || appwnd.input().key_held(VirtualKeyCode::I));
        set_button_state(&mut controllers[0].c_down , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickY)<0.2) || appwnd.input().key_held(VirtualKeyCode::K));
        set_button_state(&mut controllers[0].c_left , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickX)<0.2) || appwnd.input().key_held(VirtualKeyCode::J));
        set_button_state(&mut controllers[0].c_right, (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickX)>0.2) || appwnd.input().key_held(VirtualKeyCode::L));

        // DPad - Arrow keys
        set_button_state(&mut controllers[0].d_up   , appwnd.gamepad_ispressed(0, gilrs::Button::DPadUp)      || appwnd.input().key_held(VirtualKeyCode::Up));
        set_button_state(&mut controllers[0].d_down , appwnd.gamepad_ispressed(0, gilrs::Button::DPadDown)    || appwnd.input().key_held(VirtualKeyCode::Down));
        set_button_state(&mut controllers[0].d_left , appwnd.gamepad_ispressed(0, gilrs::Button::DPadLeft)    || appwnd.input().key_held(VirtualKeyCode::Left));
        set_button_state(&mut controllers[0].d_right, appwnd.gamepad_ispressed(0, gilrs::Button::DPadRight)   || appwnd.input().key_held(VirtualKeyCode::Right));

        // Analog Stick - WASD
        // x-axis assign -1 to A and 1 to D
        let mut x_kb = (-(appwnd.input().key_held(VirtualKeyCode::A) as i32) + (appwnd.input().key_held(VirtualKeyCode::D) as i32)) as f32;
        x_kb += appwnd.gamepad_getaxis(0, gilrs::Axis::LeftStickX);
        controllers[0].x_axis = x_kb.clamp(-1.0, 1.0);

        // y-axis assign -1 to S and 1 to W
        let mut y_kb = (-(appwnd.input().key_held(VirtualKeyCode::S) as i32) + (appwnd.input().key_held(VirtualKeyCode::W) as i32)) as f32;
        y_kb += appwnd.gamepad_getaxis(0, gilrs::Axis::LeftStickY);
        controllers[0].y_axis = y_kb.clamp(-1.0, 1.0);
    }

    fn create_color_texture(&mut self, appwnd: &AppWindow, name: &str, width: u32, height: u32, is_copy_dst: bool, is_filtered: bool) -> (wgpu::Texture, wgpu::BindGroup) {
        let device = appwnd.device();

        // create an offscreen render target for the actual game render
        // we double buffer so we don't get flickering when the n64/hle code is drawing too slowly
        // TODO need to resize texture with the window resize
        // OR maybe the render texture should be mapped to the n64 viewport?
        let texture = device.create_texture(
            &wgpu::TextureDescriptor {
                label: Some(format!("Game Texture: {name}").as_str()),
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
            label: Some(format!("Render Game Texture Bind Group: {name}").as_str()),
            layout: &self.game_render_color_texture_bind_group_layout,
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

    fn create_depth_texture(&mut self, appwnd: &AppWindow, name: &str, width: u32, height: u32) -> (wgpu::Texture, wgpu::BindGroup, wgpu::TextureView) {
        let device = appwnd.device();

        // create texture for the depth buffer
        // TODO need to resize texture with the window resize
        // OR maybe the render texture should be mapped to the n64 viewport?
        let texture = device.create_texture(
            &wgpu::TextureDescriptor {
                label: Some(format!("Game Depth Texture: {name}").as_str()),
                size: wgpu::Extent3d {
                    width: width,
                    height: height,
                    ..Default::default()
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                // TODO at some point probably need COPY_SRC to copy the buffer into RDRAM
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            }
        );

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter    : wgpu::FilterMode::Linear,
            min_filter    : wgpu::FilterMode::Linear,
            mipmap_filter : wgpu::FilterMode::Nearest,
            //compare: Some(wgpu::CompareFunction::LessEqual),
            lod_min_clamp: 0.0,
            lod_max_clamp: 100.0,
            ..Default::default()
        });

        let bind_group = device.create_bind_group( &wgpu::BindGroupDescriptor {
            label: Some(format!("Game Depth Texture Bind Group: {name}").as_str()),
            layout: &self.game_render_depth_texture_bind_group_layout,
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

        let render_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        (texture, bind_group, render_view)
    }

    fn new_game_texture(&mut self, appwnd: &AppWindow, mapped_texture: &hle::MappedTexture) -> usize {
        let texture_size = wgpu::Extent3d {
            width: mapped_texture.width as u32,
            height: mapped_texture.height as u32,
            depth_or_array_layers: 1,
        };

        let texture_index = self.game_textures.len();
        let device = appwnd.device();
        let texture = device.create_texture(
            &wgpu::TextureDescriptor {
                size: texture_size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                label: Some("Game Texture Cache"),
                view_formats: &[],
            }
        );

        let texture_view_linear = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler_linear = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter    : wgpu::FilterMode::Linear,
            min_filter    : wgpu::FilterMode::Linear,
            mipmap_filter : wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let texture_view_nearest = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler_nearest = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter    : wgpu::FilterMode::Nearest,
            min_filter    : wgpu::FilterMode::Nearest,
            mipmap_filter : wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Game Texture Cache Bind Group"),
            layout: &self.game_texture_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&texture_view_linear),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&sampler_linear),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&texture_view_nearest),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::Sampler(&sampler_nearest),
                },
            ],
        });

        self.game_textures.push(texture);
        self.game_texture_bind_groups.push(bind_group);
        assert!(self.game_textures.len() == self.game_texture_bind_groups.len());
        texture_index
    }

    fn render_game(&mut self, appwnd: &AppWindow) -> bool {
        while let Some(cmd) = self.hle_command_buffer.try_pop() {
            match cmd {
                HleRenderCommand::DefineColorImage {
                    framebuffer_address: addr,
                    ..
                } => {
                    if !self.game_render_textures.contains_key(&addr) {
                        let width = (self.args.window_scale as u32) * 320;
                        let height = (self.args.window_scale as u32) * 240;
                        let (texture, bind_group) = self.create_color_texture(appwnd, format!("${:08X}", addr).as_str(), width, height, false, false);
                        self.game_render_textures.insert(addr, texture);
                        self.game_render_texture_bind_groups.insert(addr, bind_group);
                        info!(target: "RENDER", "created color render target for address ${:08X} (width={})", addr, width);
                    }
                },

                HleRenderCommand::DefineDepthImage {
                    framebuffer_address: addr,
                    ..
                } => {
                    if !self.game_depth_textures.contains_key(&addr) {
                        let width = (self.args.window_scale as u32) * 320;
                        let height = (self.args.window_scale as u32) * 240;
                        let (texture, bind_group, render_view) = self.create_depth_texture(appwnd, format!("${:08X}", addr).as_str(), width, height);
                        self.game_depth_textures.insert(addr, texture);
                        self.game_depth_texture_bind_groups.insert(addr, bind_group);
                        self.game_depth_texture_views.insert(addr, render_view);
                        info!(target: "RENDER", "created depth render target for address ${:08X} (width={})", addr, width);
                    }
                },


                HleRenderCommand::VertexData(v) => {
                    appwnd.queue().write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::IndexData(mut v) => {
                    if ((std::mem::size_of::<u16>() * v.len()) as u64 % wgpu::COPY_BUFFER_ALIGNMENT) != 0 {
                        v.push(0);
                    }
                    appwnd.queue().write_buffer(&self.index_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::MatrixData(v) => {
                    appwnd.queue().write_buffer(&self.mvp_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::ColorCombinerStateData(v) => {
                    appwnd.queue().write_buffer(&self.color_combiner_state_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::LightStateData(v) => {
                    appwnd.queue().write_buffer(&self.light_state_buffer, 0, bytemuck::cast_slice(&v));
                },


                HleRenderCommand::UpdateTexture(mapped_texture_lock) => {
                    let mapped_texture = mapped_texture_lock.read().unwrap();

                    let texture = {
                        match self.game_texture_map.get(&mapped_texture.id) {
                            Some(ti) => {
                                self.game_textures.get(*ti).unwrap()
                            },

                            // new id, new texture
                            None => {
                                let ti = self.new_game_texture(appwnd, &mapped_texture);
                                self.game_texture_map.insert(mapped_texture.id, ti);
                                self.game_textures.get(ti).unwrap()
                            }
                        }
                    };

                    appwnd.queue().write_texture(
                        wgpu::ImageCopyTexture {
                            texture: texture,
                            mip_level: 0,
                            origin: wgpu::Origin3d::ZERO,
                            aspect: wgpu::TextureAspect::All,
                        },
                        bytemuck::cast_slice(&mapped_texture.data),
                        wgpu::ImageDataLayout {
                            offset: 0,
                            bytes_per_row: Some(4 * mapped_texture.width as u32),
                            rows_per_image: Some(mapped_texture.height as u32),
                        },
                        wgpu::Extent3d {
                            width: mapped_texture.width as u32,
                            height: mapped_texture.height as u32,
                            depth_or_array_layers: 1,
                        },
                    );
                },

                HleRenderCommand::RenderPass(rp) => {
                    // determine the color render target, which we should always have
                    let res = self.game_render_textures.get(&rp.color_buffer.or(Some(0xFFFF_FFFF)).unwrap()); // always pass a valid # to .get()
                    let color_texture: &wgpu::Texture = if res.is_none() {
                        warn!(target: "HLE", "render pass without a color target (rp.color_buffer={:X?}!", rp.color_buffer);
                        let res = self.game_depth_textures.get(&rp.color_buffer.or(Some(0xFFFF_FFFF)).unwrap());
                        if res.is_some() {
                            warn!(target: "HLE", "weird. it's a depth buffer!");
                        }
                        continue;
                    } else {
                        res.unwrap()
                    };
                    let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());

                    // determine the depth render target, and if none is set we can use a dummy target with depth_write disabled
                    let depth_view = self.game_depth_texture_views.get(&rp.depth_buffer.or(Some(0xFFFF_FFFF)).unwrap()); // always pass a valid # to .get()
                    let depth_stencil_attachment = if depth_view.is_none() {
                        None
                    } else {
                        // select the pipeline based on depth_write and depth_compare_enable:
                        Some(wgpu::RenderPassDepthStencilAttachment {
                            view: depth_view.unwrap(),
                            depth_ops: Some(wgpu::Operations {
                                // if clear depth load 1.0, if compare is disabled we load 1.0 so all compares pass
                                load: if rp.clear_depth {
                                    wgpu::LoadOp::Clear(1.0) 
                                } else { 
                                    wgpu::LoadOp::Load
                                },

                                // if clear depth or write is enabled, set store
                                store: true,
                            }),

                            stencil_ops: None,
                        })
                    };

                    let mut encoder: wgpu::CommandEncoder =
                        appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Game Render Pass Encoder") });
                    {
                        let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                            label: Some("Game Render Pass"),
                            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                                view: &color_view,
                                resolve_target: None,
                                ops: wgpu::Operations {
                                    load: if let Some(c) = rp.clear_color { 
                                        wgpu::LoadOp::Clear(wgpu::Color { r: c[0] as f64, g: c[1] as f64, b: c[2] as f64, a: c[3] as f64 }) 
                                    } else { 
                                        wgpu::LoadOp::Load 
                                    },
                                    store: true,
                                },
                            })],
                            depth_stencil_attachment: depth_stencil_attachment.clone(),
                        });

                        // set this one only once
                        if depth_stencil_attachment.is_none() {
                            render_pass.set_pipeline(&self.game_pipeline_no_depth_attachment);
                        }

                        render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                        for dl in rp.draw_list {
                            let scale = self.args.window_scale as f32;
                            match dl.viewport {
                                Some(vp) => {
                                    render_pass.set_viewport(scale*vp.x, scale*vp.y, scale*vp.w, scale*vp.h, 0.0, 1.0);
                                },
                                None => {
                                    render_pass.set_viewport(0.0, 0.0, 320.0*scale, 240.0*scale, 0.0, 1.0);
                                }
                            }

                            // if we have a depth attachment, select the pipeline based on depth write and compare
                            if depth_stencil_attachment.is_some() {
                                let index = ((dl.depth_write as u8) + ((dl.depth_compare_enable as u8) << 1)) as usize;
                                render_pass.set_pipeline(&self.game_pipelines[index]);
                            }

                            // set the selected texture index, which even if not textured should be set to some value
                            // texture 0 is the null texture
                            if let Some(mti) = dl.mapped_texture_index {
                                if let Some(ti) = self.game_texture_map.get(&mti) {
                                    render_pass.set_bind_group(0, &self.game_texture_bind_groups.get(*ti).unwrap(), &[]);
                                } else {
                                    render_pass.set_bind_group(0, &self.game_texture_bind_groups.get(0).unwrap(), &[]);
                                }
                            } else {
                                render_pass.set_bind_group(0, &self.game_texture_bind_groups.get(0).unwrap(), &[]);
                            }

                            // using the dynamic offset into the mvp uniform buffer, we can select which matrix and CC state is used for the triangle list
                            render_pass.set_bind_group(1, &self.mvp_bind_group, &[MatrixState::offset_of(dl.matrix_index as usize) as wgpu::DynamicOffset,
                                                                                  ColorCombinerState::offset_of(dl.color_combiner_state_index as usize) as wgpu::DynamicOffset,
                                                                                  LightState::offset_of(dl.light_state_index.or(Some(0)).unwrap() as usize) as wgpu::DynamicOffset]);

                            let last_index = dl.start_index + dl.num_indices;
                            render_pass.draw_indexed(dl.start_index..last_index as _, 0, 0..1);
                        }
                    }

                    appwnd.queue().submit(Some(encoder.finish()));
                },

                HleRenderCommand::Sync => {
                    self.game_frame_count += 1;
                    if (self.game_frame_count % 10) == 0 {
                        self.game_fps = 10.0 / self.game_last_fps_time.elapsed().as_secs_f64();
                        self.game_last_fps_time = Instant::now();
                    }

                    self.check_inputs = true;
                    self.reset_render_state();

                    // trigger RDP interrupt to signal render is done
                    if let Some(mi) = &self.comms.mi_interrupts_tx {
                        mi.send(InterruptUpdate(IMask_DP, InterruptUpdateMode::SetInterrupt)).unwrap();
                        self.comms.break_cpu();
                    }
                    
                    if self.capturing_display_list {
                        if self.capture_display_list == 1 {
                            self.capturing_display_list = false;
                            appwnd.change_logging("HLE=info", Level::INFO);
                        }
                        self.capture_display_list -= 1;
                    } else {
                        if self.capture_display_list > 0 {
                            self.capturing_display_list = true;
                            appwnd.change_logging("HLE=trace", Level::INFO);
                        }
                    }

                    return true;
                },
    
                z => unimplemented!("unhandled HLE render comand {:?}", z),
            };
        }
        false
    }

    fn reset_render_state(&mut self) {
        self.game_modelview  = cgmath::Matrix4::identity();
        self.game_projection = cgmath::Matrix4::identity();
    }
}

