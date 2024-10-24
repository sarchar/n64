use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::Instant;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};
use tracing_core::Level;

use winit::keyboard::KeyCode;
use wgpu::util::DeviceExt;

use atomic_counter::AtomicCounter;

use image::GenericImageView;

use crate::*;
use crate::windows::breakpoints::Breakpoints;
use crate::windows::cop0::Cop0State;
use crate::windows::cop1::Cop1State;
use crate::windows::listing::Listing;
use crate::windows::memory::Memory;
use crate::windows::registers::Registers;
use crate::windows::symbols::Symbols;
use crate::windows::tlb::Tlb;
use gui::{App, AppWindow};

use n64::{SystemCommunication, ButtonState};
use n64::hle::{
    self,
    HleRenderCommand, 
    HleCommandBuffer, 
    MatrixState, Vertex, VertexFlags,
    ColorCombinerState, LightState, FogState
};

use n64::mips::{InterruptUpdate, InterruptUpdateMode, IMask_DP};

trait ShaderData: Sized {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as wgpu::BufferAddress,
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
          + std::mem::size_of::<[u64; 24]>() // alignment
        ) as usize
    }
}

impl ShaderData for Vertex {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute { // view_position
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
                    offset: std::mem::size_of::<[f32; 8]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute { // tex_coords
                    offset: std::mem::size_of::<[f32; 11]>() as wgpu::BufferAddress,
                    shader_location: 3,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute { // tex_coords1
                    offset: std::mem::size_of::<[f32; 13]>() as wgpu::BufferAddress,
                    shader_location: 4,
                    format: wgpu::VertexFormat::Float32x2,
                },
                wgpu::VertexAttribute { // tex_params
                    offset: std::mem::size_of::<[f32; 15]>() as wgpu::BufferAddress,
                    shader_location: 5,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // tex_params1
                    offset: std::mem::size_of::<[f32; 19]>() as wgpu::BufferAddress,
                    shader_location: 6,
                    format: wgpu::VertexFormat::Float32x4,
                },
                wgpu::VertexAttribute { // maskshift
                    offset: std::mem::size_of::<[f32; 23]>() as wgpu::BufferAddress,
                    shader_location: 7,
                    format: wgpu::VertexFormat::Uint32,
                },
                wgpu::VertexAttribute { // maskshift1
                    offset: (std::mem::size_of::<[f32; 23]>() + std::mem::size_of::<u32>() * 1) as wgpu::BufferAddress,
                    shader_location: 8,
                    format: wgpu::VertexFormat::Uint32,
                },
                wgpu::VertexAttribute { // flags
                    offset: (std::mem::size_of::<[f32; 23]>() + std::mem::size_of::<u32>() * 2) as wgpu::BufferAddress,
                    shader_location: 9,
                    format: wgpu::VertexFormat::Uint32,
                },
            ]
        }
    }

    fn size() -> usize {
        (std::mem::size_of::<[f32; 23]>() + std::mem::size_of::<u32>() * 2) as usize
    }
}

// The implementation here needs to match the layout in hle
impl ShaderData for ColorCombinerState {
    fn size() -> usize {
        (std::mem::size_of::<[u8; 16]>() + std::mem::size_of::<[f32; 14]>() + std::mem::size_of::<[u64; 23]>()) as usize
    }
}

impl ShaderData for LightState {
    fn size() -> usize {
        std::mem::size_of::<[f32; 4*LightState::NUM_LIGHTS]>()        // lights[]
            + std::mem::size_of::<[f32; 4*LightState::NUM_LIGHTS]>()  // color[]
            + std::mem::size_of::<[u64; 4]>()                         // _alignment
    }
}

impl ShaderData for FogState {
    fn size() -> usize {
        std::mem::size_of::<[f32; 4]>()  // color
            + std::mem::size_of::<f32>() // multiplier
            + std::mem::size_of::<f32>() // offset
            + std::mem::size_of::<[u64; 29]>()  // _alignment
    }
}

// Y texture coordinate is inverted to flip the resulting image
// default sampler is Nearest, so we just need the textured flag 
const DISPLAY_GAME_TEXTURE_VERTICES: &[Vertex] = &[
    Vertex { view_position: [-1.0,  1.0, 0.0, 1.0], tex_coords: [0.0, 0.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // TL
    Vertex { view_position: [ 1.0,  1.0, 0.0, 1.0], tex_coords: [1.0, 0.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // TR
    Vertex { view_position: [-1.0, -1.0, 0.0, 1.0], tex_coords: [0.0, 1.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // BL
    Vertex { view_position: [ 1.0, -1.0, 0.0, 1.0], tex_coords: [1.0, 1.0], flags: VertexFlags::TEXTURED, ..Vertex::const_default() }, // BR
];

const DISPLAY_GAME_TEXTURE_INDICES: &[u16] = &[2, 1, 0, 1, 3, 2];

#[derive(Debug,Copy,Clone,PartialEq)]
enum ViewMode {
    Game,
    Color(usize),
    Depth(usize),
    TextureCache(usize),
    ViOrigin,
}

pub struct Game {
    args: crate::Args,
    comms: SystemCommunication,

    check_inputs: bool,

    hle_command_buffer: Arc<HleCommandBuffer>,

    view_mode: ViewMode,

    // basic bind group layouts for creating bind groups
    color_texture_bind_group_layout: wgpu::BindGroupLayout,
    depth_texture_bind_group_layout: wgpu::BindGroupLayout,

    // pipeline for rendering a game color texture (or RDRAM framebuffer) to screen
    display_game_color_texture_pipeline: wgpu::RenderPipeline,
    // pipeline for rendering a game depth texture to screen (for debug purposes)
    display_game_depth_texture_pipeline: wgpu::RenderPipeline,
    // pipeline for rendering a game texture cache to screen
    display_game_texture_pipeline: wgpu::RenderPipeline,
    // vertex and index buffers for a quad to render a game color or depth texture to screen
    display_game_texture_vertex_buffer: wgpu::Buffer,
    display_game_texture_index_buffer: wgpu::Buffer,

    // texture for rendering the RDRAM framebuffer into
    rdram_framebuffer_texture: Option<wgpu::Texture>,
    rdram_framebuffer_texture_bind_group: Option<wgpu::BindGroup>,
    rdram_framebuffer_format: (usize, usize, u32),

    // the color textures that the N64 game will render to
    game_color_textures: HashMap<u32, wgpu::Texture>,
    // the depth textures that the N64 game will render to
    game_depth_textures: HashMap<u32, wgpu::Texture>,
    // bind groups for the above textures
    game_color_texture_bind_groups: HashMap<u32, wgpu::BindGroup>,
    game_depth_texture_bind_groups: HashMap<u32, wgpu::BindGroup>,

    // set of pipelines for rendering the N64 game into a texture
    game_pipelines: Vec<wgpu::RenderPipeline>,
    // pipeline used when there's no depth texture attachment
    game_pipeline_no_depth_attachment: wgpu::RenderPipeline,

    // vertex and index buffers containing all the tris to be rendered this frame
    game_vertex_buffer: wgpu::Buffer,
    game_index_buffer: wgpu::Buffer,

    // textures used by the triangles rendered this frame
    // updated when they change, and usually only the last element in the list changes
    game_textures: Vec<wgpu::Texture>,
    // bind groups for the above game textures (one for each) 
    game_texture_bind_groups: Vec<wgpu::BindGroup>,
    // bind group layout for the above bind groups
    game_texture_bind_group_layout: wgpu::BindGroupLayout,
    // map from the ID used by the HLE backend to the index in the game textures array
    game_texture_map: HashMap<u32, usize>,

    // modelview-projection matrix uniform buffer used by all the tris in this frame
    mvp_matrix_buffer: wgpu::Buffer,
    // color combiner state uniform buffer used by all the tris in this frame
    color_combiner_state_buffer: wgpu::Buffer,
    // light state uniform buffer used by the tris this frame
    light_state_buffer: wgpu::Buffer,
    // fog state uniform buffer used by the tris this frame
    fog_state_buffer: wgpu::Buffer,

    // bind group describing the above buffers
    game_uniforms_bind_group: wgpu::BindGroup,

    //speed: f32,
    //is_forward_pressed: bool,
    //is_backward_pressed: bool,
    //is_left_pressed: bool,
    //is_right_pressed: bool,
    demo_open: bool,
    stats_window_opened: bool,
    tweakables_window_opened: bool,

    // open windows
    windows: Vec<Box<dyn GameWindow>>,
    breakpoints: Option<Breakpoints>,
    cop0state: Option<Cop0State>,
    cop1state: Option<Cop1State>,
    listing: Option<Listing>,
    memory: Option<Memory>,
    registers: Option<Registers>,
    symbols: Option<Symbols>,
    tlb: Option<Tlb>,

    // copy of tweakables so we don't need the lock every frame
    tweakables: n64::Tweakables,

    ui_frame_count: u64,
    ui_last_fps_time: Instant,
    ui_fps: f64,

    game_frame_count: u64,
    game_last_fps_time: Instant,
    game_fps: f64,

    capture_display_list: u32,
    capturing_display_list: bool,

    active_controller_port: u32,

    last_cpu_cycle_time: Instant,
    last_cpu_cycle_count: usize,
    last_prefetch_count: usize,
    last_prefetch_reset_time: Instant,

    last_interrupt_time: Instant,
}

impl App for Game {
    fn create(appwnd: &AppWindow, mut comms: SystemCommunication, args: crate::Args) -> Self {
        let device: &wgpu::Device = appwnd.device();

        // create the main color texture render shader
        let display_game_color_texture_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Display Game Color Texture Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("gamecolor.wgsl").into()),
        });

        // create the depth texture render shader
        let display_game_depth_texture_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Display Game Depth Texture Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("gamedepth.wgsl").into()),
        });

        // create the bind group for the rendering color textures
        let color_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Color Texture Bind Group Layout"),
            entries: &[
                // Texture View
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
                // Sampler
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });

        // create the bind group for the rendering depth textures
        let depth_texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Depth Texture Bind Group Layout"),
            entries: &[
                // Texture View
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
                // Sampler
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                    count: None,
                },
            ],
        });

        let display_game_color_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Display Game Color Texture Pipeline Layout"),
            bind_group_layouts: &[&color_texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let display_game_color_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Display Game Color Texture Pipeline"),
            layout: Some(&display_game_color_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &display_game_color_texture_shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &display_game_color_texture_shader,
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

        let display_game_depth_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Display Game Depth Texture Pipeline Layout"),
            bind_group_layouts: &[&depth_texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let display_game_depth_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Display Game Depth Texture Pipeline"),
            layout: Some(&display_game_depth_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &display_game_depth_texture_shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &display_game_depth_texture_shader,
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

        // create and upload the full screen quad to render game textures
        let display_game_texture_vertex_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Display Game Texture Vertex Buffer"),
                contents: bytemuck::cast_slice(DISPLAY_GAME_TEXTURE_VERTICES),
                usage: wgpu::BufferUsages::VERTEX,
            }
        );

        // create and upload the indices for the render game textures quad
        let display_game_texture_index_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Display Game Texture Index Buffer"),
                contents: bytemuck::cast_slice(DISPLAY_GAME_TEXTURE_INDICES),
                usage: wgpu::BufferUsages::INDEX,
            }
        );

        // create the bind group layout used by textures in-game
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

        let display_game_texture_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Display Game Texture Pipeline Layout"),
            bind_group_layouts: &[&game_texture_bind_group_layout], // use the game texture layout with 4 bindings
            push_constant_ranges: &[],
        });

        let display_game_texture_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Display Game Texture Texture Pipeline"),
            layout: Some(&display_game_texture_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &display_game_color_texture_shader, // can use the same shader
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &display_game_color_texture_shader,
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


        // create the MVP matrix uniform buffer
        let mvp_matrix_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("MVP Matrix Buffer"),
                size : (MatrixState::size() * 2048) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // create color combiner state uniform buffer
        let color_combiner_state_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Color Combiner State Buffer"),
                size : (ColorCombinerState::size() * 1024) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // create light state uniform buffer
        let light_state_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Light State Buffer"),
                size : (LightState::size() * 1024) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // create fog state uniform buffer
        let fog_state_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Fog State Buffer"),
                size : (FogState::size() * 128) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // create the game uniforms bind group layout
        let game_uniforms_bind_group_layout = device.create_bind_group_layout(
            &wgpu::BindGroupLayoutDescriptor {
                label: Some("Game Uniforms Bind Group Layout"),
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
                    wgpu::BindGroupLayoutEntry { // fog state
                        binding: 3,
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

        // create the game uniforms bind group
        let game_uniforms_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Game Uniforms Bind Group"),
            layout: &game_uniforms_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::Buffer(
                        wgpu::BufferBinding {
                            buffer: &mvp_matrix_buffer,
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
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::Buffer(
                        wgpu::BufferBinding {
                            buffer: &fog_state_buffer,
                            offset: 0,
                            size  : core::num::NonZeroU64::new(FogState::size() as u64),
                        }
                    ),
                },
            ],
        });

        // create the game pipeline layout
        let game_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Game Pipeline Layout"),
            bind_group_layouts: &[
                &game_texture_bind_group_layout,
                &game_texture_bind_group_layout,
                &game_uniforms_bind_group_layout,
            ],
            push_constant_ranges: &[],
        });

        // create the game ubershader
        let game_pipeline_uber_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Game Pipeline Uber Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("game.wgsl").into()),
        });

        // create the VertexState for the game pipeline
        let game_pipeline_vertex_state = wgpu::VertexState {
            module: &game_pipeline_uber_shader,
            entry_point: "vs_main",
            buffers: &[Vertex::desc()],
        };

        // create the FragmentState for the game pipeline
        let game_pipeline_fragment_state = wgpu::FragmentState {
            module: &game_pipeline_uber_shader,
            entry_point: "fs_main",
            targets: &[Some(wgpu::ColorTargetState {
                format: appwnd.surface_config().format,
                blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        };

        // create the PrimitiveState for the game pipeline
        let game_pipeline_primitive_state = wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None,
            //cull_mode: Some(wgpu::Face::Back),
            polygon_mode: wgpu::PolygonMode::Fill,
            unclipped_depth: false,
            conservative: false,
        };

        // create the MultisampleState for the game pipeline
        let game_pipeline_multisample_state = wgpu::MultisampleState {
            count: 1,
            mask: !0,
            alpha_to_coverage_enabled: false,
        };

        // We need four different game pipelines for the various depth buffer states:
        //         no compare +    no write (commonish)     00
        //         no compare + depth write (rare)          01
        //      depth compare +    no write (common)        10
        //      depth compare + depth write (commonest)     11
        let game_pipelines = vec![
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ No DT+No WE"),
                layout       : Some(&game_pipeline_layout),
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
                label        : Some("Game Pipeline w/ WE+No DT"),
                layout       : Some(&game_pipeline_layout),
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
                label        : Some("Game Pipeline w/ DT+No WE"),
                layout       : Some(&game_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: false,
                    depth_compare      : wgpu::CompareFunction::LessEqual,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),

            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label        : Some("Game Pipeline w/ DT+WE"),
                layout       : Some(&game_pipeline_layout),
                vertex       : game_pipeline_vertex_state.clone(),
                fragment     : Some(game_pipeline_fragment_state.clone()),
                primitive    : game_pipeline_primitive_state,
                depth_stencil: Some(wgpu::DepthStencilState {
                    format             : wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: true,
                    depth_compare      : wgpu::CompareFunction::LessEqual,
                    stencil            : wgpu::StencilState::default(),
                    bias               : wgpu::DepthBiasState::default()
                }),
                multisample: game_pipeline_multisample_state,
                multiview: None,
            }),
        ];

        // create the game pipeline used when there's no depth attachment
        let game_pipeline_no_depth_attachment = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Game Pipeline w/ No Depth Attachment"),
            layout: Some(&game_pipeline_layout),
            vertex: game_pipeline_vertex_state.clone(),
            fragment: Some(game_pipeline_fragment_state.clone()),
            primitive: game_pipeline_primitive_state,
            depth_stencil: None,
            multisample: game_pipeline_multisample_state,
            multiview: None,
        });

        // reserve space for 64k vertices
        let game_vertex_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Game Vertex Buffer"),
                size : (Vertex::size() * 64 * 1024) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // and 10k indices
        let game_index_buffer = device.create_buffer(
            &wgpu::BufferDescriptor {
                label: Some("Game Index Buffer"),
                size : (std::mem::size_of::<u16>() * 20 * 1024) as u64,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }
        );

        // Grab ownership of the hle_command_buffer
        let hle_command_buffer = std::mem::replace(&mut comms.hle_command_buffer, None).unwrap();

        // Grab copy of tweakables
        let tweakables = *comms.tweakables.read().unwrap();

        let mut ret = Self {
            args,
            comms: comms.clone(),

            // immediately check inputs once
            check_inputs: true, 

            hle_command_buffer,

            // default to Game view
            view_mode: ViewMode::Game,

            // default bind group layouts
            color_texture_bind_group_layout,
            depth_texture_bind_group_layout,

            // display game
            display_game_color_texture_pipeline, // render the color framebuffer
            display_game_depth_texture_pipeline, // debug view of the depth buffer
            display_game_texture_pipeline,       // debug view of the game texture caches
            display_game_texture_vertex_buffer,   
            display_game_texture_index_buffer,

            // render rdram
            rdram_framebuffer_texture: None,
            rdram_framebuffer_texture_bind_group: None,
            rdram_framebuffer_format: (0, 0, 0),

            // game color and depth render target textures
            game_color_textures: HashMap::new(),
            game_depth_textures: HashMap::new(),
            game_color_texture_bind_groups: HashMap::new(),
            game_depth_texture_bind_groups: HashMap::new(),

            // game pipelines
            game_pipelines,
            game_pipeline_no_depth_attachment,

            // render buffers
            game_vertex_buffer,
            game_index_buffer,

            // ingame textures
            game_textures: vec![],
            game_texture_bind_groups: vec![],
            game_texture_bind_group_layout,
            game_texture_map: HashMap::new(),

            // uniforms
            mvp_matrix_buffer,
            color_combiner_state_buffer,
            light_state_buffer,
            fog_state_buffer,
            game_uniforms_bind_group,

            //speed: 0.2,
            //is_forward_pressed: false,
            //is_backward_pressed: false,
            //is_left_pressed: false,
            //is_right_pressed: false,
            demo_open: false,
            stats_window_opened: true,
            tweakables_window_opened: false,
            windows: Vec::new(),
            breakpoints: None,
            cop0state: None,
            cop1state: None,
            listing: None,
            memory: None,
            registers: None,
            symbols: None,
            tlb: None,

            tweakables,

            ui_frame_count: 0,
            ui_last_fps_time: Instant::now(),
            ui_fps: 0.0,

            game_frame_count: 0,
            game_last_fps_time: Instant::now(),
            game_fps: 0.0,

            capture_display_list: 0,
            capturing_display_list: false,

            active_controller_port: 0,

            last_cpu_cycle_time: Instant::now(),
            last_cpu_cycle_count: 0,
            last_prefetch_count: 0,
            last_prefetch_reset_time: Instant::now(),

            last_interrupt_time: Instant::now(),
        };

        // Upload a null texture to texture 0 so that a game render always has a texture attached
        // to satisfy the pipeline layout requirements
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

        // TEMP
        if ret.args.enable_docking {
            ret.listing     = Some(Listing::new(comms.clone()));
            ret.registers   = Some(Registers::new(comms.clone()));
            ret.breakpoints = Some(Breakpoints::new(comms.clone()));
            ret.cop0state   = Some(Cop0State::new(comms.clone()));
            ret.cop1state   = Some(Cop1State::new(comms.clone()));
            ret.symbols     = Some(Symbols::new(comms.clone()));
            ret.tlb         = Some(Tlb::new(comms.clone()));
            ret.memory      = Some(Memory::new(comms.clone()));
        }

        ret
    }

    fn update(&mut self, appwnd: &AppWindow, _delta_time: f32, game_has_focus: bool) {
        self.ui_frame_count += 1;
        if (self.ui_frame_count % 10) == 0 {
            self.ui_fps = 10.0 / self.ui_last_fps_time.elapsed().as_secs_f64();
            self.ui_last_fps_time = Instant::now();
        }

        // CTRL+F(5+n) to generate interrupt signal n
        if appwnd.input().held_control() {
            const KEYS: &[KeyCode] = &[
                KeyCode::F5, KeyCode::F6, KeyCode::F7,
                KeyCode::F8, KeyCode::F9, KeyCode::F10,
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
            if appwnd.input().key_pressed(KeyCode::KeyV) {
                self.view_mode = match self.view_mode {
                    ViewMode::Game => {
                        if self.game_color_texture_bind_groups.len() > 0 {
                            ViewMode::Color(0)
                        } else if self.game_depth_texture_bind_groups.len() > 0 {
                            ViewMode::Depth(0)
                        } else if self.game_texture_bind_groups.len() > 0 {
                            ViewMode::TextureCache(0)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::Color(i) => {
                        if self.game_color_texture_bind_groups.len() > (i + 1) {
                            ViewMode::Color(i + 1)
                        } else if self.game_depth_texture_bind_groups.len() > 0 {
                            ViewMode::Depth(0)
                        } else if self.game_texture_bind_groups.len() > 0 {
                            ViewMode::TextureCache(0)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::Depth(i) => {
                        if self.game_depth_texture_bind_groups.len() > (i + 1) {
                            ViewMode::Depth(i + 1)
                        } else if self.game_texture_bind_groups.len() > 0 {
                            ViewMode::TextureCache(0)
                        } else {
                            ViewMode::ViOrigin
                        }
                    },
                    ViewMode::TextureCache(i) => {
                        if self.game_texture_bind_groups.len() > (i + 1) {
                            ViewMode::TextureCache(i + 1)
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
            if appwnd.input().key_pressed(KeyCode::KeyT) {
                let mut ef = self.comms.tweakables.write().unwrap();
                ef.disable_textures = !ef.disable_textures;
            }

            // CTRL+U to view the texture map 
            // CTRL+SHIFT+U to cycle the maps
            if appwnd.input().key_pressed(KeyCode::KeyU) {
                match self.view_mode {
                    ViewMode::TextureCache(_) => {
                        self.view_mode = ViewMode::Game;
                    },
                    _ => {
                        if self.game_texture_bind_groups.len() > 1 {
                            self.view_mode = ViewMode::TextureCache(1);
                        } else {
                            self.view_mode = ViewMode::TextureCache(0);
                        }
                    }
                }
            }

            // CTLR+L to disable lighting (and view normals as if they're the model color)
            if appwnd.input().key_pressed(KeyCode::KeyL) {
                let mut ef = self.comms.tweakables.write().unwrap();
                ef.disable_lighting = !ef.disable_lighting;
            }

            // CTLR+F to disable fog
            if appwnd.input().key_pressed(KeyCode::KeyF) {
            }

            // CTRL+P to print one frame of displaylists
            if appwnd.input().key_pressed(KeyCode::KeyP) {
                self.capture_display_list = 1;
            }

            // CTRL+S to toggle sync_ui_to_game
            if appwnd.input().key_pressed(KeyCode::KeyS) {
                self.args.sync_ui_to_game = !self.args.sync_ui_to_game;
            }

            // CTRL+1,2,3,4 to set the controller input port
            const NUMS: [KeyCode; 4] = [KeyCode::Digit1, KeyCode::Digit2, KeyCode::Digit3, KeyCode::Digit4];
            for i in 0..4 {
                if appwnd.input().key_pressed(NUMS[i]) {
                    self.active_controller_port = i as _;
                }
            }
        }

        // Reset
        if appwnd.input().key_pressed(KeyCode::F2) {
            let soft_reset = appwnd.input().key_held(KeyCode::ControlLeft);

            // send reset 1 for hard reset, 2 for soft reset
            // then break out any cpu step cycle
            self.comms.reset_signal.store(if soft_reset { 2 } else { 1 }, Ordering::SeqCst);
            self.comms.break_cpu();

            // TODO reset rendering states ?
            // TODO free texture cache ?
        }

        // fast-forward
        if appwnd.input().key_held(KeyCode::Backquote) {
            self.comms.cpu_throttle.store(0, Ordering::Relaxed);
        } else if self.comms.cpu_throttle.load(Ordering::Relaxed) == 0 { // don't reset value `2` to 1
            self.comms.cpu_throttle.store(1, Ordering::Relaxed);
        }

        // Show demo window
        if appwnd.input().key_pressed(KeyCode::F12) {
            self.demo_open = true;
        }

        if self.check_inputs {
            self.update_game_inputs(appwnd, game_has_focus);
            self.check_inputs = false;
        }

        //let input = appwnd.input();
        //self.is_forward_pressed  = input.key_held(KeyCode::KeyW) || input.key_held(KeyCode::ArrowUp);
        //self.is_backward_pressed = input.key_held(KeyCode::KeyS) || input.key_held(KeyCode::ArrowDown);
        //self.is_left_pressed     = input.key_held(KeyCode::KeyA) || input.key_held(KeyCode::ArrowLeft);
        //self.is_right_pressed    = input.key_held(KeyCode::KeyD) || input.key_held(KeyCode::ArrowRight);

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
        while !self.render_game_to_texture(appwnd) && self.args.sync_ui_to_game {}

        // always check inputs
        self.check_inputs = true;

        let mut encoder: wgpu::CommandEncoder =
            appwnd.device().create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("Display Game Texture Encoder") });
        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Display Game Texture Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        // the old color doesn't matter, so LoadOp::Load is more efficient
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.0, g: 0.0, b: 0.0, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            // look for the texture associated with the color image address
            match self.view_mode {
                ViewMode::Game | ViewMode::ViOrigin => {
                    // we need the VI_ORIGIN value to know what to render..
                    let mut video_buffer = self.comms.vi_origin.load(Ordering::Relaxed);
                    if video_buffer == 0 { 
                        // Throw away the render pass and encoder, no biggie
                        return; 
                    }

                    // force video_buffer to not exist in the bind groups so we fall through to rendering ram
                    if self.view_mode == ViewMode::ViOrigin {
                        video_buffer = 1280;
                    }
                    
                    // The video buffer pointer is either exact or off by 640, or it doesn't exist at all
                    let bind_group = if self.game_color_texture_bind_groups.contains_key(&video_buffer) {
                        self.game_color_texture_bind_groups.get(&video_buffer).unwrap()
                    } else if self.game_color_texture_bind_groups.contains_key(&(video_buffer.wrapping_sub(640))) { // video_buffer is + 640 on NTSC?
                        self.game_color_texture_bind_groups.get(&(video_buffer - 640)).unwrap()
                    } else if self.game_color_texture_bind_groups.contains_key(&(video_buffer.wrapping_sub(1280))) { // hmmm?
                        self.game_color_texture_bind_groups.get(&(video_buffer - 1280)).unwrap()
                    } else {
                        // restore video_buffer address for render
                        let video_buffer = self.comms.vi_origin.load(Ordering::Relaxed);

                        // no game render texture found, if video_buffer is valid, render directly from RDRAM if possible
                        let width = self.comms.vi_width.load(Ordering::Relaxed) as usize;
                        let height = if width == 320 { 240 } else if width == 640 { 480 } else if width == 400 { 240 } else { /*warn!(target: "RENDER", "unknown render size {}", width);*/ return; } as usize;
                        let format = self.comms.vi_format.load(Ordering::Relaxed);
                        // println!("buffer=${:08X} width={} height={} format={}", video_buffer, width, height, format);

                        if self.rdram_framebuffer_texture.is_none() || self.rdram_framebuffer_format != (width, height, format) {
                            //println!("(re-)creating framebuffer texture with height {}", height);
                            let (texture, bind_group) = self.create_color_texture(appwnd, format!("${:08X}", video_buffer).as_str(), width as u32, height as u32, true, false);
                            self.rdram_framebuffer_texture = Some(texture);
                            self.rdram_framebuffer_texture_bind_group = Some(bind_group);
                            self.rdram_framebuffer_format = (width, height, format);
                        }

                        // access RDRAM directly
                        // would be nice if I could copy RGB555 into a texture, but this copy seems acceptable for now
                        if format != 0 {
                            if let Some(rdram) = self.comms.rdram.read().as_deref().unwrap() { // rdram = &[u32]
                                let start = (video_buffer >> 2) as usize;

                                let bpp = if format == 2 { 2 } else if format == 3 { 4 } else { todo!("unknown format {}", format) };
                                let size = ((width * height * bpp) >> 2) as usize;
                                if start + size >= rdram.len() { return; }

                                let mut image_data = vec![0u32; width*height];
                                for y in 0..height {
                                    for x in 0..width {
                                        let i = (y * width) + x;
                                        let iy = (y * width) + x;

                                        match format {
                                            2 => { // RGBX5551 (thanks, Rasky!)
                                                let shift = 16 - ((i & 1) << 4);
                                                let pix = (rdram[start + (i >> 1)] >> shift) as u16;
                                                let mut r = ((pix >> 11) & 0x1F) as u8;
                                                let rb = r & 0x01;
                                                r = (r << 3) | (rb << 2) | (rb << 1) | rb;
                                                let mut g = ((pix >>  6) & 0x1F) as u8;
                                                let gb = g & 0x01;
                                                g = (g << 3) | (gb << 2) | (gb << 1) | gb;
                                                let mut b = ((pix >>  1) & 0x1F) as u8;
                                                let bb = b & 0x01;
                                                b = (b << 3) | (bb << 2) | (bb << 1) | bb;
                                                image_data[iy] = (0xFFu32 << 24) | ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);
                                            },
                                            3 => {  // RGBA32
                                                let pix = rdram[start+i];
                                                let r = (pix >> 24) & 0xFF;
                                                let g = (pix >> 16) & 0xFF;
                                                let b = (pix >>  8) & 0xFF;
                                                let a = 0xFF;
                                                image_data[iy] = (a << 24) | (r << 16) | (g << 8) | b;
                                            },
                                            _ => break,
                                        }
                                    }
                                }

                                appwnd.queue().write_texture(
                                    wgpu::ImageCopyTexture {
                                        texture: self.rdram_framebuffer_texture.as_ref().unwrap(),
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
                        }

                        self.rdram_framebuffer_texture_bind_group.as_ref().unwrap()
                    };

                    render_pass.set_pipeline(&self.display_game_color_texture_pipeline);
                    render_pass.set_bind_group(0, bind_group, &[]);
                },

                ViewMode::Color(color_buffer) => {
                    let buffers: Vec<_> = self.game_color_texture_bind_groups.iter().collect();
                    if color_buffer >= buffers.len() {
                        return;
                    }

                    render_pass.set_pipeline(&self.display_game_color_texture_pipeline);
                    render_pass.set_bind_group(0, buffers[color_buffer].1, &[]);
                },

                ViewMode::Depth(depth_buffer) => {
                    let buffers: Vec<_> = self.game_depth_texture_bind_groups.iter().collect();
                    if depth_buffer >= buffers.len() {
                        return;
                    }

                    render_pass.set_pipeline(&self.display_game_depth_texture_pipeline);
                    render_pass.set_bind_group(0, buffers[depth_buffer].1, &[]);
                },

                ViewMode::TextureCache(texture_index) => {
                    let buffers: Vec<_> = self.game_texture_bind_groups.iter().collect();
                    if texture_index >= buffers.len() {
                        return;
                    }

                    render_pass.set_pipeline(&self.display_game_texture_pipeline);
                    render_pass.set_bind_group(0, buffers[texture_index], &[]);
                }
            };

            // render a quad with the selected texture
            render_pass.set_vertex_buffer(0, self.display_game_texture_vertex_buffer.slice(..));
            render_pass.set_index_buffer(self.display_game_texture_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..DISPLAY_GAME_TEXTURE_INDICES.len() as _, 0, 0..1);
        }
        appwnd.queue().submit(Some(encoder.finish()));
    }

    fn render_ui(&mut self, _appwnd: &AppWindow, ui: &imgui::Ui) {
        if self.demo_open {
            ui.show_demo_window(&mut self.demo_open);
        }

        //let mut ef = self.comms.tweakables.write().unwrap();
        //ef.disable_lighting = !ef.disable_lighting;
        if ui.is_mouse_released(imgui::MouseButton::Right) {
            ui.open_popup("main_context");
        }

        if let Some(main_context_token) = ui.begin_popup("main_context") {
            if let Some(windows_menu_token) = ui.begin_menu("Windows") {
                ui.checkbox("Stats", &mut self.stats_window_opened);
                ui.checkbox("Tweakables", &mut self.tweakables_window_opened);

                if let Some(debugger_windows_token) = ui.begin_menu("Debugger") {
                    let mut listing_open = self.listing.is_some();
                    if ui.checkbox("Listing", &mut listing_open) {
                        if self.listing.is_some() {
                            self.listing = None;
                        } else {
                            self.listing = Some(Listing::new(self.comms.clone()));
                        }
                    }

                    let mut registers_open = self.registers.is_some();
                    if ui.checkbox("Registers", &mut registers_open) {
                        if self.registers.is_some() {
                            self.registers = None;
                        } else {
                            self.registers = Some(Registers::new(self.comms.clone()));
                        }
                    }

                    let mut breakpoints_open = self.breakpoints.is_some();
                    if ui.checkbox("Breakpoints", &mut breakpoints_open) {
                        if self.breakpoints.is_some() {
                            self.breakpoints = None;
                        } else {
                            self.breakpoints = Some(Breakpoints::new(self.comms.clone()));
                        }
                    }

                    let mut cop0state_open = self.cop0state.is_some();
                    if ui.checkbox("Cop0 State", &mut cop0state_open) {
                        if self.cop0state.is_some() {
                            self.cop0state = None;
                        } else {
                            self.cop0state = Some(Cop0State::new(self.comms.clone()));
                        }
                    }

                    let mut cop1state_open = self.cop1state.is_some();
                    if ui.checkbox("Cop1 State", &mut cop1state_open) {
                        if self.cop1state.is_some() {
                            self.cop1state = None;
                        } else {
                            self.cop1state = Some(Cop1State::new(self.comms.clone()));
                        }
                    }

                    let mut symbols_open = self.symbols.is_some();
                    if ui.checkbox("Symbols", &mut symbols_open) {
                        if self.symbols.is_some() {
                            self.symbols = None;
                        } else {
                            self.symbols = Some(Symbols::new(self.comms.clone()));
                        }
                    }

                    let mut tlb_open = self.tlb.is_some();
                    if ui.checkbox("TLB Viewer", &mut tlb_open) {
                        if self.tlb.is_some() {
                            self.tlb = None;
                        } else {
                            self.tlb = Some(Tlb::new(self.comms.clone()));
                        }
                    }

                    let mut memory_open = self.memory.is_some();
                    if ui.checkbox("Memory Viewer", &mut memory_open) {
                        if self.memory.is_some() {
                            self.memory = None;
                        } else {
                            self.memory = Some(Memory::new(self.comms.clone()));
                        }
                    }

                    debugger_windows_token.end();
                }
                windows_menu_token.end();
            }
            main_context_token.end();
        }

        if self.stats_window_opened {
            let window = ui.window("Stats");
            let total_cpu_steps = self.comms.total_cpu_steps.get();
            let duration = self.last_cpu_cycle_time.elapsed().as_secs_f64();
            window.size([300.0, 100.0], imgui::Condition::FirstUseEver)
                  .position([0.0, 0.0], imgui::Condition::FirstUseEver)
                  .opened(&mut self.stats_window_opened) // TODO: remember closed state between runs
                  .build(|| {
                      ui.text(format!("UI   FPS: {}", self.ui_fps as u64));
                      ui.text(format!("GAME FPS: {:.2}", self.game_fps));
                      ui.text(format!("VIEW    : {:?} (Ctrl+V)", self.view_mode));
                      ui.text(format!("CPU     : {:.4} MIPS", (((total_cpu_steps - self.last_cpu_cycle_count) as f64) / duration) / 1_000_000.0));
                      ui.text(format!("PFCNT   : {}/s", self.last_prefetch_count));
                  }
            );
            if duration >= 0.2 { // arbitrary time for resetting the freq timer
                self.last_cpu_cycle_time = Instant::now();
                self.last_cpu_cycle_count = total_cpu_steps;
            }
            if self.last_prefetch_reset_time.elapsed().as_secs_f64() >= 1.0 {
                self.last_prefetch_count = self.comms.reset_prefetch_counter();
                self.last_prefetch_reset_time = Instant::now();
            }
        }

        if self.tweakables_window_opened {
            let mut tweakables_dirty = false;
            let window = ui.window("Tweakables");
            window.size([300.0, 500.0], imgui::Condition::FirstUseEver)
                  .position([400.0, 0.0], imgui::Condition::FirstUseEver)
                  .opened(&mut self.tweakables_window_opened)
                  .build(|| {
                      let mut texturing = !self.tweakables.disable_textures;
                      if ui.checkbox("Texturing", &mut texturing) {
                          self.tweakables.disable_textures = !texturing;
                          tweakables_dirty = true;
                      }
                      let mut lighting = !self.tweakables.disable_lighting;
                      if ui.checkbox("Lighting", &mut lighting) {
                          self.tweakables.disable_lighting = !lighting;
                          tweakables_dirty = true;
                      }
                      let mut fog = !self.tweakables.disable_fog;
                      if ui.checkbox("Fog", &mut fog) {
                          self.tweakables.disable_fog = !fog;
                          tweakables_dirty = true;
                      }
                  }
            );

            if tweakables_dirty {
                let mut tw = self.comms.tweakables.write().unwrap();
                *tw = self.tweakables;
            }
        }

        // render child windows.. anything returning false will be removed from the window list
        if let Some(w) = self.listing.as_mut() {
            if !w.render_ui(ui) {
                self.listing = None;
            }
        }

        if let Some(w) = self.breakpoints.as_mut() {
            if !w.render_ui(ui) {
                self.breakpoints = None;
            }
        }

        if let Some(w) = self.registers.as_mut() {
            if !w.render_ui(ui) {
                self.registers = None;
            }
        }

        if let Some(w) = self.cop0state.as_mut() {
            if !w.render_ui(ui) {
                self.cop0state = None;
            }
        }

        if let Some(w) = self.cop1state.as_mut() {
            if !w.render_ui(ui) {
                self.cop1state = None;
            }
        }

        if let Some(w) = self.symbols.as_mut() {
            if !w.render_ui(ui) {
                self.symbols = None;
            }
        }

        if let Some(w) = self.tlb.as_mut() {
            if !w.render_ui(ui) {
                self.tlb = None;
            }
        }

        if let Some(w) = self.memory.as_mut() {
            if !w.render_ui(ui) {
                self.memory = None;
            }
        }

        self.windows.retain_mut(|window| {
            window.render_ui(ui)
        });

        // process pubsub messages
        self.comms.pubsub.update();
    }
}

impl Game {
    fn update_game_inputs(&mut self, appwnd: &AppWindow, allow_keyboard_input: bool) {
        let update_button_state = |state: &mut ButtonState, is_pressed: bool| {
            let pressed  = (!state.pressed && !state.held) &&  is_pressed;
            let held     = ( state.pressed ||  state.held) &&  is_pressed;
            let released = ( state.pressed ||  state.held) && !is_pressed;
            *state = ButtonState { held, pressed, released };
        };

        let is_key_down = |keycode| {
            allow_keyboard_input && (appwnd.input().key_pressed(keycode) || appwnd.input().key_held(keycode))
        };

        let controllers = &mut self.comms.controllers.write().unwrap();
        let controller = &mut controllers[self.active_controller_port as usize];
        update_button_state(&mut controller.a    , appwnd.gamepad_ispressed(0, gilrs::Button::South)         || is_key_down(KeyCode::Slash));
        update_button_state(&mut controller.b    , appwnd.gamepad_ispressed(0, gilrs::Button::West)          || is_key_down(KeyCode::Period));
        update_button_state(&mut controller.z    , appwnd.gamepad_ispressed(0, gilrs::Button::LeftTrigger2)  || is_key_down(KeyCode::Space));
        update_button_state(&mut controller.start, appwnd.gamepad_ispressed(0, gilrs::Button::Start)         || is_key_down(KeyCode::Enter));

        update_button_state(&mut controller.l_trigger, appwnd.gamepad_ispressed(0, gilrs::Button::LeftTrigger)  || is_key_down(KeyCode::Semicolon));
        update_button_state(&mut controller.r_trigger, appwnd.gamepad_ispressed(0, gilrs::Button::RightTrigger) || is_key_down(KeyCode::Quote));

        // C buttons - IJKL
        const DEADZONE: f32 = 0.2;
        update_button_state(&mut controller.c_up   , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickY) >  DEADZONE) || is_key_down(KeyCode::KeyI));
        update_button_state(&mut controller.c_down , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickY) < -DEADZONE) || is_key_down(KeyCode::KeyK));
        update_button_state(&mut controller.c_left , (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickX) < -DEADZONE) || is_key_down(KeyCode::KeyJ));
        update_button_state(&mut controller.c_right, (appwnd.gamepad_getaxis(0, gilrs::Axis::RightStickX) >  DEADZONE) || is_key_down(KeyCode::KeyL));

        // DPad - Arrow keys
        update_button_state(&mut controller.d_up   , appwnd.gamepad_ispressed(0, gilrs::Button::DPadUp)      || is_key_down(KeyCode::ArrowUp));
        update_button_state(&mut controller.d_down , appwnd.gamepad_ispressed(0, gilrs::Button::DPadDown)    || is_key_down(KeyCode::ArrowDown));
        update_button_state(&mut controller.d_left , appwnd.gamepad_ispressed(0, gilrs::Button::DPadLeft)    || is_key_down(KeyCode::ArrowLeft));
        update_button_state(&mut controller.d_right, appwnd.gamepad_ispressed(0, gilrs::Button::DPadRight)   || is_key_down(KeyCode::ArrowRight));

        // Analog Stick - WASD
        // x-axis assign -1 to A and 1 to D
        let mut x_kb = (-(is_key_down(KeyCode::KeyA) as i32) + (is_key_down(KeyCode::KeyD) as i32)) as f32;
        x_kb += appwnd.gamepad_getaxis(0, gilrs::Axis::LeftStickX);
        controller.x_axis = x_kb.clamp(-1.0, 1.0);

        // y-axis assign -1 to S and 1 to W
        let mut y_kb = (-(is_key_down(KeyCode::KeyS) as i32) + (is_key_down(KeyCode::KeyW) as i32)) as f32;
        y_kb += appwnd.gamepad_getaxis(0, gilrs::Axis::LeftStickY);
        controller.y_axis = y_kb.clamp(-1.0, 1.0);
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
            layout: &self.color_texture_bind_group_layout,
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

    fn create_depth_texture(&mut self, appwnd: &AppWindow, name: &str, width: u32, height: u32) -> (wgpu::Texture, wgpu::BindGroup) {
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
            mag_filter    : wgpu::FilterMode::Nearest,
            min_filter    : wgpu::FilterMode::Nearest,
            mipmap_filter : wgpu::FilterMode::Nearest,
            //compare: Some(wgpu::CompareFunction::LessEqual),
            lod_min_clamp: 0.0,
            lod_max_clamp: 100.0,
            ..Default::default()
        });

        let bind_group = device.create_bind_group( &wgpu::BindGroupDescriptor {
            label: Some(format!("Game Depth Texture Bind Group: {name}").as_str()),
            layout: &self.depth_texture_bind_group_layout,
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

    fn render_game_to_texture(&mut self, appwnd: &AppWindow) -> bool {
        while let Some(cmd) = self.hle_command_buffer.try_pop() {
            match cmd {
                HleRenderCommand::DefineColorImage {
                    framebuffer_address: addr,
                    ..
                } => {
                    if !self.game_color_textures.contains_key(&addr) {
                        let width = (self.args.window_scale as u32) * 320;
                        let height = (self.args.window_scale as u32) * 240;
                        let (texture, bind_group) = self.create_color_texture(appwnd, format!("${:08X}", addr).as_str(), width, height, false, false);
                        self.game_color_textures.insert(addr, texture);
                        self.game_color_texture_bind_groups.insert(addr, bind_group);
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
                        let (texture, bind_group) = self.create_depth_texture(appwnd, format!("${:08X}", addr).as_str(), width, height);
                        self.game_depth_textures.insert(addr, texture);
                        self.game_depth_texture_bind_groups.insert(addr, bind_group);
                        info!(target: "RENDER", "created depth render target for address ${:08X} (width={})", addr, width);
                    }
                },


                HleRenderCommand::VertexData(v) => {
                    appwnd.queue().write_buffer(&self.game_vertex_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::IndexData(mut v) => {
                    if ((std::mem::size_of::<u16>() * v.len()) as u64 % wgpu::COPY_BUFFER_ALIGNMENT) != 0 {
                        v.push(0);
                    }
                    appwnd.queue().write_buffer(&self.game_index_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::MatrixData(v) => {
                    appwnd.queue().write_buffer(&self.mvp_matrix_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::ColorCombinerStateData(v) => {
                    appwnd.queue().write_buffer(&self.color_combiner_state_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::LightStateData(v) => {
                    appwnd.queue().write_buffer(&self.light_state_buffer, 0, bytemuck::cast_slice(&v));
                },

                HleRenderCommand::FogStateData(v) => {
                    appwnd.queue().write_buffer(&self.fog_state_buffer, 0, bytemuck::cast_slice(&v));
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
                    let res = self.game_color_textures.get(&rp.color_buffer.or(Some(0xFFFF_FFFF)).unwrap()); // always pass a valid # to .get()
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
                    let res = self.game_depth_textures.get(&rp.depth_buffer.or(Some(0xFFFF_FFFF)).unwrap()); // always pass a valid # to .get()
                    let depth_view;
                    let depth_stencil_attachment = if res.is_none() {
                        None
                    } else {
                        depth_view = res.unwrap().create_view(&wgpu::TextureViewDescriptor::default());

                        // select the pipeline based on depth_write and depth_compare_enable:
                        Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &depth_view,
                            depth_ops: Some(wgpu::Operations {
                                // if clear depth load 1.0, if compare is disabled we load 1.0 so all compares pass
                                load: if rp.clear_depth {
                                    wgpu::LoadOp::Clear(1.0) 
                                } else { 
                                    wgpu::LoadOp::Load
                                },

                                // if clear depth or write is enabled, set store
                                store: wgpu::StoreOp::Store,
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
                                    store: wgpu::StoreOp::Store,
                                },
                            })],
                            depth_stencil_attachment: depth_stencil_attachment.clone(),
                            occlusion_query_set: None,
                            timestamp_writes: None,
                        });

                        // set this one only once
                        if depth_stencil_attachment.is_none() {
                            render_pass.set_pipeline(&self.game_pipeline_no_depth_attachment);
                        }

                        render_pass.set_vertex_buffer(0, self.game_vertex_buffer.slice(..));
                        render_pass.set_index_buffer(self.game_index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                        for dl in rp.draw_list {
                            let scale = self.args.window_scale as f32;
                            match dl.viewport {
                                Some(vp) => {
                                    render_pass.set_viewport(0.0f32.max(scale*vp.x), 0.0f32.max(scale*vp.y), 
                                                             (320.0*scale).min(scale*vp.w), (240.0*scale).min(scale*vp.h), 
                                                             0.0, 1.0);
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

                            // set the selected texture index for TEXEL1, which even if not textured should be set to some value
                            // texture 0 is the null texture
                            if let Some(mti) = dl.mapped_texture_index1 {
                                if let Some(ti) = self.game_texture_map.get(&mti) {
                                    render_pass.set_bind_group(1, &self.game_texture_bind_groups.get(*ti).unwrap(), &[]);
                                } else {
                                    render_pass.set_bind_group(1, &self.game_texture_bind_groups.get(0).unwrap(), &[]);
                                }
                            } else {
                                render_pass.set_bind_group(1, &self.game_texture_bind_groups.get(0).unwrap(), &[]);
                            }

                            // using the dynamic offset into the mvp uniform buffer, we can select which matrix and CC state is used for the triangle list
                            render_pass.set_bind_group(2, &self.game_uniforms_bind_group, &[MatrixState::offset_of(dl.matrix_index as usize) as wgpu::DynamicOffset,
                                                                                            ColorCombinerState::offset_of(dl.color_combiner_state_index as usize) as wgpu::DynamicOffset,
                                                                                            LightState::offset_of(dl.light_state_index.or(Some(0)).unwrap() as usize) as wgpu::DynamicOffset,
                                                                                            FogState::offset_of(dl.fog_state_index as usize) as wgpu::DynamicOffset]);

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

                    // trigger RDP interrupt to signal render is done
                    if let Some(mi) = &self.comms.mi_interrupts_tx {
                        //while self.last_interrupt_time.elapsed().as_secs_f64() < (1.0 / 60.0) {
                        //    unsafe { std::arch::asm!("nop"); }
                        //}
                        self.last_interrupt_time = Instant::now();
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
}

pub struct Utils;

impl Utils {
    pub const BUTTON_COLOR        : [f32; 4] = [    1.0    , 0.0, 0.0, 1.0];
    pub const BUTTON_HOVERED_COLOR: [f32; 4] = [196.0/255.0, 0.0, 0.0, 1.0];

    pub const DATA_CHANGED_FADE_COLOR: [f32; 4] = [0.8, 0.8, 0.8, 1.0];
    pub const DATA_CHANGED_FADE_TIME : f32      = 0.5;

    // create a toggle-able toolbar button, returning true when the state changes and the current state in `state`
    // pass None for no state tracking
    pub fn flag_button<T: AsRef<str>, U: AsRef<str>>(ui: &imgui::Ui, state: Option<&mut bool>, label: T, tooltip: Option<U>) -> bool {
        let mut tokens = Vec::new();
        if state.as_ref().is_some_and(|v| **v) {
            tokens.push(ui.push_style_color(imgui::StyleColor::Button, Utils::BUTTON_COLOR));
            tokens.push(ui.push_style_color(imgui::StyleColor::ButtonHovered, Utils::BUTTON_HOVERED_COLOR));
        }

        let pressed = ui.small_button(label);
        if pressed {
            if let Some(v) = state {
                *v = !*v;
            }
        }

        if tooltip.is_some() && ui.is_item_hovered() {
            ui.tooltip_text(tooltip.unwrap());
        }

        for token in tokens {
            token.pop();
        }

        pressed
    }

    pub fn messagebox<T: AsRef<str>, U: AsRef<str>>(ui: &imgui::Ui, title: T, msg: U) {
        ui.modal_popup(format!("{}###messagebox", title.as_ref()), || {
            ui.text(msg);

            if ui.button("OK") {
                ui.close_current_popup();
            }
        });
    }

    pub fn show_messagebox(ui: &imgui::Ui) {
        ui.open_popup("###messagebox");
    }

    // Parse `s` as either hexadecimal or decimal without sign extension
    pub fn parse_u64<T: AsRef<str>>(s: &T) -> Option<u64> {
        let s: &str = s.as_ref().trim();
        if let Some(hex) = {
            if s.starts_with("$") {
                Some(&s[1..])
            } else if s.starts_with("0x") {
                Some(&s[2..])
            } else {
                None
            }
        } {
            if let Ok(v) = u64::from_str_radix(hex, 16) {
                return Some(v);
            }
        } else {
            if let Ok(v) = u64::from_str_radix(s, 10) {
                return Some(v);
            }
        }
        None
    }

    // Parse `s` as either hexadecimal or decimal. Hex numbers of length exactly 8
    // will be sign extended
    pub fn parse_u64_se32<T: AsRef<str>>(s: &T) -> Option<u64> {
        let s: &str = s.as_ref().trim();
        if let Some(hex) = {
            if s.starts_with("$") {
                Some(&s[1..])
            } else if s.starts_with("0x") {
                Some(&s[2..])
            } else {
                None
            }
        } {
            if let Ok(mut v) = u64::from_str_radix(hex, 16) {
                if hex.len() == 8 {
                    v = (v as i32) as u64;
                }
                return Some(v);
            }
        } else {
            if let Ok(v) = u64::from_str_radix(s, 10) {
                return Some(v);
            }
        }
        None
    }

    pub fn parse_u64_float<F: std::str::FromStr, T: AsRef<str>>(s: &T) -> Option<F> {
        let s: &str = s.as_ref().trim();
        if let Ok(float) = s.parse::<F>() {
            Some(float)
        } else {
            None
        }
    }

    // take an imgui::Ui for a required context to exist
    pub fn good_separator(_ui: &imgui::Ui) {
        unsafe { 
            imgui::sys::igSeparatorEx(imgui::sys::ImGuiSeparatorFlags_Horizontal as i32); 
        }
    }

    pub fn average_colors(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
        [
            (a[0] + b[0]) / 2.0,
            (a[1] + b[1]) / 2.0,
            (a[2] + b[2]) / 2.0,
            (a[3] + b[3]) / 2.0,
        ]
    }

    
    pub fn interpolate_colors(zero: [f32; 4], one: [f32; 4], t: f32) -> [f32; 4] {
        [
            zero[0] * (1.0 - t) + one[0] * t,
            zero[1] * (1.0 - t) + one[1] * t,
            zero[2] * (1.0 - t) + one[2] * t,
            zero[3] * (1.0 - t) + one[3] * t,
        ]
    }
}

pub trait GameWindow {
    // return false to have the window closed
    fn render_ui(&mut self, ui: &imgui::Ui) -> bool;
}
