use std::collections::HashMap;
use std::mem;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use cgmath::prelude::*;
use cgmath::{Matrix4, Matrix3, Vector4, Vector3};

use crate::*;

#[derive(Debug, Clone)]
pub enum HleRenderCommand {
    Noop,
    DefineColorImage { bpp: u8, width: u16, framebuffer_address: u32 },
    DefineDepthImage { framebuffer_address: u32 },
    VertexData(Vec<Vertex>),
    IndexData(Vec<u16>),
    MatrixData(Vec<MatrixState>),
    ColorCombinerStateData(Vec<ColorCombinerState>),
    LightStateData(Vec<LightState>),
    FogStateData(Vec<FogState>),
    UpdateTexture(Arc<RwLock<MappedTexture>>),
    RenderPass(RenderPassState),
    Sync,
}

pub type HleCommandBuffer = atomicring::AtomicRingBuffer<HleRenderCommand>;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum RenderPassType {
    #[default]
    DrawTriangles,
    FillRectangles,
}

// There are actually a lot of variations within the RSP ucodes that share a common GBI. 
#[derive(PartialEq, Debug)]
enum HleRspSoftwareVersion {
    Uninitialized,
    Unknown,
    S3DEX2, // RSP SW 2.0X (SM64)
    //.F3DEX1, // Star Fox 64
    F3DEX2, // Fast3D 2.0 (Zelda)
}


// An F3DZEX vertex has two forms, but only varies with the last color value
// being either used for prelit color or the normal value
#[repr(C)]
#[derive(Copy,Clone,Default,Debug)]
pub struct F3DZEX2_Vertex {
    pub position: [i16; 3],
    pub reserved: u16,
    pub texcoord: [i16; 2],
    pub color_or_normal: [u8; 4],
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub view_position: [f32; 4],
    pub color        : [f32; 4],
    pub view_normal  : [f32; 3],
    pub tex_coords   : [f32; 2],
    pub tex_coords1  : [f32; 2],
    pub tex_params   : [f32; 4],
    pub tex_params1  : [f32; 4],
    pub maskshift    : u32,
    pub maskshift1   : u32,
    pub flags        : u32,
}

impl Vertex {
    pub const fn const_default() -> Self {
        Self {
            view_position: [0.0, 0.0, 0.0, 1.0],
            color        : [0.0, 0.0, 0.0, 1.0],
            view_normal  : [1.0, 0.0, 0.0],
            tex_coords   : [0.0, 0.0],
            tex_coords1  : [0.0, 0.0],
            tex_params   : [0.0, 1.0, 0.0, 1.0],
            tex_params1  : [0.0, 1.0, 0.0, 1.0],
            maskshift    : 0,
            maskshift1   : 0,
            flags        : 0,
        }
    }
}

impl Default for Vertex {
    fn default() -> Self {
        Self::const_default()
    }

}

pub struct VertexFlags;
impl VertexFlags {
    pub const TEXTURED                 : u32 = 1u32 << 0;
    pub const LINEAR_FILTER            : u32 = 1u32 << 1;
    pub const TEXMODE_S_SHIFT          : u32 = 2; // 00 = nomirror+wrap, 01 = mirror, 10 = clamp, 11 = n/a
    pub const TEXMODE_T_SHIFT          : u32 = 4; // 00 = nomirror+wrap, 01 = mirror, 10 = clamp, 11 = n/a
    pub const TEXMODE_S1_SHIFT         : u32 = 6; // 00 = nomirror+wrap, 01 = mirror, 10 = clamp, 11 = n/a
    pub const TEXMODE_T1_SHIFT         : u32 = 8; // 00 = nomirror+wrap, 01 = mirror, 10 = clamp, 11 = n/a
    pub const LIT                      : u32 = 1u32 << 10;
    pub const TWO_CYCLE                : u32 = 1u32 << 11;
    pub const ZMODE_SHIFT              : u32 = 12; // 00 = Opaque, 01 = Interpenetrating, 10 = Translucent, 11 = Decal
    pub const ALPHA_COMPARE_MODE_SHIFT : u32 = 14; // 00 = None, 01 = Threshold, 11 = Dither
    pub const TEX_EDGE                 : u32 = 16;
    // next VERTEX_FLAG at "<< 16"
}

// TODO at some point other ucode might support different types of lighting?
#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LightState {
    pub lights : [[f32; 4]; LightState::NUM_LIGHTS], // w coordinate != 0.0 indicates light is enabled
    pub colors : [[f32; 4]; LightState::NUM_LIGHTS], // alpha component not used but present for alignment
    
    pub _alignment: [u64; 4],
}

impl LightState {
    pub const NUM_LIGHTS: usize = 7;
}

impl Default for LightState {
    fn default() -> Self {
        Self {
            lights: [[0.0; 4]; LightState::NUM_LIGHTS],
            colors: [[0.0; 4]; LightState::NUM_LIGHTS],

            _alignment: [0; 4],
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FogState {
    pub color : [f32; 4],
    pub multiplier  : f32,  // multiplier == 0.0 && offset == 0.0 => fog is disabled
    pub offset      : f32,
    // need to align to 256 bytes, uhg
    pub _alignment  : [u64; 29],
}

impl Default for FogState {
    fn default() -> Self {
        FogState {
            color     : [0.0; 4],
            multiplier: 0.0,
            offset    : 0.0,
            _alignment: [0; 29],
        }
    }
}

#[derive(Default)]
struct DLStackEntry {
    dl: Vec<u32>,
    pc: u32,
    base_address: u32,
}

#[derive(Clone,Debug,Default)]
pub struct RenderPassState {
    pub pass_type: Option<RenderPassType>,

    // current render targets for tris
    pub color_buffer: Option<u32>,
    pub depth_buffer: Option<u32>,

    pub clear_color: Option<[f32; 4]>,
    pub clear_depth: bool,

    // draw_list an array of triangle lists, where each triangle shares common state
    pub draw_list: Vec<TriangleList>,

    // reason this render pass was terminated
    pub reason: Option<String>,
}

#[derive(Clone,Debug,Default,PartialEq)]
pub struct Viewport {
    pub x: f32, 
    pub y: f32, 
    pub z: f32, 
    pub w: f32, 
    pub h: f32, 
    pub d: f32,
}

#[derive(Clone,Debug,Default)]
pub struct TriangleList {
    // viewport used in this draw call
    pub viewport: Option<Viewport>,

    // pipeline state for this draw call
    pub depth_compare_enable: bool,
    pub depth_write: bool,

    // textures for this draw call
    pub mapped_texture_index: Option<u32>,
    pub mapped_texture_index1: Option<u32>,

    // uniform buffer indices to use for this draw list
    // only valid when num_indices > 0 (i.e., when the first triangle is added) but defaults to 0
    pub matrix_index: u32,
    pub color_combiner_state_index: u32,
    pub light_state_index: Option<u32>,
    pub fog_state_index: u32,

    // start index and number of indices
    pub start_index: u32,
    pub num_indices: u32,
}

pub struct Hle {
    comms: SystemCommunication,

    hle_command_buffer: Arc<HleCommandBuffer>,
    software_version: HleRspSoftwareVersion,
    software_crc: u32,

    dl_stack: Vec<DLStackEntry>,
    segments: [u32; 16],

    geometry_mode: u32,

    // RDP Other Modes
    other_modes: OtherModes,

    // we have a temporary space for vertices that have not had their texcoords transformed yet
    // these vertices won't be transfered to the renderer if they never get used in a draw call
    vertices_internal: Vec<Vertex>,          // non-transformed, not rendered
    vertices: Vec<Vertex>,                   // post texture transform, rendered
    vertex_stack: [u16; 32],                 // F3DZEX has storage for 32 vertices
    indices: Vec<u16>,

    matrices: Vec<MatrixState>,              // all the unique matrices in the DL
    matrix_stack: Vec<Matrix4<f32>>,         // modelview only, not multiplied by proj
    current_projection_matrix: Matrix4<f32>, // current projection matrix
    current_modelview_matrix: Matrix4<f32>,  // current modelview matrix (equal to the top of the matrix stack or identity)
    current_mv_inverse_matrix: Option<Matrix3<f32>>, // inverse of modelview, only Some() when actually needed

    matrix_index_override: Option<u32>,      // set to force a specific matrix index rather than use current_matrix
    disable_depth_override: Option<()>,      // set to force no depth on the next draw call

    current_viewport: Option<Viewport>,

    color_images: HashMap<u32, HleRenderCommand>,
    depth_images: HashMap<u32, HleRenderCommand>,
    clear_images: HashMap<u32, [f32; 4]>,
    current_color_image: Option<u32>,
    current_color_image_format: Option<HleRenderCommand>,
    current_depth_image: Option<u32>,

    // list of render_passes. the last of the array is always the current render pass
    render_passes: Vec<RenderPassState>,
    num_draws: u32,
    num_tris: u32,
    num_texels: u32,
    total_texture_data: u32,

    fill_color: u32,

    rdp_half_hi: u32,
    rdp_half_lo: u32,

    current_color_combiner_state: ColorCombinerState,
    color_combiner_states: Vec<ColorCombinerState>,
    color_combiner_uses_texel1: bool,

    // texture state
    tex: TextureState,

    // mapped textures
    // when a new texture overflows in y, we start a new texture
    mapped_textures: Vec<Arc<RwLock<MappedTexture>>>,

    // lighting
    num_lights: u32,
    ambient_light_color: [f32; 3],
    current_light_state: LightState,
    light_states: Vec<LightState>,

    // fog
    current_fog_state: FogState,
    fog_states: Vec<FogState>,

    command_table: [DLCommand; 256],
    command_address: u32,
    command: u64,
    command_op: u8,
    command_words: u32,
    command_prefix: String,

    // copied tweakables
    tweakables: Tweakables,
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MatrixState {
    pub projection: [[f32; 4]; 4],
    // need some alignment
    pub _alignment: [u64; 24],
}

impl Default for MatrixState {
    fn default() -> Self {
        Self {
            projection: Matrix4::identity().into(),
            _alignment: [0; 24],
        }
    }
}

#[derive(Debug,Default)]
pub struct MappedTexture {
    pub id         : u32,
    pub data       : Vec<u8>,
    pub width      : usize,
    pub height     : usize,
    pub alloc_x    : u32,
    pub alloc_y    : u32,
    pub alloc_max_h: u32,
    pub dirty      : bool,
}

struct TextureState {
    tmem: [u32; 1024], // 4KiB of TMEM

    // texture map cache
    mapped_texture_cache: HashMap<u64, (u32, u32, u32)>,

    // gSPTexture
    s_scale: f32,  // s coordinate texture scale
    t_scale: f32,  // t coordinate texture scale
    mipmaps: u8,   // number of mipmap levels
    tile   : u8,   // currently selected tile in TMEM
    enabled: bool, // enable/disable texturing on the current primitive

    // gDPSetTextureImage
    format : u8,   // G_IM_FMT_*
    size   : u8,   // G_IM_SIZ_*
    width  : u16,  // 1..4096
    address: u32,  // physical address in DRAM

    // gDPSetTile
    rdp_tiles: [RdpTileState; 8],
}

#[derive(Default,Copy,Clone)]
struct RdpTileState {
    format : u8,  // G_IM_FMT_*
    size   : u8,  // G_IM_SIZ_*
    line   : u16, // number of qwords per image row
    tmem   : u16, // location of texture in tmem in qwords (actual address is tmem * sizeof(u64))
    palette: u8,  // selected palette
    clamp_s: u8,  // s,t clamp
    clamp_t: u8,
    mask_s : u8,  // s,t bit mask
    mask_t : u8, 
    shift_s: u8,  // s,t shift after perspective correction
    shift_t: u8,
    ul     : (f32, f32), // upper-left coordinate
    lr     : (f32, f32), // lower-right coordinate, for wrapping

    // texture_index, texture_x, texture_y
    mapped_coordinates: Option<(u32, u32, u32)>,
    // true if the CC uses TEXEL1 in any input and we need to set a second texture
    // if so, the tile to use is always the next tile.
    has_texel1: bool,
}

impl TextureState {
    fn new() -> Self {
        Self {
            tmem: [0u32; 1024],
            mapped_texture_cache: HashMap::new(),
            s_scale: 0.0, t_scale: 0.0, mipmaps: 0, tile: 0,
            enabled: false, format: 0, size: 0, width: 0,
            address: 0, 
            rdp_tiles: [RdpTileState::default(); 8],
        }
    }

    fn reset(&mut self) {
        // basically, leave tmem and texture cache alone
        self.s_scale = 0.0;
        self.t_scale = 0.0;
        self.mipmaps = 0;
        self.tile = 0;
        self.enabled = false;
        self.format = 0;
        self.size = 0;
        self.width = 0;
        self.address = 0;
        self.rdp_tiles = [RdpTileState::default(); 8];
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ColorCombinerState {
    pub color1_source: u32, //[u8; 4], // a, b, c, d
    pub alpha1_source: u32, //[u8; 4],
    pub color2_source: u32, //[u8; 4], // a, b, c, d
    pub alpha2_source: u32, //[u8; 4],
    pub prim_color   : [f32; 4],
    pub env_color    : [f32; 4],
    pub blend_color  : [f32; 4],
    pub lodfrac      : f32,
    pub _unused      : f32, // only present for alignment, this space can be used in the future

    // alignment to 256 bytes
    pub _alignment   : [u64; 23],
}

impl ColorCombinerState {
    fn new() -> Self {
        Self {
            color1_source: 0, //[0, 0, 0, 0],
            alpha1_source: 0, //[0, 0, 0, 0],
            color2_source: 0, //[0, 0, 0, 0],
            alpha2_source: 0, //[0, 0, 0, 0],
            prim_color   : [0.0, 0.0, 0.0, 0.0],
            env_color    : [0.0, 0.0, 0.0, 0.0],
            blend_color  : [0.0, 0.0, 0.0, 0.0],
            lodfrac      : 0.0,
            _unused      : 0.0,
            _alignment   : [0; 23],
        }
    }
}

pub const TEXSIZE_WIDTH: usize = 512;
pub const TEXSIZE_HEIGHT: usize = 512;

const DL_FETCH_SIZE: u32 = 168; // dunno why 168 is used so much on LoZ, but let's use it too?

type DLCommand = fn(&mut Hle) -> ();

impl Hle {
    pub fn new(comms: SystemCommunication, hle_command_buffer: Arc<HleCommandBuffer>) -> Self {
        // keep a default vertex in self.vertices_internal in case anything draws before calling G_VTX
        let vertices_internal = vec![Vertex { color: [1.0, 0.0, 0.0, 1.0], ..Default::default() }];

        let mut ret = Self {
            comms: comms,

            hle_command_buffer: hle_command_buffer,
            software_version: HleRspSoftwareVersion::Uninitialized,
            software_crc: 0,

            dl_stack: vec![],
            segments: [0u32; 16],

            geometry_mode: 0,

            other_modes: OtherModes::default(),

            vertices_internal: vertices_internal,
            vertices: vec![],
            vertex_stack: [0; 32],
            indices: vec![],

            matrices: vec![],
            matrix_stack: vec![],
            current_projection_matrix: Matrix4::identity(),
            current_modelview_matrix: Matrix4::identity(),
            current_mv_inverse_matrix: None,
            matrix_index_override: None,
            disable_depth_override: None,

            current_viewport: None,

            color_images: HashMap::new(),
            depth_images: HashMap::new(),
            clear_images: HashMap::new(),
            current_color_image: None,
            current_color_image_format: None,
            current_depth_image: None,

            render_passes: vec![],
            num_draws: 0,
            num_tris: 0,
            num_texels: 0,
            total_texture_data: 0,

            fill_color: 0,

            rdp_half_hi: 0,
            rdp_half_lo: 0,

            current_color_combiner_state: ColorCombinerState::new(),
            color_combiner_states: vec![],
            color_combiner_uses_texel1: false,

            tex: TextureState::new(),

            mapped_textures: vec![],

            num_lights: 0,
            ambient_light_color: [1.0; 3],
            current_light_state: LightState::default(),
            light_states: vec![],

            current_fog_state: FogState::default(),
            fog_states: vec![FogState::default()],

            command_table: [Hle::handle_unknown; 256],
            command_address: 0,
            command: 0,
            command_op: 0,
            command_words: 0,
            command_prefix: String::new(),

            tweakables: Tweakables::default(),
        };

        ret.new_texture_cache();
        ret
    }

    fn new_texture_cache(&mut self) {
        let mut texdata = Vec::new();
        texdata.resize(std::mem::size_of::<u32>()*TEXSIZE_WIDTH*TEXSIZE_HEIGHT, 0);
        // At some point this for loop should go away (for performance?)
        // and we can either all 0 or all 0xFF in the resize() call
        for v in texdata.chunks_mut(4) {
            v[1] = 0x7F;
            v[3] = 0xFF;
        }

        let index = self.mapped_textures.len();

        let mt = MappedTexture {
            id    : index as u32,
            data  : texdata,
            width : TEXSIZE_WIDTH,
            height: TEXSIZE_HEIGHT,
            ..Default::default()
        };

        self.mapped_textures.push(Arc::new(RwLock::new(mt)));
    }

    fn reset_display_list(&mut self) {
        self.dl_stack.clear();
        self.segments = [0u32; 16];
        self.vertices_internal.truncate(1); // keep element 0 in the list
        self.vertices.clear();
        self.vertex_stack = [0; 32];
        self.indices.clear();
        self.matrices.clear();
        self.matrix_stack.clear();
        self.geometry_mode = 1 << 19; // G_CLIPPING is on by default
        self.matrix_index_override = None;
        self.disable_depth_override = None;
        self.current_projection_matrix = Matrix4::identity();
        self.current_modelview_matrix = Matrix4::identity();
        self.current_mv_inverse_matrix = None;
        self.color_images.clear();
        self.depth_images.clear();
        self.clear_images.clear();
        self.render_passes.clear();
        self.num_draws = 0;
        self.num_tris = 0;
        self.num_texels = 0;
        self.total_texture_data = 0;
        self.fill_color = 0;
        self.color_combiner_states.clear();
        self.color_combiner_uses_texel1 = false;
        self.tex.reset(); // TODO this probably shouldn't be done, as it looks like
                          // RDP state isn't reset from frame to frame

        // clear dirty flag on all textures
        for mtl in &self.mapped_textures {
            mtl.write().unwrap().dirty = false;
        }

        self.num_lights = 0;
        self.ambient_light_color = [1.0; 3];
        self.current_light_state = LightState::default();
        self.light_states.clear();

        self.current_fog_state = FogState::default();
        self.fog_states.truncate(1);

        self.current_viewport = None;

        // keep color and depth images from pass to pass
        //self.current_color_image = None;
        //self.current_color_image_format = None;
        //self.current_depth_image = None;

        // set matrix 0 to be an ortho projection
        self.matrices.push(MatrixState {
            projection: cgmath::ortho(-1.0, 1.0, -1.0, 1.0, 0.1, 100.0).into(),
            ..Default::default()
        });

        // always have a render pass and draw list started
        self.next_render_pass(None);

        // copy over tweakables once per frame
        self.tweakables = *self.comms.tweakables.read().unwrap();
    }

    fn detect_software_version(&mut self, ucode_address: u32) -> bool {
        let ucode = self.load_from_rdram(ucode_address, 4 * 1024);

        self.software_crc = 0;

        // the CRCs are based on the first 3k of ucode
        for i in 0..(3072 >> 2) {
            self.software_crc = self.software_crc.wrapping_add(ucode[i]);
        }

        // TODO: need a way to organize these into a data or config file
        self.software_version = match self.software_crc {
            0xB54E7F93 => HleRspSoftwareVersion::S3DEX2, // Nintendo 64 demos
            0x3A1CBAC3 => HleRspSoftwareVersion::S3DEX2, // Super Mario 64 (U)
            0x3F7247FB => HleRspSoftwareVersion::S3DEX2, // Tetrisphere

            0xAD0A6292 => HleRspSoftwareVersion::F3DEX2, // Nintendo 64 devkit f3dex2
            0x22099872 => HleRspSoftwareVersion::F3DEX2, // Zelda MM Release
            0x21F91874 => HleRspSoftwareVersion::F3DEX2, // Zelda OoT Debug
            0x5D3099F1 => HleRspSoftwareVersion::F3DEX2, // Zelda OoT Release
            0xC901CE73 => HleRspSoftwareVersion::F3DEX2, // More demos?
            0x21F91834 => HleRspSoftwareVersion::F3DEX2, // Paper Mario, (NuSys?), 
            0xBC45382E => HleRspSoftwareVersion::F3DEX2, // Kirby 64
            0x65201989 => HleRspSoftwareVersion::F3DEX2, // Gauntlet Legends


            _ => HleRspSoftwareVersion::Unknown,
        };

        info!(target: "HLE", "{:?} detected", self.software_version);

        match self.software_version {
            HleRspSoftwareVersion::S3DEX2 => {
                // basically none of these are tested
                //self.command_table[0x00] = Hle::handle_spnoop;
                self.command_table[0x01] = Hle::handle_mtx;
                self.command_table[0x03] = Hle::handle_movemem00;
                self.command_table[0x04] = Hle::handle_vtx;
                self.command_table[0x06] = Hle::handle_displaylist;
                //self.command_table[0xC0] = Hle::handle_noop;

                let base = (-0x41i8 as u8) as usize;
                self.command_table[base-0] = Hle::handle_tri1;
                self.command_table[base-2] = Hle::handle_popmtx;
                self.command_table[base-3] = Hle::handle_moveword00;
                self.command_table[base-4] = Hle::handle_texture;
                self.command_table[base-5] = Hle::handle_setothermode_h;
                self.command_table[base-6] = Hle::handle_setothermode_l;
                self.command_table[base-7] = Hle::handle_enddl;
                self.command_table[base-8] = Hle::handle_setgeometrymode;
                self.command_table[base-9] = Hle::handle_cleargeometrymode;
                self.command_table[base-11] = Hle::handle_rdphalf_1;
                self.command_table[base-12] = Hle::handle_rdphalf_2;
                //self.command_table[base-14] = Hle::handle_tri2;

                // RDP commands
                self.command_table[0xFF] = Hle::handle_setcimg;
                self.command_table[0xFE] = Hle::handle_setzimg;
                self.command_table[0xFD] = Hle::handle_settimg;
                self.command_table[0xFC] = Hle::handle_setcombine;
                self.command_table[0xFB] = Hle::handle_setenvcolor;
                self.command_table[0xFA] = Hle::handle_setprimcolor;
                self.command_table[0xF9] = Hle::handle_setblendcolor;
                self.command_table[0xF8] = Hle::handle_setfogcolor;
                self.command_table[0xF7] = Hle::handle_setfillcolor;
                self.command_table[0xF6] = Hle::handle_fillrect;
                self.command_table[0xF5] = Hle::handle_settile;
                self.command_table[0xF4] = Hle::handle_loadtile;
                self.command_table[0xF3] = Hle::handle_loadblock;
                self.command_table[0xF2] = Hle::handle_settilesize;
                self.command_table[0xF0] = Hle::handle_loadlut;
                //self.command_table[0xEF] = Hle::handle_rdpsetothermode;
                //self.command_table[0xEE] = Hle::handle_setprimdepth;
                self.command_table[0xED] = Hle::handle_setscissor;
                //self.command_table[0xEC] = Hle::handle_setconvert;
                //self.command_table[0xEB] = Hle::handle_setkeyr;
                //self.command_table[0xEA] = Hle::handle_setkeygb;
                self.command_table[0xE9] = Hle::handle_rdpfullsync;
                self.command_table[0xE8] = Hle::handle_rdptilesync;
                self.command_table[0xE7] = Hle::handle_rdppipesync;
                self.command_table[0xE6] = Hle::handle_rdploadsync;
                self.command_table[0xE4] = Hle::handle_texrect;
            },

            HleRspSoftwareVersion::F3DEX2 => {
                self.command_table[0x00] = Hle::handle_noop;
                self.command_table[0x01] = Hle::handle_vtx;
                // 0x02 - modify vtx
                self.command_table[0x03] = Hle::handle_culldl;
                self.command_table[0x04] = Hle::handle_branch_z;
                self.command_table[0x05] = Hle::handle_tri1;
                self.command_table[0x06] = Hle::handle_tri2;
                self.command_table[0x07] = Hle::handle_quad;
                self.command_table[0x0B] = Hle::handle_obj_rendermode;

                self.command_table[0xD7] = Hle::handle_texture;
                self.command_table[0xD8] = Hle::handle_popmtx;
                self.command_table[0xD9] = Hle::handle_geometrymode;
                self.command_table[0xDA] = Hle::handle_mtx;
                self.command_table[0xDB] = Hle::handle_moveword02;
                self.command_table[0xDC] = Hle::handle_movemem02;
                self.command_table[0xDD] = Hle::handle_load_ucode;
                self.command_table[0xDE] = Hle::handle_displaylist;
                self.command_table[0xDF] = Hle::handle_enddl;
                self.command_table[0xE2] = Hle::handle_setothermode_l;
                self.command_table[0xE3] = Hle::handle_setothermode_h;

                // RDP commands
                self.command_table[0xFF] = Hle::handle_setcimg;
                self.command_table[0xFE] = Hle::handle_setzimg;
                self.command_table[0xFD] = Hle::handle_settimg;
                self.command_table[0xFC] = Hle::handle_setcombine;
                self.command_table[0xFB] = Hle::handle_setenvcolor;
                self.command_table[0xFA] = Hle::handle_setprimcolor;
                self.command_table[0xF9] = Hle::handle_setblendcolor;
                self.command_table[0xF8] = Hle::handle_setfogcolor;
                self.command_table[0xF7] = Hle::handle_setfillcolor;
                self.command_table[0xF6] = Hle::handle_fillrect;
                self.command_table[0xF5] = Hle::handle_settile;
                self.command_table[0xF4] = Hle::handle_loadtile;
                self.command_table[0xF3] = Hle::handle_loadblock;
                self.command_table[0xF2] = Hle::handle_settilesize;
                self.command_table[0xF1] = Hle::handle_rdphalf_2;
                self.command_table[0xF0] = Hle::handle_loadlut;
                self.command_table[0xEF] = Hle::handle_rdpsetothermode;
                self.command_table[0xEE] = Hle::handle_setprimdepth;
                self.command_table[0xED] = Hle::handle_setscissor;
                self.command_table[0xEC] = Hle::handle_setconvert;
                self.command_table[0xEB] = Hle::handle_setkeyr;
                self.command_table[0xEA] = Hle::handle_setkeygb;
                self.command_table[0xE9] = Hle::handle_rdpfullsync;
                self.command_table[0xE8] = Hle::handle_rdptilesync;
                self.command_table[0xE7] = Hle::handle_rdppipesync;
                self.command_table[0xE6] = Hle::handle_rdploadsync;
                self.command_table[0xE4] = Hle::handle_texrect;
                self.command_table[0xE1] = Hle::handle_rdphalf_1;
                self.command_table[0xE0] = Hle::handle_spnoop;
            },

            HleRspSoftwareVersion::Unknown => {},
            _ => {
                unimplemented!("unsupported software {:?}", self.software_version);
            },
        };

        !(self.software_version == HleRspSoftwareVersion::Unknown)
    }

    pub fn process_display_list(&mut self, dl_start: u32, dl_length: u32, ucode_address: u32) {
        trace!(target: "HLE", "processing display list from ${:08X}, length {} bytes", dl_start, dl_length);
        
        if let HleRspSoftwareVersion::Uninitialized = self.software_version {
            if !self.detect_software_version(ucode_address) { 
                unimplemented!("unknown RSP graphics task microcode (CRC 0x{:08X})", self.software_crc);
            }
        }

        self.reset_display_list();

        let cur_dl = DLStackEntry {
            dl: self.load_from_rdram(dl_start, dl_length).to_vec(),
            base_address: dl_start,
            ..Default::default()
        };
        self.dl_stack.push(cur_dl);

        while self.dl_stack.len() > 0 {
            let addr = self.current_display_list_address();
            let cmd = self.next_display_list_command();
            self.process_display_list_command(addr, cmd);
        }

        // upload dirty textures
        let dirty_textures: Vec<_> = self.mapped_textures.iter().map(|mtl| {
            let mt = mtl.read().unwrap();
            if mt.dirty {
                Some(HleRenderCommand::UpdateTexture(mtl.clone()))
            } else {
                None
            }
        }).collect();

        for dt in dirty_textures {
            if let Some(cmd) = dt {
                self.send_hle_render_command(cmd);
            }
        }

        // finalize current render pass
        self.finalize_render_pass(Some(format!("end of display list")));

        debug!(target: "HLE", "found {} matrices, {} vertices, {} indices, {} render passes, {} draw calls, \
                               {} total tris, {} texels read ({} bytes), {} cc states, {} light states", 
               self.matrices.len(), self.vertices.len(), self.indices.len(), self.render_passes.len(), self.num_draws, 
               self.num_tris, self.num_texels, self.total_texture_data, self.color_combiner_states.len(),
               self.light_states.len());

        // if there's nothing to draw, just Sync with the renderer and return
        if self.render_passes.len() == 0 {
            debug!(target: "HLE", "no render passes created, nothing to do");
            self.send_hle_render_command(HleRenderCommand::Sync);
            return;
        }

        // depth buffers are cleared by being used as color images, so remove them from being
        // created as actual color targets
        let depth_images = std::mem::replace(&mut self.depth_images, HashMap::new());
        for (key, buffer_cmd) in depth_images {
            if self.color_images.contains_key(&key) {
                self.color_images.remove(&key);
            }
            self.send_hle_render_command(buffer_cmd);
        }

        // create the color buffers
        let color_images = std::mem::replace(&mut self.color_images, HashMap::new());
        for (_, buffer_cmd) in color_images {
            self.send_hle_render_command(buffer_cmd);
        }

        // upload verts
        let vertices = std::mem::replace(&mut self.vertices, vec![]);
        self.send_hle_render_command(HleRenderCommand::VertexData(vertices));

        // upload indices
        let indices = std::mem::replace(&mut self.indices, vec![]);
        self.send_hle_render_command(HleRenderCommand::IndexData(indices));

        // upload matrices
        let matrices = std::mem::replace(&mut self.matrices, vec![]);
        self.send_hle_render_command(HleRenderCommand::MatrixData(matrices));

        // upload color combiner states
        let cc_states = std::mem::replace(&mut self.color_combiner_states, vec![]);
        self.send_hle_render_command(HleRenderCommand::ColorCombinerStateData(cc_states));

        // upload light states
        let light_states = std::mem::replace(&mut self.light_states, vec![]);
        self.send_hle_render_command(HleRenderCommand::LightStateData(light_states));

        // upload fog states
        let fog_states = std::mem::replace(&mut self.fog_states, vec![FogState::default()]);
        self.send_hle_render_command(HleRenderCommand::FogStateData(fog_states));

        // run each render pass
        for i in 0..self.render_passes.len() {
            let rp = std::mem::replace(&mut self.render_passes[i], RenderPassState::default());
            //println!("sending RP that was terminated because: {}", rp.reason.as_ref().unwrap_or(&String::from("None")));
            //println!("rp{} pass_type={:?} rp.color_buffer={:?} depth_buffer={:?} rp.clear_depth={:?} draw_list.len={}", 
            //         i, rp.pass_type, rp.color_buffer, rp.depth_buffer, rp.clear_depth, rp.draw_list.len());
            self.send_hle_render_command(HleRenderCommand::RenderPass(rp));
        }

        self.send_hle_render_command(HleRenderCommand::Sync);
        //println!("sync");
    }

    fn current_display_list_address(&mut self) -> u32 {
        let cur = self.dl_stack.last().unwrap();
        cur.base_address + cur.pc
    }

    fn next_display_list_command(&mut self) -> u64 {
        let needs_data = {
            let cur = self.dl_stack.last().unwrap();
            cur.pc == cur.dl.len() as u32
        };

        if needs_data {
            let load_address = {
                let cur = self.dl_stack.last().unwrap();
                cur.base_address + cur.pc * 4
            };

            let dl = self.load_from_rdram(load_address, DL_FETCH_SIZE);
            {
                let cur = self.dl_stack.last_mut().unwrap();
                cur.dl.extend_from_slice(&dl);
            }
        }

        let cur = self.dl_stack.last_mut().unwrap();
        let r = ((cur.dl[cur.pc as usize] as u64) << 32) | (cur.dl[cur.pc as usize + 1] as u64);
        cur.pc += 2;
        r
    }

    fn load_from_rdram(&self, start: u32, length: u32) -> Vec<u32> {
        let access = self.comms.rdram.read();
        let rdram: &[u32] = access.as_deref().unwrap().as_ref().unwrap();
        let length = ((length + 7) & !7) as usize;
        let start = ((start & !0x8000_0000) >> 2) as usize;
        let end   = start + (length >> 2);
        if end > rdram.len() {
            panic!("read from ${:08X} length {} reads outside of RDRAM", start << 2, length);
        }
        rdram[start..end].into()
    }

    // read memory until a \0 is encountered, and decode into a printable string
    // ideally block_size would be larger than the expected string length, but
    // not too large as to make the memory copy slow
    fn load_string(&mut self, mut start: u32, block_size: u32) -> String {
        let mut v: Vec<u8> = Vec::with_capacity(block_size as usize);

        if (start & 0x03) != 0 {
            warn!(target: "HLE", "load_string on unaligned address ${:08X}", start);
        }

        start = (start + 3) & !3;
        let mut skip_one = false;//(start & 0x04) == 0x04;

        loop {
            let block = self.load_from_rdram(start, block_size);
            for e in block {
                if skip_one {
                    skip_one = false;
                    continue;
                }

                let mut c = ((e >> 24) & 0xFF) as u8;
                if c != 0 {
                    v.push(c);
                    c = ((e >> 16) & 0xFF) as u8;
                    if c != 0 {
                        v.push(c);
                        c = ((e >> 8) & 0xFF) as u8;
                        if c != 0 {
                            v.push(c);
                            c = (e & 0xFF) as u8;
                            if c != 0 {
                                v.push(c);
                            }
                        }
                    }
                }

                if c == 0 {
                    let (res, _enc, _errors) = encoding_rs::EUC_JP.decode(&v);
                    return res.to_string();
                }
            }

            start += block_size;
        }
    }


    fn handle_unknown(&mut self) {
        unimplemented!("unimplemented DL command ${:02X}", self.command_op);
    }

    fn handle_rdphalf_1(&mut self) { // G_RDPHALF_1 (S3DEX2, F3DEX2)
        let wordhi = self.command as u32;
        trace!(target: "HLE", "{} gsDPWord(0x{:08X}, wordlo);", self.command_prefix, wordhi);
        self.rdp_half_hi = wordhi;
    }

    fn handle_rdphalf_2(&mut self) { // G_RDPHALF_2 (S3DEX2, F3DEX2)
        let wordlo = self.command as u32;
        trace!(target: "HLE", "{} gsDPWord(wordhi, 0x{:08X});", self.command_prefix, wordlo);
        self.rdp_half_lo = wordlo;
    }

    fn handle_spnoop(&mut self) { // G_SPNOOP
        trace!(target: "HLE", "{} gsSPNoOp()", self.command_prefix);
    }

    fn handle_noop(&mut self) { // G_NOOP
        let addr = (self.command & 0xFFFF_FFFF) as u32;
        if addr != 0 {
            let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else {
                let segment = ((addr >> 24) & 0x0F) as usize;
                self.segments[segment] + (addr & 0x007F_FFFF)
            } & 0x007F_FFFF);

            let s = self.load_string(translated_addr, 64);
            trace!(target: "HLE", "{} gsDPNoOpString([0x{:08X}] \"{}\")", self.command_prefix, addr, s);
        } else {
            trace!(target: "HLE", "{} gsDPNoOp()", self.command_prefix);
        }
    }

    fn handle_mtx(&mut self) { // G_MTX (S3DEX2, F3DEX2)
        let (push, mul, proj) = match self.software_version {
            HleRspSoftwareVersion::S3DEX2 => {
                let params = (self.command >> 48) as u8;
                let push = (params & 0x04) != 0;
                let mul  = (params & 0x02) == 0;
                let proj = (params & 0x01) != 0;
                (push, mul, proj)
            },
            HleRspSoftwareVersion::F3DEX2 => {
                let params = (self.command >> 32) as u8;
                let push = (params & 0x01) == 0; // G_MTX_PUSH is inverted on F3DEX2
                let mul  = (params & 0x02) == 0;
                let proj = (params & 0x04) != 0; // true = projection matrix, false = modelview
                (push, mul, proj)
            },
            _ => todo!(),
        };

        let addr   = self.command as u32;

        let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x007F_FFFF)
        } & 0x007F_FFFF);


        let mut s = String::from("0");
        if push { s.push_str("|G_MTX_PUSH"); } else { s.push_str("|G_MTX_NOPUSH"); }
        if mul  { s.push_str("|G_MTX_MUL"); } else { s.push_str("|G_MTX_LOAD"); }
        if proj { s.push_str("|G_MTX_PROJECTION"); } else { s.push_str("|G_MTX_MODELVIEW"); }
        trace!(target: "HLE", "{} gsSPMatrix(0x{:08X}, {})", self.command_prefix, addr, s);

        let mtx_data = self.load_from_rdram(translated_addr, 64);

        // incoming data (0-9a-f are whole part, g-v are the fractional parts)
        // 00001111 22223333 44445555 66667777
        // 88889999 aaaabbbb ccccdddd eeeeffff
        // gggghhhh iiiijjjj kkkkllll mmmmnnnn
        // oooopppp qqqqrrrr sssstttt uuuuvvvv
        //
        // becomes
        //
        // [ 0000.gggg 1111.hhhh 2222.iiii 3333.jjjj ]
        // [ ..        ..        ..        ..        ]
        // [ cccc.ssss dddd.tttt eeee.uuuu ffff.vvvv ]
        let elem = |i, s| (((mtx_data[i] >> s) as i16) as f32) + (((mtx_data[i + 8] >> s) as u16) as f32) / 65536.0;
        let c0 = [elem(0, 16), elem(2, 16), elem(4, 16), elem(6, 16)];
        let c1 = [elem(0,  0), elem(2,  0), elem(4,  0), elem(6,  0)];
        let c2 = [elem(1, 16), elem(3, 16), elem(5, 16), elem(7, 16)];
        let c3 = [elem(1,  0), elem(3,  0), elem(5,  0), elem(7,  0)];
        let mut cgmat = Matrix4::from_cols(c0.into(), c1.into(), c2.into(), c3.into());
        //println!("data: {mtx_data:?}");
        trace!(target: "HLE", "matrix: {cgmat:?}");

        if proj {
            //println!("proj: {cgmat:?}");
            if mul {
                self.current_projection_matrix = cgmat * self.current_projection_matrix;
            } else {
                self.current_projection_matrix = cgmat;
            }
        } else {
            let count = self.matrix_stack.len();
            if mul && count > 0 {
                let other = &self.matrix_stack[count - 1];
                cgmat = cgmat * other;
            }

            if count == 0 || (push && count < 10) {
                self.matrix_stack.push(cgmat);
            } else {
                self.matrix_stack[count - 1] = cgmat;
            }

            self.current_modelview_matrix = cgmat.transpose();
            self.current_mv_inverse_matrix = None; // only compute this when necessary
        }
    }

    fn make_mv_inverse(&mut self) {
        let inv = self.current_modelview_matrix.invert().unwrap_or(Matrix4::identity()).transpose();
        self.current_mv_inverse_matrix = Some(Matrix3::from_cols(inv.x.truncate(), inv.y.truncate(), inv.z.truncate()));
    }

    fn handle_popmtx(&mut self) { // G_POPMTX
        let num = match self.software_version {
            HleRspSoftwareVersion::S3DEX2 => 1, // can only pop 1
            HleRspSoftwareVersion::F3DEX2 => ((self.command & 0xFFFF_FFFF) >> 6) as u32, // num / 64
            _ => todo!(),
        };
        trace!(target: "HLE", "{} gsSPPopMatrixN(G_MTX_MODELVIEW, {})", self.command_prefix, num);
        let new_size = self.matrix_stack.len().saturating_sub(num as usize);
        self.matrix_stack.truncate(new_size);

        self.current_modelview_matrix = (*self.matrix_stack.last().unwrap_or(&Matrix4::identity())).transpose();
        self.current_mv_inverse_matrix = None;
    }

    fn handle_moveword(&mut self, index: u8, offset: u16, data: u32) {
        match index {
            2 => { // G_MW_NUMLIGHTS
                match self.software_version {
                    HleRspSoftwareVersion::S3DEX2 => {
                        self.num_lights = std::cmp::min((((self.command & 0x7FFF_FFFF) >> 5) - 1) as u32, LightState::NUM_LIGHTS as u32);
                    },
                    HleRspSoftwareVersion::F3DEX2 => {
                        self.num_lights = data / 24;
                    },
                    _ => todo!(),
                }
                trace!(target: "HLE", "{} gsSPNumLights({})", self.command_prefix, self.num_lights);
            },

            6 => { // G_MW_SEGMENT
                trace!(target: "HLE", "{} gsSPSegment({}, 0x{:08X})", self.command_prefix, offset >> 2, data);
                self.segments[((offset >> 2) & 0x0F) as usize] = data;
            },

            8 => { // G_MW_FOG
                let multiplier = (((self.command & 0xFFFF_0000) >> 16) as i16) as f32 / 256.0;
                let offset     = (((self.command & 0x0000_FFFF)      ) as i16) as f32 / 256.0;
                trace!(target: "HLE", "{} gpSPFog(multiplier=${:04X} [{}], offset=${:04X} [{}])", self.command_prefix, (self.command & 0xFFFF_0000) >> 16, multiplier, self.command as u16, offset);
                self.current_fog_state.multiplier = multiplier;
                self.current_fog_state.offset = offset;
            },

            10 => { // G_MW_LIGHTCOL
                info!(target: "HLE", "G_MW_LIGHTCOL not implemented yet");
            },

            14 => { // G_MW_PERSPNORM
                trace!(target: "HLE", "{} gsSPPerspNormalize(0x{:08X})", self.command_prefix, data);
                // Perspective Normalization is ignored in HLE. While there are precision issues
                // with f32, we don't need the fixed point correction
            },

            _ => {
                trace!(target: "HLE", "{} gsMoveWd({}, 0x{:04X}, 0x{:08X})", self.command_prefix, index, offset, data);
            },
        };
    }

    fn handle_moveword00(&mut self) { // G_MOVEWORD (S3DEX2)
        let offset = ((self.command >> 40) & 0xFFFF) as u16;
        let index  = ((self.command >> 32) & 0xFF) as u8;
        let data   = self.command as u32;
        self.handle_moveword(index, offset, data)
    }

    fn handle_moveword02(&mut self) { // G_MOVEWORD (F3DEX2)
        let index  = ((self.command >> 48) & 0xFF) as u8;
        let offset = ((self.command >> 32) & 0xFFFF) as u16;
        let data   = self.command as u32;
        self.handle_moveword(index, offset, data)
    }

    fn handle_movemem(&mut self, index: u8, addr: u32, size: u64) { // G_MOVEMEM
        match index {
            2 => todo!("G_MV_MMTX"),
            6 => todo!("G_MV_PMTX"),
            8 => { // G_VIEWPORT
                let segment = (addr >> 24) as u8;
                let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else { 
                    (addr & 0x007F_FFFF) + self.segments[segment as usize]
                } & 0x007F_FFFF);

                let vp = self.load_from_rdram(translated_addr, size as u32);

                let xs = (vp[0] >> 16) as i16;
                let ys = vp[0] as i16;
                let zs = (vp[1] >> 16) as i16;
                let x_scale = (xs as f32) / 4.0;
                let y_scale = (ys as f32) / 4.0;
                let z_scale = (zs as f32) / 4.0;
                let xt = (vp[2] >> 16) as i16;
                let yt = vp[2] as i16;
                let zt = (vp[3] >> 16) as i16;
                let x_translate = (xt as f32) / 4.0;
                let y_translate = (yt as f32) / 4.0;
                let z_translate = (zt as f32) / 4.0;

                trace!(target: "HLE", "{} gsSPViewport(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);
                trace!(target: "HLE", "Viewport {{ vscale: [ {}, {}, {} ], vtrans: [ {}, {}, {} ] }}    ", x_scale, y_scale, z_scale, x_translate, y_translate, z_translate);

                let l = -x_scale + x_translate;
                let w = 2.0 * x_scale;
                let t = -y_scale + y_translate;
                let h = 2.0 * y_scale;

                self.current_viewport = Some(Viewport {
                    x: l, 
                    y: t,
                    z: 0.0,
                    w: w,
                    h: h,
                    d: 1.0,
                });

                // if the viewport changed from the current drawcall, we need to move on to a new draw call
                let vp = self.current_viewport.clone().unwrap();
                if self.current_triangle_list().viewport.as_ref().is_some_and(|v| *v != vp) {
                    self.next_triangle_list();
                }

                trace!(target: "HLE", "Converted viewport: {:?}", self.current_viewport);
            },

            10 | 0x86..=0x94 => { // G_MV_LIGHT
                let mut light_index = if index > 10 { 2 + ((index - 0x86) >> 1) } else { ((((self.command >> 40) & 0xFF) << 3) / 24) as u8 } as usize;
                trace!(target: "HLE", "{} gsSPLight(index={}, size={}, address=${:08X}) [num_lights={}]", self.command_prefix, light_index, size, addr, size / 16);

                if light_index < 2 {
                    debug!(target: "HLE", "TODO: G_MV_LIGHT with light_index < 2");
                    return;
                }
                light_index -= 2;

                if light_index >= LightState::NUM_LIGHTS {
                    warn!(target: "HLE", "invalid light index {}", light_index);
                    return;
                }

                let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else { 
                    let segment = (addr >> 24) as u8;
                    (addr & 0x007F_FFFF) + self.segments[segment as usize]
                } & 0x007F_FFFF);

                let light_data = self.load_from_rdram(translated_addr, size as u32);

                let num_lights = size / 16; // Light structure is 16 bytes long

                let norm_scale = |i: i8| if i < 0 { (i as f32) / 128.0 } else { (i as f32) / 127.0 };

                let cur_draw_is_lit = self.current_triangle_list().light_state_index.is_some();

                for li in 0..num_lights as usize {
                    // if the write index is the ambient light, done
                    if light_index == self.num_lights as usize { break; }

                    let data = &light_data[(16 * li) >> 2..];

                    let color = [
                        (((data[0] >> 24) as u8) as f32) / 255.0,
                        (((data[0] >> 16) as u8) as f32) / 255.0,
                        (((data[0] >>  8) as u8) as f32) / 255.0,
                        1.0,
                    ];

                    let light = [
                        norm_scale((data[0] >> 24) as i8),
                        norm_scale((data[0] >> 16) as i8),
                        norm_scale((data[0] >>  8) as i8),
                        1.0, // enable light
                    ];
                    
                    if cur_draw_is_lit && (self.current_light_state.lights[light_index] != light || self.current_light_state.colors[light_index] != color) {
                        // light state changed, new draw call
                        self.next_triangle_list();
                    }

                    self.current_light_state.lights[light_index] = light;
                    self.current_light_state.colors[light_index] = color;
                    trace!(target: "HLE", "light{} color = {:?}", light_index, color);
                    light_index += 1;
                }

                if light_index == self.num_lights as usize {
                    // disable the remaining lights
                    for li in self.num_lights as usize..LightState::NUM_LIGHTS {
                        if cur_draw_is_lit && (self.current_light_state.lights[li][3] != 0.0) {
                            // disabling a light -> new draw call
                            self.next_triangle_list();
                        }
                        self.current_light_state.lights[li][3] = 0.0;
                    }

                    // the last light is the ambient color
                    let ambient_data = &light_data[((16 * (num_lights - 1)) as usize) >> 2..];

                    // ambient color change doesn't affect light state since we pass it in the 
                    // color value of the vertex, so we can change ambient lighting without issuing
                    // a new draw call
                    self.ambient_light_color = [
                        (((ambient_data[0] >> 24) as u8) as f32) / 255.0,
                        (((ambient_data[0] >> 16) as u8) as f32) / 255.0,
                        (((ambient_data[0] >>  8) as u8) as f32) / 255.0,
                    ];

                    trace!(target: "HLE", "amblient light color = {:?}", self.ambient_light_color);
                }
            },

            14 => { // G_MV_MATRIX
                todo!();
            },

            0x82 => { // G_LOOKATY - use this vector for lighting?
                trace!(target: "HLE", "{} gsSPLookAtY(...)", self.command_prefix);
                static mut TODO: bool = true;
                if unsafe { TODO } { 
                    unsafe { TODO = false; }
                    info!(target: "HLE", "gsSPLookAtY lighting TODO");
                }
            },

            0x84 => { // G_LOOKATX - use this vector for lighting?
                let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else { 
                    let segment = (addr >> 24) as u8;
                    (addr & 0x007F_FFFF) + self.segments[segment as usize]
                } & 0x007F_FFFF);

                let lookat_data = self.load_from_rdram(translated_addr, size as u32);
                let _x = ((((lookat_data[2] >> 24) & 0xFF) as i8) as f32) / 127.0;
                let _y = ((((lookat_data[2] >> 16) & 0xFF) as i8) as f32) / 127.0;
                let _z = ((((lookat_data[2] >>  8) & 0xFF) as i8) as f32) / 127.0;

                trace!(target: "HLE", "{} gsSPLookAtX(...)", self.command_prefix);
                static mut TODO: bool = true;
                if unsafe { TODO } { 
                    unsafe { TODO = false; }
                    info!(target: "HLE", "gsSPLookAtX lighting TODO");
                }
            },

            _ => {
                trace!(target: "HLE", "{} gsSPMoveMem?({}, ...)", self.command_prefix, index);
            },
        };
    }

    fn handle_movemem00(&mut self) { // G_MOVEMEM (S3DEX2)
        let index = ((self.command >> 48) & 0xFF) as u8;
        let addr = self.command as u32;
        match index {
            0x80 => { // G_VIEWPORT
                self.handle_movemem(8, addr, 16);
            },
            0x82 | 0x84 => { // G_LOOKATX, Y
                self.handle_movemem(index, addr, 16); // sizeof(Light)
            },
            0x86..=0x94 => { // G_LIGHT
                self.handle_movemem(index, addr, 16); // sizeof(Light)
            },
            _ => todo!("unimplemented G_MOVEMEM cmd ${:02X}", index),
        }
        //self.handle_movemem(index, offset, data)
    }

    fn handle_movemem02(&mut self) { // G_MOVEMEM (F3DEX2)
        let size  = ((((self.command >> 48) & 0xFF) >> 3) + 1) << 3;
        let index = (self.command >> 32) as u8;
        let addr  = self.command as u32;
        self.handle_movemem(index, addr, size);
    }

    fn handle_load_ucode(&mut self) { // G_LOAD_UCODE
        let _cmd1 = self.next_display_list_command();
        self.command_words += 2;

        // nop
        trace!(target: "HLE", "{} glSPLoadUcodeEx(...)", self.command_prefix);
        todo!();
    }

    fn handle_displaylist(&mut self) { // G_DL (S3DEX2, F3DEX2)
        let is_link = (self.command & 0x00FF_0000_0000_0000) == 0;
        let addr    = self.command as u32;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            (addr & 0x007F_FFFF) + self.segments[segment]
        } & 0x007F_FFFF;

        if is_link {
            trace!(target: "HLE", "{} gsSPDisplayList(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);
        
            // append a DL stack entry
            let new_dl = DLStackEntry {
                base_address: translated_addr,
                ..Default::default()
            };

            self.dl_stack.push(new_dl);
        } else {
            trace!(target: "HLE", "{} gsSPBranchList(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);
        
            // replace the current DL with a new one
            let cur = self.dl_stack.last_mut().unwrap();
            cur.dl.clear();
            cur.base_address = translated_addr;
            cur.pc = 0;
        }
    }

    fn handle_culldl(&mut self) { // G_CULLDL
        let vfirst = ((self.command >> 32) as u16) >> 1;
        let vlast  = (self.command as u16) >> 1;

        trace!(target: "HLE", "{} gsSPCullDisplayList({}, {})", self.command_prefix, vfirst, vlast);

        // TODO 
        // loop over the vertices and find min and max of each component, call them vmin and vmax
        // transform vmin and vmax using the current mvp matrix
        // persp correct vmin and vmax
        // check if the entire bounding volumn is outside of viewspace (-1..1, -1..1, 0..1)
        // if entirely outside of viewspace, execute gsSPEndDisplayList()
    }

    fn handle_branch_z(&mut self) { // G_BRANCH_Z
        let addr = self.rdp_half_hi;

        let vbidx = ((self.command >> 32) & 0xFFF) >> 1;
        let zval = self.command as u32;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            (self.segments[segment] + (addr & 0x007F_FFFF)) & 0x007F_FFFF
        };

        trace!(target: "HLE", "{} gsSPBranchLessZraw(0x{:08X} [0x{:08X}], {}, 0x{:08X})", self.command_prefix, addr, translated_addr, vbidx, zval);
    }

    fn handle_enddl(&mut self) { // G_ENDDL (S3DEX2, F3DEX2)
        trace!(target: "HLE", "{} gsSPEndDisplayList()", self.command_prefix);
        self.dl_stack.pop();
    }

    fn handle_texture(&mut self) { // G_TEXTURE (S3DEX2, F3DEX2)
        let ts    = self.command as u16;
        let ss    = (self.command >> 16) as u16;
        let on    = ((self.command >> 32) & 0xFF) != 0;
        let tile  = (self.command >> 40) & 0x07;
        let level = (self.command >> 43) & 0x07;
        trace!(target: "HLE", "{} gsSPTexture(0x{:04X}, 0x{:04X}, {}, {}, {})", self.command_prefix, ss, ts, level, tile, on);

        if on {
            self.tex.enabled = true;
            self.tex.s_scale = (ss as f32) / 65536.0;
            self.tex.t_scale = (ts as f32) / 65536.0;
            self.tex.mipmaps = (level + 1) as u8;
            self.tex.tile    = tile as u8;
            if self.tex.mipmaps != 1 { warn!(target: "HLE", "TODO: mipmaps > 0 not implemented"); }
        } else {
            self.tex.enabled = false;
            self.tex.s_scale = 0.0;
            self.tex.t_scale = 0.0;
            self.tex.mipmaps = 0;
            self.tex.tile    = 0;
        }
    }

    fn handle_vtx(&mut self) { // G_VTX
        let (numv, vbidx) = match self.software_version {
            HleRspSoftwareVersion::S3DEX2 => {
                let numv = (((self.command >> 52) & 0x0F) + 1) as u8;
                let vbidx = ((self.command >> 48) & 0x0F) as u8;
                (numv, vbidx)
            },
            HleRspSoftwareVersion::F3DEX2 => {
                let numv  = (self.command >> 44) as u8;
                let vbidx = (((self.command >> 33) & 0x7F) as u8) - numv;
                (numv, vbidx)
            },
            _ => todo!(),
        };

        let addr  = self.command as u32;

        let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x007F_FFFF)
        } & 0x007F_FFFF);

        let vtx_size = mem::size_of::<F3DZEX2_Vertex>();
        let data_size = numv as usize * vtx_size;
        trace!(target: "HLE", "{} gsSPVertex(0x{:08X} [0x{:08X}], {}, {}) (size_of<vtx>={}, data_size={})", self.command_prefix, addr, translated_addr, numv, vbidx, vtx_size, data_size);

        let vtx_data = self.load_from_rdram(translated_addr, data_size as u32);
        assert!(data_size == vtx_data.len() * 4);
        assert!((vtx_size % 4) == 0);

        for i in 0..numv {
            let data = &vtx_data[(vtx_size * i as usize) >> 2..];

            // view position is calculated the same for both
            // MV matrix is transposed, since cgmath doesn't support left multiply??
            let pos = Vector4::new(((data[0] >> 16) as i16) as f32, (data[0] as i16) as f32, ((data[1] >> 16) as i16) as f32, 1.0);
            let view_pos: Vector4<f32> = self.current_modelview_matrix * pos;

            // convert F3DZEX2_Vertex to Vertex
            let vtx = if self.tweakables.disable_lighting || (self.geometry_mode & 0x0002_0000) == 0 { // !G_LIGHTING
                // Lighting disabled, we have vertex colors
                Vertex {
                    view_position: view_pos.into(),
                    color: [
                        ((data[3] >> 24) as u8) as f32 / 255.0, 
                        ((data[3] >> 16) as u8) as f32 / 255.0, 
                        ((data[3] >>  8) as u8) as f32 / 255.0, 
                        ( data[3]        as u8) as f32 / 255.0,
                    ],
                    tex_coords: [
                        // s,t are S10.5 format, and are scaled by the current s,t scale factors
                        // many games set s and t scale to 0.5, so this coordinate effectively becomes
                        // S9.6, and ranges -512..511
                        self.tex.s_scale * (((data[2] >> 16) as i16) as f32 / 32.0), 
                        self.tex.t_scale * (( data[2]        as i16) as f32 / 32.0),
                    ],
                    ..Default::default()
                }
            } else {
                let norm_scale = |i: i8| if i < 0 { (i as f32) / 128.0 } else { (i as f32) / 127.0 };

                // Lighting enabled, so we have a normal and lights the normal vector must be normalized by the game
                // and to calculate proper world space lighting, we need the modelview inverse matrix
                self.make_mv_inverse();

                let normal = Vector3::new(norm_scale((data[3] >> 24) as i8), norm_scale((data[3] >> 16) as i8), norm_scale((data[3] >>  8) as i8));
                let view_normal = self.current_mv_inverse_matrix.unwrap() * normal;

                Vertex {
                    view_position: view_pos.into(),
                    color: [
                        self.ambient_light_color[0],
                        self.ambient_light_color[1],
                        self.ambient_light_color[2],
                        // Alpha is still present
                        (data[3] as u8) as f32 / 255.0,
                    ],
                    view_normal: view_normal.normalize().into(),
                    tex_coords: [
                        self.tex.s_scale * (((data[2] >> 16) as i16) as f32 / 32.0), 
                        self.tex.t_scale * (( data[2]        as i16) as f32 / 32.0),
                    ],
                    flags: VertexFlags::LIT,
                    ..Default::default()
                }
            };

            trace!(target: "HLE", "v{}: {:?}", i+vbidx, vtx);

            // place the vertex in the internal buffer, and set the stack to refer to it
            let cur_pos = self.vertices_internal.len() as u16;
            self.vertices_internal.push(vtx);
            self.vertex_stack[(i + vbidx) as usize] = cur_pos;
        }
    }
        
    fn update_render_pass_state(&mut self) {
        let color_image = self.current_color_image;
        let depth_image = self.current_depth_image;

        let rp = self.current_render_pass();
        rp.color_buffer         = color_image;
        rp.depth_buffer         = depth_image;
    }

    fn current_render_pass(&mut self) -> &mut RenderPassState {
        self.render_passes.last_mut().expect("must always have a valid RP")
    }

    // true if this render pass has something drawn in it
    fn current_render_pass_has_tris(&mut self) -> bool {
        self.current_render_pass().draw_list.len() > 1 ||
            self.current_render_pass().draw_list.last().is_some_and(|v| v.num_indices != 0)
    }

    fn next_render_pass(&mut self, reason: Option<String>) {
        if self.render_passes.len() > 0 {
            // don't create a new render pass if this one isn't rendering anything, however, keep the current state
            // if there's no draw_list or the one there is empty, this render pass can still be used and doesn't need to be finalized
            if !self.current_render_pass_has_tris() {
                // No draw calls, so update render state
                self.update_render_pass_state();
                return;
            }

            self.finalize_render_pass(reason);
        }

        let rp = RenderPassState::default();
        self.render_passes.push(rp);
        self.update_render_pass_state();

        self.next_triangle_list();
    }

    fn finalize_render_pass(&mut self, reason: Option<String>) {
        // if the current draw list has no indices, drop it
        if self.current_render_pass().draw_list.last().is_some_and(|v| v.num_indices == 0) {
            self.current_render_pass().draw_list.pop().unwrap();
            self.num_draws -= 1;
        }

        // if there are no draw_calls, drop this render pass
        // this could happen at the end of a frame, but not from next_render_pass()
        if self.current_render_pass().draw_list.len() == 0 {
            self.render_passes.pop().unwrap();
            return;
        }

        match self.current_render_pass().pass_type {
            // no depth on FillRectangles
            Some(RenderPassType::FillRectangles) => {
                // disabling the depth buffer makes the draw calls 
                // depth_write/compare variables go unused
                self.current_render_pass().depth_buffer = None;
            },

            Some(RenderPassType::DrawTriangles) => {},

            None => warn!(target: "HLE", "finalizing buffer that has no pass type"),
        }

        trace!(target: "HLE", "changing to new render pass: {} (now {} complete passe(s))", reason.as_ref().unwrap(), self.render_passes.len());
        self.current_render_pass().reason = reason;

        // check to see if the render targets are cleared with full screen tris
        let (color_buffer, depth_buffer) = {
            let rp = self.current_render_pass();
            (rp.color_buffer, rp.depth_buffer)
        };

        if let Some(color_addr) = color_buffer {
            let clear_color = self.clear_images.remove(&color_addr);
            self.current_render_pass().clear_color = clear_color;
        }

        if let Some(depth_addr) = depth_buffer {
            if self.clear_images.remove(&depth_addr).is_some() {
                self.current_render_pass().clear_depth = true;
            }
        }
    }

    fn add_triangles(&mut self, pass_type: RenderPassType, v: &[u16]) {
        assert!((v.len() % 3) == 0); // must be a tri

        // check if the render pass type has changed, and if so, start a new render pass
        if self.current_render_pass().pass_type.is_some_and(|v| v != pass_type) {
            let cur_pass_type = self.current_render_pass().pass_type.unwrap();
            self.next_render_pass(Some(format!("change pass type from {:?} to {:?}", cur_pass_type, pass_type)));
        }

        // set pass type for the new render pass (or one that is None)
        self.current_render_pass().pass_type = Some(pass_type);

        // check if any states need to change to start a new draw call
        if self.current_triangle_list().num_indices != 0 {
            // if requested no depth, change to a new draw call
            if self.disable_depth_override.is_some() {
                if self.current_triangle_list().depth_write || self.current_triangle_list().depth_compare_enable {
                    self.next_triangle_list();
                }
            }

            // if the matrix index requested is different than the current matrix
            if let Some(matrix_index_override) = self.matrix_index_override {
                if self.current_triangle_list().matrix_index != matrix_index_override { // must be valid here
                    // pass type is the same, draw calls exist, matrix changed, we need a new list
                    self.next_triangle_list();
                }
            }

            // if the mapped TEXEL0 index has changed, switch to a new draw call
            if !self.tex.enabled && self.current_triangle_list().mapped_texture_index.is_some() { 
                // textures have been disabled
                self.next_triangle_list();
            } else if self.tex.enabled {
                if self.current_triangle_list().mapped_texture_index.is_none() {
                    // textures have been enabled but no mapped texture set yet
                    self.next_triangle_list();
                } else {
                    let ti = self.current_triangle_list().mapped_texture_index.unwrap();
                    // tile.mapped_coordinates must be set (you have to call map_current_textures()
                    // before adding textured triangles)
                    if ti != self.tex.rdp_tiles[self.tex.tile as usize].mapped_coordinates.unwrap().0 {
                        self.next_triangle_list();
                    }
                }
            }

            // if the mapped TEXEL1 index has changed, switch to a new draw call
            if !self.tex.enabled && self.current_triangle_list().mapped_texture_index1.is_some() { 
                // textures have been disabled
                self.next_triangle_list();
            } else if self.tex.enabled {
                // if this draw doesn't use TEXEL1, don't care what is currently set
                if self.tex.rdp_tiles[self.tex.tile as usize].has_texel1 {
                    if self.current_triangle_list().mapped_texture_index1.is_none() {
                        // texture has TEXEL1 but no mapped_texture_index1 set on this draw call, start a new one
                        self.next_triangle_list();
                    } else {
                        // if the TEXEL1 texture index changes, need a new draw call
                        let ti = self.current_triangle_list().mapped_texture_index1.unwrap();

                        // tile.mapped_coordinates must be set (you have to call map_current_textures() before adding textured triangles)
                        // and we know that tex.tile+1 is valid
                        if ti != self.tex.rdp_tiles[self.tex.tile as usize + 1].mapped_coordinates.unwrap().0 {
                            self.next_triangle_list();
                        }
                    }
                }
            }

            // if fog state changed, need a new draw call
            let fs_index = self.current_triangle_list().fog_state_index;
            if *self.fog_states.get(fs_index as usize).unwrap() != self.current_fog_state {
                self.next_triangle_list();
            }
        }

        // upon the first addition of a triangle, we need to note what the current uniform buffers are
        if self.current_triangle_list().num_indices == 0 {
            // if this is the first draw call in the render pass, update render state
            if self.current_render_pass().draw_list.len() == 1 {
                self.update_render_pass_state();
            }

            // set the current viewport on the draw call
            let viewport = self.current_viewport.clone();

            // set the pipeline state
            let (depth_write, depth_compare_enable) = if self.disable_depth_override.is_some() {
                (false, false)
            } else {
                (self.other_modes.get_depth_update_enable(),
                 self.other_modes.get_depth_compare_enable())
            };

            // transfer the current matrix to the uniform buffer but allow a matrix index override
            // you need to set self.matrix_index_override before any add_triangles call that requires it
            let matrix_index = match self.matrix_index_override {
                Some(matrix_index_override) => {
                    matrix_index_override
                },
                None => {
                    let matrix_index = self.matrices.len() as u32;
                    self.matrices.push(MatrixState {
                        projection: self.current_projection_matrix.into(),
                        ..Default::default()
                    });
                    matrix_index
                },
            };

            trace!(target: "HLE", "new draw call uses proj matrix: {:?}", self.current_projection_matrix);
            //println!("new draw call uses proj matrix: {:?}", self.current_projection_matrix);

            // set the mapped texture index for this draw call (TEXEL0)
            let mapped_texture_index = {
                let rdp_tile = &self.tex.rdp_tiles[self.tex.tile as usize];
                match rdp_tile.mapped_coordinates {
                    Some((ti, _, _)) => Some(ti),
                    _ => None,
                }
            };

            // setup TEXEL1 input
            let mapped_texture_index1 = {
                if mapped_texture_index.is_some() && self.tex.rdp_tiles[self.tex.tile as usize].has_texel1 {
                    let rdp_tile1 = &self.tex.rdp_tiles[self.tex.tile as usize + 1];
                    match rdp_tile1.mapped_coordinates {
                        Some((ti, _, _)) => Some(ti),
                        _ => None,
                    }
                } else {
                    None
                }
            };

            // transfer the current combiner state to the uniform buffer
            let cc_index = {
                let need_new = self.color_combiner_states.len() == 0 
                                || self.current_color_combiner_state != *self.color_combiner_states.last().unwrap();
                if need_new {
                    let cc_index = self.color_combiner_states.len() as u32;
                    self.color_combiner_states.push(self.current_color_combiner_state);
                    cc_index
                } else {
                    (self.color_combiner_states.len() - 1) as u32
                }
            };

            trace!(target: "HLE", "CC State: {:?}", self.color_combiner_states[cc_index as usize]);

            // transfer the current light state to the uniform buffer
            let ls_index = {
                // we only enable lighting on vertices that are light, otherwise ls_index can be left empty
                if (self.vertices[v[0] as usize].flags & VertexFlags::LIT) != 0 {
                    let need_new = self.light_states.len() == 0 || self.current_light_state != *self.light_states.last().unwrap();
                    if need_new {
                        let index = self.light_states.len();
                        self.light_states.push(self.current_light_state);
                        Some(index as u32)
                    } else {
                        Some((self.light_states.len() - 1) as u32)
                    }
                } else {
                    None
                }
            };

            // transfer current fog state to uniform buffers
            let fs_index = {
                let rsp_fog = (self.geometry_mode & 0x0001_0000) != 0;
                let rdp_fog = self.other_modes.get_fog_enabled();
                // G_BL_A_SHADE 
                let vertex_fog = self.other_modes.get_cycle1_a_shade();
                let _prim_fog = self.other_modes.get_cycle1_a_fog();

                if rsp_fog && rdp_fog && vertex_fog && !self.tweakables.disable_fog {
                    let need_new = self.current_fog_state != *self.fog_states.last().unwrap();
                    if need_new {
                        let index = self.fog_states.len();
                        self.fog_states.push(self.current_fog_state);
                        index as u32
                    } else {
                        (self.fog_states.len() - 1) as u32
                    }
                } else {
                    0
                }
            };

            // set all the state values
            // now with num_indices > 0, they can't change. a new draw call is needed
            let tl = self.current_triangle_list();
            tl.viewport = viewport;
            tl.depth_compare_enable = depth_compare_enable;
            tl.depth_write = depth_write;
            tl.matrix_index = matrix_index; // set the matrix index
            tl.mapped_texture_index = mapped_texture_index;
            tl.mapped_texture_index1 = mapped_texture_index1;
            tl.color_combiner_state_index = cc_index;
            tl.light_state_index = ls_index;
            tl.fog_state_index = fs_index;
        }

        // clear overrides even if they weren't used
        self.matrix_index_override = None;
        self.disable_depth_override = None;

        self.indices.extend_from_slice(v);
        self.current_triangle_list().num_indices += v.len() as u32;
        self.num_tris += v.len() as u32 / 3;
    }

    // get current triangle list, and if the pass type changes we need a new render pass
    fn current_triangle_list(&mut self) -> &mut TriangleList {
        // create a draw_list if none exists
        if self.current_render_pass().draw_list.len() == 0 {
            self.next_triangle_list();
        }

        self.current_render_pass().draw_list.last_mut().unwrap()
    }

    // start a new draw list of type pass_type
    fn next_triangle_list(&mut self) {
        // if the current list exists and has no indices, then no need to create a new draw list
        if self.current_render_pass().draw_list.last().is_some_and(|v| v.num_indices == 0) { return; }

        // definitely need a new draw list then
        let tl = TriangleList {
            // start index with the current indices doesn't change
            start_index : self.indices.len() as u32,

            ..Default::default()
        };

        self.current_render_pass().draw_list.push(tl);
        self.num_draws += 1;
    }

    // return a transformed vertex along with the index into self.indices where it is placed
    // the vertex is `final` in the sense that it's the exact data that will go to the shader
    fn finalize_vertex(&mut self, vertex_index: usize) -> Option<(&Vertex, u16)> {
        let mut vtx = self.vertices_internal.get(vertex_index)?.clone();
        if self.tex.enabled {
            let current_tile = self.tex.tile;
            let rdp_tile = &self.tex.rdp_tiles[current_tile as usize];

            // copy because we're modifying vtx.tex_coords;
            let tex_coords = vtx.tex_coords;

            if let Some((_, mx, my)) = rdp_tile.mapped_coordinates {
                // adjust texcoords to be relative to the ul coordinate
                vtx.tex_coords[0] = tex_coords[0] - rdp_tile.ul.0;
                vtx.tex_coords[1] = tex_coords[1] - rdp_tile.ul.1;

                // the texture parameters of the tile need to be sent to the shader
                // mx,my correspond to 0,0 of the texture (not rdp_tile.ul!)
                vtx.tex_params[0] = mx as f32;        // x start
                vtx.tex_params[1] = mx as f32 + (rdp_tile.lr.0 - rdp_tile.ul.0) + 1.0;  // x end
                vtx.tex_params[2] = my as f32;        // y start
                vtx.tex_params[3] = my as f32 + (rdp_tile.lr.1 - rdp_tile.ul.1) + 1.0;  // y end

                // all the tex coords and params stay in the unscaled form 
                // they will be scaled in the shader
            }

            // mask and shift values for this tile
            vtx.maskshift = ((rdp_tile.mask_s as u32) << 24)
                            | ((rdp_tile.shift_s as u32) << 16)
                            | ((rdp_tile.mask_t as u32) << 8)
                            | ((rdp_tile.shift_t as u32) << 0);

            // set flags on the vertex for this tile shifting
            vtx.flags |= ((rdp_tile.clamp_s as u32) << VertexFlags::TEXMODE_S_SHIFT)
                        | ((rdp_tile.clamp_t as u32) << VertexFlags::TEXMODE_T_SHIFT);

            // if there's a second texture to sample, pass along those coordinates
            if rdp_tile.has_texel1 && current_tile < 7 {
                let rdp_tile1 = &self.tex.rdp_tiles[current_tile as usize + 1];

                if let Some((_, mx, my)) = rdp_tile1.mapped_coordinates {
                    // adjust texcoords to be relative to the ul coordinate
                    vtx.tex_coords1[0] = tex_coords[0] - rdp_tile1.ul.0;
                    vtx.tex_coords1[1] = tex_coords[1] - rdp_tile1.ul.1;

                    // the texture parameters of the tile need to be sent to the shader
                    // mx,my correspond to 0,0 of the texture (not rdp_tile.ul!)
                    vtx.tex_params1[0] = mx as f32;        // x start
                    vtx.tex_params1[1] = mx as f32 + (rdp_tile1.lr.0 - rdp_tile1.ul.0) + 1.0;  // x end
                    vtx.tex_params1[2] = my as f32;        // y start
                    vtx.tex_params1[3] = my as f32 + (rdp_tile1.lr.1 - rdp_tile1.ul.1) + 1.0;  // y end

                    // all the tex coords and params stay in the unscaled form 
                    // they will be scaled in the shader
                }

                // mask and shift values for this tile
                vtx.maskshift1 = ((rdp_tile1.mask_s as u32) << 24)
                                | ((rdp_tile1.shift_s as u32) << 16)
                                | ((rdp_tile1.mask_t as u32) << 8)
                                | ((rdp_tile1.shift_t as u32) << 0);

                // set flags on the vertex for this tile shifting
                vtx.flags |= ((rdp_tile1.clamp_s as u32) << VertexFlags::TEXMODE_S1_SHIFT)
                            | ((rdp_tile1.clamp_t as u32) << VertexFlags::TEXMODE_T1_SHIFT);
            }

            if self.tweakables.disable_textures {
                vtx.flags &= !VertexFlags::TEXTURED;
            } else {
                vtx.flags |= VertexFlags::TEXTURED;

                if self.other_modes.get_texture_filter() == TextureFilter::Bilinear {
                    vtx.flags |= VertexFlags::LINEAR_FILTER;
                }
            }
        }   

        if self.other_modes.get_cycle_type() == CycleType::TwoCycle {
            vtx.flags |= VertexFlags::TWO_CYCLE;
        }

        // pass ZMode through
        let mut zmode = self.other_modes.get_zmode();
        if zmode == ZMode::Opaque && self.other_modes.get_force_blend() { zmode = ZMode::Translucent; }
        vtx.flags |= (zmode as u32) << VertexFlags::ZMODE_SHIFT;

        // alpha compare mode
        if self.other_modes.get_alpha_compare() == AlphaCompare::Threshold {
            vtx.flags |= 1 << VertexFlags::ALPHA_COMPARE_MODE_SHIFT; //(self.other_modes.get_alpha_compare() as u32) << VertexFlags::ALPHA_COMPARE_MODE_SHIFT;
        }

        // tex edge mode
        if self.other_modes.guess_tex_edge() {
            vtx.flags |= 1 << VertexFlags::TEX_EDGE;
        }

        let index = self.vertices.len();
        trace!(target: "HLE", "final: {:?}", vtx);
        self.vertices.push(vtx);
        Some((self.vertices.get(index).unwrap(), index as u16))
    }

    fn allocate_mapped_texture_space(&mut self, width: u32, height: u32) -> (u32, u32, u32) {
        trace!(target: "HLE", "allocating texture space for {}x{} tile", width, height);
        assert!(width < (TEXSIZE_WIDTH as u32) && height < (TEXSIZE_HEIGHT as u32));

        // TODO I guess I need some spacial tree structure to allocate rectangular regions.
        // It wouldn't really need to be super space efficient, just not needlessly wasteful.
        // Modern gpus have tons of ram to work with.
        
        // loop over mapped textures in reverse order looking for space before allocating new textures
        let mut i = self.mapped_textures.len() - 1;
        loop {
            let fits = {
                let mut mt = self.mapped_textures.get(i).unwrap().write().unwrap();

                // For now, we allocate left to right, top to bottom. To move from row to row, we keep
                // track of the tallest texture
                if ((mt.width as u32) - mt.alloc_x) < width {
                    // move down and to the beginning of the row
                    mt.alloc_x = 0;
                    mt.alloc_y += mt.alloc_max_h;

                    // reset max h for this row
                    mt.alloc_max_h = 0;
                }

                // if Y doesn't fit then we ran out of space
                !(((mt.height as u32) - mt.alloc_y) < height)
            };

            // check another texture if it doesn't fit
            if !fits {
                if i > 0 {
                    i -= 1;
                    continue;
                } 

                // otherwise create a new texture and then it definitely fits
                self.new_texture_cache();
                i = self.mapped_textures.len() - 1; // index of the newly created texture
            }

            // update values and return result
            let mut mt = self.mapped_textures.get(i).unwrap().write().unwrap();
            let rx = mt.alloc_x; // texture start position
            let ry = mt.alloc_y;
            mt.alloc_x += width;
            if height > mt.alloc_max_h {
                mt.alloc_max_h = height;
            }

            mt.dirty = true;

            break (i as u32, rx, ry);
        }
    }

    #[inline]
    fn tmem_offset_4b(x: u32, y: u32, tmem_address: u32, line_bytes: u32) -> (usize, u32, u32) {
        // on odd lines, flip to read 8 texels (32-bits) ahead/back
        let sx = if (y & 0x01) == 0x01 { x ^ 0x08 } else { x };
        
        // offset based on x,y of texture data
        // tmem_address is in 64-bit words, self.tex.tmem is 32-bit
        let offset  = (y * line_bytes) + (sx >> 1);    // sx is is texels, convert to half bytes!
        let address = (tmem_address << 3) + offset;    // address in bytes
        let rshift  = 28 - ((sx & 0x07) << 2);         // multiply by 4 to select bits 31..28, 27..24, 23..20, etc

        (address as usize, rshift, 0x0F)
    }

    #[inline]
    fn tmem_offset_8b(x: u32, y: u32, tmem_address: u32, line_bytes: u32) -> (usize, u32, u32) {
        // on odd lines, flip to read 4 texels (32-bits) ahead/back
        let sx = if (y & 0x01) == 0x01 { x ^ 0x04 } else { x };

        // offset based on x,y of texture data
        // tmem_address is in 64-bit words, self.tex.tmem is 32-bit
        let offset  = (y * line_bytes) + sx;           // sx is is texels (8bpp), convert to bytes!
        let address = (tmem_address << 3) + offset;    // address in bytes
        let rshift  = 24 - ((sx & 0x03) << 3);         // multiply by 8 to select bits 31..24, 23..16, 15..8, 7..0

        (address as usize, rshift, 0xFF)
    }

    #[inline]
    fn tmem_offset_16b(x: u32, y: u32, tmem_address: u32, line_bytes: u32) -> (usize, u32, u32) {
        // on odd lines, flip to read 2 texels (32-bits) ahead/back
        let sx = if (y & 0x01) == 0x01 { x ^ 0x02 } else { x };

        // offset based on x,y of texture data
        // tmem_address is in 64-bit words, self.tex.tmem is 32-bit
        let offset  = (y * line_bytes) + (sx * 2);      // sx is is texels (16bpp), convert to bytes!
        let address = (tmem_address << 3) + offset;     // address in bytes
        let rshift  = 16 - ((address & 0x02) << 3);     // multiply by 16 to select bits 31..16, 15..0

        (address as usize, rshift, 0xFFFF)
    }

    // Convert CI 4b in TMEM to RGBA 32bpp
    fn map_tmem_ci_4b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {

        let tlut_mode = self.other_modes.get_tlut_mode();

        // determine palette
        let palette = (self.tex.rdp_tiles[self.tex.tile as usize].palette << 4) as u32;

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_4b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                let index = palette | src;
                let color_address = 0x200 | (index >> 1);
                let shift = 16 - ((index & 1) << 4);
                let color = (self.tex.tmem[color_address as usize] >> shift) & 0xFFFF;
                let p = match tlut_mode {
                    TlutMode::Rgba16 => {
                        let r = (color >> 11) & 0x1F;
                        let g = (color >>  6) & 0x1F;
                        let b = (color >>  1) & 0x1F;
                        [((r << 3) | (r >> 2)) as u8,
                         ((g << 3) | (g >> 2)) as u8,
                         ((b << 3) | (b >> 2)) as u8,
                         if (color & 0x01) != 0 { 255 } else { 0 }]
                    },
                    TlutMode::Ia16 => {
                        let i = color >> 8;
                        let a = color & 0xFF;
                        [i as u8, i as u8, i as u8, a as u8]
                    },
                    TlutMode::None => {
                        warn!(target: "HLE", "map_tmem_ci_4b with TlutMode::None");
                        [255, 0, 0, 255]
                    },
                };

                plot(x, y, &p);
                //plot(x, y, &[207, 52, 235, 255]);
            }
        }
        //plot(0, 0, &[255, 0, 0, 200]); // type of image
    }

    // Convert IA 4b in TMEM to RGBA 32bpp
    //
    // Overworld minimap in OoT
    fn map_tmem_ia_4b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_4b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                // duplicate the nibble in both halves to give a more gradual flow and maximum
                // range (0b0000 maps to 0b0000_0000 and 0b1111 maps to 0b1111_1111)
                let c = src >> 1;
                let v = (c << 5) | (c << 2) | (c >> 1);
                plot(x, y, &[v as u8, v as u8, v as u8, if (src & 0x01) != 0 { 255 } else { 0 }]);
            }
        }
        //plot(0, 0, &[255, 1, 0, 200]); // type of image
    }

    // Convert I 4b in TMEM to RGBA 32bpp
    fn map_tmem_i_4b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {
        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_4b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                // duplicate the nibble in both halves to give a more gradual flow and maximum
                // range (0b0000 maps to 0b0000_0000 and 0b1111 maps to 0b1111_1111)
                let v = (src << 4) | src;
                plot(x, y, &[v as u8, v as u8, v as u8, v as u8]);
                //plot(x, y, &[207, 52, 235, 255]);
            }
        }
        //plot(0, 0, &[255, 2, 0, 200]); // type of image
    }

    // Convert CI 8b in TMEM to RGBA 32bpp
    //
    // Environment/Skybox in OoT
    fn map_tmem_ci_8b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {

        let tlut_mode = self.other_modes.get_tlut_mode();

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_8b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                // select the 16-bit color from TLUT
                let color_address = 0x200 | (src >> 1); // two colors per 32-bit word 
                let shift = 16 - ((src & 1) << 4);
                let color = ((self.tex.tmem[color_address as usize] >> shift) & 0xFFFF) as u16;
                let p = match tlut_mode {
                    TlutMode::Rgba16 => {
                        let r = (color >> 11) & 0x1F;
                        let g = (color >>  6) & 0x1F;
                        let b = (color >>  1) & 0x1F;
                        [((r << 3) | (r >> 2)) as u8,
                         ((g << 3) | (g >> 2)) as u8,
                         ((b << 3) | (b >> 2)) as u8,
                         if (color & 0x01) != 0 { 255 } else { 0 }]
                    },
                    TlutMode::Ia16 => {
                        let i = color >> 8;
                        let a = color & 0xFF;
                        [i as u8, i as u8, i as u8, a as u8]
                    },
                    TlutMode::None => {
                        warn!(target: "HLE", "map_tmem_ci_8b with TlutMode::None");
                        [255, 0, 0, 255]
                    },
                };

                plot(x, y, &p);
                //plot(x, y, &[207, 52, 235, 255]);
            }
        }
        //plot(0, 0, &[255, 0, 1, 200]); // type of image
    }

    // Convert IA 8b in TMEM to RGBA 32bpp
    //
    // Moon in title scren OoT
    fn map_tmem_ia_8b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_8b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                let c = src >> 4;
                let v = (c << 4) | c;
                let a = src & 0x0F;
                plot(x, y, &[v as u8, v as u8, v as u8, ((a << 4) | a) as u8]);
            }
        }
        //plot(0, 0, &[255, 1, 1, 200]); // type of image
    }

    // Convert I 8b in TMEM to RGBA 32bpp
    //
    // NINTENDO64 logo in OoT
    fn map_tmem_i_8b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {
        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_8b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };
                plot(x, y, &[src as u8, src as u8, src as u8, src as u8]);
                //plot(x, y, &[52, 207, 235, 255]);
            }
        }
        //plot(0, 0, &[255, 2, 1, 200]); // type of image
    }

    // Convert IA 16b in TMEM to RGBA 32bpp
    fn map_tmem_ia_16b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F) 
        where 
            F: FnMut(u32, u32, &[u8]) {

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_16b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };

                let c = src >> 8;
                let a = src & 0xFF;
                plot(x, y, &[c as u8, c as u8, c as u8, a as u8]);
            }
        }
        //plot(0, 0, &[255, 1, 2, 200]); // type of image
    }


    // Convert RGBA 16b in TMEM to RGBA 32bpp
    fn map_tmem_rgba_16b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F)
        where 
            F: FnMut(u32, u32, &[u8]) {
        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    let (address, rshift, mask) = Self::tmem_offset_16b(x, y, tmem_address, line_bytes);
                    (self.tex.tmem[address >> 2] >> rshift) & mask
                };
                let r = ((src >> 11) & 0x1F) as u8;
                let g = ((src >>  6) & 0x1F) as u8;
                let b = ((src >>  1) & 0x1F) as u8;
                let a = (src & 0x01) != 0;
                plot(x, y, &[(r << 3) | (r >> 2), (g << 3) | (g >> 2), (b << 3) | (b >> 2), if a {255} else {0}]);
            }
        }
        //plot(0, 0, &[255, 3, 2, 200]); // type of image
    }

    fn map_tmem_rgba_32b<F>(&mut self, tmem_address: u32, texture_width: u32, texture_height: u32, line_bytes: u32, mut plot: F)
        where 
            F: FnMut(u32, u32, &[u8]) {

        for y in 0..texture_height {
            for x in 0..texture_width {
                let src = {
                    // swap by two texels on odd lines
                    // not 1 because texels are 16-bit (separated) in size
                    let sx = if (y & 0x01) == 0x01 { x ^ 2 } else { x };

                    // offset based on x,y of texture data
                    // every 16-bits represents one 32-bit texel because the other half is stored in high mem
                    let offset = (y * line_bytes) + (sx * 2);
                    let shift = 16 - ((sx & 1) << 4); // odd x takes the lower 16-bits from value in tmem

                    // tmem_address is in 64-bit words, self.tex.tmem is 32-bit
                    // offset is in bytes
                    let address = (tmem_address << 3) + offset;

                    // combine colors from high and low
                    let a = (self.tex.tmem[(address >> 2) as usize | 0x000] >> shift) & 0xFFFF;
                    let b = (self.tex.tmem[(address >> 2) as usize | 0x200] >> shift) & 0xFFFF;
                    (a << 16) | b
                };
                plot(x, y, &src.to_be_bytes());
            }
        }
        //plot(0, 0, &[255, 3, 3, 200]); // type of image
    }

    // Need to map both TEXEL0 (if textures are enabled) and TEXEL1 (if the color combiner has it set)
    // TODO: mipmaps
    fn map_current_textures(&mut self) {
        // if texturing isn't enabled, then the current tile index isn't valid and it doesn't
        // make sense to map it into a texture
        if !self.tex.enabled { return; }

        // when rendering, tile should be set to mipmap level 0
        let current_tile_index = self.tex.tile;
        self.map_texture(current_tile_index);

        // if any of the color combiner states use parameter 2 (G_CCMUX_TEXEL1 or G_ACMUX_TEXEL1) 
        // or if `C` uses parameter 9 (G_CCMUX_TEXEL1_ALPHA), we need to include the second texture
        self.tex.rdp_tiles[self.tex.tile as usize].has_texel1 = false;
        if current_tile_index < 7 && self.color_combiner_uses_texel1 { // last tile can't be used here
            let valid_tile1 = self.tex.rdp_tiles[self.tex.tile as usize + 1].line != 0;
            if valid_tile1 {
                self.tex.rdp_tiles[self.tex.tile as usize].has_texel1 = true;
                self.map_texture(current_tile_index + 1);
            }
        }
    }

    fn map_texture(&mut self, current_tile_index: u8) {
        let current_tile = self.tex.rdp_tiles[current_tile_index as usize];

        // if current tile has coordinates, we don't need to do anything
        if current_tile.mapped_coordinates.is_some() { return; }

        // calculate row stride of the source texture data, not always the same as width
        let line_bytes = (current_tile.line as u32) * 8;

        // TODO we really shouldn't use lr coordinates if clamp or wrapping
        // isn't set. We could just use padded_width to map the width of the texture,
        // but I'm not sure where the height of the texture would come from

        // tile bounds are inclusive of texels, so add 1 in each dim
        // we're going to ignore the ul coordinate of the tile and map from 0,0 to lr
        // so we end up mapping a texture larger than required but 
        let texture_width = match current_tile.size {
            0 => line_bytes << 1, // 4b
            1 => line_bytes     , // 8b
            2 => line_bytes >> 1, // 16b
            3 => line_bytes >> 1, // 32b have a width twice the line size due to the split
                                  //     nature of the data
            5 => todo!("SIZ_DD"),
            _ => { error!(target: "HLE", "invalid texture size"); 0 },
        };

        // cap texture_height to the availble tmem memory
        let max_height = if current_tile.format == 2 { // CI
            2048 / line_bytes
        } else {
            4096 / line_bytes
        };

        let texture_height = std::cmp::min((current_tile.lr.1 as u32).saturating_sub(current_tile.ul.1 as u32) + 1, max_height);

        // TODO tiles with upper left coordinates not at 0,0 are getting the wrong texture data
        //if current_tile.ul.0 != 0.0 || current_tile.ul.1 != 0.0 {
        //    warn!(target: "HLE", "TODO: mapping tile with ul != (0,0): {:?}", current_tile.ul);
        //}

        // calculate CRC of texture data
        let crc: u64 = {
            let crc_start = ((current_tile.tmem as u32) << 1) as usize;
            let len = ((line_bytes * texture_height) >> 2) as usize;
            let data = (self.tex.tmem[crc_start..][..len])
                            .iter()
                            .map(|v| v.to_be_bytes())
                            .flatten()
                            .collect::<Vec<u8>>();

            let mut crc = crc64::crc64(0, &data);

            // include the texture dims, format and data size in the CRC
            // (i.e., a blank texture in IA is not the same as a blank texture in CI)
            let mut metadata: Vec<_> = vec![];
            metadata.push(current_tile.format.into());
            metadata.push(current_tile.size.into());
            metadata.extend_from_slice(&texture_width.to_be_bytes());
            metadata.extend_from_slice(&texture_height.to_be_bytes());
            metadata.extend_from_slice(&data.len().to_be_bytes());
            crc = crc64::crc64(crc, &metadata);

            // 32-bit textures need to crc the high half of memory too
            if current_tile.size == 3 {
                let data = (self.tex.tmem[crc_start ^ 0x200..][..len])
                                .iter()
                                .map(|v| v.to_be_bytes())
                                .flatten()
                                .collect::<Vec<u8>>();

                crc64::crc64(crc, &data)
            } else if current_tile.format == 2 { // Color-indexed textures, 
                // Palette textures need their palette CRC'd so we can get palette animations
                // But it could be very wasteful and slow
                match current_tile.size { 
                    0 => { // 4b CI needs to crc 16 palette entries (*2 bytes ea) indicated by current_tile.pal
                           // only 512 bytes of high mem is used because the tlut has to be duplicated 4x
                           // (TODO: maybe something takes advantage of this?)
                        let palette = (current_tile.palette << 4) as usize; // half word index into memory
                        let data = (self.tex.tmem[crc_start | (0x200 + (palette >> 1))..][..8])
                                        .iter()
                                        .map(|v| v.to_be_bytes())
                                        .flatten()
                                        .collect::<Vec<u8>>();
                        crc64::crc64(crc, &data)
                    },
                    1 => { // 8b CI needs to CRC the entirety of the 256-color TLUT
                        let data = (self.tex.tmem[crc_start | 0x200..][..128])
                                        .iter()
                                        .map(|v| v.to_be_bytes())
                                        .flatten()
                                        .collect::<Vec<u8>>();
                        crc64::crc64(crc, &data)
                    },
                    _ => panic!("invalid size for CI"),
                }
            } else {
                crc
            }
        };

        if let Some((ti, mx, my)) = self.tex.mapped_texture_cache.get(&crc) {
            let current_tile = &mut self.tex.rdp_tiles[current_tile_index as usize];
            current_tile.mapped_coordinates = Some((*ti, *mx, *my));
            //println!("{}. saw crc=${:X} before, mapped_coordinates = {:?}, ul.0={}", current_tile_index, crc, current_tile.mapped_coordinates, current_tile.ul.0);
            return;
        }

        // we need to duplicate the borders of every texture so that filters don't look ugly
        let (ti, mut mx, mut my) = self.allocate_mapped_texture_space(texture_width+2, texture_height+2);
        trace!(target: "HLE", "allocated texture space at {},{} in ti={} for texture size {}x{} (crc=${:X})", mx, my, ti, texture_width, texture_height, crc);

        // coordinates that the polys use will be increased by 1 to accomodate the duplicate texture data
        mx += 1;
        my += 1;

        // we need to take the mapped texture out of the array so we don't keep a mutable reference to self
        // don't take the data directly either, since the render thread might be about to read it
        let mapped_texture_lock = {
            std::mem::replace(self.mapped_textures.get_mut(ti as usize).unwrap(),
                              Arc::new(RwLock::new(MappedTexture::default())))
        };
        let mut mapped_texture = mapped_texture_lock.write().unwrap();

        let mtw = mapped_texture.width as u32;
        let texdata = &mut mapped_texture.data;

        // now we need to copy the texture from tmem to mapped texture space, converting
        // before formats and unswizzling odd rows. this plot fuction shifts x and y right/down by
        // 1, and will duplicate a pixel if x == 0 or y == 0
        let calc_offset = |x, y| (4 * (((my + y) * mtw) + (mx + x))) as usize; // *4 => sizeof(u32)
        let plot = move |x: u32, y: u32, color: &[u8]| {
            let offset = calc_offset(x, y);
            {
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
            }

            #[allow(dead_code)]
            const BORDER_COLOR: [u8; 4] = [255, 0, 0, 255];

            // copy color into (-1,-1)
            if x == 0 && y == 0 {
                let offset = offset - 4*(mtw as usize + 1);
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            } 
            // copy color into (w, h)
            else if x == texture_width - 1 && y == texture_height - 1 {
                let offset = offset + 4*(mtw as usize + 1);
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            } 
            // copy color into (-1, h)
            else if x == 0 && y == texture_height - 1 {
                let offset = offset + 4*(mtw as usize - 1);
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            }
            // copy color into (x, -1)
            else if x == texture_width - 1 && y == 0 {
                let offset = offset - 4*(mtw as usize - 1);
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            }

            // copy color into (-1, y)
            if x == 0 {
                let offset = offset - 4; // sizeof(u32)
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            } 
            // copy color into (w, y)
            else if x == texture_width - 1 {
                let offset = offset + 4; // sizeof(u32)
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            }

            // copy color into (x, -1)
            if y == 0 {
                let offset = offset - 4*mtw as usize; // sizeof(u32)
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            }
            // copy color into (x, h)
            else if y == texture_height - 1 {
                let offset = offset + 4*mtw as usize; // sizeof(u32)
                let mapped_texture_dest = &mut texdata[offset as usize..][..4];
                mapped_texture_dest.copy_from_slice(&color);
                //mapped_texture_dest.copy_from_slice(&BORDER_COLOR);
            }
        };

        match (current_tile.format, current_tile.size) {
            (0, 2) => { // RGBA_16b
                self.map_tmem_rgba_16b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (0, 3) => { // RGBA_32b
                self.map_tmem_rgba_32b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (2, 0) => { // CI_4b
                self.map_tmem_ci_4b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (2, 1) => { // CI_8b
                self.map_tmem_ci_8b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (3, 0) => { // IA_4b
                self.map_tmem_ia_4b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (3, 1) => { // IA_8b
                self.map_tmem_ia_8b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (3, 2) => { // IA_16b
                self.map_tmem_ia_16b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (4, 0) => { // I_4b
                self.map_tmem_i_4b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },

            (4, 1) => { // I_8b
                self.map_tmem_i_8b(current_tile.tmem as u32, texture_width, texture_height, line_bytes, plot);
            },
            
            _ => {
                warn!(target: "HLE", "unsupported texture format ({}, {})", current_tile.format, current_tile.size);
                todo!();
            },
        };

        // put the mapped texture back into the array
        drop(mapped_texture);
        let _ = std::mem::replace(self.mapped_textures.get_mut(ti as usize).unwrap(), mapped_texture_lock);

        // mapped tile needs to be used to transform upcoming texture coordinates
        let current_tile = &mut self.tex.rdp_tiles[current_tile_index as usize];
        current_tile.mapped_coordinates = Some((ti, mx, my));
        self.tex.mapped_texture_cache.insert(crc, (ti, mx, my));
    }

    fn handle_tri1(&mut self) { // G_TRI1
        let (v0, v1, v2) = match self.software_version {
            HleRspSoftwareVersion::S3DEX2 => {
                let v0 = (((self.command >> 16) & 0xFF) / 10) as u16;
                let v1 = (((self.command >>  8) & 0xFF) / 10) as u16;
                let v2 = (((self.command >>  0) & 0xFF) / 10) as u16;
                (v0, v1, v2)
            },
            HleRspSoftwareVersion::F3DEX2 => { 
                let v0 = ((self.command >> 49) & 0x1F) as u16;
                let v1 = ((self.command >> 41) & 0x1F) as u16;
                let v2 = ((self.command >> 33) & 0x1F) as u16;
                (v0, v1, v2)
            },
            _ => todo!(),
        };

        trace!(target: "HLE", "{} gsSP1Triangle({}, {}, {})", self.command_prefix, v0, v1, v2);

        // if a previous *RECT call forced depth we need to re-enable depth
        if self.disable_depth_override.is_some() {
            self.disable_depth_override = None;
            self.check_othermode_state_change();
        }

        // make sure textures are mapped
        self.map_current_textures();

        // transform texture coordinates
        let vi = self.vertex_stack.get(v0 as usize).unwrap();
        let (_iv, v0) = self.finalize_vertex(*vi as usize).unwrap();
        //println!("iv0: {:?}", _iv);
        let vi = self.vertex_stack.get(v1 as usize).unwrap();
        let (_iv, v1) = self.finalize_vertex(*vi as usize).unwrap();
        //println!("iv1: {:?}", _iv);
        let vi = self.vertex_stack.get(v2 as usize).unwrap();
        let (_iv, v2) = self.finalize_vertex(*vi as usize).unwrap();
        //println!("iv2: {:?}", _iv);

        // adjust color according to G_SHADE_SMOOTH. If smooth shading is not enabled,
        // all other vertices take the color of the first vertex
        let is_lit = (self.vertices[v0 as usize].flags & VertexFlags::LIT) != 0;
        if !is_lit && (self.geometry_mode & 0x0020_0004) == 0x04 { // G_SHADE and !G_SHADE_SMOOTH (TODO: this value is different in different ucode!)
            self.vertices[v1 as usize].color = self.vertices[v0 as usize].color;
            self.vertices[v2 as usize].color = self.vertices[v0 as usize].color;
        }

        // place indices into draw list
        self.add_triangles(RenderPassType::DrawTriangles, &[v0, v1, v2]);
    }

    fn handle_tri2(&mut self) { // G_TRI2
        let v00 = ((self.command >> 49) & 0x1F) as u16;
        let v01 = ((self.command >> 41) & 0x1F) as u16;
        let v02 = ((self.command >> 33) & 0x1F) as u16;
        let v10 = ((self.command >> 17) & 0x1F) as u16;
        let v11 = ((self.command >>  9) & 0x1F) as u16;
        let v12 = ((self.command >>  1) & 0x1F) as u16;
        trace!(target: "HLE", "{} gsSP2Triangle({}, {}, {}, 0, {}, {}, {}, 0)", self.command_prefix, v00, v01, v02, v10, v11, v12);

        // if a previous *RECT call forced depth we need to re-enable depth
        if self.disable_depth_override.is_some() {
            self.disable_depth_override = None;
            self.check_othermode_state_change();
        }

        // make sure texture are mapped
        self.map_current_textures();

        // translate to global vertex stack
        let vi = self.vertex_stack.get(v00 as usize).unwrap();
        let (_, v00) = self.finalize_vertex(*vi as usize).unwrap();
        let vi = self.vertex_stack.get(v01 as usize).unwrap();
        let (_, v01) = self.finalize_vertex(*vi as usize).unwrap();
        let vi = self.vertex_stack.get(v02 as usize).unwrap();
        let (_, v02) = self.finalize_vertex(*vi as usize).unwrap();
        let vi = self.vertex_stack.get(v10 as usize).unwrap();
        let (_, v10) = self.finalize_vertex(*vi as usize).unwrap();
        let vi = self.vertex_stack.get(v11 as usize).unwrap();
        let (_, v11) = self.finalize_vertex(*vi as usize).unwrap();
        let vi = self.vertex_stack.get(v12 as usize).unwrap();
        let (_, v12) = self.finalize_vertex(*vi as usize).unwrap();

        // adjust color according to G_SHADE_SMOOTH. If smooth shading is not enabled,
        // all other vertices take the color of the first vertex
        let is_lit = (self.vertices[v00 as usize].flags & VertexFlags::LIT) != 0;
        if !is_lit && (self.geometry_mode & 0x0020_0004) == 0x04 { // G_SHADE and !G_SHADE_SMOOTH (TODO: this value is different in different ucode!)
            self.vertices[v01 as usize].color = self.vertices[v00 as usize].color;
            self.vertices[v02 as usize].color = self.vertices[v00 as usize].color;
            self.vertices[v11 as usize].color = self.vertices[v10 as usize].color;
            self.vertices[v12 as usize].color = self.vertices[v10 as usize].color;
        }

        self.add_triangles(RenderPassType::DrawTriangles, &[v00, v01, v02, v10, v11, v12]);
    }

    fn handle_quad(&mut self) { // G_QUAD
        // equiv. to TRI2
        trace!(target: "HLE", "{} next_call_is_actually gsSPQuadrangle(...)", self.command_prefix);
        self.handle_tri2();
    }

    fn handle_texrect(&mut self) { // G_TEXRECT (S3DEX2, F3DEX2)
        let cmd1 = self.next_display_list_command();
        let cmd2 = self.next_display_list_command();
        self.command_words += 4;

        let lrx  = ((self.command >> 44) & 0xFFF) as u16;
        let lry  = ((self.command >> 32) & 0xFFF) as u16;
        let tile = ((self.command >> 24) & 0x07) as u8;
        let ulx  = ((self.command >> 12) & 0xFFF) as u16;
        let uly  = ((self.command >>  0) & 0xFFF) as u16;
        let uls  = (cmd1 >> 16) as i16;
        let ult  = (cmd1 >>  0) as i16;
        let dsdx = (cmd2 >> 16) as i16;
        let dtdy = (cmd2 >>  0) as i16;

        let (vw, vh) = match self.current_viewport.clone() {
            Some(Viewport { w: vw, h: vh, .. }) => (vw, vh),
            _ => {
                // no viewport set?
                //warn!(target: "HLE", "gsSPTextureRectangle with empty viewport!");
                (319.0, 239.0)
            },
        };

        // convert values to fp
        let ulx  = (ulx  as f32) / 4.0;
        let uly  = (uly  as f32) / 4.0;
        let lrx  = (lrx  as f32) / 4.0;
        let lry  = (lry  as f32) / 4.0;
        let uls  = (uls  as f32) / 32.0;
        let ult  = (ult  as f32) / 32.0;
        let dsdx = (dsdx as f32) / 1024.0;
        let dtdy = (dtdy as f32) / 1024.0;
        trace!(target: "HLE", "{} gsSPTextureRectangle(ulx={}, uly={}, lrx={}, lry={}, tile={}, uls={}, ult={}, dsdx={}, dtdy={})", self.command_prefix, ulx, uly, lrx, lry, tile, uls, ult, dsdx, dtdy);

        let w    = lrx - ulx;
        let h    = lry - uly;
        let sw   = w * dsdx;
        let th   = h * dtdy;

        // apparently textures don't have to be enabled with gSPTexture to use TextureRectangle..
        let old_enabled = self.tex.enabled;
        self.tex.enabled = true;
        let old_tile = self.tex.tile;
        self.tex.tile = tile;

        self.map_current_textures();

        // map screen coordinate to -1..1
        // invert y
        let scale_x = |s, maxs| ((s / maxs) * 2.0) - 1.0;
        let scale_y = |s, maxs| (((s / maxs) * 2.0) - 1.0) * -1.0;

        let cur_pos = self.vertices_internal.len() as u16; // save start index
        self.vertices_internal.push(Vertex { // TL
            view_position  : [scale_x(ulx, vw), scale_y(uly, vh), 0.0, 1.0], 
            color          : [1.0, 0.0, 0.0, 1.0], 
            tex_coords     : [uls, ult], 
            ..Default::default()
        });
        self.vertices_internal.push(Vertex { // TR
            view_position  : [scale_x(ulx+w, vw), scale_y(uly, vh), 0.0, 1.0], 
            color          : [0.0, 1.0, 0.0, 1.0], 
            tex_coords     : [uls+sw, ult], 
            ..Default::default()
        });
        self.vertices_internal.push(Vertex { // BL
            view_position  : [scale_x(ulx, vw), scale_y(uly+h, vh), 0.0, 1.0], 
            color          : [0.0, 0.0, 1.0, 1.0], 
            tex_coords     : [uls, ult+th], 
            ..Default::default()
        });
        self.vertices_internal.push(Vertex { // BR
            view_position  : [scale_x(ulx+w, vw), scale_y(uly+h, vh), 0.0, 1.0], 
            color          : [1.0, 0.0, 1.0, 1.0], 
            tex_coords     : [uls+sw, ult+th], 
            ..Default::default()
        });

        let (_, v0) = self.finalize_vertex((cur_pos+0) as usize).unwrap();
        let (_, v1) = self.finalize_vertex((cur_pos+1) as usize).unwrap();
        let (_, v2) = self.finalize_vertex((cur_pos+2) as usize).unwrap();
        let (_, v3) = self.finalize_vertex((cur_pos+3) as usize).unwrap();

        // start or change the current draw list to use matrix 0 (our ortho projection)
        // and disable the depth buffer
        self.matrix_index_override = Some(0);
        self.disable_depth_override = Some(());
        self.add_triangles(RenderPassType::DrawTriangles, &[v0, v1, v2, v1, v2, v3]);

        self.tex.enabled = old_enabled;
        self.tex.tile = old_tile;

        // G_TRI* calls coming up will need to change to a new draw call if depth state
        // is different than being disabled, so we reuse disable_depth_override
        self.disable_depth_override = Some(());
    }

    fn handle_settilesize(&mut self) { // G_SETTILESIZE (S3DEX2, F3DEX2)
        let uls = ((self.command >> 44) & 0xFFF) as u16;
        let ult = ((self.command >> 32) & 0xFFF) as u16;
        let tile = ((self.command >> 24) & 0x07) as u8;
        let lrs = ((self.command >> 12) & 0xFFF) as u16;
        let lrt = ((self.command >>  0) & 0xFFF) as u16;

        // ul and lr coordinates are 10.2
        let rdp_tile = &mut self.tex.rdp_tiles[tile as usize];
        rdp_tile.ul = ((uls as f32) / 4.0, (ult as f32) / 4.0);
        rdp_tile.lr = ((lrs as f32) / 4.0, (lrt as f32) / 4.0);

        trace!(target: "HLE", "{} gsDPSetTileSize(tile={}, uls={} [{}], ult={} [{}], lrs={} [{}], lrt={} [{}])", 
               self.command_prefix, tile, uls, rdp_tile.ul.0, ult, rdp_tile.ul.1, lrs, rdp_tile.lr.0, lrt, rdp_tile.lr.1);
    }

    fn handle_loadblock(&mut self) { // G_LOADBLOCK (S3DEX2, F3DEX2)
        let to_f32 = |i| (i as f32) / (4.0 * 1024.0);
        let uls    = ((self.command >> 44) & 0xFFF) as u16;
        let ult    = ((self.command >> 32) & 0xFFF) as u16;
        let tile   = ((self.command >> 24) & 0x07) as u8;
        let texels = (((self.command >> 12) & 0xFFF) as u32) + 1;
        let dxt    = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPLoadBlock(tile={}, uls={} [{}], ult={} [{}], texels={}, dxt={})", self.command_prefix, 
                                    tile, uls, to_f32(uls), ult, to_f32(ult), texels, dxt);

        if uls != 0 || ult != 0 { warn!(target: "HLE", "TODO: non-zero uls/t coordinate"); }

        // size of data to load
        let data_size = match self.tex.size {
            0 => texels >> 1,      // 4b
            1 => texels,           // 8b
            2 => texels * 2,       // 16b
            3 => texels * 4,       // 32b
            5 => todo!("SIZ_DD?"), // DD
            _ => panic!("invalid texture size"), // invalid
        };

        let selected_tile = &self.tex.rdp_tiles[tile as usize];

        // RDP is limited to 2048 texel loads
        if texels > 2048 { 
            warn!(target: "HLE", "{} gsDPLoadBlock(tile={}, uls={} [{}], ult={} [{}], texels={}, dxt={})", self.command_prefix, 
                                        tile, uls, to_f32(uls), ult, to_f32(ult), texels, dxt);
            warn!(target: "HLE", "format={} size={}", selected_tile.format, selected_tile.size);
            warn!(target: "HLE", "too large of copy, {} bytes from ${:08X} to TMEM=${:04X} -- IGNORING", data_size, self.tex.address, selected_tile.tmem << 3);
            return; 
        }

        // load all texels from rdram in one go
        let padded_size = (data_size + 7) & !7;
        let data = if ((self.tex.address + padded_size) & 0x7FFF_FFFF) >= 0x0080_0000 {
            warn!(target: "HLE", "invalid read outside of RDRAM (address=${:08X}, length={})!", self.tex.address, padded_size);
            let mut v = Vec::new();    
            v.resize((padded_size >> 2) as usize, 0xFF0000FF);
            v
        } else {
            self.load_from_rdram(self.tex.address, padded_size)
        };

        // copy line by line, incrementing a counter by dxt, and every time the whole value portion
        // of the counter rolls over, increment the line number
        // if dxt is nonzero, we know how tall the texture is, otherwise the texture has been pre-swizzled
        // TODO it would be nice to not swizzle at all since it costs performance, but some load
        // blocks with dxt=0 transfer preswizzled data. I could mark address selected_tile.tmem as
        // swizzed or not, but a program could use a tmem address that's inside the data that's
        // loaded here.
        let dst = &mut self.tex.tmem[(selected_tile.tmem << 1) as usize..]; // destination in tmem -- (tmem * sizeof(u64)) / sizeof(u32)
        let mut dest_offset   = 0 as usize;
        let mut source_offset = 0 as usize;

        let dxt               = (dxt as f32) / 2048.0; // 1.11
        let mut counter       = 0.0f32;
        let mut whole_part    = counter as u32;
        let mut line_number   = 0u32;

        trace!(target: "HLE", "copy {} bytes from ${:08X} to TMEM=${:04X}", data_size, self.tex.address, selected_tile.tmem << 3);
        for _ in 0..(data_size / 8) { // number of 64-bit dwords to copy
            let src_ptr = &data[source_offset..source_offset+2]; // grab two 32-bit words
            source_offset += 2;

            match selected_tile.size {
                // TODO check swizzling on 8b textures, not sure if we can just use this.
                1 | 2 => { // 16b
                    // rows are 4 texels long
                    if (line_number & 0x01) == 0 {
                        if dst[dest_offset..].len() < 2 || src_ptr.len() < 2 {
                            error!(target: "HLE", "{} gsDPLoadBlock(tile={}, uls={} [{}], ult={} [{}], texels={}, dxt={})", self.command_prefix, 
                                                        tile, uls, to_f32(uls), ult, to_f32(ult), texels, dxt);
                            error!(target: "HLE", "copy {} bytes from ${:08X} to TMEM=${:04X}", data_size, self.tex.address, selected_tile.tmem << 3);
                            error!(target: "HLE", "format={} size={}", selected_tile.format, selected_tile.size);
                        }
                        dst[dest_offset+0] = src_ptr[0];
                        dst[dest_offset+1] = src_ptr[1];
                    } else {
                        dst[dest_offset+0] = src_ptr[1];
                        dst[dest_offset+1] = src_ptr[0];
                    }
                    dest_offset += 2;
                },
                3 => { // 32b
                    // 32b mode stores red+green in low half of tmem and blue+alpha in high mem
                    // incoming data is already padded appropriately but not split in RG/BA
                    if (line_number & 0x01) == 0 {
                        // red and green from the two texels into the low half
                        dst[dest_offset | 0x000] = (src_ptr[0] & 0xFFFF0000) | ((src_ptr[1] & 0xFFFF0000) >> 16);
                        // blue and alpha from the two texels into the high half
                        dst[dest_offset | 0x200] = ((src_ptr[0] & 0x0000FFFF) << 16) | (src_ptr[1] & 0x0000FFFF);
                    } else {
                        // swizzled
                        dst[(dest_offset^1) | 0x000] = (src_ptr[0] & 0xFFFF0000) | ((src_ptr[1] & 0xFFFF0000) >> 16); // RG
                        dst[(dest_offset^1) | 0x200] = ((src_ptr[0] & 0x0000FFFF) << 16) | (src_ptr[1] & 0x0000FFFF); // BA
                    }

                    dest_offset += 1;
                },
                _ => todo!("swizzle LOADBLOCK for other sizes"),
            };

            // counter is incremented for every 64-bits of texture read, or 2 u32s
            counter += dxt;
            if whole_part != (counter as u32) {
                whole_part = counter as u32;
                line_number += 1;
            }
        }

        self.num_texels += texels;
        self.total_texture_data += padded_size;

        // have to clear all mapped coordinates since textures are changing
        for rdp_tile in &mut self.tex.rdp_tiles {
            rdp_tile.mapped_coordinates = None;
            rdp_tile.has_texel1 = false;
        }
    }

    fn handle_loadtile(&mut self) { // G_LOADTILE (S3DEX2, F3DEX2)
        let uls  = ((self.command >> 44) & 0xFFF) as u16;
        let ult  = ((self.command >> 32) & 0xFFF) as u16;
        let tile = ((self.command >> 24) & 0x07) as u8;
        let lrs  = ((self.command >> 12) & 0xFFF) as u16;
        let lrt  = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPLoadTile({}, {}, {}, {}, {})", self.command_prefix, tile, uls, ult, lrs, lrt);

        // convert to fp
        let uls  = (uls as f32) / 4.0;
        let ult  = (ult as f32) / 4.0;
        let lrs  = (lrs as f32) / 4.0;
        let lrt  = (lrt as f32) / 4.0;

        let tile_width  = (lrs - uls) as u32 + 1;
        let tile_height = (lrt - ult) as u32 + 1;

        let rdp_tile    = self.tex.rdp_tiles[tile as usize];

        // adjust_size gives byte size from texel count
        let tile_size   = rdp_tile.size;
        let adjust_size = |texels: u32| match tile_size {
            0 => texels >> 1, // 4b
            1 => texels     , // 8b
            2 => texels << 1, // 16b
            3 => texels << 2, // 32b
            5 => todo!("SIZ_DD"),
            _ => { error!(target: "HLE", "invalid texture size"); 0 },
        };

        // load enough data to cover the entire tile, despite loading too much data
        // it may be (seems, at least) better than locking rdram once for every row
        let image_bytes_per_row = adjust_size(self.tex.width as u32); // width of the texture in DRAM
        // the amount of data to load is determined by the width of the tile
        let tile_bytes_per_row = adjust_size(tile_width);           // width of the tile in qwords
        let tile_words_per_row = (tile_bytes_per_row >> 2) as usize;

        //println!("image_bytes_per_row={} tile_bytes_per_row={} uls={} ult={} lrs={} lrt={} tile_width={} tile_height={}", 
        //         image_bytes_per_row, tile_bytes_per_row, uls, ult, lrs, lrt, tile_width, tile_height);

        // start address = base + (y * texture_width + x) * bpp
        let start_dram = self.tex.address + (ult as u32) * image_bytes_per_row + adjust_size(uls as u32);

        // load enough data to contain the entire tile
        let load_size = (tile_height - 1) * image_bytes_per_row + tile_bytes_per_row;
        let image_data = self.load_from_rdram(start_dram, load_size);

        // rdp_tile.tmem is in 64-bit words, tmem[] is 32-bit
        let dst = &mut self.tex.tmem[(rdp_tile.tmem << 1) as usize..];

        // TODO if this happens, a tile is being uploaded starting at an odd line.  I don't know if
        // that matters, actually, but might be worth checking into.
        if ((rdp_tile.tmem / rdp_tile.line) & 0x01) != 0 { todo!(); }

        // loop over the rows and swizzle as necessary
        let mut source_offset = 0;
        let mut dest_offset   = 0;
        for y in 0..(tile_height as usize) {
            let mut row = image_data[source_offset..][..tile_words_per_row].to_owned();
            source_offset += (image_bytes_per_row >> 2) as usize; // increment by one row in DRAM

            // store data in TMEM based on size
            match rdp_tile.size { 
                1 | 2 => { // 8,16b
                    if (y & 0x01) == 1 {
                        // if tile_words_per_row was odd, extend with one blank word
                        if (tile_words_per_row % 2) == 1 {
                            row.push(0);
                        }
                        // now swap for TMEM layout
                        for r in row.chunks_mut(2) {
                            r.swap(0, 1);
                        }
                    }

                    dst[dest_offset..][..row.len()].copy_from_slice(&row);

                    // increment by one row in TMEM
                    dest_offset += (rdp_tile.line as usize) << 1;
                },

                3 => { // 32b half is stored in low memory, other half in high memory
                    let mut i = 0;
                    for texels in row.chunks_mut(2) {
                        if (y & 0x01) == 0 {
                            dst[(( dest_offset+i)   & 0x1FF) | 0x000] = (texels[0] & 0xFFFF0000) | (texels[1] >> 16);
                            dst[(( dest_offset+i)   & 0x1FF) | 0x200] = (texels[0] << 16)        | (texels[1] & 0x0000FFFF);
                        } else {
                            dst[(((dest_offset+i)^1)& 0x1FF) | 0x000] = (texels[0] & 0xFFFF0000) | (texels[1] >> 16);
                            dst[(((dest_offset+i)^1)& 0x1FF) | 0x200] = (texels[0] << 16)        | (texels[1] & 0x0000FFFF);
                        }
                        i += 1;
                    }

                    // the increment in TMEM is determined by the line value of SETTILE
                    dest_offset += (rdp_tile.line as usize) << 1;
                },

                _ => todo!("load tile size {}", rdp_tile.size),
            }
        }

        let texels = (tile_width * tile_height) as u32;
        self.num_texels += texels;
        self.total_texture_data += adjust_size(texels);

        // have to clear all mapped coordinates since textures are changing
        for rdp_tile in &mut self.tex.rdp_tiles {
            rdp_tile.mapped_coordinates = None;
            rdp_tile.has_texel1 = false;
        }
    }

    fn handle_settile(&mut self) { // G_SETTILE (S3DEX2, F3DEX2)
        let fmt     = (self.command >> 53) & 0x07;
        let sz      = (self.command >> 51) & 0x03;
        let line    = (self.command >> 41) & 0x1FF;
        let tmem    = (self.command >> 32) & 0x1FF;
        let tile    = ((self.command >> 24) & 0x07) as usize;
        let pal     = (self.command >> 20) & 0x0F;
        let clamp_t = (self.command >> 18) & 0x03;
        let mask_t  = (self.command >> 14) & 0x0F;
        let shift_t = (self.command >> 10) & 0x0F;
        let clamp_s = (self.command >>  8) & 0x03;
        let mask_s  = (self.command >>  4) & 0x0F;
        let shift_s = (self.command >>  0) & 0x0F;

        let fmtstr = match fmt {
            0 => "G_IM_FMT_RGBA", 1 => "G_IM_FMT_YUV", 2 => "G_IM_FMT_CI", 3 => "G_IM_FMT_IA",
            4 => "G_IM_FMT_I", _ => "G_IM_FMT_unknown",
        };

        let szstr = match sz {
            0 => "G_IM_SIZ_4b", 1 => "G_IM_SIZ_8b", 2 => "G_IM_SIZ_16b", 3 => "G_IM_SIZ_32b",
            5 => "G_IM_SIZ_DD", _ => "G_IM_SIZ_unknown",
        };

        trace!(target: "HLE", "{} gsDPSetTile({}, {}, line={}, tmem=0x{:03X}, tile={}, pal={}, cmT={}, maskT={}, shiftT={}, cmS={}, maskS={}, shiftS={})",
                    self.command_prefix, fmtstr, szstr, line, tmem, tile, pal, clamp_t, mask_t, shift_t, clamp_s, mask_s, shift_s);

        self.tex.rdp_tiles[tile].format  = fmt     as u8;
        self.tex.rdp_tiles[tile].size    = sz      as u8;
        self.tex.rdp_tiles[tile].line    = line    as u16;
        self.tex.rdp_tiles[tile].tmem    = tmem    as u16;
        self.tex.rdp_tiles[tile].palette = pal     as u8;
        self.tex.rdp_tiles[tile].clamp_s = clamp_s as u8;
        self.tex.rdp_tiles[tile].clamp_t = clamp_t as u8;
        self.tex.rdp_tiles[tile].mask_s  = mask_s  as u8;
        self.tex.rdp_tiles[tile].mask_t  = mask_t  as u8;
        self.tex.rdp_tiles[tile].shift_s = shift_s as u8;
        self.tex.rdp_tiles[tile].shift_t = shift_t as u8;
    }

    fn handle_cleargeometrymode(&mut self) { // G_CLEARGEOMETRYMODE (S3DEX2)
        let bits = self.command as u32;
        trace!(target: "HLE", "{} gsSPClearGeometryMode(0x{:08X})", self.command_prefix, bits);
        self.geometry_mode &= !bits;
    }

    fn handle_setgeometrymode(&mut self) { // G_SETGEOMETRYMODE (S3DEX2)
        let bits = self.command as u32;
        trace!(target: "HLE", "{} gsSPSetGeometryMode(0x{:08X})", self.command_prefix, bits);
        self.geometry_mode |= bits;
    }

    fn handle_geometrymode(&mut self) { // G_GEOMETRYMODE (F3DEX2)
        let clearbits = !(((self.command >> 32) & 0x00FF_FFFF) as u32);
        let setbits   = self.command as u32;
        if clearbits == 0 && setbits != 0 {
            trace!(target: "HLE", "{} gsSPLoadGeometryMode(0x{:08X})", self.command_prefix, setbits);
        } else if clearbits != 0 && setbits == 0 {
            trace!(target: "HLE", "{} gsSPClearGeometryMode(0x{:08X})", self.command_prefix, clearbits);
        } else {
            trace!(target: "HLE", "{} gsSPGeometryMode(0x{:08X}, 0x{:08X})", self.command_prefix, clearbits, setbits);
        }

        self.geometry_mode = (self.geometry_mode & !clearbits) | setbits;
    }

    fn handle_rdploadsync(&mut self) { // G_RDPLOADSYNC (S3DEX2, F3DEX2)
        trace!(target: "HLE", "{} gsDPLoadSync()", self.command_prefix);
    }

    fn handle_rdppipesync(&mut self) { // G_RDPPIPESYNC (S3DEX2, F3DEX2)
        trace!(target: "HLE", "{} gsDPPipeSync()", self.command_prefix);
    }

    fn handle_rdptilesync(&mut self) { // G_RDPTILESYNC (S3DEX2, F3DEX2)
        trace!(target: "HLE", "{} gsDPTileSync()", self.command_prefix);
    }

    fn handle_rdpfullsync(&mut self) { // G_RDPFULLSYNC (S3DEX2, F3DEX2)
        trace!(target: "HLE", "{} gsDPFullSync()", self.command_prefix);
        self.comms.rdp_full_sync.store(1, Ordering::SeqCst);
    }

    fn handle_loadlut(&mut self) { // G_LOADTLUT (S3DEX2, F3DEX2)
        let tile  = ((self.command >> 24) & 0x07) as u8;
        let count = ((self.command >> 14) & 0x03FF) + 1;
        trace!(target: "HLE", "{} gsDPLoadTLUTCmd({}, {})", self.command_prefix, tile, count);

        let rdp_tile = &self.tex.rdp_tiles[tile as usize];

        // TLUT is always 16b
        let mut load_size = (count << 1) as u32; // in bytes

        // prevent overflow of tmem
        let dest_addr = (rdp_tile.tmem as u32) << 1; // convert address to u32/word index
        if (dest_addr + (load_size >> 2)) >= 0x400 {
            load_size = (0x400 - dest_addr) << 2;
        }

        if (self.tex.address & 0x07) != 0 {
            warn!(target: "HLE", "TLUT source address is not 64b aligned");
        }

        // load tlut data into tmem starting at rdp_tiles.tmem from rdram self.tex.address
        let tlut_data = self.load_from_rdram(self.tex.address, load_size);

        for i in 0..count as u32 {
            let src_shift = 16 - ((i & 1) << 4);
            let src_color = (tlut_data[(i >> 1) as usize] >> src_shift) & 0xFFFF;

            let da = dest_addr + (i >> 1);
            let dst_shift = 16 - ((i & 1) << 4);
            let v = &mut self.tex.tmem[da as usize];
            *v = (*v & (0xFFFF0000 >> dst_shift)) | (src_color << dst_shift);
        }
    }

    // start a new render pass if any of the critical render states change, i.e., depth compare is disabled
    fn check_othermode_state_change(&mut self) {
        if self.current_triangle_list().depth_compare_enable != self.other_modes.get_depth_compare_enable() {
            self.next_triangle_list();
            return;
        }

        if self.current_triangle_list().depth_write != self.other_modes.get_depth_update_enable() {
            self.next_triangle_list();
            return;
        }
    }

    fn handle_rdpsetothermode(&mut self) { // G_(RDP)SETOTHERMODE
        let hi = ((self.command >> 32) & 0x00FF_FFFF) as u32;
        let lo = self.command as u32;
        trace!(target: "HLE", "{} gsDPSetOtherMode(0x{:08X}, 0x{:08X})", self.command_prefix, hi, lo);
        self.other_modes.hi = hi;
        self.other_modes.lo = lo;

        self.check_othermode_state_change();
    }

    fn handle_setothermode_h(&mut self) { // G_SETOTHERMODE_H (S3DEX2, F3DEX2)
        let (length, shift) = match self.software_version {
            HleRspSoftwareVersion::F3DEX2 => {
                let length = ((self.command >> 32) & 0xFF) + 1;
                let shift = 32 - length - ((self.command >> 40) & 0xFF);
                (length, shift)
            },

            _ => {
                let length = (self.command >> 32) & 0xFF;
                let shift = (self.command >> 40) & 0xFF;
                (length, shift)
            }
        };

        let data = self.command as u32;

        trace!(target: "HLE", "{} gsSPSetOtherModeH(shift={}, length={}, data=0x{:08X})", self.command_prefix, shift, length, data);

        let mask = ((1 << length) - 1) << shift;
        self.other_modes.hi &= !mask;
        self.other_modes.hi |= data & mask;

        self.check_othermode_state_change();
    }

    fn handle_setothermode_l(&mut self) { // G_SETOTHERMODE_L (S3DEX2, F3DEX2)
        let (length, shift) = match self.software_version {
            HleRspSoftwareVersion::F3DEX2 => {
                let length = ((self.command >> 32) & 0xFF) + 1;
                let shift = 32 - length - ((self.command >> 40) & 0xFF);
                (length, shift)
            },

            _ => {
                let length = (self.command >> 32) & 0xFF;
                let shift = (self.command >> 40) & 0xFF;
                (length, shift)
            }
        };

        let data = self.command as u32;

        trace!(target: "HLE", "{} gsSPSetOtherModeL(shift={}, length={}, data=0x{:08X})", self.command_prefix, shift, length, data);

        let mask = ((1 << length) - 1) << shift;
        self.other_modes.lo &= !mask;
        self.other_modes.lo |= data & mask;

        self.check_othermode_state_change();

        // Opaque surface: has Z_CMP, Z_UPD, ALPHA_CVG_SEL,           GL_BL_A_MEM
        // Transl surface: has               CLR_ON_CVG,    FORCE_BL, G_BL_1MA
    }

    fn handle_obj_rendermode(&mut self) {
        trace!(target: "HLE", "{} gsDPObjectRenderMode???(...)", self.command_prefix);
    }

    fn handle_setprimdepth(&mut self) { // G_SETPRIMDEPTH
        trace!(target: "HLE", "{} gsDPSetPrimDepth(...)", self.command_prefix);
    }

    fn handle_setscissor(&mut self) { // G_SETSCISSOR (S3DEX2, F3DEX2)
        // TODO: I think the x/y values are 12bit 10.2 fixed point format?
        let x0 = ((self.command >> 44) & 0xFFF) as u16;
        let y0 = ((self.command >> 32) & 0xFFF) as u16;
        let m  = ((self.command >> 28) & 0x0F) as u8;
        let x1 = ((self.command >> 12) & 0xFFF) as u16;
        let y1 = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPSetScissor({}, {}, {}, {}, {})", self.command_prefix, m, x0, y0, x1 >> 2, y1 >> 2);
    }

    fn handle_setkeyr(&mut self) { // G_SETKEYR
        trace!(target: "HLE", "{} gsDPSetKeyR(...)", self.command_prefix);
    }

    fn handle_setkeygb(&mut self) { // G_SETKEYGB
        trace!(target: "HLE", "{} gsDPSetKeyGB(...)", self.command_prefix);
    }

    fn handle_setconvert(&mut self) { // G_SETCONVERT
        trace!(target: "HLE", "{} gsDPSetConvert(...)", self.command_prefix);
    }

    fn handle_fillrect(&mut self) { // G_FILLRECT (S3DEX2, F3DEX2)
        let x1 = (((self.command >> 44) & 0xFFF) as u16) >> 2;
        let y1 = (((self.command >> 32) & 0xFFF) as u16) >> 2;
        let x0 = (((self.command >> 12) & 0xFFF) as u16) >> 2;
        let y0 = (((self.command >>  0) & 0xFFF) as u16) >> 2;
        let w = x1 - x0 + 1;
        let h = y1 - y0 + 1;

        let color = match self.current_color_image_format {
            Some(HleRenderCommand::DefineColorImage { bpp, .. }) => {
                match bpp {
                    2 => { // 16b 
                        [((self.fill_color >> 11) & 0x1F) as f32 / 31.0, 
                         ((self.fill_color >> 6) & 0x1F) as f32 / 31.0,
                         ((self.fill_color >>  1) & 0x1F) as f32 / 31.0, 
                         if (self.fill_color & 0x01) != 0 { 1.0 } else { 0.0 }]
                    },
                    3 => { // 32b
                        [((self.fill_color >> 24) & 0xFF) as f32 / 255.0,
                         ((self.fill_color >> 16) & 0xFF) as f32 / 255.0,
                         ((self.fill_color >>  8) & 0xFF) as f32 / 255.0,
                         ((self.fill_color >>  0) & 0xFF) as f32 / 255.0]
                    },
                    _ => {
                        // Unimplemented mode
                        [0.0, 1.0, 1.0, 1.0]
                    },
                }
            },
            _ => {
                // No color image defined, give a weird color that stands out
                [1.0, 1.0, 0.0, 1.0]
            },
        };

        trace!(target: "HLE", "{} gsDPFillRectangle({}, {}, {}, {})", self.command_prefix, x0, y0, x1, y1);

        // we can only render fill rects when there's a valid viewport
        let (vx, _vy, vw, vh) = match self.current_viewport.clone() {
            Some(Viewport { x: vx, y: vy, w: vw, h: vh, .. }) => (vx, vy, vw, vh),
            _ => {
                // no viewport set?
                (0.0, 0.0, 320.0, 240.0)
            },
        };

        // Check if this rectangle is a full-screen (i.e., clear) render
        // and if so, make sure the color is compatible with a fill
        if self.other_modes.get_cycle_type() == CycleType::Fill {
            // Some games letterbox the top and bottom, so let's let them do that even though we
            // might be clearing area we aren't supposed to... (could be a graphical bug somewhere)
            let letterbox_size = y0;

            if (x0 == vx as u16 && w == vw as u16) && (h == (vh as u16) - letterbox_size * 2) {
                let addr = self.current_render_pass().color_buffer.clone().unwrap();
                match self.current_color_image_format {
                    Some(HleRenderCommand::DefineColorImage { bpp, .. }) => {
                        match bpp {
                            2 => { // 16b
                                if (self.fill_color >> 16) == (self.fill_color & 0xFFFF) { // can't have alternating colors
                                    // we can't just mark clear_color in the current render pass, because
                                    // the N64 uses the color buffer to clear depth buffers, so we mark this
                                    // address as cleared in a separate buffer and only when next_render_pass() is
                                    // called do we know that the render targets are set
                                    trace!(target: "HLE", "clear image ${:08X}", addr);
                                    self.clear_images.insert(addr, color);
                                    return;
                                }
                            },
                            3 => { // 32b
                                // the color isn't important because it's 32b, so any value will do. see note above
                                trace!(target: "HLE", "clear image ${:08X}", addr);
                                self.clear_images.insert(addr, color);
                                return;
                            },
                            _ => {},
                        }
                    },
                    _ => {},
                }
            }
        }

        // otherwise, we need to render this quad, which might require a new render pass
        // but the coordinates are specified in device coordinate space (0..320, 0..240, etc)
        // we can map them into view space (-1..1) and render a quad with depth testing disabled

        // map (0,vx) -> (-1,1)
        // so (rx/vx * 2) - 1
        let scale_x = |s, maxs| (((s as f32) / maxs) * 2.0) - 1.0;
        let scale_y = |s, maxs| ((((s as f32) / maxs) * 2.0) - 1.0) * -1.0;
        let cur_pos = self.vertices_internal.len() as u16; // save start index
        self.vertices_internal.push(Vertex {  // TL
            view_position: [ scale_x(x0  , vw), scale_y(y0+h, vh), 0.0, 1.0], 
            color: color, 
            ..Default::default() 
        });
        self.vertices_internal.push(Vertex {  // TR
            view_position: [ scale_x(x0+w, vw), scale_y(y0+h, vh), 0.0, 1.0], 
            color: color, 
            ..Default::default() 
        });
        self.vertices_internal.push(Vertex {  // BL
            view_position: [ scale_x(x0  , vw), scale_y(y0  , vh), 0.0, 1.0], 
            color: color, 
            ..Default::default() 
        }); 
        self.vertices_internal.push(Vertex {  // BR
            view_position: [ scale_x(x0+w, vw), scale_y(y0  , vh), 0.0, 1.0], 
            color: color, 
            ..Default::default() 
        });

        let (_, v0) = self.finalize_vertex((cur_pos+0) as usize).unwrap();
        let (_, v1) = self.finalize_vertex((cur_pos+1) as usize).unwrap();
        let (_, v2) = self.finalize_vertex((cur_pos+2) as usize).unwrap();
        let (_, v3) = self.finalize_vertex((cur_pos+3) as usize).unwrap();

        // start or change the current draw list to use matrix 0 (our ortho projection)
        // and disable the depth buffer
        self.matrix_index_override = Some(0);
        self.disable_depth_override = Some(());
        self.add_triangles(RenderPassType::DrawTriangles, &[v0, v1, v2, v1, v2, v3]);

        // G_TRI* calls coming up will need to change to a new draw call if depth state
        // is different than being disabled, so we reuse disable_depth_override
        self.disable_depth_override = Some(());
    }

    fn handle_setfogcolor(&mut self) { // G_SETFOGCOLOR (S3DEX2, F3DEX2)
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPSetFogColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);

        self.current_fog_state.color = [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, a as f32 / 255.0];
    }

    fn handle_setfillcolor(&mut self) { // G_SETFILLCOLOR (S3DEX2, F3DEX2)
        self.fill_color = self.command as u32;
        trace!(target: "HLE", "{} gsDPSetFillColor(0x{:08X})", self.command_prefix, self.fill_color);
    }

    fn handle_setblendcolor(&mut self) { // G_SETBLENDCOLOR (S3DEX2, F3DEX2)
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPBlendColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);

        // TODO This actually changes the blender, not the color combiner.  But who knows if the
        // performance matters here?
        self.current_color_combiner_state.blend_color = [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, a as f32 / 255.0];

        self.color_combiner_state_changed();
    }

    fn handle_setprimcolor(&mut self) { // G_SETPRIMCOLOR (S3DEX2, F3DEX2)
        let minlevel = (self.command >> 40) as u8;
        let lodfrac  = (self.command >> 32) as u8;
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;

        trace!(target: "HLE", "{} gsDPSetPrimColor({}, {}, {}, {}, {}, {})", self.command_prefix, minlevel, lodfrac, r, g, b, a);

        self.current_color_combiner_state.prim_color = [
            (r as f32) / 255.0,
            (g as f32) / 255.0,
            (b as f32) / 255.0,
            (a as f32) / 255.0
        ];

        self.current_color_combiner_state.lodfrac = (lodfrac as f32) / 256.0;

        self.color_combiner_state_changed();
    }

    fn handle_setenvcolor(&mut self) { // G_SETENVCOLOR (S3DEX2, F3DEX2)
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPSetEnvColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);

        self.current_color_combiner_state.env_color = [
            (r as f32) / 255.0,
            (g as f32) / 255.0,
            (b as f32) / 255.0,
            (a as f32) / 255.0
        ];

        self.color_combiner_state_changed();
    }

    fn handle_setcombine(&mut self) { // G_SETCOMBINE (S3DEX2, F3DEX2)
        let a0c = ((self.command >> 52) & 0x0F) as u8;
        let b0c = ((self.command >> 28) & 0x0F) as u8;
        let c0c = ((self.command >> 47) & 0x1F) as u8;
        let d0c = ((self.command >> 15) & 0x07) as u8;

        let a0a = ((self.command >> 44) & 0x07) as u8;
        let b0a = ((self.command >> 12) & 0x07) as u8;
        let c0a = ((self.command >> 41) & 0x07) as u8;
        let d0a = ((self.command >>  9) & 0x07) as u8;

        let a1c = ((self.command >> 37) & 0x0F) as u8;
        let b1c = ((self.command >> 24) & 0x0F) as u8;
        let c1c = ((self.command >> 32) & 0x1F) as u8;
        let d1c = ((self.command >>  6) & 0x07) as u8;

        let a1a = ((self.command >> 21) & 0x07) as u8;
        let b1a = ((self.command >>  3) & 0x07) as u8;
        let c1a = ((self.command >> 18) & 0x07) as u8;
        let d1a = ((self.command >>  0) & 0x07) as u8;

        trace!(target: "HLE", "{} gsDPSetCombineLERP({}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {})", self.command_prefix,
                a0c, b0c, c0c, d0c, a0a, b0a, c0a, d0a, a1c, b1c, c1c, d1c, a1a, b1a, c1a, d1a);

        self.current_color_combiner_state.color1_source = u32::from_be_bytes([a0c, b0c, c0c, d0c]);
        self.current_color_combiner_state.alpha1_source = u32::from_be_bytes([a0a, b0a, c0a, d0a]);
        self.current_color_combiner_state.color2_source = u32::from_be_bytes([a1c, b1c, c1c, d1c]);
        self.current_color_combiner_state.alpha2_source = u32::from_be_bytes([a1a, b1a, c1a, d1a]);
        self.color_combiner_state_changed();
    }

    fn color_combiner_state_changed(&mut self) {
        // if the current triangle list isn't using the current combiner state, switch to a new list.
        // num_indices has to be non-zero for the color state index to be valid
        // if num_indices is 0, then the color index will be set on the next draw call
        let cc_index = self.current_triangle_list().color_combiner_state_index as usize;
        if (self.current_triangle_list().num_indices > 0)
            && (self.color_combiner_states[cc_index] != self.current_color_combiner_state) {
            self.next_triangle_list();
        }

        // set uses_texel1 if any color source makes use of TEXEL1
        let cc = &self.current_color_combiner_state;
        if   ((cc.color1_source & 0xFF00_0000) == 0x0200_0000) // G_CCMUX_TEXEL1
          || ((cc.color1_source & 0x00FF_0000) == 0x0002_0000) // G_CCMUX_TEXEL1
          || ((cc.color1_source & 0x0000_FF00) == 0x0000_0200) // G_CCMUX_TEXEL1
          || ((cc.color1_source & 0x0000_00FF) == 0x0000_0002) // G_CCMUX_TEXEL1
          || ((cc.color1_source & 0x0000_FF00) == 0x0000_0900) // G_CCMUX_TEXEL1_ALPHA 
          || ((cc.alpha1_source & 0xFF00_0000) == 0x0200_0000) // G_ACMUX_TEXEL1
          || ((cc.alpha1_source & 0x00FF_0000) == 0x0002_0000) // G_ACMUX_TEXEL1
          || ((cc.alpha1_source & 0x0000_FF00) == 0x0000_0200) // G_ACMUX_TEXEL1
          || ((cc.alpha1_source & 0x0000_00FF) == 0x0000_0002) // G_ACMUX_TEXEL1
          || ((cc.color2_source & 0xFF00_0000) == 0x0200_0000) // G_CCMUX_TEXEL1
          || ((cc.color2_source & 0x00FF_0000) == 0x0002_0000) // G_CCMUX_TEXEL1
          || ((cc.color2_source & 0x0000_FF00) == 0x0000_0200) // G_CCMUX_TEXEL1
          || ((cc.color2_source & 0x0000_00FF) == 0x0000_0002) // G_CCMUX_TEXEL1
          || ((cc.color2_source & 0x0000_FF00) == 0x0000_0900) // G_CCMUX_TEXEL1_ALPHA 
          || ((cc.alpha2_source & 0xFF00_0000) == 0x0200_0000) // G_ACMUX_TEXEL1
          || ((cc.alpha2_source & 0x00FF_0000) == 0x0002_0000) // G_ACMUX_TEXEL1
          || ((cc.alpha2_source & 0x0000_FF00) == 0x0000_0200) // G_ACMUX_TEXEL1
          || ((cc.alpha2_source & 0x0000_00FF) == 0x0000_0002) // G_ACMUX_TEXEL1
                                                               {
            self.color_combiner_uses_texel1 = true;
        } else {
            self.color_combiner_uses_texel1 = false;
        }
    }

    fn handle_settimg(&mut self) { // G_SETTIMG (S3DEX2, F3DEX2)
        let fmt   = (self.command >> 53) & 0x07;
        let sz    = (self.command >> 51) & 0x03;
        let width = (self.command >> 32) & 0x0FFF;
        let addr  = self.command as u32;

        let translated_addr = (if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x007F_FFFF)
        } & 0x007F_FFFF);

        let fmtstr = match fmt {
            0 => "G_IM_FMT_RGBA", 1 => "G_IM_FMT_YUV", 2 => "G_IM_FMT_CI", 3 => "G_IM_FMT_IA",
            4 => "G_IM_FMT_I", _ => "G_IM_FMT_unknown",
        };

        let szstr = match sz {
            0 => "G_IM_SIZ_4b", 1 => "G_IM_SIZ_8b", 2 => "G_IM_SIZ_16b", 3 => "G_IM_SIZ_32b",
            5 => "G_IM_SIZ_DD", _ => "G_IM_SIZ_unknown",
        };

        trace!(target: "HLE", "{} gsDPSetTextureImage({}, {}, width={}, 0x{:08X} [0x{:08X}])", self.command_prefix, fmtstr, szstr, width, addr, translated_addr);
        self.tex.format  = fmt as u8;
        self.tex.size    = sz as u8;
        self.tex.width   = (width as u16) + 1;
        self.tex.address = translated_addr;
    }

    fn handle_setzimg(&mut self) { // G_SETZIMG (S3DEX2, F3DEX2)
        let addr = self.command as u32;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            (self.segments[segment] + (addr & 0x007F_FFFF)) & 0x007F_FFFF
        } & 0x07FF_FFFF;

        trace!(target: "HLE", "{} gsDPSetDepthImage(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);

        // set before next_render_pass(), which will use the new value
        self.current_depth_image = Some(translated_addr);

        // if the depth image changes, start a new render pass
        if self.current_render_pass().depth_buffer.is_none() 
            || self.current_render_pass().depth_buffer.is_some_and(|v| v != translated_addr) {
            self.next_render_pass(Some(format!("new depth target ${:08X}", translated_addr)));
        }

        let hle_render_command = HleRenderCommand::DefineDepthImage { framebuffer_address: translated_addr };
        if !self.depth_images.contains_key(&translated_addr) {
            self.depth_images.insert(translated_addr, hle_render_command);
        }
    }

    fn handle_setcimg(&mut self) { // G_SETCIMG (S3DEX2, F3DEX2)
        let addr  = self.command as u32;
        let width = ((self.command >> 32) & 0x0FFF) as u16 + 1;
        let bpp   = ((self.command >> 51) & 0x03) as u8;
        let fmt   = ((self.command >> 53) & 0x07) as u8;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            (self.segments[segment] + (addr & 0x007F_FFFF)) & 0x007F_FFFF
        } & 0x07FF_FFFF;

        trace!(target: "HLE", "{} gsDPSetColorImage({}, {}, {}, 0x{:08X} [0x{:08X}])", self.command_prefix, fmt, bpp, width, addr, translated_addr);

        if fmt != 0 { // G_IM_FMT_RGBA
            error!("color targets not of RGBA not yet supported: {}", fmt);
            return;
        }

        // set before next_render_pass(), which will use this value
        self.current_color_image = Some(translated_addr);

        // if the color image changes, start a new render pass
        // I guess its possible there have been draw calls before setting a color image
        if self.current_render_pass().color_buffer.is_none() 
            || self.current_render_pass().color_buffer.is_some_and(|v| v != translated_addr) {
            self.next_render_pass(Some(format!("new color target ${:08X}", translated_addr)));
        }

        // create the color buffer
        let hle_render_command = HleRenderCommand::DefineColorImage { bpp: bpp, width: width, framebuffer_address: translated_addr };
        self.current_color_image_format = Some(hle_render_command.clone());
        if !self.color_images.contains_key(&translated_addr) {
            self.color_images.insert(translated_addr, hle_render_command);
        }
    }

    fn send_hle_render_command(&mut self, hle_render_command: HleRenderCommand) {
        loop {
            match self.hle_command_buffer.try_push(hle_render_command.clone()) {
                Ok(_) => break,
                Err(_) => continue,
            };
        }
    }

    fn process_display_list_command(&mut self, addr: u32, cmd: u64) {
        let depth = self.dl_stack.len() - 1;

        let mut spacing = String::new();
        for _ in 0..depth { spacing.push_str("  "); }

        self.command_prefix  = format!("${:08X}: ${:08X}_${:08X}:{}", addr, cmd >> 32, cmd & 0xFFFF_FFFF, spacing);
        self.command_address = addr;
        self.command_op      = (cmd >> 56) as u8;
        self.command         = cmd;
        self.command_words   = 2;
        self.command_table[self.command_op as usize](self);
    }
}

#[allow(dead_code)]
#[derive(Debug,PartialEq)]
enum AlphaDither {
    Pattern,
    NotPattern,
    Noise,
    Disable
}

#[derive(Debug, PartialEq)]
enum TextureFilter {
    Point,
    Average,
    Bilinear,
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum AlphaCompare {
    None = 0,
    Threshold = 1,
    Dither = 3
}

#[allow(dead_code)]
#[derive(Debug,PartialEq)]
enum TlutMode {
    None,
    Rgba16,
    Ia16,
}

#[allow(dead_code)]
#[derive(Debug,PartialEq)]
enum ZSourceSelect {
    Pixel,
    Primitive
}

#[derive(Debug,PartialEq)]
enum ZMode {
    Opaque = 0,
    Interpenetrating = 1,
    Translucent = 2,
    Decal = 3
}

#[allow(dead_code)]
#[derive(Debug,PartialEq)]
enum CycleType {
    OneCycle,
    TwoCycle,
    Copy,
    Fill
}

#[derive(Debug)]
struct OtherModes {
    lo: u32,
    hi: u32,
}

impl Default for OtherModes {
    fn default() -> Self {
        Self {
            lo: 0,
            hi: 0,
        }
    }
}

#[allow(dead_code)]
impl OtherModes {
    // HI
    const ALPHA_DITHER_SHIFT   : u32 = 4;
    const ALPHA_DITHER_MASK    : u32 = 0x03;
    const TEXTURE_FILTER_SHIFT : u32 = 12;
    const TEXTURE_FILTER_MASK  : u32 = 0x03;
    const TLUT_MODE_SHIFT      : u32 = 14;
    const TLUT_MODE_MASK       : u32 = 0x03;
    const CYCLE_TYPE_SHIFT     : u32 = 20;
    const CYCLE_TYPE_MASK      : u32 = 0x3;

    // LO
    const ALPHA_COMPARE_SHIFT  : u32 = 0;
    const ALPHA_COMPARE_MASK   : u32 = 0x03;
    const Z_SOURCE_SELECT_SHIFT: u32 = 2;
    const Z_SOURCE_SELECT_MASK : u32 = 0x01;
    const RENDER_MODE_SHIFT    : u32 = 3;
    const RENDER_MODE_MASK     : u32 = 0x1FFFFFFF;

    // RenderMode
    // AA_EN starts the render mode flags and it is set to 0x0008
    // because RENDER_MODE_SHIFT is 3.  So subtract 3 from the actual bit flag to get the shift
    // amount
    const Z_CMP_SHIFT          : u32 = 1;
    const Z_UPD_SHIFT          : u32 = 2;
    const ZMODE_SHIFT          : u32 = 7;
    const ZMODE_MASK           : u32 = 0x03;
    const CVG_X_ALPHA_SHIFT    : u32 = 9;
    const ALPHA_CVG_SEL_SHIFT  : u32 = 10;
    const FORCE_BL_SHIFT       : u32 = 11;

    fn get_render_mode(&self) -> u32 {
        (self.lo >> Self::RENDER_MODE_SHIFT) & Self::RENDER_MODE_MASK
    }

    fn get_alpha_compare(&self) -> AlphaCompare {
        match (self.lo >> Self::ALPHA_COMPARE_SHIFT) & Self::ALPHA_COMPARE_MASK {
            0 => AlphaCompare::None,
            1 => AlphaCompare::Threshold,
            3 => AlphaCompare::Dither,
            x => {
                warn!(target: "HLE", "invalid alpha compare mode {}", x);
                AlphaCompare::None
            }
        }
    }

    fn get_z_source_select(&self) -> ZSourceSelect {
        match (self.lo >> Self::Z_SOURCE_SELECT_SHIFT) & Self::Z_SOURCE_SELECT_MASK {
            0 => ZSourceSelect::Pixel,
            1 => ZSourceSelect::Primitive,
            _ => panic!("invalid"),
        }
    }

    fn get_alpha_dither(&self) -> AlphaDither {
        match (self.lo >> Self::ALPHA_DITHER_SHIFT) & Self::ALPHA_DITHER_MASK {
			0 => AlphaDither::Pattern,
			1 => AlphaDither::NotPattern,
			2 => AlphaDither::Noise,
			3 => AlphaDither::Disable,
			_ => panic!("invalid"),
		}
	}

    fn get_texture_filter(&self) -> TextureFilter {
        match (self.lo >> Self::TEXTURE_FILTER_SHIFT) & Self::TEXTURE_FILTER_MASK {
            0 => TextureFilter::Point,
            2 => TextureFilter::Bilinear,
            3 => TextureFilter::Average,
            x => {
                warn!(target: "HLE", "invalid texture filter mode {}", x);
                TextureFilter::Point
            }
        }
    }

    fn get_tlut_mode(&self) -> TlutMode {
        match (self.hi >> Self::TLUT_MODE_SHIFT) & Self::TLUT_MODE_MASK {
            0 => TlutMode::None,
            2 => TlutMode::Rgba16,
            3 => TlutMode::Ia16,
            x => {
                warn!(target: "HLE", "invalid tlut mode {}", x);
                TlutMode::None
            }
        }
    }

    fn get_depth_compare_enable(&self) -> bool {
        ((self.get_render_mode() >> Self::Z_CMP_SHIFT) & 0x01) != 0
    }

    fn get_depth_update_enable(&self) -> bool {
        ((self.get_render_mode() >> Self::Z_UPD_SHIFT) & 0x01) != 0
    }

    fn get_cycle_type(&self) -> CycleType {
        match (self.hi >> Self::CYCLE_TYPE_SHIFT) & Self::CYCLE_TYPE_MASK {
            0 => CycleType::OneCycle,
            1 => CycleType::TwoCycle,
            2 => CycleType::Copy,
            3 => CycleType::Fill,
            _ => panic!("invalid"),
        }
    }

    fn get_zmode(&self) -> ZMode {
        match (self.get_render_mode() >> Self::ZMODE_SHIFT) & Self::ZMODE_MASK {
            0 => ZMode::Opaque,
            1 => ZMode::Interpenetrating,
            2 => ZMode::Translucent,
            3 => ZMode::Decal,
            _ => panic!("invalid"),
        }
    }

    fn get_force_blend(&self) -> bool {
        (self.get_render_mode() & (1 << OtherModes::FORCE_BL_SHIFT)) != 0
    }

    // determine if a TEX_EDGE mode is in use.  TEX_EDGE doesn't actually set any bits...well,
    // apparently it used to, so that is checked first. otherwise, it's a guessing game?
    // TODO is there a better way?
    fn guess_tex_edge(&self) -> bool {
        // first check old bits
        if (self.lo & 0x8000) != 0 { return true; }

        // otherwise, mask out and test the following:
        // !FORCE_BL && ZMODE_OPA && CVG_X_ALPHA && ALPHA_CVG_SEL && Z_CMP
        const MASK: u32 = (1 << OtherModes::FORCE_BL_SHIFT)
                          | (OtherModes::ZMODE_MASK << OtherModes::ZMODE_SHIFT)
                          | (1 << OtherModes::CVG_X_ALPHA_SHIFT)
                          | (1 << OtherModes::ALPHA_CVG_SEL_SHIFT)
                          | (1 << OtherModes::Z_CMP_SHIFT);
        const VAL: u32 = (1 << OtherModes::CVG_X_ALPHA_SHIFT)
                          | (1 << OtherModes::ALPHA_CVG_SEL_SHIFT)
                          | (1 << OtherModes::Z_CMP_SHIFT);

        let rm = self.get_render_mode();
        (rm & MASK) == VAL
    }

    fn get_fog_enabled(&self) -> bool {
        // Fogging is enabled by the blender in Cycle 1 (for both 1CYC and 2CYC modes)
        // 0xC000 selects "P" in Cycle 1
        // value 0b11 is G_BL_CLR_FOG
        ((self.lo >> 16) & 0xC000) == 0xC000
    }

    // returns true if "A" in the blender of cycle 1 mode is G_BL_A_FOG (0b01)
    fn get_cycle1_a_fog(&self) -> bool {
        ((self.lo >> 16) & 0x0C00) == 0x0400
    }

    // returns true if "A" in the blender of cycle 1 mode is G_BL_A_SHADE (0b10)
    fn get_cycle1_a_shade(&self) -> bool {
        ((self.lo >> 16) & 0x0C00) == 0x0800
    }
}
