use std::collections::HashMap;
use std::mem;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use cgmath::prelude::*;
use cgmath::{Matrix4, Vector4};

use crate::*;

// Z values in OpenGL ranges from -1..1 but DirectX (what WGPU uses) 
// uses a Z range of 0..1. This matrix converts from OpenGL to DirectX.
// Note that the matrix is actually transposed from how it looks written out
// (col 2 row 3 is 0.5)
#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: Matrix4<f32> = Matrix4::from_cols(
    Vector4::new(1.0, 0.0, 0.0, 0.0),
    Vector4::new(0.0, 1.0, 0.0, 0.0),
    Vector4::new(0.0, 0.0, 0.5, 0.5),
    Vector4::new(0.0, 0.0, 0.0, 1.0),
);


#[derive(Debug, Clone)]
pub enum HleRenderCommand {
    Noop,
    DefineColorImage { bytes_per_pixel: u8, width: u16, framebuffer_address: u32 },
    DefineDepthImage { framebuffer_address: u32 },
    Viewport { x: f32, y: f32, z: f32, w: f32, h: f32, d: f32 },
    VertexData(Vec<F3DZEX2_Vertex>),
    IndexData(Vec<u16>),
    MatrixData(Vec<Matrix4<f32>>),
    FillRectangle { framebuffer_address: Option<u32>, x: f32, y: f32, w: f32, h: f32, c: [f32; 4] },
    RenderPass(RenderPassState),
    Sync,
}

pub type HleCommandBuffer = atomicring::AtomicRingBuffer<HleRenderCommand>;

// There are actually a lot of variations within the RSP ucodes that share
// a common GBI. 
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

#[allow(non_camel_case_types)]
pub type F3DZEX2_Matrix = [[i32; 4]; 4];

#[derive(Default)]
struct DLStackEntry {
    dl: Vec<u32>,
    pc: u32,
    base_address: u32,
}

#[derive(Clone,Debug,Default)]
pub struct RenderPassState {
    // current render targets for tris
    pub color_buffer: Option<u32>,
    pub depth_buffer: Option<u32>,

    pub clear_color: Option<[f32; 4]>,
    pub clear_depth: bool,

    // draw_list an array of triangle lists, where each triangle shares common state
    pub draw_list: Vec<TriangleList>,
}

#[derive(Clone,Debug,Default)]
pub struct TriangleList {
    pub matrix_index: u32,
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

    // F3DZEX has storage for 32 vertices
    vertices: Vec<F3DZEX2_Vertex>,
    vertex_stack: [u16; 32],
    indices: Vec<u16>,

    matrices: Vec<Matrix4<f32>>,             // all the unique matrices in the DL
    matrix_stack: Vec<Matrix4<f32>>,         // modelview only, not multiplied by proj
    current_matrix: Matrix4<f32>,            // current modelview matrix multiplied by projection
    current_projection_matrix: Matrix4<f32>, // current projection matrix
    current_matrix_index: Option<u32>,       // index into matrices[]; None when the current matrix isn't pushed yet

    current_viewport: Option<HleRenderCommand>,

    color_images: HashMap<u32, HleRenderCommand>,
    depth_images: HashMap<u32, HleRenderCommand>,
    clear_images: HashMap<u32, [f32; 4]>,

    // list of render_passes. the last of the array is always the current render pass
    render_passes: Vec<RenderPassState>,
    num_draws: u32,
    num_tris: u32,

    fill_color: u32,

    command_table: [DLCommand; 256],
    command_address: u32,
    command: u64,
    command_op: u8,
    command_words: u32,
    command_prefix: String,
}

const DL_FETCH_SIZE: u32 = 168; // dunno why 168 is used so much on LoZ, but let's use it too?

type DLCommand = fn(&mut Hle) -> ();

impl Hle {
    pub fn new(comms: SystemCommunication, hle_command_buffer: Arc<HleCommandBuffer>) -> Self {
        Self {
            comms: comms,

            hle_command_buffer: hle_command_buffer,
            software_version: HleRspSoftwareVersion::Uninitialized,
            software_crc: 0,

            dl_stack: vec![],
            segments: [0u32; 16],

            vertices: vec![],
            vertex_stack: [0; 32],
            indices: vec![],

            matrices: vec![],
            matrix_stack: vec![],
            current_matrix: Matrix4::identity(),
            current_projection_matrix: Matrix4::identity(),
            current_matrix_index: None,
            current_viewport: None,

            color_images: HashMap::new(),
            depth_images: HashMap::new(),
            clear_images: HashMap::new(),

            render_passes: vec![],
            num_draws: 0,
            num_tris: 0,

            fill_color: 0,

            command_table: [Hle::handle_unknown; 256],
            command_address: 0,
            command: 0,
            command_op: 0,
            command_words: 0,
            command_prefix: String::new(),
        }
    }

    fn reset_display_list(&mut self) {
        self.dl_stack.clear();
        self.segments = [0u32; 16];
        self.vertices.clear();
        self.vertex_stack = [0; 32];
        self.indices.clear();
        self.matrices.clear();
        self.matrix_stack.clear();
        self.current_matrix = Matrix4::identity();
        self.current_projection_matrix = Matrix4::identity();
        self.current_matrix_index = None;
        self.current_viewport = None;
        self.color_images.clear();
        self.depth_images.clear();
        self.clear_images.clear();
        self.render_passes.clear();
        self.next_render_pass();
        self.num_draws = 0;
        self.num_tris = 0;
        self.fill_color = 0;
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
            0xAD0A6292 => HleRspSoftwareVersion::F3DEX2, // Nintendo 64 devkit f3dex2
            0x21F91874 => HleRspSoftwareVersion::F3DEX2, // Zelda OoT
            0xC901CE73 => HleRspSoftwareVersion::F3DEX2, // More demos?

            0xB54E7F93 => HleRspSoftwareVersion::S3DEX2, // Nintendo 64 demos
            0x3A1CBAC3 => HleRspSoftwareVersion::S3DEX2, // Super Mario 64 (U)

            _ => HleRspSoftwareVersion::Unknown,
        };

        match self.software_version {
            HleRspSoftwareVersion::F3DEX2 => {
                self.command_table[0x00] = Hle::handle_noop;
                self.command_table[0x01] = Hle::handle_vtx;
                self.command_table[0x05] = Hle::handle_tri1;
                self.command_table[0x06] = Hle::handle_tri2;

                self.command_table[0xD7] = Hle::handle_texture;
                self.command_table[0xD8] = Hle::handle_popmtx;
                self.command_table[0xD9] = Hle::handle_geometrymode;
                self.command_table[0xDA] = Hle::handle_mtx;
                self.command_table[0xDB] = Hle::handle_moveword;
                self.command_table[0xDC] = Hle::handle_movemem;
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
            },

            HleRspSoftwareVersion::Unknown => {},
            _ => {
                unimplemented!("unsupported software {:?}", self.software_version);
            },
        };

        !(self.software_version == HleRspSoftwareVersion::Unknown)
    }

    pub fn process_display_list(&mut self, dl_start: u32, dl_length: u32, ucode_address: u32) {
        info!(target: "HLE", "processing display list from ${:08X}, length {} bytes", dl_start, dl_length);

        if let HleRspSoftwareVersion::Uninitialized = self.software_version {
            if !self.detect_software_version(ucode_address) { 
                unimplemented!("unknown RSP graphics task microcode (CRC 0x{:08X})", self.software_crc);
            }
        }

        self.reset_display_list();

        // sometimes dl_length ends up greater than DL_FETCH_SIZE, so it would reduce the # of DMAs
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

        // finalize current render pass
        self.finalize_render_pass();

        println!("found {} matrices", self.matrices.len());
        println!("found {} vertices", self.vertices.len());
        println!("found {} indices", self.indices.len());
        println!("found {} render passes", self.render_passes.len());
        println!("found {} draw calls", self.num_draws);
        println!("found {} tris", self.num_tris);

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

        // run each render pass
        for i in 0..self.render_passes.len() {
            let rp = std::mem::replace(&mut self.render_passes[i], RenderPassState::default());
            self.send_hle_render_command(HleRenderCommand::RenderPass(rp));
        }

        self.send_hle_render_command(HleRenderCommand::Sync);
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
        rdram[start..end].into()
    }

    // read memory until a \0 is encountered, and decode into a printable string
    // ideally block_size would be larger than the expected string length, but
    // not too large as to make the memory copy slow
    fn load_string(&mut self, mut start: u32, block_size: u32) -> String {
        let mut v: Vec<u8> = Vec::with_capacity(block_size as usize);

        assert!((start & 0x03) == 0); // if this happens we need smarter code
        let mut skip_one = (start & 0x04) == 0x04;

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

    fn handle_noop(&mut self) { // G_NOOP
        let addr = (self.command & 0xFFFF_FFFF) as u32;
        if addr != 0 {
            let s = self.load_string(addr, 64);
            trace!(target: "HLE", "{} gsDPNoOpString([0x{:08X}] \"{}\")", self.command_prefix, addr, s);
        } else {
            trace!(target: "HLE", "{} gsDPNoOp()", self.command_prefix);
        }
    }

    fn handle_mtx(&mut self) { // G_MTX
        let params = (self.command >> 32) as u8;
        let addr   = self.command as u32;

        let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x7F) as usize;
            self.segments[segment] + (addr & 0x00FF_FFFF)
        };

        let push = (params & 0x01) == 0;
        let mul  = (params & 0x02) == 0;
        let proj = (params & 0x04) != 0; // true = projection matrix, false = modelview

        let mut s = String::from("0");
        if push { s.push_str("|G_MTX_PUSH"); } else { s.push_str("|G_MTX_NOPUSH"); }
        if mul  { s.push_str("|G_MTX_MUL"); } else { s.push_str("|G_MTX_LOAD"); }
        if proj { s.push_str("|G_MTX_PROJECTION"); } else { s.push_str("|G_MTX_MODELVIEW"); }
        trace!(target: "HLE", "{} gsSPMatrix(0x{:08X}, {})", self.command_prefix, addr, s);

        let mtx_data = self.load_from_rdram(translated_addr, 64);
        //let mut mtx: F3DZEX2_Matrix = [[0i32; 4]; 4];
        //let mut fmtx = [[0f32; 4]; 4];

        // incoming data (numbers are the whole part, letters the fractional part):
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
        //println!("matrix: {cgmat:?}");

        //.for row in 0..4 {
        //.    match row {
        //.        0   => print!(" / "),
        //.        1|2 => print!("|  "),
        //.        3   => print!(" \\ "),
        //.        _   => {},
        //.    }

        //.    for col in 0..4 {
        //.        let idx16 = row*4+col;
        //.        let idx   = idx16 >> 1;
        //.        //let y     = idx >> 3;
        //.        //let x     = (idx >> 1) & 0x03;
        //.        let shift = 16 - ((idx16 & 1) << 4);

        //.        let intpart = (mtx_data[idx] >> shift) as i16;
        //.        let fracpart = (mtx_data[8 + idx] >> shift) as u16;

        //.        print!("{:04x}.{:04x} ", intpart, fracpart);
        //.        mtx[row][col] = ((intpart as i32) << 16) | (fracpart as i32);
        //.    }

        //.    match row {
        //.        0   => print!("\\   /"),
        //.        1|2 => print!(" | | "),
        //.        3   => print!("/   \\"),
        //.        _   => {},
        //.    }

        //.    for col in 0..4 {
        //.        let e = mtx[row][col];
        //.        let f = (e >> 16) as f32 + ((e as u16) as f32) / 65536.0;
        //.        fmtx[row][col] = f;
        //.        print!("{:8.4}", cgmat[row][col]);
        //.    }

        //.    match row {
        //.        0   => println!(" \\"),
        //.        1|2 => println!("  |"),
        //.        3   => println!(" /   "),
        //.        _   => {},
        //.    }

        //.}

        if proj {
            //println!("proj: {cgmat:?}");
            self.current_projection_matrix = cgmat * OPENGL_TO_WGPU_MATRIX;
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

            // the matrices used on n64 expect the vertex to be left-multiplied: v' = v*(M*V*P)
            self.current_matrix = cgmat * self.current_projection_matrix;
        }

        // clear the current matrix index without creating a new state, since the matrix could be updated further
        self.current_matrix_index = None;
    }

    fn handle_popmtx(&mut self) { // G_POPMTX
        let num = ((self.command & 0xFFFF_FFFF) >> 6) as u32; // num / 64

        trace!(target: "HLE", "{} gsSPPopMatrixN(G_MTX_MODELVIEW, {})", self.command_prefix, num);
        for _ in 0..num { self.matrix_stack.pop(); }
        self.current_matrix_index = None;
    }

    fn handle_moveword(&mut self) { // G_MOVEWORD
        let index  = ((self.command >> 48) & 0xFF) as u8;
        let offset = ((self.command >> 32) & 0xFFFF) as u16;
        let data   = self.command as u32;
        
        match index {
            6 => { // G_MW_SEGMENT
                trace!(target: "HLE", "{} gsSPSegment({}, 0x{:08X})", self.command_prefix, offset >> 2, data);
                self.segments[(offset >> 2) as usize] = data;
            },
        
            _ => {
                trace!(target: "HLE", "{} gsMoveWd({}, 0x{:04X}, 0x{:08X})", self.command_prefix, index, offset, data);
            },
        };
    }

    fn handle_movemem(&mut self) { // G_MOVEMEM
        let size  = ((((self.command >> 48) & 0xFF) >> 3) + 1) << 3;
        let index = (self.command >> 32) as u8;
        let addr  = (self.command & 0xFFFF_FFFF) as u32;

        match index {
            8 => { // G_VIEWPORT
                let segment = (addr >> 24) as u8;
                let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else { 
                    (addr & 0x00FF_FFFF) + self.segments[segment as usize] 
                };

                let vp = self.load_from_rdram(translated_addr, size as u32);
                let frac: [f32; 4] = [0.00, 0.25, 0.5, 0.75];
                let xs = (vp[0] >> 16) as i16;
                let ys = vp[0] as i16;
                let zs = (vp[1] >> 16) as i16;
                let x_scale = (xs >> 2) as f32 + frac[(xs & 3) as usize];
                let y_scale = (ys >> 2) as f32 + frac[(ys & 3) as usize];
                let z_scale = (zs >> 2) as f32 + frac[(zs & 3) as usize];
                let xt = (vp[2] >> 16) as i16;
                let yt = vp[2] as i16;
                let zt = (vp[3] >> 16) as i16;
                let x_translate = (xt >> 2) as f32 + frac[(xt & 3) as usize];
                let y_translate = (yt >> 2) as f32 + frac[(yt & 3) as usize];
                let z_translate = (zt >> 2) as f32 + frac[(zt & 3) as usize];

                trace!(target: "HLE", "{} gsSPViewport(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);
                println!("Viewport {{ vscale: [ {}, {}, {} ], vtrans: [ {}, {}, {} ] }}    ", x_scale, y_scale, z_scale, x_translate, y_translate, z_translate);

                self.current_viewport = Some(HleRenderCommand::Viewport {
                    x: -1.0 * x_scale + x_translate,
                    y: -1.0 * y_scale + y_translate,
                    z: -1.0 * z_scale + z_translate,
                    w:  2.0 * x_scale,
                    h:  2.0 * y_scale,
                    d:  2.0 * z_scale,
                });
                println!("{:?}", self.current_viewport);
            },

            _ => {
                trace!(target: "HLE", "{} gsSPMoveMem?({}, ...)", self.command_prefix, index);
            },
        };
    }

    fn handle_displaylist(&mut self) { // G_DL
        let is_link = (self.command & 0x00FF_0000_0000_0000) == 0;
        let addr    = (self.command & 0x1FFF_FFFF) as u32;
        let segment = (addr >> 24) as usize;

        let translated_addr = (addr & 0x00FF_FFFF) + self.segments[segment];

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

    fn handle_enddl(&mut self) {
        trace!(target: "HLE", "{} gsSPEndDisplayList()", self.command_prefix);
        self.dl_stack.pop();
    }

    fn handle_texture(&mut self) { // G_TEXTURE
        let ts    = self.command as u16;
        let ss    = (self.command >> 16) as u16;
        let on    = ((self.command >> 33) & 0x7F) != 0;
        let tile  = ((self.command >> 40) & 0x07) as u8;
        let level = (self.command >> 43) & 0x07;
        trace!(target: "HLE", "{} gsSPTexture(0x{:04X}, 0x{:04X}, {}, {}, {})", self.command_prefix, ss, ts, level, tile, on);
    }

    fn handle_vtx(&mut self) { // G_VTX
        let numv  = (self.command >> 44) as u8;
        let vbidx = (((self.command >> 33) & 0x7F) as u8) - numv;
        let addr  = self.command as u32;

        let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x00FF_FFFF)
        };

        let vtx_size = mem::size_of::<F3DZEX2_Vertex>();
        let data_size = numv as usize * vtx_size;
        trace!(target: "HLE", "{} gsSPVertex(0x{:08X} [0x{:08X}], {}, {}) (size_of<vtx>={}, data_size={})", self.command_prefix, addr, translated_addr, numv, vbidx, vtx_size, data_size);

        let vtx_data = self.load_from_rdram(translated_addr, data_size as u32);
        assert!(data_size == vtx_data.len() * 4);
        assert!((vtx_size % 4) == 0);

        for i in 0..numv {
            let data = &vtx_data[(vtx_size * i as usize) >> 2..];
            let vtx = F3DZEX2_Vertex {
                position: [(data[0] >> 16) as i16, data[0] as i16, (data[1] >> 16) as i16],
                reserved: data[1] as u16,
                texcoord: [(data[2] >> 16) as i16, data[2] as i16],
                color_or_normal: [
                    (data[3] >> 24) as u8, (data[3] >> 16) as u8, (data[3] >> 8) as u8, data[3] as u8
                ]
            };

            //println!("v{}: {:?}", i+vbidx, vtx);
            let cur_pos = self.vertices.len() as u16;
            self.vertices.push(vtx);
            self.vertex_stack[(i + vbidx) as usize] = cur_pos;
        }

        //self.send_hle_render_command(HleRenderCommand::VertexData(v, vbidx as usize));
    }
        
    fn current_render_pass(&mut self) -> &mut RenderPassState {
        self.render_passes.last_mut().expect("must always have a valid RP")
    }

    fn next_render_pass(&mut self) {
        // don't create a new render pass if this one isn't rendering anything, however, keep the current state
        if self.render_passes.len() > 0 {
            if self.render_passes.last().unwrap().draw_list.len() == 0 {
                return;
            }

            self.finalize_render_pass();
        }

        let rp = RenderPassState {
            ..Default::default()
        };
        self.render_passes.push(rp);
    }

    fn finalize_render_pass(&mut self) {
        let (color_buffer, depth_buffer) = {
            let rp = self.current_render_pass();
            (rp.color_buffer, rp.depth_buffer)
        };

        if let Some(color_addr) = color_buffer {
            let clear_color = self.clear_images.get(&color_addr).copied();
            self.current_render_pass().clear_color = clear_color;
        }

        if let Some(depth_addr) = depth_buffer {
            if self.clear_images.contains_key(&depth_addr) {
                self.current_render_pass().clear_depth = true;
            }
        }
    }

    fn current_triangle_list(&mut self) -> &mut TriangleList {
        if let None = self.current_matrix_index { // TODO: other ways to check if the state has changed
            if self.matrices.len() == 0 || self.current_matrix != *self.matrices.last().unwrap() {
                self.matrices.push(self.current_matrix);
            }
            let matrix_index = (self.matrices.len() as u32) - 1;
            self.current_matrix_index = Some(matrix_index);

            let tl = TriangleList {
                matrix_index: matrix_index,
                ..Default::default()
            };

            let rp = self.current_render_pass();
            rp.draw_list.push(tl);
            self.num_draws += 1;
        }

        self.current_render_pass().draw_list.last_mut().expect("must have one draw list!")
    }

    fn handle_tri1(&mut self) { // G_TRI1
        let v0 = ((self.command >> 49) & 0x1F) as u16;
        let v1 = ((self.command >> 41) & 0x1F) as u16;
        let v2 = ((self.command >> 33) & 0x1F) as u16;
        trace!(target: "HLE", "{} gsSP1Triangle({}, {}, {})", self.command_prefix, v0, v1, v2);
        // translate to global vertex index
        let v0 = self.vertex_stack[v0 as usize];
        let v1 = self.vertex_stack[v1 as usize];
        let v2 = self.vertex_stack[v2 as usize];
        let tl = self.current_triangle_list();
        tl.num_indices += 3;
        self.indices.extend_from_slice(&[v0, v1, v2]);
        self.num_tris += 1;
    }

    fn handle_tri2(&mut self) { // G_TRI2
        let v00 = ((self.command >> 49) & 0x1F) as u16;
        let v01 = ((self.command >> 41) & 0x1F) as u16;
        let v02 = ((self.command >> 33) & 0x1F) as u16;
        let v10 = ((self.command >> 17) & 0x1F) as u16;
        let v11 = ((self.command >>  9) & 0x1F) as u16;
        let v12 = ((self.command >>  1) & 0x1F) as u16;
        trace!(target: "HLE", "{} gsSP2Triangle({}, {}, {}, 0, {}, {}, {}, 0)", self.command_prefix, v00, v01, v02, v10, v11, v12);
        // translate to global vertex stack
        let v00 = self.vertex_stack[v00 as usize];
        let v01 = self.vertex_stack[v01 as usize];
        let v02 = self.vertex_stack[v02 as usize];
        let v10 = self.vertex_stack[v10 as usize];
        let v11 = self.vertex_stack[v11 as usize];
        let v12 = self.vertex_stack[v12 as usize];
        let tl = self.current_triangle_list();
        tl.num_indices += 6;
        self.indices.extend_from_slice(&[v00, v01, v02, v10, v11, v12]);
        self.num_tris += 2;
    }

    fn handle_texrect(&mut self) { // G_TEXRECT
        let cmd1 = self.next_display_list_command();
        let cmd2 = self.next_display_list_command();
        self.command_words += 4;
        let x1   = ((self.command >> 44) & 0xFFF) as u16;
        let y1   = ((self.command >> 32) & 0xFFF) as u16;
        let tile = ((self.command >> 24) & 0x0F) as u8;
        let x0   = ((self.command >> 12) & 0xFFF) as u16;
        let y0   = ((self.command >>  0) & 0xFFF) as u16;
        let s0   = (cmd1 >> 16) as u16;
        let t0   = (cmd1 >>  0) as u16;
        let dsdx = (cmd2 >> 16) as u16;
        let dtdy = (cmd2 >>  0) as u16;
        trace!(target: "HLE", "{} gsSPTextureRectange({}, {}, {}, {}, {}, {}, {}, {}, {})", self.command_prefix, x0, y0, x1, y1, tile, s0, t0, dsdx, dtdy);
        //self.send_hle_render_command(HleRenderCommand::FillRectangle {
        //    x: x0 as f32,
        //    y: y0 as f32,
        //    w: (x1 - x0) as f32 + 1.0,
        //    h: (y1 - y0) as f32 + 1.0,
        //    c: [((self.fill_color >> 11) & 0x1F) as f32 / 32.0, ((self.fill_color >> 6) & 0x1F) as f32 / 32.0,
        //        ((self.fill_color >>  1) & 0x1F) as f32 / 32.0, 1.0],
        //});
    }

    fn handle_settilesize(&mut self) { // G_SETTILESIZE
        let x0 = ((self.command >> 44) & 0xFFF) as u16;
        let y0 = ((self.command >> 32) & 0xFFF) as u16;
        let tile = ((self.command >> 24) & 0x0F) as u8;
        let x1 = ((self.command >> 12) & 0xFFF) as u16;
        let y1 = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPSetTileSize({}, {}, {}, {}, {})", self.command_prefix, tile, x0, y0, x1, y1);
    }

    fn handle_loadblock(&mut self) { // G_LOADBLOCK
        let x0     = ((self.command >> 44) & 0xFFF) as u16;
        let y0     = ((self.command >> 32) & 0xFFF) as u16;
        let tile   = ((self.command >> 24) & 0x0F) as u8;
        let texels = ((self.command >> 12) & 0xFFF) as u16;
        let dxt    = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPLoadBlock({}, {}, {}, {}, {})", self.command_prefix, tile, x0, y0, texels, dxt);
    }

    fn handle_loadtile(&mut self) { // G_LOADTILE
        let s0     = ((self.command >> 44) & 0xFFF) as u16;
        let t0     = ((self.command >> 32) & 0xFFF) as u16;
        let tile   = ((self.command >> 24) & 0x0F) as u8;
        let s1     = ((self.command >> 12) & 0xFFF) as u16;
        let t1     = ((self.command >>  0) & 0xFFF) as u16;
        trace!(target: "HLE", "{} gsDPLoadTile({}, {}, {}, {}, {})", self.command_prefix, tile, s0, t0, s1, t1);
    }

    fn handle_settile(&mut self) { // G_SETTILE
        trace!(target: "HLE", "{} gsDPSetTile(...)", self.command_prefix);
    }

    fn handle_geometrymode(&mut self) { // G_GEOMETRYMODE
        let clearbits = ((self.command >> 32) & 0x00FF_FFFF) as u32;
        let setbits   = self.command as u32;
        if clearbits == 0 && setbits != 0 {
            trace!(target: "HLE", "{} gsSPLoadGeometryMode(0x{:08X})", self.command_prefix, setbits);
        } else if clearbits != 0 && setbits == 0 {
            trace!(target: "HLE", "{} gsSPClearGeometryMode(0x{:08X})", self.command_prefix, clearbits);
        } else {} {
            trace!(target: "HLE", "{} gsSPGeometryMode(0x{:08X}, 0x{:08X})", self.command_prefix, clearbits, setbits);
        }
    }

    fn handle_rdploadsync(&mut self) { // G_RDPLOADSYNC
        trace!(target: "HLE", "{} gsDPLoadSync()", self.command_prefix);
    }

    fn handle_rdppipesync(&mut self) { // G_RDPPIPESYNC
        trace!(target: "HLE", "{} gsDPPipeSync()", self.command_prefix);
    }

    fn handle_rdptilesync(&mut self) { // G_RDPTILESYNC
        trace!(target: "HLE", "{} gsDPTileSync()", self.command_prefix);
    }

    fn handle_rdpfullsync(&mut self) { // G_RDPFULLSYNC
        trace!(target: "HLE", "{} gsDPFullSync()", self.command_prefix);
    }

    fn handle_rdpsetothermode(&mut self) { // G_(RDP)SETOTHERMODE
        let hi = ((self.command >> 32) & 0x00FF_FFFF) as u32;
        let lo = self.command as u32;
        trace!(target: "HLE", "{} gsDPSetOtherMode(0x{:08X}, 0x{:08X})", self.command_prefix, hi, lo);
    }

    fn handle_loadlut(&mut self) { // G_LOADTLUT
        let tile  = ((self.command >> 24) & 0x0F) as u8;
        let count = ((self.command >> 14) & 0x03FF) as u16;
        trace!(target: "HLE", "{} gsDPLoadTLUTCmd({}, {})", self.command_prefix, tile, count);
    }

    fn handle_setothermode_h(&mut self) { // G_SETOTHERMODE_H
        trace!(target: "HLE", "{} gsSPSetOtherModeH(...)", self.command_prefix);
    }

    fn handle_setothermode_l(&mut self) { // G_SETOTHERMODE_L
        trace!(target: "HLE", "{} gsSPSetOtherModeL(...)", self.command_prefix);
    }

    fn handle_setprimdepth(&mut self) { // G_SETPRIMDEPTH
        trace!(target: "HLE", "{} gsDPSetPrimDepth(...)", self.command_prefix);
    }

    fn handle_setscissor(&mut self) { // G_SETSCISSOR
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

    fn handle_fillrect(&mut self) { // G_FILLRECT
        let x1 = (((self.command >> 44) & 0xFFF) as u16) >> 2;
        let y1 = (((self.command >> 32) & 0xFFF) as u16) >> 2;
        let x0 = (((self.command >> 12) & 0xFFF) as u16) >> 2;
        let y0 = (((self.command >>  0) & 0xFFF) as u16) >> 2;
        trace!(target: "HLE", "{} gsDPFillRectangle({}, {}, {}, {})", self.command_prefix, x0, y0, x1, y1);
        if let Some(HleRenderCommand::Viewport { x: vx, y: vy, w: vw, h: vh, .. }) = &self.current_viewport {
            if (self.fill_color >> 16) == (self.fill_color & 0xFFFF) {
                let w = x1 - x0 + 1;
                let h = y1 - y0 + 1;
                if x0 == *vx as u16 && y0 == *vy as u16 && w == *vw as u16 && h == *vh as u16 {
                    let addr = self.current_render_pass().color_buffer.clone().unwrap();
                    // we can't just mark clear_color in the current render pass, because
                    // the N64 uses the color buffer to clear depth buffers, so we mark this
                    // address as cleared in a separate buffer and only when next_render_pass() is
                    // called do we know that the render targets are set
                    let color = [((self.fill_color >> 11) & 0x1F) as f32 / 32.0, ((self.fill_color >> 6) & 0x1F) as f32 / 32.0,
                                 ((self.fill_color >>  1) & 0x1F) as f32 / 32.0, 1.0];
                    self.clear_images.insert(addr, color);
                }
            }
        }
        //let color_buffer = self.current_render_pass().color_buffer.clone();
        //self.send_hle_render_command(HleRenderCommand::FillRectangle {
        //    framebuffer_address: color_buffer,
        //    x: x0 as f32,
        //    y: y0 as f32,
        //    w: (x1 - x0) as f32 + 1.0,
        //    h: (y1 - y0) as f32 + 1.0,
        //    c: [((self.fill_color >> 11) & 0x1F) as f32 / 32.0, ((self.fill_color >> 6) & 0x1F) as f32 / 32.0,
        //        ((self.fill_color >>  1) & 0x1F) as f32 / 32.0, 1.0],
        //});
    }

    fn handle_setfogcolor(&mut self) { // G_SETFOGCOLOR
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPSetFogColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);
    }

    fn handle_setfillcolor(&mut self) { // G_SETFILLCOLOR
        assert!((self.command >> 16) as u16 == (self.command & 0xFFFF) as u16); // Someday some code will fill a rect with alternating colors
        self.fill_color = self.command as u32;
        trace!(target: "HLE", "{} gsDPSetFillColor(0x{:08X})", self.command_prefix, self.fill_color);
    }

    fn handle_setblendcolor(&mut self) { // G_SETBLENDCOLOR
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPBlendColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);
    }

    fn handle_setprimcolor(&mut self) { // G_SETPRIMCOLOR
        let minlevel = (self.command >> 40) as u8;
        let lodfrac  = (self.command >> 32) as u8;
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPSetPrimColor({}, {}, {}, {}, {}, {})", self.command_prefix, minlevel, lodfrac, r, g, b, a);
    }

    fn handle_setenvcolor(&mut self) { // G_SETENVCOLOR
        let r = (self.command >> 24) as u8;
        let g = (self.command >> 16) as u8;
        let b = (self.command >>  8) as u8;
        let a = (self.command >>  0) as u8;
        trace!(target: "HLE", "{} gsDPSetEnvColor({}, {}, {}, {})", self.command_prefix, r, g, b, a);
    }

    fn handle_setcombine(&mut self) { // G_SETCOMBINE
        trace!(target: "HLE", "{} gsDPSetCombineLERP(...)", self.command_prefix);
    }

    fn handle_settimg(&mut self) { // G_SETTIMG
        let addr = self.command as u32;
        trace!(target: "HLE", "{} gsDPSetTextureImage(..., 0x{:08X})", self.command_prefix, addr);
    }

    fn handle_setzimg(&mut self) { // G_SETZIMG
        let addr = self.command as u32;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x00FF_FFFF)
        };

        trace!(target: "HLE", "{} gsDPSetDepthImage(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);

        // if the color depth changes, start a new render pass
        if self.current_render_pass().depth_buffer.is_some_and(|v| v != translated_addr) {
            self.next_render_pass();
        }
        self.current_render_pass().depth_buffer = Some(translated_addr);

        let hle_render_command = HleRenderCommand::DefineDepthImage { framebuffer_address: translated_addr };
        if !self.depth_images.contains_key(&translated_addr) {
            self.depth_images.insert(translated_addr, hle_render_command);
        }
    }

    fn handle_setcimg(&mut self) { // G_SETCIMG
        let addr  = self.command as u32;
        let width = ((self.command >> 32) & 0x0FFF) as u16 + 1;
        let bpp   = ((self.command >> 51) & 0x03) as u8;
        let fmt   = ((self.command >> 53) & 0x07) as u8;

        let translated_addr = if (addr & 0xE000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x00FF_FFFF)
        };

        trace!(target: "HLE", "{} gsDPSetColorImage({}, {}, {}, 0x{:08X} [0x{:08X}])", self.command_prefix, fmt, bpp, width, addr, translated_addr);

        if fmt != 0 { // G_IM_FMT_RGBA
            unimplemented!("color targets not of RGBA not yet supported");
        }

        // if the color buffer changes, start a new render pass
        if self.current_render_pass().color_buffer.is_some_and(|v| v != translated_addr) {
            self.next_render_pass();
        }
        self.current_render_pass().color_buffer = Some(translated_addr);

        // create the color buffer
        let hle_render_command = HleRenderCommand::DefineColorImage { bytes_per_pixel: bpp, width: width, framebuffer_address: translated_addr };
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

        //let mut size: u32 = 0;
        //let op = (cmd >> 56) & 0xFF;
        //match op {
    }
}
