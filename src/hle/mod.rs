use std::mem;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use cgmath::prelude::*;
use cgmath::Matrix4;

use crate::*;
use rcp::DmaInfo;

#[derive(Debug, Clone)]
pub enum HleRenderCommand {
    Noop,
    SetColorImage { bytes_per_pixel: u8, width: u16, framebuffer_address: u32 },
    Viewport { x: f32, y: f32, w: f32, h: f32 },
    SetProjectionMatrix(Matrix4<f32>),
    SetModelViewMatrix(Matrix4<f32>),
    VertexData(Vec<F3DZEX2_Vertex>, usize),
    FillRectangle { x: f32, y: f32, w: f32, h: f32, c: [f32; 4] },
    DrawTriangle(u16, u16, u16),
    //Vertices(u32),
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

pub struct Hle {
    hle_command_buffer: Arc<HleCommandBuffer>,
    software_version: HleRspSoftwareVersion,
    software_crc: u32,

    dl_stack: Vec<DLStackEntry>,

    segments: [u32; 16],
    matrices: Vec<Matrix4<f32>>,
    // F3DZEX has storage for 32 vertices
    vertices: [F3DZEX2_Vertex; 32],

    fill_color: u32,

    dma_completed_rx: mpsc::Receiver<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,
    start_dma_tx: mpsc::Sender<DmaInfo>,

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
    pub fn new(start_dma_tx: mpsc::Sender<DmaInfo>, hle_command_buffer: Arc<HleCommandBuffer>) -> Self {
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        Self {
            hle_command_buffer: hle_command_buffer,
            software_version: HleRspSoftwareVersion::Uninitialized,
            software_crc: 0,

            dl_stack: vec![],
            segments: [0u32; 16],
            matrices: vec![],
            fill_color: 0,

            vertices: [F3DZEX2_Vertex::default(); 32],
            dma_completed_rx: dma_completed_rx,
            dma_completed_tx: dma_completed_tx,
            start_dma_tx: start_dma_tx,

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
        self.vertices = [F3DZEX2_Vertex::default(); 32];
        self.matrices.clear();
    }

    fn detect_software_version(&mut self, ucode_address: u32) -> bool {
        let ucode = self.load_display_list(ucode_address, 4 * 1024);

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
        trace!(target: "HLE", "processing display list from ${:08X}, length {} bytes", dl_start, dl_length);

        if let HleRspSoftwareVersion::Uninitialized = self.software_version {
            if !self.detect_software_version(ucode_address) { 
                unimplemented!("unknown RSP graphics task microcode (CRC 0x{:08X})", self.software_crc);
            }
        }

        self.reset_display_list();

        // sometimes dl_length ends up greater than DL_FETCH_SIZE, so it would reduce the # of DMAs
        let cur_dl = DLStackEntry {
            dl: self.load_display_list(dl_start, dl_length),
            base_address: dl_start,
            ..Default::default()
        };
        self.dl_stack.push(cur_dl);

        while self.dl_stack.len() > 0 {
            let addr = self.current_display_list_address();
            let cmd = self.next_display_list_command();
            self.process_display_list_command(addr, cmd);
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

            let dl = self.load_display_list(load_address, DL_FETCH_SIZE);
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

    fn load_display_list(&self, start: u32, length: u32) -> Vec<u32> {
        let dma_info = DmaInfo {
            initiator      : "HLE-DL",
            source_address : start,
            dest_address   : 0xFFFF_FFFF,
            count          : 1,
            length         : length,
            completed      : Some(self.dma_completed_tx.clone()),
            ..Default::default()
        };

        self.start_dma_tx.send(dma_info).unwrap();
        match self.dma_completed_rx.recv() {
            Ok(dma_info) => {
                dma_info.internal_buffer.unwrap()
            },

            Err(e) => {
                panic!("shouldn't happen: {}", e);
            },
        }
    }

    // read memory until a \0 is encountered, and decode into a printable string
    // ideally block_size would be larger than the expected string length, but
    // not too large as to make the memory copy slow
    fn load_string(&mut self, mut start: u32, block_size: u32) -> String {
        let mut v: Vec<u8> = Vec::with_capacity(block_size as usize);

        assert!((start & 0x03) == 0); // if this happens we need smarter code
        let mut skip_one = (start & 0x04) == 0x04;

        loop {
            let block = self.load_display_list(start, block_size);
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

        let mtx_data = self.load_display_list(translated_addr, 64);
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
        // 0000.gggg 1111.hhhh 2222.iiii 3333.jjjj
        // ..
        // cccc.kkkk dddd.tttt eeee.uuuu ffff.vvvv
        let elem = |i, s| (((mtx_data[i] >> s) as i16) as f32) + (((mtx_data[i + 8] >> s) as u16) as f32) / 65536.0;
        let c0 = [elem(0, 16), elem(2, 16), elem(4, 16), elem(6, 16)];
        let c1 = [elem(0,  0), elem(2,  0), elem(4,  0), elem(6,  0)];
        let c2 = [elem(1, 16), elem(3, 16), elem(5, 16), elem(7, 16)];
        let c3 = [elem(1,  0), elem(3,  0), elem(5,  0), elem(7,  0)];
        let mut cgmat = Matrix4::from_cols(c0.into(), c1.into(), c2.into(), c3.into());

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

        let set_index = self.matrices.len();
        if set_index != 0 && mul {
            let other = &self.matrices[set_index - 1];
            cgmat = cgmat * other;
        }

        if set_index == 0 || push {
            self.matrices.push(cgmat);
            assert!(self.matrices.len() < 16);
        } else {
            self.matrices[set_index - 1] = cgmat;
        }

        if proj {
            self.send_hle_render_command(HleRenderCommand::SetProjectionMatrix(cgmat));
        } else {
            self.send_hle_render_command(HleRenderCommand::SetModelViewMatrix(cgmat));
        }
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

                let vp = self.load_display_list(translated_addr, size as u32);
                let frac: [f32; 4] = [0.00, 0.25, 0.5, 0.75];
                let xs = (vp[0] >> 16) as i16;
                let ys = vp[0] as i16;
                let zs = (vp[1] >> 16) as i16;
                let x_scale = (xs >> 2) as f32 + frac[(xs & 3) as usize];
                let y_scale = (ys >> 2) as f32 + frac[(ys & 3) as usize];
                let _z_scale = (zs >> 2) as f32 + frac[(zs & 3) as usize];
                let xt = (vp[2] >> 16) as i16;
                let yt = vp[2] as i16;
                let zt = (vp[3] >> 16) as i16;
                let x_translate = (xt >> 2) as f32 + frac[(xt & 3) as usize];
                let y_translate = (yt >> 2) as f32 + frac[(yt & 3) as usize];
                let _z_translate = (zt >> 2) as f32 + frac[(zt & 3) as usize];

                trace!(target: "HLE", "{} gsSPViewport(0x{:08X} [0x{:08X}])", self.command_prefix, addr, translated_addr);
                //println!("Viewport {{ vscale: [ {}, {}, {}, 0.0 ], vtrans: [ {}, {}, {}, 0.0 ] }}    ", x_scale, y_scale, z_scale, x_translate, y_translate, z_translate);

                self.send_hle_render_command(HleRenderCommand::Viewport {
                    x: -1.0 * x_scale + x_translate,
                    y: -1.0 * y_scale + y_translate,
                    w:  2.0 * x_scale,
                    h:  2.0 * y_scale,
                });
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

        let vtx_data = self.load_display_list(translated_addr, data_size as u32);
        assert!(data_size == vtx_data.len() * 4);
        assert!((vtx_size % 4) == 0);

        let mut v = Vec::new();
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
            self.vertices[(i + vbidx) as usize] = vtx;
            v.push(vtx);
        }

        self.send_hle_render_command(HleRenderCommand::VertexData(v, vbidx as usize));
    }
        
    fn handle_tri1(&mut self) { // G_TRI1
        let v0 = ((self.command >> 49) & 0x7F) as u16;
        let v1 = ((self.command >> 41) & 0x7F) as u16;
        let v2 = ((self.command >> 33) & 0x7F) as u16;
        trace!(target: "HLE", "{} gsSP1Triangle({}, {}, {})", self.command_prefix, v0, v1, v2);
        self.send_hle_render_command(HleRenderCommand::DrawTriangle(v0, v1, v2));
    }

    fn handle_tri2(&mut self) { // G_TRI2
        let v00 = ((self.command >> 49) & 0x7F) as u16;
        let v01 = ((self.command >> 41) & 0x7F) as u16;
        let v02 = ((self.command >> 33) & 0x7F) as u16;
        let v10 = ((self.command >> 17) & 0x7F) as u16;
        let v11 = ((self.command >>  9) & 0x7F) as u16;
        let v12 = ((self.command >>  1) & 0x7F) as u16;
        trace!(target: "HLE", "{} gsSP2Triangle({}, {}, {}, 0, {}, {}, {}, 0)", self.command_prefix, v00, v01, v02, v10, v11, v12);
        self.send_hle_render_command(HleRenderCommand::DrawTriangle(v00, v01, v02));
        self.send_hle_render_command(HleRenderCommand::DrawTriangle(v10, v11, v12));
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
        self.send_hle_render_command(HleRenderCommand::FillRectangle {
            x: x0 as f32,
            y: y0 as f32,
            w: (x1 - x0) as f32 + 1.0,
            h: (y1 - y0) as f32 + 1.0,
            c: [((self.fill_color >> 11) & 0x1F) as f32 / 32.0, ((self.fill_color >> 6) & 0x1F) as f32 / 32.0,
                ((self.fill_color >>  1) & 0x1F) as f32 / 32.0, 1.0],
        });
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
        trace!(target: "HLE", "{} gsDPSetDepthImage(..., 0x{:08X})", self.command_prefix, addr);
    }

    fn handle_setcimg(&mut self) { // G_SETCIMG
        let addr  = self.command as u32;
        let width = ((self.command >> 32) & 0x0FFF) as u16 + 1;
        let bpp   = ((self.command >> 51) & 0x03) as u8;
        let fmt   = ((self.command >> 53) & 0x07) as u8;

        let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else {
            let segment = ((addr >> 24) & 0x0F) as usize;
            self.segments[segment] + (addr & 0x00FF_FFFF)
        };

        trace!(target: "HLE", "{} gsDPSetColorImage({}, {}, {}, 0x{:08X} [0x{:08X}])", self.command_prefix, fmt, bpp, width, addr, translated_addr);

        if fmt != 0 { // G_IM_FMT_RGBA
            unimplemented!("color targets not of RGBA not yet supported");
        }

        self.send_hle_render_command(HleRenderCommand::SetColorImage { bytes_per_pixel: bpp, width: width, framebuffer_address: translated_addr });
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
