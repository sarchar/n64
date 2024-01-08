use std::mem;
use std::sync::mpsc;

#[allow(unused_imports)]
use tracing::{trace, debug, error, info, warn};

use crate::*;
use rcp::DmaInfo;

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum HleRenderCommand {
    Noop,
    Viewport { x: f32, y: f32, w: f32, h: f32 },
    SetProjectionMatrix([[f32; 4]; 4]),
    SetModelViewMatrix([[f32; 4]; 4]),
    FillRectangle { x: f32, y: f32, w: f32, h: f32, c: [f32; 4] },
    //Vertices(u32),
    Sync,
}

pub type HleCommandBuffer = atomicring::AtomicRingBuffer<HleRenderCommand>;

// An F3DZEX vertex has two forms, but only varies with the last color value
// being either used for prelit color or the normal value
#[repr(C)]
#[derive(Copy,Clone,Default,Debug)]
struct F3DZEX2_Vertex {
    position: [i16; 3],
    reserved: u16,
    texcoord: [i16; 2],
    color_or_normal: [u8; 4],
}

#[allow(non_camel_case_types)]
type F3DZEX2_Matrix = [[i32; 4]; 4];

#[derive(Default)]
struct DLStackEntry {
    dl: Vec<u32>,
    pc: u32,
    base_address: u32,
}

pub struct Hle {
    hle_command_buffer: Arc<HleCommandBuffer>,

    dl_stack: Vec<DLStackEntry>,

    segments: [u32; 16],
    // F3DZEX has storage for 32 vertices
    vertices: [F3DZEX2_Vertex; 32],

    fill_color: u32,

    dma_completed_rx: mpsc::Receiver<DmaInfo>,
    dma_completed_tx: mpsc::Sender<DmaInfo>,
    start_dma_tx: mpsc::Sender<DmaInfo>,
}

const DL_FETCH_SIZE: u32 = 168; // dunno why 168 is used so much on LoZ, but let's use it too?

impl Hle {
    pub fn new(start_dma_tx: mpsc::Sender<DmaInfo>, hle_command_buffer: Arc<HleCommandBuffer>) -> Self {
        let (dma_completed_tx, dma_completed_rx) = mpsc::channel();

        Self {
            hle_command_buffer: hle_command_buffer,

            dl_stack: vec![],
            segments: [0u32; 16],
            fill_color: 0,

            vertices: [F3DZEX2_Vertex::default(); 32],
            dma_completed_rx: dma_completed_rx,
            dma_completed_tx: dma_completed_tx,
            start_dma_tx: start_dma_tx,
        }
    }

    fn reset_display_list(&mut self) {
        self.dl_stack.clear();
        self.segments = [0u32; 16];
        self.vertices = [F3DZEX2_Vertex::default(); 32];
    }

    pub fn process_display_list(&mut self, dl_start: u32, dl_length: u32) {
        debug!(target: "HLE", "processing display list from ${:08X}, length {} bytes", dl_start, dl_length);

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

    fn process_display_list_command(&mut self, addr: u32, cmd: u64) {
        print!("${:08X}: ${:08X}_${:08X}: ", addr, cmd >> 32, cmd & 0xFFFF_FFFF);
        let depth = self.dl_stack.len() - 1;
        for _ in 0..depth { print!("  "); }

        let mut size: u32 = 0;
        let op = (cmd >> 56) & 0xFF;
        match op {
            0x00 => { // G_NOOP
                let addr = (cmd & 0xFFFF_FFFF) as u32;
                if addr != 0 {
                    let s = self.load_string(addr, 64);
                    print!("gsDPNoOpString([0x{:08X}] \"{}\")", addr, s);
                } else {
                    print!("gsDPNoOp()");
                }
            },

            0x01 => { // G_VTX
                let numv  = (cmd >> 44) as u8;
                let vbidx = (((cmd >> 33) & 0x7F) as u8) - numv;
                let addr  = cmd as u32;

                let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else {
                    let segment = ((addr >> 24) & 0x0F) as usize;
                    self.segments[segment] + (addr & 0x00FF_FFFF)
                };

                let vtx_size = mem::size_of::<F3DZEX2_Vertex>();
                let data_size = numv as usize * vtx_size;
                print!("gsSPVertex(0x{:08X} [0x{:08X}], {}, {}) (size_of<vtx>={}, data_size={})", addr, translated_addr, numv, vbidx, vtx_size, data_size);

                let vtx_data = self.load_display_list(translated_addr, data_size as u32);
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

                    print!("\nv{}: {:?}", i+vbidx, vtx);
                    self.vertices[(i + vbidx) as usize] = vtx;
                }
            },

            0x05 => { // G_TRI1
                let v0 = ((cmd >> 49) & 0x7F) as u8;
                let v1 = ((cmd >> 41) & 0x7F) as u8;
                let v2 = ((cmd >> 33) & 0x7F) as u8;
                print!("gsSP1Triangle({}, {}, {})", v0, v1, v2);
            },

            0x06 => { // G_TRI2
                let v00 = ((cmd >> 49) & 0x7F) as u8;
                let v01 = ((cmd >> 41) & 0x7F) as u8;
                let v02 = ((cmd >> 33) & 0x7F) as u8;
                let v10 = ((cmd >> 17) & 0x7F) as u8;
                let v11 = ((cmd >>  9) & 0x7F) as u8;
                let v12 = ((cmd >>  1) & 0x7F) as u8;
                print!("gsSP2Triangle({}, {}, {}, 0, {}, {}, {}, 0)", v00, v01, v02, v10, v11, v12);
            },

            0xD7 => { // G_TEXTURE
                let ts    = cmd as u16;
                let ss    = (cmd >> 16) as u16;
                let on    = ((cmd >> 33) & 0x7F) != 0;
                let tile  = ((cmd >> 40) & 0x07) as u8;
                let level = (cmd >> 43) & 0x07;
                print!("gsSPTexture(0x{:04X}, 0x{:04X}, {}, {}, {})", ss, ts, level, tile, on);
            },

            0xD9 => { // G_GEOMETRYMODE
                let clearbits = ((cmd >> 32) & 0x00FF_FFFF) as u32;
                let setbits   = cmd as u32;
                if clearbits == 0 && setbits != 0 {
                    print!("gsSPLoadGeometryMode(0x{:08X})", setbits);
                } else if clearbits != 0 && setbits == 0 {
                    print!("gsSPClearGeometryMode(0x{:08X})", clearbits);
                } else {
                    print!("gsSPGeometryMode(0x{:08X}, 0x{:08X})", clearbits, setbits);
                }
            },

            0xDA => { // G_MTX
                let params = (cmd >> 32) as u8;
                let addr   = cmd as u32;

                let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else {
                    let segment = ((addr >> 24) & 0x7F) as usize;
                    self.segments[segment] + (addr & 0x00FF_FFFF)
                };

                let push = (params & 0x01) != 0;
                let mul  = (params & 0x02) == 0;
                let proj = (params & 0x04) != 0; // true = projection matrix, false = modelview

                let mut s = String::from("0");
                if push { s.push_str("|G_MTX_PUSH"); } else { s.push_str("|G_MTX_NOPUSH"); }
                if mul  { s.push_str("|G_MTX_MUL"); } else { s.push_str("|G_MTX_LOAD"); }
                if proj { s.push_str("|G_MTX_PROJECTION"); } else { s.push_str("|G_MTX_MODELVIEW"); }
                println!("gsSPMatrix(0x{:08X}, {})", addr, s);

                let mtx_data = self.load_display_list(translated_addr, 64);
                let mut mtx: F3DZEX2_Matrix = [[0i32; 4]; 4];
                let mut fmtx = [[0f32; 4]; 4];

                for row in 0..4 {
                    match row {
                        0   => print!(" / "),
                        1|2 => print!("|  "),
                        3   => print!(" \\ "),
                        _   => {},
                    }

                    // 00001111 22223333 44445555 66667777
                    // 88889999 aaaabbbb ccccdddd eeeeffff
                    // gggghhhh iiiijjjj kkkkllll mmmmnnnn
                    // oooopppp qqqqrrrr sssstttt uuuuvvvv
                    // becomes
                    // 0000.gggg 1111.hhhh 2222.iiii 3333.jjjj
                    // ..
                    // cccc.kkkk dddd.tttt eeee.uuuu ffff.vvvv
                    for col in 0..4 {
                        let idx16 = row*4+col;
                        let idx   = idx16 >> 1;
                        //let y     = idx >> 3;
                        //let x     = (idx >> 1) & 0x03;
                        let shift = 16 - ((idx16 & 1) << 4);

                        let intpart = (mtx_data[idx] >> shift) as i16;
                        let fracpart = (mtx_data[8 + idx] >> shift) as u16;

                        print!("{:04x}.{:04x} ", intpart, fracpart);
                        mtx[row][col] = ((intpart as i32) << 16) | (fracpart as i32);
                    }

                    match row {
                        0   => print!("\\   /"),
                        1|2 => print!(" | | "),
                        3   => print!("/   \\"),
                        _   => {},
                    }

                    for col in 0..4 {
                        let e = mtx[row][col];
                        let f = (e >> 16) as f32 + ((e as u16) as f32) / 65536.0;
                        fmtx[row][col] = f;
                        print!("{:8.4}", f);
                    }

                    match row {
                        0   => println!(" \\"),
                        1|2 => println!("  |"),
                        3   => print!(" /   "),
                        _   => {},
                    }

                }

                if proj {
                    self.hle_command_buffer.try_push(HleRenderCommand::SetProjectionMatrix(fmtx))
                } else {
                    self.hle_command_buffer.try_push(HleRenderCommand::SetModelViewMatrix(fmtx))
                }.expect("HLE command buffer full");
            },

            0xDB => { // G_MOVEWORD
                let index  = ((cmd >> 48) & 0xFF) as u8;
                let offset = ((cmd >> 32) & 0xFFFF) as u16;
                let data   = cmd as u32;

                match index {
                    6 => { // G_MW_SEGMENT
                        print!("gsSPSegment({}, 0x{:08X})", offset >> 2, data);
                        self.segments[(offset >> 2) as usize] = data;
                    },

                    _ => {
                        print!("gsMoveWd({}, 0x{:04X}, 0x{:08X})", index, offset, data);
                    },
                };
            },

            0xDC => { // G_MOVEMEM
                let size  = ((((cmd >> 48) & 0xFF) >> 3) + 1) << 3;
                let index = (cmd >> 32) as u8;
                let addr  = (cmd & 0xFFFF_FFFF) as u32;

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
                        let z_scale = (zs >> 2) as f32 + frac[(zs & 3) as usize];
                        let xt = (vp[2] >> 16) as i16;
                        let yt = vp[2] as i16;
                        let zt = (vp[3] >> 16) as i16;
                        let x_translate = (xt >> 2) as f32 + frac[(xt & 3) as usize];
                        let y_translate = (yt >> 2) as f32 + frac[(yt & 3) as usize];
                        let z_translate = (zt >> 2) as f32 + frac[(zt & 3) as usize];

                        println!("gsSPViewport(0x{:08X} [0x{:08X}])", addr, translated_addr);
                        print!("Viewport {{ vscale: [ {}, {}, {}, 0.0 ], vtrans: [ {}, {}, {}, 0.0 ] }}    ", x_scale, y_scale, z_scale, x_translate, y_translate, z_translate);

                        self.hle_command_buffer.try_push(HleRenderCommand::Viewport {
                            x: -1.0 * x_scale + x_translate,
                            y: -1.0 * y_scale + y_translate,
                            w:  2.0 * x_scale,
                            h:  2.0 * y_scale,
                        }).expect("HLE command buffer full");
                    },

                    _ => {
                        print!("gsSPMoveMem?({}, ...)", index);
                    },
                };
            },

            0xDE => { // G_DL
                let is_link = (cmd & 0x00FF_0000_0000_0000) == 0;
                let addr = (cmd & 0xFFFF_FFFF) as u32;
                let segment = (cmd >> 24) as u8;
                if is_link {
                    print!("gsSPDisplayList(0x{:08X})", addr);

                    let translated_addr = if (addr & 0x8000_0000) != 0 { addr } else { (addr & 0x00FF_FFFF) + self.segments[segment as usize] };

                    // append a DL stack entry
                    let new_dl = DLStackEntry {
                        base_address: translated_addr,
                        ..Default::default()
                    };
                    self.dl_stack.push(new_dl);
                } else {
                    print!("gsSPBranchList(0x{:08X})", addr);

                    // replace the current DL with a new one
                    let cur = self.dl_stack.last_mut().unwrap();
                    cur.dl.clear();
                    cur.base_address = addr;
                    cur.pc = 0;
                }
            },

            0xDF => { // G_ENDDL
                print!("gsSPEndDisplayList()");
                self.dl_stack.pop();
            },

            0xE2 => { // G_SETOTHERMODE_L
                print!("gsSPSetOtherModeL(...)");
            }

            0xE3 => { // G_SETOTHERMODE_H
                print!("gsSPSetOtherModeH(...)");
            }

            0xE4 => { // G_TEXRECT(E4)
                let cmd1 = self.next_display_list_command();
                let cmd2 = self.next_display_list_command();
                size += 4;
                let x1   = ((cmd >> 44) & 0xFFF) as u16;
                let y1   = ((cmd >> 32) & 0xFFF) as u16;
                let tile = ((cmd >> 24) & 0x0F) as u8;
                let x0   = ((cmd >> 12) & 0xFFF) as u16;
                let y0   = ((cmd >>  0) & 0xFFF) as u16;
                let s0   = (cmd1 >> 16) as u16;
                let t0   = (cmd1 >>  0) as u16;
                let dsdx = (cmd2 >> 16) as u16;
                let dtdy = (cmd2 >>  0) as u16;
                print!("gsSPTextureRectange({}, {}, {}, {}, {}, {}, {}, {}, {})", x0, y0, x1, y1, tile, s0, t0, dsdx, dtdy);
            },

            0xE6 => { // G_RDPLOADSYNC
                print!("gsDPLoadSync()");
            },

            0xE7 => { // G_PIPESYNC
                print!("gsDPPipeSync()");
            },

            0xE8 => { // G_RDPTILESYNC
                print!("gsDPTileSync()");
            },

            0xE9 => { // G_FULLSYNC
                print!("gsDPFullSync()");
                self.hle_command_buffer.try_push(HleRenderCommand::Sync).expect("HLE command buffer full");
            },

            0xED => { // G_SETSCISSOR
                // TODO: I think the x/y values are 12bit 10.2 fixed point format?
                let x0 = ((cmd >> 44) & 0xFFF) as u16;
                let y0 = ((cmd >> 32) & 0xFFF) as u16;
                let m  = ((cmd >> 28) & 0x0F) as u8;
                let x1 = ((cmd >> 12) & 0xFFF) as u16;
                let y1 = ((cmd >>  0) & 0xFFF) as u16;
                print!("gsDPSetScissor({}, {}, {}, {}, {})", m, x0, y0, x1 >> 2, y1 >> 2);
            },

            0xEF => { // G_SETOTHERMODE
                let hi = ((cmd >> 32) & 0x00FF_FFFF) as u32;
                let lo = cmd as u32;
                print!("gsDPSetOtherMode(0x{:08X}, 0x{:08X})", hi, lo);
            },

            0xF0 => { // G_LOADTLUT
                let tile  = ((cmd >> 24) & 0x0F) as u8;
                let count = ((cmd >> 14) & 0x03FF) as u16;
                print!("gsDPLoadTLUTCmd({}, {})", tile, count);
            },

            0xF2 => { // G_SETTILESIZE
                let x0 = ((cmd >> 44) & 0xFFF) as u16;
                let y0 = ((cmd >> 32) & 0xFFF) as u16;
                let tile = ((cmd >> 24) & 0x0F) as u8;
                let x1 = ((cmd >> 12) & 0xFFF) as u16;
                let y1 = ((cmd >>  0) & 0xFFF) as u16;
                print!("gsDPSetTileSize({}, {}, {}, {}, {})", tile, x0, y0, x1, y1);
            },

            0xF3 => { // G_LOADBLOCK
                let x0     = ((cmd >> 44) & 0xFFF) as u16;
                let y0     = ((cmd >> 32) & 0xFFF) as u16;
                let tile   = ((cmd >> 24) & 0x0F) as u8;
                let texels = ((cmd >> 12) & 0xFFF) as u16;
                let dxt    = ((cmd >>  0) & 0xFFF) as u16;
                print!("gsDPLoadBlock({}, {}, {}, {}, {})", tile, x0, y0, texels, dxt);
            },

            0xF4 => { // G_LOADTILE
                let s0     = ((cmd >> 44) & 0xFFF) as u16;
                let t0     = ((cmd >> 32) & 0xFFF) as u16;
                let tile   = ((cmd >> 24) & 0x0F) as u8;
                let s1     = ((cmd >> 12) & 0xFFF) as u16;
                let t1     = ((cmd >>  0) & 0xFFF) as u16;
                print!("gsDPLoadTile({}, {}, {}, {}, {})", tile, s0, t0, s1, t1);
            },

            0xF5 => { // G_SETTILE
                print!("gsDPSetTile(...)");
            },

            0xF6 => { // G_FILLRECT
                let x1 = (((cmd >> 44) & 0xFFF) as u16) >> 2;
                let y1 = (((cmd >> 32) & 0xFFF) as u16) >> 2;
                let x0 = (((cmd >> 12) & 0xFFF) as u16) >> 2;
                let y0 = (((cmd >>  0) & 0xFFF) as u16) >> 2;
                print!("gsDPFillRectangle({}, {}, {}, {})", x0, y0, x1, y1);
                self.hle_command_buffer.try_push(HleRenderCommand::FillRectangle {
                    x: x0 as f32,
                    y: y0 as f32,
                    w: (x1 - x0) as f32 + 1.0,
                    h: (y1 - y0) as f32 + 1.0,
                    c: [((self.fill_color >> 11) & 0x1F) as f32 / 32.0, ((self.fill_color >> 6) & 0x1F) as f32 / 32.0,
                        ((self.fill_color >>  1) & 0x1F) as f32 / 32.0, 1.0],
                }).expect("HLE command buffer full");
            },

            0xF7 => { // G_SETFILLCOLOR 
                assert!((cmd >> 16) as u16 == (cmd & 0xFFFF) as u16); // Someday some code will fill a rect with alternating colors
                self.fill_color = cmd as u32;
                print!("gsDPSetFillColor(0x{:08X})", self.fill_color);
            },

            0xF9 => { // G_SETBLENDCOLOR
                let r = (cmd >> 24) as u8;
                let g = (cmd >> 16) as u8;
                let b = (cmd >>  8) as u8;
                let a = (cmd >>  0) as u8;
                print!("gsDPBlendColor({}, {}, {}, {})", r, g, b, a);
            },

            0xFA => { // G_SETPRIMCOLOR
                let minlevel = (cmd >> 40) as u8;
                let lodfrac  = (cmd >> 32) as u8;
                let r = (cmd >> 24) as u8;
                let g = (cmd >> 16) as u8;
                let b = (cmd >>  8) as u8;
                let a = (cmd >>  0) as u8;
                print!("gsDPSetPrimColor({}, {}, {}, {}, {}, {})", minlevel, lodfrac, r, g, b, a);
            },

            0xFB => { // G_SETENVCOLOR
                let r = (cmd >> 24) as u8;
                let g = (cmd >> 16) as u8;
                let b = (cmd >>  8) as u8;
                let a = (cmd >>  0) as u8;
                print!("gsDPSetEnvColor({}, {}, {}, {})", r, g, b, a);
            },


            0xFC => { // G_SETCOMBINE
                print!("gsDPSetCombineLERP(...)");
            },

            0xFD => { // G_SETTIMG
                let addr = cmd as u32;
                print!("gsDPSetTextureImage(..., 0x{:08X})", addr);
            },

            0xFE => { // G_SETCIMG
                let addr = cmd as u32;
                print!("gsDPSetDepthImage(..., 0x{:08X})", addr);
            },

            0xFF => { // G_SETCIMG
                let addr = cmd as u32;
                print!("gsDPSetColorImage(..., 0x{:08X})", addr);
            },

            _ => {
                unimplemented!("unimplemented DL command ${:02X}", op);
            },
        };

        println!(" {{{}}}", size + 2);
    }
}
