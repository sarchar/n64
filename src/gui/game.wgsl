const ENABLE_TEXTURE: u32 = 0x01u;
const LINEAR_FILTER : u32 = 0x02u;

struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(1) @binding(0)
var<uniform> camera: CameraUniform;

struct ColorCombinerState {
    color1_source: u32,
    alpha1_source: u32,
    color2_source: u32,
    alpha2_source: u32,
    prim_color   : vec4<f32>,
    env_color    : vec4<f32>,
};

@group(1) @binding(1)
var<uniform> color_combiner_state: ColorCombinerState;

struct VertexInput {
    @location(0) position: vec4<f32>,
    @location(1) tex_coords: vec2<f32>,
    @location(2) color: vec4<f32>,
    @location(3) flags: u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) tex_coords: vec2<f32>,
    @location(1) color: vec4<f32>,
    @location(2) flags: u32,
};

struct FragmentOutput {
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;


    out.tex_coords    = vec2<f32>(in.tex_coords.x, in.tex_coords.y);
    out.clip_position = in.position * camera.view_proj;
    out.color         = in.color;
    out.flags         = in.flags;

    return out;
}

@group(0) @binding(0)
var t_diffuse_linear: texture_2d<f32>;
@group(0) @binding(1)
var s_diffuse_linear: sampler;

@group(0) @binding(2)
var t_diffuse_nearest: texture_2d<f32>;
@group(0) @binding(3)
var s_diffuse_nearest: sampler;

@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {
    var out: FragmentOutput;

    let rc = rasterizer_color(in);
    let cc = color_combine(rc, in);

    out.color = cc;
    return out;
}

fn rasterizer_color(in: VertexOutput) -> vec4<f32> {
    let tex_linear  = textureSample(t_diffuse_linear, s_diffuse_linear, in.tex_coords);
    let tex_nearest = textureSample(t_diffuse_nearest, s_diffuse_nearest, in.tex_coords);

    if ((in.flags & ENABLE_TEXTURE) == ENABLE_TEXTURE) {
        if ((in.flags & LINEAR_FILTER) == LINEAR_FILTER) {
            return tex_linear;
        } else {
            return tex_nearest;
        }
    } else {
        return in.color;
    }
}

fn select_colorA(src: u32, rc: vec4<f32>, in: VertexOutput) -> vec4<f32> {
    switch(src) {
        case 1u: {
            return rc;
        }

        default: {
            return vec4(1.0, 0.0, 1.0, 1.0);
        }
    }
}

// (A-B)*C+D
fn color_combine(rc: vec4<f32>, in: VertexOutput) -> vec4<f32> {
    // So ugly.  Why can't I use u8 types in this language?
    let a0c_source = color_combiner_state.color1_source / 16777216u;
    let b0c_source = (color_combiner_state.color1_source / 65536u) % 256u; 
    let c0c_source = (color_combiner_state.color1_source / 256u) % 256u;
    let d0c_source = color_combiner_state.color1_source % 256u;

    let a = select_colorA(a0c_source, rc, in);

    if a0c_source == 1u && c0c_source == 3u {
        // A=texture color
        // C=prim color
        // B=D=0
        return rc * color_combiner_state.prim_color;
    } else {
        return rc;
    }
}
