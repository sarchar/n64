const ENABLE_TEXTURE: u32 = 0x01u;
const LINEAR_FILTER : u32 = 0x02u;

struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(1) @binding(0)
var<uniform> camera: CameraUniform;

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

    let tex_linear  = textureSample(t_diffuse_linear, s_diffuse_linear, in.tex_coords);
    let tex_nearest = textureSample(t_diffuse_nearest, s_diffuse_nearest, in.tex_coords);

    if ((in.flags & ENABLE_TEXTURE) == ENABLE_TEXTURE) {
        if ((in.flags & LINEAR_FILTER) == LINEAR_FILTER) {
            out.color = tex_linear;
        } else {
            out.color = tex_nearest;
        }
    } else {
        out.color = in.color;
    }

    return out;
}
