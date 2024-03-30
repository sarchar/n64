struct VertexInput {
    @location(0) position   : vec4<f32>,
    @location(1) color      : vec4<f32>,
    @location(2) normal     : vec3<f32>,
    @location(3) tex_coords : vec2<f32>,
    @location(4) tex_coords1: vec2<f32>,
    @location(5) tex_params : vec4<f32>,
    @location(6) tex_params1: vec4<f32>,
    @location(7) maskshift  : u32,
    @location(8) maskshift1 : u32,
    @location(9) flags      : u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) tex_coords: vec2<f32>,
};

@vertex
fn vs_main(
    model: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = model.position;
    out.tex_coords = model.tex_coords;
    return out;
}

@group(0) @binding(0)
var t_diffuse: texture_2d<f32>;
@group(0) @binding(1)
var s_diffuse: sampler;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return textureSample(t_diffuse, s_diffuse, in.tex_coords);
}
