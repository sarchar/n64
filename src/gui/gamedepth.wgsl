struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) tex_coords: vec2<f32>,
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
    out.tex_coords = vec2<f32>(model.tex_coords.x, model.tex_coords.y);
    out.clip_position = vec4<f32>(model.position, 1.0);
    return out;
}

@group(0) @binding(0)
var t_depth: texture_depth_2d;
@group(0) @binding(1)
var s_depth: sampler;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // TODO: no depth sampling for you! 
    // var cmp: f32 = textureSample(t_depth, s_depth, in.tex_coords.xy);
    let cmp = 0.5;
    return vec4<f32>(cmp, cmp, cmp, 1.0);
}
