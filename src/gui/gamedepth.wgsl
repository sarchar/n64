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
    out.tex_coords = model.tex_coords;
    out.clip_position = model.position;
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
    //let cmp = 0.5;
    var g = textureGather(t_depth, s_depth, in.tex_coords.xy);
    if(g.x < 0.8) {
        return vec4<f32>(1.0, 0.0, 0.0, 1.0);
    } else {
        g = 1.0 - ((g - 0.8) * 5);
        return vec4<f32>(g.xxx, 1.0);
    }
}
