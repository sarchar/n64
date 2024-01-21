struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(1) @binding(0)
var<uniform> camera: CameraUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) tex_coords: vec2<f32>,
    @location(2) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) tex_coords: vec2<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_main(
    in: VertexInput,
) -> VertexOutput {
    var out: VertexOutput;
    out.tex_coords    = vec2<f32>(in.tex_coords.x, in.tex_coords.y);
    out.clip_position = vec4<f32>(in.position, 1.0) * camera.view_proj;
    out.color         = in.color;
    return out;
}

@group(0) @binding(0)
var t_diffuse: texture_2d<f32>;
@group(0) @binding(1)
var s_diffuse: sampler;

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
//    // texcoords are coming in 0..16
//    // assuming texture is 16x16, but the data is actually linear in a 64x32 texture
//    let half_texel_x = 1.0/(2.0*64.0);
//    let half_texel_y = 1.0/(2.0*32.0);
//
//    // compute integer index into linear texture data
//    // also, flip y
//    let i: i32 = i32(16.0 - in.tex_coords.y) * 16 + i32(in.tex_coords.x);
//
//    // now convert to 64x32 texture
//    let yi = i / 64;
//    let xi = i - (yi * 64);
//
//    // now convert to 0.0..1.0 and center the sample
//    let coords = vec2(f32(xi)/64.0 + half_texel_x, f32(yi)/32.0 + half_texel_y);
//
//    // dont forget to change the y coordinate in the vertex shader!
//    let tex = textureSample(t_diffuse, s_diffuse, coords);
    let tex = textureSample(t_diffuse, s_diffuse, in.tex_coords);

    // if in.color.a is 1.0, give 1.0
    // if in.color.a is 0.0, give tex.a
    // in.color.a * 1.0 + (1.0 - in.color.a) * tex.a
    return vec4(in.color.rgb * in.color.a + tex.rgb * (1.0 - in.color.a), 1.0 * in.color.a + (1.0 - in.color.a) * tex.a);
}
