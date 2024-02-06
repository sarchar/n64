const VERTEX_FLAG_TEXTURED_SHIFT     : u32 = 0x00u;
const VERTEX_FLAG_LINEAR_FILTER_SHIFT: u32 = 0x01u;
const VERTEX_FLAG_TEXMODE_S_SHIFT    : u32 = 0x02u;
const VERTEX_FLAG_TEXMODE_T_SHIFT    : u32 = 0x04u;
const VERTEX_FLAG_LIT                : u32 = 0x06u;

const VERTEX_FLAG_TEXMODE_MASK : u32 = 0x03u;

struct CameraUniform {
    projection: mat4x4<f32>,
    modelview : mat4x4<f32>,
    mv_inverse: mat4x4<f32>,
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

struct LightState {
    lights: array<vec4<f32>, 7>,
    colors: array<vec3<f32>, 7>,
}

@group(1) @binding(2)
var<uniform> light_state: LightState;

struct VertexInput {
    @location(0) position  : vec4<f32>,
    @location(1) color     : vec4<f32>,
    @location(2) normal    : vec3<f32>,
    @location(3) tex_coords: vec2<f32>,
    @location(4) tex_params: vec4<f32>,
    @location(5) flags     : u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,

    // color and tex_coords are interpolated across the poly
    @location(0) color     : vec4<f32>,
    @location(1) normal    : vec3<f32>,
    @location(2) tex_coords: vec2<f32>,
    // flags and tex_params are the same at each vertex, so look as if they're constant
    @location(3) tex_params: vec4<f32>,
    @location(4) flags     : u32,
};

struct FragmentOutput {
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    let mv_inverse = mat3x3(camera.mv_inverse[0].xyz, camera.mv_inverse[1].xyz, camera.mv_inverse[2].xyz);

    // the matrices used on n64 expect the vertex to be left-multiplied: v' = v*(M*V*P)
    out.clip_position = in.position * camera.modelview * camera.projection;
    out.normal        = normalize(in.normal * transpose(mv_inverse));
    out.color         = in.color;
    out.tex_coords    = in.tex_coords;
    out.tex_params    = in.tex_params;
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

fn texcoord(st: f32, minval: f32, maxval: f32, mode: u32) -> f32 {
    switch(mode) {
        case 0u: { // NOMIRROR | WRAP
            // we need to wrap within the (minval,maxval) range
            let range = maxval - minval;
            return minval + fract(st / range) * range;
        }
        case 1u: { // MIRROR
            // TODO mirror might only make sense after maskST and shiftST are implemented
            let range = maxval - minval;
            let wraps = st / range;
            let count = u32(trunc(wraps));
            if (extractBits(count, 0u, 1u) == 1u) { 
                return maxval - (1.0 - fract(wraps)) * range;
            } else {
                return minval + fract(wraps) * range;
            }
        }
        case 2u: { // CLAMP
            let size = maxval - minval;
            return minval + clamp(st, 0.0, size);
        }
        default: { // invalid, shouldn't happen
            return 0.0;
        }
    }
}

fn rasterizer_color(in: VertexOutput) -> vec4<f32> {
    let texmode_s  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_S_SHIFT, 2u);
    let texmode_t  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_T_SHIFT, 2u);

    let tex_coords = vec2(texcoord(in.tex_coords.x, in.tex_params.x, in.tex_params.y, texmode_s),
                          texcoord(in.tex_coords.y, in.tex_params.z, in.tex_params.w, texmode_t));

    let tex_linear  = textureSample(t_diffuse_linear, s_diffuse_linear, tex_coords / 512.0);
    let tex_nearest = textureSample(t_diffuse_nearest, s_diffuse_nearest, tex_coords / 512.0);

    if (extractBits(in.flags, VERTEX_FLAG_TEXTURED_SHIFT, 1u) == 1u) {
        if (extractBits(in.flags, VERTEX_FLAG_LINEAR_FILTER_SHIFT, 1u) == 1u) {
            return tex_linear;
        } else {
            return tex_nearest;
        }
    } else {
        return in.color;
    }
}

fn shade(in: VertexOutput) -> vec4<f32> {
    if (extractBits(in.flags, VERTEX_FLAG_LIT, 1u) == 0u) {
        return in.color;
    } else {
        // perform lighting with the ambient from in.color
        var color = in.color.rgb;
        for (var i: i32 = 0; i < 7; i++) {
            if (light_state.lights[i].w != 0.0) {
                let contrib = max(dot(in.normal, normalize(-light_state.lights[i].xyz)), 0.0);
                color += light_state.colors[i].rgb * contrib;
            }
        }
        return vec4(color, in.color.a);
    }
}

fn select_color(letter: u32, src: u32, rc: vec3<f32>, shade: vec3<f32>, in: VertexOutput) -> vec3<f32> {
    switch(src) {
        case 1u: {
            return rc;
        }

        case 2u: { // TODO TEXEL1
            return rc; //vec3(0.0, 0.0, 1.0);
        }

        case 3u: {
            return color_combiner_state.prim_color.rgb;
        }

        case 4u: {
            return shade;
        }

        case 5u: {
            return color_combiner_state.env_color.rgb;
        }
        
        case 6u: {
            switch(letter) {
                case 0u, 3u: {
                    return vec3(1.0, 1.0, 1.0);
                }
                // TODO CCMUX_CENTER
                // TODO CCMUX_SCALE
                default: {
                    return vec3(1.0, 0.0, 0.0);
                }
            }
        }

        case 7u: {
            switch(letter) {
                case 3u: {
                    return vec3(0.0, 0.0, 0.0);
                }
                // TODO CCMUX_NOISE
                // TODO CCMUX_K4
                // TODO CCMUX_COMBINED_ALPHA
                default: {
                    return vec3(1.0, 1.0, 0.0);
                }
            }
        }

        case 15u, 31u: {
            return vec3(0.0, 0.0, 0.0);
        }

        default: {
            return vec3(1.0, 0.0, 1.0);
        }
    }
}

fn select_alpha(letter: u32, src: u32, rc: f32, shade: f32, in: VertexOutput) -> f32 {
    // cases 0 and 6 will use `letter`

    switch(src) {
        case 1u: {
            return rc;
        }
        
        case 2u: { // TODO TEXEL1
            return rc;
        }

        case 3u: {
            return color_combiner_state.prim_color.a;
        }

        case 4u: {
            return shade;
        }

        case 5u: {
            return color_combiner_state.env_color.a;
        }

        case 6u: {
            switch(letter) {
                case 0u, 1u, 3u: {
                    return 1.0;
                }
                // TODO PRIM_LOD_FRAC
                default: {
                    return 0.5;
                }
            }
        }

        case 7u: {
            return 0.0;
        }

        default: {
            return 1.0;
        }
    }
}

// (A-B)*C+D
fn color_combine(rc: vec4<f32>, in: VertexOutput) -> vec4<f32> {
    let shade_color = shade(in);

    let a0c_source = extractBits(color_combiner_state.color1_source, 24u, 8u);
    let b0c_source = extractBits(color_combiner_state.color1_source, 16u, 8u);
    let c0c_source = extractBits(color_combiner_state.color1_source,  8u, 8u);
    let d0c_source = extractBits(color_combiner_state.color1_source,  0u, 8u);

    let a0a_source = extractBits(color_combiner_state.alpha1_source, 24u, 8u);
    let b0a_source = extractBits(color_combiner_state.alpha1_source, 16u, 8u);
    let c0a_source = extractBits(color_combiner_state.alpha1_source,  8u, 8u);
    let d0a_source = extractBits(color_combiner_state.alpha1_source,  0u, 8u);

    let a = vec4(select_color(0u, a0c_source, rc.rgb, shade_color.rgb, in), select_alpha(0u, a0a_source, rc.a, shade_color.a, in));
    let b = vec4(select_color(1u, b0c_source, rc.rgb, shade_color.rgb, in), select_alpha(1u, b0a_source, rc.a, shade_color.a, in));
    let c = vec4(select_color(2u, c0c_source, rc.rgb, shade_color.rgb, in), select_alpha(2u, c0a_source, rc.a, shade_color.a, in));
    let d = vec4(select_color(3u, d0c_source, rc.rgb, shade_color.rgb, in), select_alpha(3u, d0a_source, rc.a, shade_color.a, in));

    return (a - b) * c + d;
}
