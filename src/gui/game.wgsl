const VERTEX_FLAG_TEXTURED_SHIFT     : u32 = 0u;
const VERTEX_FLAG_LINEAR_FILTER_SHIFT: u32 = 1u;
const VERTEX_FLAG_TEXMODE_S_SHIFT    : u32 = 2u;
const VERTEX_FLAG_TEXMODE_T_SHIFT    : u32 = 4u;
const VERTEX_FLAG_LIT_SHIFT          : u32 = 6u;
const VERTEX_FLAG_TWO_CYCLE_SHIFT    : u32 = 7u;

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
    @location(5) maskshift : u32,
    @location(6) flags     : u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,

    // color and tex_coords are interpolated across the poly
    @location(0) color     : vec4<f32>,
    @location(1) normal    : vec3<f32>,
    @location(2) tex_coords: vec2<f32>,
    // flags and tex_params are the same at each vertex, so look as if they're constant
    @location(3) tex_params: vec4<f32>,
    @location(4) maskshift : u32,
    @location(5) flags     : u32,
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
    out.maskshift     = in.maskshift;
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

fn texcoord(st_in: f32, minval: f32, maxval: f32, mask: f32, mode: u32) -> f32 {
    if mask < 0.1 { // when not masking, clamp is implicit and mode is ignored
        return clamp(minval + st_in, minval, maxval);
    } else {
        var st: f32;
        let mask2 = pow(2.0, mask);

        // optionally clamp input coordinate before mirror/wrap
        if(extractBits(mode, 2u, 1u) == 1u) {
            let range = maxval - minval;
            st = clamp(st_in / range, 0.0, 1.0) * range;
        } else {
            st = st_in;
        }

        // -8, -7, .. -2, -1, 0, 1, 2, 3, 4, .. 8, 9, 10, ..
        //  0   1      6   7  0, 1, 2, 3, 4, .. 0, 1, 2, ..   <-- WRAP
        //      7      2   1  0, 1, 2, 3, 4, .. 7, 7, 5, ..   <-- MIRROR
        if(extractBits(mode, 0u, 1u) == 0u) { // WRAP (performed by masking)
            if st < 0.0 {
                return minval + (mask2 + (st % mask2));
            } else {
                return minval + (st % mask2);
            }
        } else { // MIRROR
            let count = u32(trunc(st / mask2));
            if (extractBits(count, 0u, 1u) == 1u) { 
                let inverted = mask2 - (abs(st) % mask2);
                return minval + inverted;
            } else {
                return minval + (abs(st) % mask2);
            }
        }
    }
}

fn shift(st: f32, shift: f32) -> f32 {
    if shift < 0.1 {  // no shift, leave texcoord alone
        return st;
    } else if shift <= 10.0 {
        return st / pow(2.0, shift);
    } else {
        return st * pow(2.0, 5.0 - (shift - 11.0));
    }
}

fn rasterizer_color(in: VertexOutput) -> vec4<f32> {
    let texmode_s  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_S_SHIFT, 2u);
    let texmode_t  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_T_SHIFT, 2u);
    let mask_s     = f32(extractBits(in.maskshift, 24u, 8u)); // 0xMSssMTst // mask_s shift_s mask_t shift_t
    let shift_s    = f32(extractBits(in.maskshift, 16u, 8u));
    let mask_t     = f32(extractBits(in.maskshift,  8u, 8u));
    let shift_t    = f32(extractBits(in.maskshift,  0u, 8u));

    let tex_coords = vec2(texcoord(shift(in.tex_coords.x, shift_s), in.tex_params.x, in.tex_params.y, mask_s, texmode_s),
                          texcoord(shift(in.tex_coords.y, shift_t), in.tex_params.z, in.tex_params.w, mask_t, texmode_t));

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
    if (extractBits(in.flags, VERTEX_FLAG_LIT_SHIFT, 1u) == 0u) {
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

fn select_color(letter: u32, src: u32, cc: vec4<f32>, rc: vec4<f32>, shade: vec4<f32>, in: VertexOutput) -> vec3<f32> {
    switch(src) {
        case 0u: {
            return cc.rgb;
        }

        case 1u: {
            return rc.rgb;
        }

        case 2u: { // TODO TEXEL1
            return rc.rgb;//vec3(1.0, 1.0, 1.0);
        }

        case 3u: {
            return color_combiner_state.prim_color.rgb;
        }

        case 4u: {
            return shade.rgb;
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
                case 0u: { // NOISE
                    return vec3(252.0/255.0, 3.0/255.0, 198.0/255.0);//vec3(1.0, 1.0, 1.0);
                }

                case 2u: { // COMBINED_ALPHA
                    return cc.aaa;
                }

                case 3u: {
                    return vec3(0.0, 0.0, 0.0);
                }

                // TODO CCMUX_K4
                default: {
                    return vec3(1.0, 1.0, 0.0);
                }
            }
        }

        case 8u, 9u: {
            switch(letter) {
                case 2u: { return rc.aaa; } // TEXELn_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        case 10u: {
            switch(letter) {
                case 2u: { return color_combiner_state.prim_color.aaa; } // PRIMITIVE_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        case 11u: {
            switch(letter) {
                case 2u: { return shade.aaa; } // SHADE_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        case 12u: {
            switch(letter) {
                case 2u: { return color_combiner_state.env_color.aaa; } // ENVIRONMENT_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        case 13u: {
            switch(letter) {
                case 2u: { return vec3(0.0, 0.0, 0.0); } // LOD_FRACTION (TODO might need to be 1.0?)
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        case 14u: {
            switch(letter) {
                case 2u: { return vec3(0.0, 0.0, 0.0); } // PRIM_LOD_FRACTION (TODO might need to be 1.0?)
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        default: {
            return vec3(0.0, 0.0, 0.0);
        }
    }
}

fn select_alpha(letter: u32, src: u32, cc: f32, rc: f32, shade: f32, in: VertexOutput) -> f32 {
    // cases 0 and 6 will use `letter`

    switch(src) {
        case 0u: {
            switch(letter) {
                case 0u, 1u, 3u: {
                    return cc;
                }
                default: {
                    return 0.0;
                }
            }
        }

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
                    return 0.0;
                }
            }
        }

        default: {
            return 0.0;
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

    let cc_in = vec4(0.5, 0.5, 1.0, 1.0);
    let a = vec4(select_color(0u, a0c_source, cc_in, rc, shade_color, in), select_alpha(0u, a0a_source, cc_in.a, rc.a, shade_color.a, in));
    let b = vec4(select_color(1u, b0c_source, cc_in, rc, shade_color, in), select_alpha(1u, b0a_source, cc_in.a, rc.a, shade_color.a, in));
    let c = vec4(select_color(2u, c0c_source, cc_in, rc, shade_color, in), select_alpha(2u, c0a_source, cc_in.a, rc.a, shade_color.a, in));
    let d = vec4(select_color(3u, d0c_source, cc_in, rc, shade_color, in), select_alpha(3u, d0a_source, cc_in.a, rc.a, shade_color.a, in));

    let cc_out = (a - b) * c + d;

    if (extractBits(in.flags, VERTEX_FLAG_TWO_CYCLE_SHIFT, 1u) == 0u) {
        return cc_out;
    } else {
        let a1c_source = extractBits(color_combiner_state.color2_source, 24u, 8u);
        let b1c_source = extractBits(color_combiner_state.color2_source, 16u, 8u);
        let c1c_source = extractBits(color_combiner_state.color2_source,  8u, 8u);
        let d1c_source = extractBits(color_combiner_state.color2_source,  0u, 8u);

        let a1a_source = extractBits(color_combiner_state.alpha2_source, 24u, 8u);
        let b1a_source = extractBits(color_combiner_state.alpha2_source, 16u, 8u);
        let c1a_source = extractBits(color_combiner_state.alpha2_source,  8u, 8u);
        let d1a_source = extractBits(color_combiner_state.alpha2_source,  0u, 8u);

        let a2 = vec4(select_color(0u, a1c_source, cc_out, rc, shade_color, in), select_alpha(0u, a1a_source, cc_out.a, rc.a, shade_color.a, in));
        let b2 = vec4(select_color(1u, b1c_source, cc_out, rc, shade_color, in), select_alpha(1u, b1a_source, cc_out.a, rc.a, shade_color.a, in));
        let c2 = vec4(select_color(2u, c1c_source, cc_out, rc, shade_color, in), select_alpha(2u, c1a_source, cc_out.a, rc.a, shade_color.a, in));
        let d2 = vec4(select_color(3u, d1c_source, cc_out, rc, shade_color, in), select_alpha(3u, d1a_source, cc_out.a, rc.a, shade_color.a, in));

        return (a2 - b2) * c2 + d2;
    }
}
