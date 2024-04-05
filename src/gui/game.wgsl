const VERTEX_FLAG_TEXTURED_SHIFT           : u32 = 0u;
const VERTEX_FLAG_LINEAR_FILTER_SHIFT      : u32 = 1u;
const VERTEX_FLAG_TEXMODE_S_SHIFT          : u32 = 2u;
const VERTEX_FLAG_TEXMODE_T_SHIFT          : u32 = 4u;
const VERTEX_FLAG_TEXMODE_S1_SHIFT         : u32 = 6u;
const VERTEX_FLAG_TEXMODE_T1_SHIFT         : u32 = 8u;
const VERTEX_FLAG_LIT_SHIFT                : u32 = 10u;
const VERTEX_FLAG_TWO_CYCLE_SHIFT          : u32 = 11u;
const VERTEX_FLAG_ZMODE_SHIFT              : u32 = 12;
const VERTEX_FLAG_ALPHA_COMPARE_MODE_SHIFT : u32 = 14;
const VERTEX_FLAG_TEX_EDGE                 : u32 = 16;

const VERTEX_FLAG_ZMODE_MASK               : u32 = 0x03u;
const VERTEX_FLAG_TEXMODE_MASK             : u32 = 0x03u;
const VERTEX_FLAG_ALPHA_COMPARE_MODE_MASK  : u32 = 0x03u;

struct CameraUniform {
    projection: mat4x4<f32>,
};

@group(2) @binding(0)
var<uniform> camera: CameraUniform;

struct ColorCombinerState {
    color1_source: u32,
    alpha1_source: u32,
    color2_source: u32,
    alpha2_source: u32,
    prim_color   : vec4<f32>,
    env_color    : vec4<f32>,
    blend_color  : vec4<f32>,
    lodfrac      : f32,
};

@group(2) @binding(1)
var<uniform> color_combiner_state: ColorCombinerState;

struct LightState {
    lights: array<vec4<f32>, 7>,
    colors: array<vec3<f32>, 7>,
}

@group(2) @binding(2)
var<uniform> light_state: LightState;

struct FogState {
    color     : vec4<f32>,
    multiplier: f32,
    offset    : f32,
};

@group(2) @binding(3)
var<uniform> fog_state: FogState;

struct VertexInput {
    @location(0) view_position: vec4<f32>,
    @location(1) color        : vec4<f32>,
    @location(2) view_normal  : vec3<f32>,
    @location(3) tex_coords   : vec2<f32>,
    @location(4) tex_coords1  : vec2<f32>,
    @location(5) tex_params   : vec4<f32>,
    @location(6) tex_params1  : vec4<f32>,
    @location(7) maskshift    : u32,
    @location(8) maskshift1   : u32,
    @location(9) flags        : u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,

    // position, color and tex_coords are interpolated across the poly
    @location(0) view_position: vec4<f32>,
    @location(1) color        : vec4<f32>,
    @location(2) view_normal  : vec3<f32>,
    @location(3) tex_coords   : vec2<f32>,
    @location(4) tex_coords1  : vec2<f32>,
    // flags and tex_params are the same at each vertex, so look as if they're constant
    @location(5) tex_params   : vec4<f32>,
    @location(6) tex_params1  : vec4<f32>,
    @location(7) maskshift    : u32,
    @location(8) maskshift1   : u32,
    @location(9) flags        : u32,
    @location(10) clip_z      : f32,
};

struct FragmentOutput {
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;

    // need a view space position for per-pixel lighting
    out.view_position = in.view_position;
    
    // transform with projection matrix
    var clip_position: vec4<f32> = out.view_position * camera.projection;
    
    // apply N64 z viewport (z' = z*scale+trans) after perspective correction
    // preserve the sign of z, as it indicates the direction from the near clip plane
    var nz = clip_position.z / abs(clip_position.w);
    nz = (nz * 127.75) + 127.75; // Z should have been -1..1, => 0..256
    nz /= 256.0; // renormalize to N64 max Z

    // for decals, move z closer to camera
    if(extractBits(in.flags, VERTEX_FLAG_ZMODE_SHIFT, 2u) == 3u) {
        clip_position.z -= 0.001 * abs(clip_position.w);
    }

    // OoT beating heart has Z value -0.00085
    // TODO: clamps everything here, but not sure if that's the right thing to do or if there's a reason why the value is <0.
    // TODO: after working on fog, I think OoT uses a z viewport of -1..1 (with almost everything Z>0.8). More investigation needed.
    // Perhaps the viewport scaling above can be inserted into the projection matrix and then this check isn't necessary
    if (clip_position.z / clip_position.w) < 0 {
        clip_position.z = 0.0;
    }

    // pass normalized clip z to fragment shader for fog
    out.clip_z = nz;

    out.clip_position = clip_position;
    out.view_normal   = in.view_normal;
    out.color         = in.color;
    out.tex_coords    = in.tex_coords;
    out.tex_coords1   = in.tex_coords1;
    out.tex_params    = in.tex_params;
    out.tex_params1   = in.tex_params1;
    out.maskshift     = in.maskshift;
    out.maskshift1    = in.maskshift1;
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

@group(1) @binding(0)
var t_diffuse_linear1: texture_2d<f32>;
@group(1) @binding(1)
var s_diffuse_linear1: sampler;

@group(1) @binding(2)
var t_diffuse_nearest1: texture_2d<f32>;
@group(1) @binding(3)
var s_diffuse_nearest1: sampler;

@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {
    var out: FragmentOutput;

    let rc  = rasterizer_color(in);
    let rc1 = texel1_color(in);
    var cc  = color_combine(rc, rc1, in);

    // Threshold alpha -- throw away if alpha is LTE the blend color register
    if(extractBits(in.flags, VERTEX_FLAG_ALPHA_COMPARE_MODE_SHIFT, 2u) == 1u) {
        if(cc.a <= color_combiner_state.blend_color.a) { 
            discard; 
            //cc = vec4<f32>(color_combiner_state.blend_color.aaa, 1.0);
        }
    } else if(extractBits(in.flags, VERTEX_FLAG_TEX_EDGE, 1u) == 1u) {
        if(cc.a == 0.0) {
            discard;
            //cc = vec4<f32>(207.0/255.0, 52.0/255.0, 235.0/255.0, 1.0);
        }
    }

    // for opaque mode, kill alpha
    if(extractBits(in.flags, VERTEX_FLAG_ZMODE_SHIFT, 2u) == 0u) {
        cc.a = 1.0;
    }

    // apply fog
    let usable_z = in.clip_z;
    if(usable_z < 1.0 && fog_state.multiplier != 0.0 && fog_state.offset != 0.0) {
        // compute alpha for fog
        let fog_alpha = clamp((usable_z * 2.0 - 1.0) * fog_state.multiplier + fog_state.offset, 0.0, 1.0);

        // apply to color
        let new_color = fog_state.color.rgb * fog_alpha + cc.rgb * (1.0 - fog_alpha);
        cc = vec4<f32>(new_color.rgb, cc.a);
    }

    //out.color = vec4<f32>(usable_z, usable_z, usable_z, 1.0); // show Z clip space value
    out.color = cc;
    return out;
}

fn texcoord(st_in: f32, minval: f32, maxval: f32, mask: f32, mode: u32) -> f32 {
    if mask < 0.1 { // when not masking, clamp is implicit and mode is ignored
        return clamp(minval + st_in, minval, maxval);
    } else {
        var st: f32;
        let mask2 = pow(2.0, mask);

        // -8, -7, .. -2, -1, 0, 1, 2, 3, 4, .. 8, 9, 10, ..
        //  0   1      6   7  0, 1, 2, 3, 4, .. 0, 1, 2, ..   <-- WRAP
        //      7      2   1  0, 1, 2, 3, 4, .. 7, 6, 5, ..   <-- MIRROR

        // Clamp and Wrap are mutually exclusive
        if(extractBits(mode, 1u, 1u) == 1u) { // CLAMP
            let range = maxval - minval;
            // guess you are allowed to have mirror on clamp, if only once on the negative side?
            if(extractBits(mode, 0u, 1u) == 1u) { // MIRROR
                var count = u32(trunc(st_in / mask2)); // negative st_in should have proper count
                if (extractBits(count, 0u, 1u) == 1u) { 
                    let inverted = mask2 - (abs(st_in) % mask2);
                    st = clamp(inverted / range, 0.0, 1.0) * range;
                } else {
                    st = clamp(st_in / range, 0.0, 1.0) * range;
                }
            } else {
                st = clamp(st_in / range, 0.0, 1.0) * range;
            }
            return minval + st;
        } else { // WRAP
            if(extractBits(mode, 0u, 1u) == 1u) { // MIRROR
                var count = u32(trunc(st_in / mask2)); // negative st_in should have proper count
                if (extractBits(count, 0u, 1u) == 1u) { 
                    let inverted = mask2 - (abs(st_in) % mask2);
                    return minval + inverted;
                } else {
                    return minval + (abs(st_in) % mask2);
                }
            } else {
                if st_in < 0.0 {
                    return minval + (mask2 + (st_in % mask2));
                } else {
                    return minval + (st_in % mask2);
                }
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

    if (extractBits(in.flags, VERTEX_FLAG_TEXTURED_SHIFT, 1u) == 1u) {
        if (extractBits(in.flags, VERTEX_FLAG_LINEAR_FILTER_SHIFT, 1u) == 1u) {
            return textureSample(t_diffuse_linear, s_diffuse_linear, tex_coords / 512.0);
        } else {
            return textureSample(t_diffuse_nearest, s_diffuse_nearest, tex_coords / 512.0);
        }
    } else {
        return in.color;
    }
}

fn texel1_color(in: VertexOutput) -> vec4<f32> {
    let texmode_s  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_S1_SHIFT, 2u);
    let texmode_t  = extractBits(in.flags, VERTEX_FLAG_TEXMODE_T1_SHIFT, 2u);
    let mask_s     = f32(extractBits(in.maskshift1, 24u, 8u)); // 0xMSssMTst // mask_s shift_s mask_t shift_t
    let shift_s    = f32(extractBits(in.maskshift1, 16u, 8u));
    let mask_t     = f32(extractBits(in.maskshift1,  8u, 8u));
    let shift_t    = f32(extractBits(in.maskshift1,  0u, 8u));

    let tex_coords = vec2(texcoord(shift(in.tex_coords1.x, shift_s), in.tex_params1.x, in.tex_params1.y, mask_s, texmode_s),
                          texcoord(shift(in.tex_coords1.y, shift_t), in.tex_params1.z, in.tex_params1.w, mask_t, texmode_t));

    if (extractBits(in.flags, VERTEX_FLAG_LINEAR_FILTER_SHIFT, 1u) == 1u) {
        return textureSample(t_diffuse_linear1, s_diffuse_linear1, tex_coords / 512.0);
    } else {
        return textureSample(t_diffuse_nearest1, s_diffuse_nearest1, tex_coords / 512.0);
    }
}


fn shade(in: VertexOutput) -> vec4<f32> {
    if (extractBits(in.flags, VERTEX_FLAG_LIT_SHIFT, 1u) == 0u) {
        return in.color;
    } else {
        // perform lighting with the ambient from in.color
        var color = in.color.rgb;
        let position = in.view_position.xyz / in.view_position.w;
        for (var i: i32 = 0; i < 7; i++) {
            if (light_state.lights[i].w != 0.0) {
                let neg_light_direction = normalize(light_state.lights[i].xyz - position);
                let contrib = max(dot(in.view_normal, neg_light_direction), 0.0);
                color += light_state.colors[i].rgb * contrib;
            }
        }
        return vec4(color, in.color.a);
    }
}

fn select_color(letter: u32, src: u32, cc: vec4<f32>, rc: vec4<f32>, rc1: vec4<f32>, shade: vec4<f32>, in: VertexOutput) -> vec3<f32> {
    switch(src) {
        case 0u: { // CC first cycle output, unknown behaviour in first cycle
            return cc.rgb;
        }

        case 1u: { // TEXEL0 color
            return rc.rgb;
        }

        case 2u: { // TEXEL1 color
            return rc1.rgb;
        }

        case 3u: { // PRIM color
            return color_combiner_state.prim_color.rgb;
        }

        case 4u: { // Vertex color
            return shade.rgb;
        }

        case 5u: { // ENV color
            return color_combiner_state.env_color.rgb;
        }
        
        case 6u: {
            switch(letter) {
                case 0u, 3u: { // ONE
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

        case 8u: {
            switch(letter) {
                case 2u: { return rc.aaa; } // TEXEL0_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); } // ZERO
            }
        }

        case 9u: {
            switch(letter) {
                case 2u: { return rc1.aaa; } // TEXEL1_ALPHA
                default: { return vec3(0.0, 0.0, 0.0); } // ZERO
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
                case 2u: { return vec3(color_combiner_state.lodfrac, color_combiner_state.lodfrac, color_combiner_state.lodfrac); } // PRIM_LOD_FRACTION
                default: { return vec3(0.0, 0.0, 0.0); }
            }
        }

        default: {
            return vec3(0.0, 0.0, 0.0);
        }
    }
}

fn select_alpha(letter: u32, src: u32, cc: f32, rc: f32, rc1: f32, shade: f32, in: VertexOutput) -> f32 {
    switch(src) {
        case 0u: { 
            switch(letter) {
                case 0u, 1u, 3u: { // CC alpha from previous cycle
                    return cc;
                }
                // TODO LOD_FRAC
                default: {
                    return 0.0;
                }
            }
        }

        case 1u: { // TEXEL0 alpha
            return rc;
        }
        
        case 2u: { // TEXEL1 alpha
            return rc1;
        }

        case 3u: { // PRIM alpha
            return color_combiner_state.prim_color.a;
        }

        case 4u: { // Vertex alpha
            return shade;
        }

        case 5u: { // ENV alpha
            return color_combiner_state.env_color.a;
        }

        case 6u: {
            switch(letter) {
                case 0u, 1u, 3u: { // ONE
                    return 1.0;
                }
                // TODO PRIM_LOD_FRAC
                default: {
                    return color_combiner_state.lodfrac;
                }
            }
        }

        default: { // ZERO (case 7u)
            return 0.0;
        }
    }
}

// (A-B)*C+D
fn color_combine(rc: vec4<f32>, rc1: vec4<f32>, in: VertexOutput) -> vec4<f32> {
    let shade_color = shade(in);

    let a0c_source = extractBits(color_combiner_state.color1_source, 24u, 8u);
    let b0c_source = extractBits(color_combiner_state.color1_source, 16u, 8u);
    let c0c_source = extractBits(color_combiner_state.color1_source,  8u, 8u);
    let d0c_source = extractBits(color_combiner_state.color1_source,  0u, 8u);

    let a0a_source = extractBits(color_combiner_state.alpha1_source, 24u, 8u);
    let b0a_source = extractBits(color_combiner_state.alpha1_source, 16u, 8u);
    let c0a_source = extractBits(color_combiner_state.alpha1_source,  8u, 8u);
    let d0a_source = extractBits(color_combiner_state.alpha1_source,  0u, 8u);

    // If combined color is used in the first cycle, you'll get a light slate blue color. Might be useful for debugging.
    let cc_in = vec4(0.5, 0.5, 1.0, 1.0);
    let a = vec4(select_color(0u, a0c_source, cc_in, rc, rc1, shade_color, in), select_alpha(0u, a0a_source, cc_in.a, rc.a, rc1.a, shade_color.a, in));
    let b = vec4(select_color(1u, b0c_source, cc_in, rc, rc1, shade_color, in), select_alpha(1u, b0a_source, cc_in.a, rc.a, rc1.a, shade_color.a, in));
    let c = vec4(select_color(2u, c0c_source, cc_in, rc, rc1, shade_color, in), select_alpha(2u, c0a_source, cc_in.a, rc.a, rc1.a, shade_color.a, in));
    let d = vec4(select_color(3u, d0c_source, cc_in, rc, rc1, shade_color, in), select_alpha(3u, d0a_source, cc_in.a, rc.a, rc1.a, shade_color.a, in));

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

        let a2 = vec4(select_color(0u, a1c_source, cc_out, rc, rc1, shade_color, in), select_alpha(0u, a1a_source, cc_out.a, rc.a, rc1.a, shade_color.a, in));
        let b2 = vec4(select_color(1u, b1c_source, cc_out, rc, rc1, shade_color, in), select_alpha(1u, b1a_source, cc_out.a, rc.a, rc1.a, shade_color.a, in));
        let c2 = vec4(select_color(2u, c1c_source, cc_out, rc, rc1, shade_color, in), select_alpha(2u, c1a_source, cc_out.a, rc.a, rc1.a, shade_color.a, in));
        let d2 = vec4(select_color(3u, d1c_source, cc_out, rc, rc1, shade_color, in), select_alpha(3u, d1a_source, cc_out.a, rc.a, rc1.a, shade_color.a, in));

        return (a2 - b2) * c2 + d2;
    }
}
