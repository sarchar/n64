use cfg_if::cfg_if;

#[cfg(target_arch="x86_64")]
use core::arch::x86_64::*;

use core::simd::Simd;

#[cfg(target_feature="avx512f")]
#[allow(non_camel_case_types)]
pub type __wm512i = __m512i;

#[cfg(not(target_feature="avx512f"))]
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
pub struct __wm512i([i64; 8]);

#[inline(always)]
pub fn _wmm512_setzero_si512() -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_setzero_si512() }
        } else {
            __wm512i([0i64; 8])
        }
    }
}

#[inline(always)]
pub fn _wmm512_set1_epi64(a: i64) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_set1_epi64(a) }
        } else {
            __wm512i([a; 8])
        }
    }
}

#[inline(always)]
pub fn _wmm512_srai_epi64(a: __wm512i, imm8: u32) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_srai_epi64(a, imm8) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] >> imm8; 
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_srli_epi64(a: __wm512i, imm8: u32) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_srli_epi64(a, imm8) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = ((a.0[i] as u64) >> imm8) as i64; 
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_slli_epi64(a: __wm512i, imm8: u32) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_slli_epi64(a, imm8) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] << imm8; 
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_add_epi64(a: __wm512i, b: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_add_epi64(a, b) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i].wrapping_add(b.0[i]);
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_and_si256(a: __wm512i, b: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_and_si512(a, b) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] & b.0[i];
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_and_epi64(a: __wm512i, b: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_and_epi64(a, b) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] & b.0[i];
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_or_epi64(a: __wm512i, b: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_or_epi64(a, b) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] | b.0[i];
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_cmpge_epi64_mask(a: __wm512i, b: __wm512i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cmpge_epi64_mask(a, b) }
        } else {
            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a.0[i] >= b.0[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_cmplt_epi64_mask(a: __wm512i, b: __wm512i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cmplt_epi64_mask(a, b) }
        } else {
            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a.0[i] < b.0[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_cmpeq_epi32_mask(a: __m256i, b: __m256i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cmpeq_epi32_mask(a, b) }
        } else {
            let a = Simd::<i32, 8>::from(a);
            let b = Simd::<i32, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] == b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_cmpge_epi32_mask(a: __m256i, b: __m256i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cmpge_epi32_mask(a, b) }
        } else {
            let a = Simd::<i32, 8>::from(a);
            let b = Simd::<i32, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] >= b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_cmple_epi32_mask(a: __m256i, b: __m256i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cmple_epi32_mask(a, b) }
        } else {
            let a = Simd::<i32, 8>::from(a);
            let b = Simd::<i32, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] <= b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_cmplt_epi32_mask(a: __m256i, b: __m256i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cmplt_epi32_mask(a, b) }
        } else {
            let a = Simd::<i32, 8>::from(a);
            let b = Simd::<i32, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] < b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}


#[inline(always)]
pub fn _wmm_cmpge_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmpge_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] >= b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm_cmpge_epu16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmpge_epu16_mask(a, b) }
        } else {
            let a = Simd::<u16, 8>::from(a);
            let b = Simd::<u16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] >= b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}


#[inline(always)]
pub fn _wmm_cmpgt_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmpgt_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] > b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm_cmple_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmple_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] <= b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm_cmplt_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmplt_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] < b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm_cmpeq_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmpeq_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] == b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm_cmpneq_epi16_mask(a: __m128i, b: __m128i) -> __mmask8 {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_cmpneq_epi16_mask(a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();

            let mut r: __mmask8 = 0;
            for i in 0..8 { 
                if a_lanes[i] != b_lanes[i] {
                    r |= 1 << i;
                }
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_mask_blend_epi64(mask: __mmask8, a: __wm512i, b: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_mask_blend_epi64(mask, a, b) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = if (mask & (1 << i)) == 0 { a.0[i] } else { b.0[i] };
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_mask_blend_epi32(mask: __mmask8, a: __m256i, b: __m256i) -> __m256i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_mask_blend_epi32(mask, a, b) }
        } else {
            let a = Simd::<i32, 8>::from(a);
            let b = Simd::<i32, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();
            let blend: [i32; 8] = std::array::from_fn(|i| if (mask & (1 << i)) == 0 { a_lanes[i] } else { b_lanes[i] });
            unsafe {
                _mm256_set_epi32(blend[7], blend[6], blend[5], blend[4], blend[3], blend[2], blend[1], blend[0])
            }
        }
    }
}


#[inline(always)]
pub fn _wmm_mask_blend_epi16(mask: __mmask8, a: __m128i, b: __m128i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_mask_blend_epi16(mask, a, b) }
        } else {
            let a = Simd::<i16, 8>::from(a);
            let b = Simd::<i16, 8>::from(b);
            let a_lanes = a.as_array();
            let b_lanes = b.as_array();
            let blend: [i16; 8] = std::array::from_fn(|i| if (mask & (1 << i)) == 0 { a_lanes[i] } else { b_lanes[i] });
            unsafe {
                _mm_set_epi16(blend[7], blend[6], blend[5], blend[4], blend[3], blend[2], blend[1], blend[0])
            }
        }
    }
}


#[inline(always)]
pub fn _wmm512_cvtepi32_epi64(v: __m256i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepi32_epi64(v) }
        } else {
            let binding = Simd::<i32, 8>::from(v);
            let lanes = binding.as_array();
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = lanes[i] as i64;
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_cvtepi64_epi16(v: __wm512i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepi64_epi16(v) }
        } else {
            unsafe {
                _mm_set_epi16(v.0[7] as i16, v.0[6] as i16, v.0[5] as i16, v.0[4] as i16,
                              v.0[3] as i16, v.0[2] as i16, v.0[1] as i16, v.0[0] as i16)
            }
        }
    }
}

#[inline(always)]
pub fn _wmm512_cvtepi64_epi32(v: __wm512i) -> __m256i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepi64_epi32(v) }
        } else {
            unsafe {
                _mm256_set_epi32(v.0[7] as i32, v.0[6] as i32, v.0[5] as i32, v.0[4] as i32,
                                 v.0[3] as i32, v.0[2] as i32, v.0[1] as i32, v.0[0] as i32)
            }
        }
    }
}

#[inline(always)]
pub fn _wmm512_cvtepu16_epi64(v: __m128i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepu16_epi64(v) }
        } else {
            let binding = Simd::<u16, 8>::from(v);
            let lanes = binding.as_array();
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 {
                r.0[i] = lanes[i] as i64;
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_cvtepu32_epi64(v: __m256i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepu32_epi64(v) }
        } else {
            //let mut storage = [0u32; 8];
            //unsafe {
            //    _mm256_store_si256(storage.as_mut_ptr() as *mut __m256i, v);
            //}
            let binding = Simd::<u32, 8>::from(v);
            let lanes = binding.as_array();
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 {
                r.0[i] = lanes[i] as i64;
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm256_cvtepi32_epi16(v: __m256i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cvtepi32_epi16(v) }
        } else {
            let binding = Simd::<i32, 8>::from(v);
            let lanes = binding.as_array();
            unsafe {
                _mm_set_epi16(lanes[7] as i16, lanes[6] as i16, lanes[5] as i16, lanes[4] as i16, 
                              lanes[3] as i16, lanes[2] as i16, lanes[1] as i16, lanes[0] as i16)
            }
        }
    }
}

#[inline(always)]
fn saturate16_i32(v: i32) -> i16 {
    if v < -32768 { 0x8000u16 as i16 } else if v > 32767 { 0x7FFFi16 } else { v as i16 }
}

#[inline(always)]
pub fn _wmm256_cvtsepi32_epi16(v: __m256i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_cvtsepi32_epi16(v) }
        } else {
            let binding = Simd::<i32, 8>::from(v);
            let lanes = binding.as_array();
            unsafe {
                _mm_set_epi16(saturate16_i32(lanes[7]), saturate16_i32(lanes[6]), saturate16_i32(lanes[5]), saturate16_i32(lanes[4]), 
                              saturate16_i32(lanes[3]), saturate16_i32(lanes[2]), saturate16_i32(lanes[1]), saturate16_i32(lanes[0]))
            }
        }
    }
}

