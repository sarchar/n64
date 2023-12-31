use cfg_if::cfg_if;

#[cfg(target_arch="x86_64")]
use core::arch::x86_64::*;

#[cfg(not(target_feature="avx512f"))]
use core::simd::Simd;

#[cfg(target_feature="avx512f")]
#[allow(non_camel_case_types)]
pub type __wm512i = __m512i;

#[cfg(not(target_feature="avx512f"))]
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
pub struct __wm512i([i64; 8]);

// This table simply takes the index i and simply doubles/duplicates the bits: 0xCA becomes 0xF0CC
#[cfg(not(target_feature="avx512f"))]
const MASK8_TO_MM128_EPI8_TABLE: [u16; 256] = [
    0x0000, 0x0003, 0x000C, 0x000F, 0x0030, 0x0033, 0x003C, 0x003F, 0x00C0, 0x00C3, 0x00CC, 0x00CF, 0x00F0, 0x00F3, 0x00FC, 0x00FF,
    0x0300, 0x0303, 0x030C, 0x030F, 0x0330, 0x0333, 0x033C, 0x033F, 0x03C0, 0x03C3, 0x03CC, 0x03CF, 0x03F0, 0x03F3, 0x03FC, 0x03FF,
    0x0C00, 0x0C03, 0x0C0C, 0x0C0F, 0x0C30, 0x0C33, 0x0C3C, 0x0C3F, 0x0CC0, 0x0CC3, 0x0CCC, 0x0CCF, 0x0CF0, 0x0CF3, 0x0CFC, 0x0CFF,
    0x0F00, 0x0F03, 0x0F0C, 0x0F0F, 0x0F30, 0x0F33, 0x0F3C, 0x0F3F, 0x0FC0, 0x0FC3, 0x0FCC, 0x0FCF, 0x0FF0, 0x0FF3, 0x0FFC, 0x0FFF,
    0x3000, 0x3003, 0x300C, 0x300F, 0x3030, 0x3033, 0x303C, 0x303F, 0x30C0, 0x30C3, 0x30CC, 0x30CF, 0x30F0, 0x30F3, 0x30FC, 0x30FF,
    0x3300, 0x3303, 0x330C, 0x330F, 0x3330, 0x3333, 0x333C, 0x333F, 0x33C0, 0x33C3, 0x33CC, 0x33CF, 0x33F0, 0x33F3, 0x33FC, 0x33FF,
    0x3C00, 0x3C03, 0x3C0C, 0x3C0F, 0x3C30, 0x3C33, 0x3C3C, 0x3C3F, 0x3CC0, 0x3CC3, 0x3CCC, 0x3CCF, 0x3CF0, 0x3CF3, 0x3CFC, 0x3CFF,
    0x3F00, 0x3F03, 0x3F0C, 0x3F0F, 0x3F30, 0x3F33, 0x3F3C, 0x3F3F, 0x3FC0, 0x3FC3, 0x3FCC, 0x3FCF, 0x3FF0, 0x3FF3, 0x3FFC, 0x3FFF,
    0xC000, 0xC003, 0xC00C, 0xC00F, 0xC030, 0xC033, 0xC03C, 0xC03F, 0xC0C0, 0xC0C3, 0xC0CC, 0xC0CF, 0xC0F0, 0xC0F3, 0xC0FC, 0xC0FF,
    0xC300, 0xC303, 0xC30C, 0xC30F, 0xC330, 0xC333, 0xC33C, 0xC33F, 0xC3C0, 0xC3C3, 0xC3CC, 0xC3CF, 0xC3F0, 0xC3F3, 0xC3FC, 0xC3FF,
    0xCC00, 0xCC03, 0xCC0C, 0xCC0F, 0xCC30, 0xCC33, 0xCC3C, 0xCC3F, 0xCCC0, 0xCCC3, 0xCCCC, 0xCCCF, 0xCCF0, 0xCCF3, 0xCCFC, 0xCCFF,
    0xCF00, 0xCF03, 0xCF0C, 0xCF0F, 0xCF30, 0xCF33, 0xCF3C, 0xCF3F, 0xCFC0, 0xCFC3, 0xCFCC, 0xCFCF, 0xCFF0, 0xCFF3, 0xCFFC, 0xCFFF,
    0xF000, 0xF003, 0xF00C, 0xF00F, 0xF030, 0xF033, 0xF03C, 0xF03F, 0xF0C0, 0xF0C3, 0xF0CC, 0xF0CF, 0xF0F0, 0xF0F3, 0xF0FC, 0xF0FF,
    0xF300, 0xF303, 0xF30C, 0xF30F, 0xF330, 0xF333, 0xF33C, 0xF33F, 0xF3C0, 0xF3C3, 0xF3CC, 0xF3CF, 0xF3F0, 0xF3F3, 0xF3FC, 0xF3FF,
    0xFC00, 0xFC03, 0xFC0C, 0xFC0F, 0xFC30, 0xFC33, 0xFC3C, 0xFC3F, 0xFCC0, 0xFCC3, 0xFCCC, 0xFCCF, 0xFCF0, 0xFCF3, 0xFCFC, 0xFCFF,
    0xFF00, 0xFF03, 0xFF0C, 0xFF0F, 0xFF30, 0xFF33, 0xFF3C, 0xFF3F, 0xFFC0, 0xFFC3, 0xFFCC, 0xFFCF, 0xFFF0, 0xFFF3, 0xFFFC, 0xFFFF
];

// Quadruple bits in a bitmask, so 0xCA becomes 0xFF00F0F0
#[cfg(not(target_feature="avx512f"))]
const MASK8_TO_MM256_EPI8_TABLE: [u32; 256] = [
    0x00000000, 0x0000000F, 0x000000F0, 0x000000FF, 0x00000F00, 0x00000F0F, 0x00000FF0, 0x00000FFF,
    0x0000F000, 0x0000F00F, 0x0000F0F0, 0x0000F0FF, 0x0000FF00, 0x0000FF0F, 0x0000FFF0, 0x0000FFFF,
    0x000F0000, 0x000F000F, 0x000F00F0, 0x000F00FF, 0x000F0F00, 0x000F0F0F, 0x000F0FF0, 0x000F0FFF,
    0x000FF000, 0x000FF00F, 0x000FF0F0, 0x000FF0FF, 0x000FFF00, 0x000FFF0F, 0x000FFFF0, 0x000FFFFF,
    0x00F00000, 0x00F0000F, 0x00F000F0, 0x00F000FF, 0x00F00F00, 0x00F00F0F, 0x00F00FF0, 0x00F00FFF,
    0x00F0F000, 0x00F0F00F, 0x00F0F0F0, 0x00F0F0FF, 0x00F0FF00, 0x00F0FF0F, 0x00F0FFF0, 0x00F0FFFF,
    0x00FF0000, 0x00FF000F, 0x00FF00F0, 0x00FF00FF, 0x00FF0F00, 0x00FF0F0F, 0x00FF0FF0, 0x00FF0FFF,
    0x00FFF000, 0x00FFF00F, 0x00FFF0F0, 0x00FFF0FF, 0x00FFFF00, 0x00FFFF0F, 0x00FFFFF0, 0x00FFFFFF,
    0x0F000000, 0x0F00000F, 0x0F0000F0, 0x0F0000FF, 0x0F000F00, 0x0F000F0F, 0x0F000FF0, 0x0F000FFF,
    0x0F00F000, 0x0F00F00F, 0x0F00F0F0, 0x0F00F0FF, 0x0F00FF00, 0x0F00FF0F, 0x0F00FFF0, 0x0F00FFFF,
    0x0F0F0000, 0x0F0F000F, 0x0F0F00F0, 0x0F0F00FF, 0x0F0F0F00, 0x0F0F0F0F, 0x0F0F0FF0, 0x0F0F0FFF,
    0x0F0FF000, 0x0F0FF00F, 0x0F0FF0F0, 0x0F0FF0FF, 0x0F0FFF00, 0x0F0FFF0F, 0x0F0FFFF0, 0x0F0FFFFF,
    0x0FF00000, 0x0FF0000F, 0x0FF000F0, 0x0FF000FF, 0x0FF00F00, 0x0FF00F0F, 0x0FF00FF0, 0x0FF00FFF,
    0x0FF0F000, 0x0FF0F00F, 0x0FF0F0F0, 0x0FF0F0FF, 0x0FF0FF00, 0x0FF0FF0F, 0x0FF0FFF0, 0x0FF0FFFF,
    0x0FFF0000, 0x0FFF000F, 0x0FFF00F0, 0x0FFF00FF, 0x0FFF0F00, 0x0FFF0F0F, 0x0FFF0FF0, 0x0FFF0FFF,
    0x0FFFF000, 0x0FFFF00F, 0x0FFFF0F0, 0x0FFFF0FF, 0x0FFFFF00, 0x0FFFFF0F, 0x0FFFFFF0, 0x0FFFFFFF,
    0xF0000000, 0xF000000F, 0xF00000F0, 0xF00000FF, 0xF0000F00, 0xF0000F0F, 0xF0000FF0, 0xF0000FFF,
    0xF000F000, 0xF000F00F, 0xF000F0F0, 0xF000F0FF, 0xF000FF00, 0xF000FF0F, 0xF000FFF0, 0xF000FFFF,
    0xF00F0000, 0xF00F000F, 0xF00F00F0, 0xF00F00FF, 0xF00F0F00, 0xF00F0F0F, 0xF00F0FF0, 0xF00F0FFF,
    0xF00FF000, 0xF00FF00F, 0xF00FF0F0, 0xF00FF0FF, 0xF00FFF00, 0xF00FFF0F, 0xF00FFFF0, 0xF00FFFFF,
    0xF0F00000, 0xF0F0000F, 0xF0F000F0, 0xF0F000FF, 0xF0F00F00, 0xF0F00F0F, 0xF0F00FF0, 0xF0F00FFF,
    0xF0F0F000, 0xF0F0F00F, 0xF0F0F0F0, 0xF0F0F0FF, 0xF0F0FF00, 0xF0F0FF0F, 0xF0F0FFF0, 0xF0F0FFFF,
    0xF0FF0000, 0xF0FF000F, 0xF0FF00F0, 0xF0FF00FF, 0xF0FF0F00, 0xF0FF0F0F, 0xF0FF0FF0, 0xF0FF0FFF,
    0xF0FFF000, 0xF0FFF00F, 0xF0FFF0F0, 0xF0FFF0FF, 0xF0FFFF00, 0xF0FFFF0F, 0xF0FFFFF0, 0xF0FFFFFF,
    0xFF000000, 0xFF00000F, 0xFF0000F0, 0xFF0000FF, 0xFF000F00, 0xFF000F0F, 0xFF000FF0, 0xFF000FFF,
    0xFF00F000, 0xFF00F00F, 0xFF00F0F0, 0xFF00F0FF, 0xFF00FF00, 0xFF00FF0F, 0xFF00FFF0, 0xFF00FFFF,
    0xFF0F0000, 0xFF0F000F, 0xFF0F00F0, 0xFF0F00FF, 0xFF0F0F00, 0xFF0F0F0F, 0xFF0F0FF0, 0xFF0F0FFF,
    0xFF0FF000, 0xFF0FF00F, 0xFF0FF0F0, 0xFF0FF0FF, 0xFF0FFF00, 0xFF0FFF0F, 0xFF0FFFF0, 0xFF0FFFFF,
    0xFFF00000, 0xFFF0000F, 0xFFF000F0, 0xFFF000FF, 0xFFF00F00, 0xFFF00F0F, 0xFFF00FF0, 0xFFF00FFF,
    0xFFF0F000, 0xFFF0F00F, 0xFFF0F0F0, 0xFFF0F0FF, 0xFFF0FF00, 0xFFF0FF0F, 0xFFF0FFF0, 0xFFF0FFFF,
    0xFFFF0000, 0xFFFF000F, 0xFFFF00F0, 0xFFFF00FF, 0xFFFF0F00, 0xFFFF0F0F, 0xFFFF0FF0, 0xFFFF0FFF,
    0xFFFFF000, 0xFFFFF00F, 0xFFFFF0F0, 0xFFFFF0FF, 0xFFFFFF00, 0xFFFFFF0F, 0xFFFFFFF0, 0xFFFFFFFF,
];

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
pub fn _wmm512_srai_epi64<const IMM8: u32>(a: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_srai_epi64::<IMM8>(a) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] >> IMM8; 
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_srli_epi64<const IMM8: u32>(a: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_srli_epi64::<IMM8>(a) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = ((a.0[i] as u64) >> IMM8) as i64; 
            }
            r
        }
    }
}

#[inline(always)]
pub fn _wmm512_slli_epi64<const IMM8: u32>(a: __wm512i) -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_slli_epi64::<IMM8>(a) }
        } else {
            let mut r = _wmm512_setzero_si512();
            for i in 0..8 { 
                r.0[i] = a.0[i] << IMM8; 
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

// this version not exactly listed at
// https://stackoverflow.com/questions/72898737/intrinsic-inverse-to-mm-movemask-epi8
// but has been converted to take 32-bit mask and make a __m256i mask usable in _mm256_blendv_epi8
#[cfg(not(target_feature="avx512f"))]
#[inline(always)]
unsafe fn mask8_to_m256i(mask: __mmask8) -> __m256i {
    // _mm256_shuffle_epi8 shuffles each 128-bit half separately, so we need the mask in epi32 lane 0 and 2
    let mask = {
        let tmp = _mm_cvtsi32_si128(MASK8_TO_MM256_EPI8_TABLE[mask as usize] as i32);
        _mm_or_si128(tmp, _mm_slli_si128(tmp, 8)) // byte shift, not bit shift. so 64 bits left
    };

    let sel = _mm256_set1_epi64x(0x80402010_08040201u64 as i64);
        // ^ 0x8040201008040201_8040201008040201_8040201008040201_8040201008040201
    _mm256_cmpeq_epi8(
        _mm256_and_si256(
            _mm256_shuffle_epi8(
                    // say mask = 0xCA, table result = 0xFF00F0F0
                _mm256_cvtepu32_epi64(mask),
                    // ^ 0x0000000000000000_0000000000000000_0000000000000000_00000000FF00F0F0 <- (mm128i)0x00000000_00000000_00000000_FF00F0F0
                _mm256_set_epi64x(0x03030303_03030303u64 as i64, 0x02020202_02020202u64 as i64, 0x01010101_01010101u64 as i64, 0)
                    // ^ 0x0303030303030303_0202020202020202_0101010101010101_0000000000000000
              
               //    (byte 3 repeated)       byte 2            byte 1           byte 0
            ), // <- 0xFFFFFFFFFFFFFFFF_0000000000000000_F0F0F0F0F0F0F0F0_F0F0F0F0F0F0F0F0
            sel
        ), // <- 0x8040201008040201_0000000000000000_8040201000000000_8040201000000000
        sel

      // so 0xCA = 1     1         0      0         1      0         1      0
    ) // <- 0xFFFFFFFFFFFFFFFF_0000000000000000_FFFFFFFF00000000_FFFFFFFF00000000
}

#[inline(always)]
pub fn _wmm256_mask_blend_epi32(mask: __mmask8, a: __m256i, b: __m256i) -> __m256i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm256_mask_blend_epi32(mask, a, b) }
        } else {
            unsafe { _mm256_blendv_epi8(a, b, mask8_to_m256i(mask)) }
        }
    }
}

// https://stackoverflow.com/questions/72898737/intrinsic-inverse-to-mm-movemask-epi8
#[cfg(not(target_feature="avx512f"))]
#[inline(always)]
unsafe fn mask8_to_m128i(mask: __mmask8) -> __m128i {
    let sel = _mm_set1_epi64x(0x80402010_08040201u64 as i64);
        // ^ 0x80402010_08040201_80402010_08040201
    _mm_cmpeq_epi8(
        _mm_and_si128(
            _mm_shuffle_epi8(
                    // say mask = 0xCA, table result = 0xF0CC
                _mm_cvtsi32_si128(MASK8_TO_MM128_EPI8_TABLE[mask as usize] as i32),
                    // ^ 0x00000000_00000000_00000000_0000F0CC
                _mm_set_epi64x(0x01010101_01010101i64, 0)
                    // ^ 0x01010101_01010101_00000000_00000000
            ), // <- 0xF0F0F0F0_F0F0F0F0_CCCCCCCC_CCCCCCCC
            sel
        ), // <- 0x80402010_00000000_80400000_08040000
        sel
    ) // <- 0xFFFFFFFF_00000000_FFFF0000_FFFF0000 // 0xCA
}

#[inline(always)]
pub fn _wmm_mask_blend_epi16(mask: __mmask8, a: __m128i, b: __m128i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm_mask_blend_epi16(mask, a, b) }
        } else {
            unsafe { _mm_blendv_epi8(a, b, mask8_to_m128i(mask)) }
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

#[cfg(not(target_feature="avx512f"))]
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

