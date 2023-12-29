use cfg_if::cfg_if;

#[cfg(target_arch="x86_64")]
use core::arch::x86_64::*;

#[cfg(target_feature="avx512f")]
#[allow(non_camel_case_types)]
pub type __wm512i = __m512i;

#[cfg(not(target_feature="avx512f"))]
#[allow(non_camel_case_types)]
pub struct __wm512i(__m256i, __m256i);

#[inline(always)]
pub fn _wmm512_setzero_si512() -> __wm512i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_setzero_si512() }
        } else {
            unsafe {
                __wm512i(_mm256_setzero_si256(), _mm256_setzero_si256())
            }
        }
    }
}

#[inline(always)]
#[cfg(not(target_feature="avx512f"))]
pub fn _wmm512_to_array(v: __wm512i) -> [i64; 8] {
    let mut arr = [0i64; 8];
    unsafe {
        _mm256_store_si256(arr.as_mut_ptr() as *mut __m256i, v.0);
        _mm256_store_si256((&arr[4..]).as_mut_ptr() as *mut __m256i, v.1);
    }
    arr
}

#[inline(always)]
pub fn _wmm512_cvtepi64_epi16(v: __wm512i) -> __m128i {
    cfg_if! {
        if #[cfg(target_feature="avx512f")] {
            unsafe { _mm512_cvtepi64_epi16(v) }
        } else {
            let mut arr = _wmm512_to_array(v);
            unsafe {
                todo!();
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
            let mut arr = _wmm512_to_array(v);
            unsafe {
                todo!();
            }
        }
    }
}
