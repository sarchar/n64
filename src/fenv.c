#include <fenv.h>
#include <math.h>
#include <stdint.h>
#include <xmmintrin.h>

int32_t c_fe_upward     = FE_UPWARD;
int32_t c_fe_downward   = FE_DOWNWARD;
int32_t c_fe_tonearest  = FE_TONEAREST;
int32_t c_fe_towardzero = FE_TOWARDZERO;

int32_t c_fe_divbyzero  = FE_DIVBYZERO;
int32_t c_fe_inexact    = FE_INEXACT;
int32_t c_fe_invalid    = FE_INVALID;
int32_t c_fe_overflow   = FE_OVERFLOW;
int32_t c_fe_underflow  = FE_UNDERFLOW;
int32_t c_fe_all_except = FE_ALL_EXCEPT;

extern uint32_t c_fesetround(int32_t round) {
    return fesetround(round);
}

extern int32_t c_fegetround() {
    return fegetround();
}

extern int32_t c_feclearexcept(int32_t excepts) {
    return feclearexcept(excepts);
}

extern int32_t c_fetestexcept(int32_t excepts) {
    return fetestexcept(excepts);
}

// these floating point operations are in C so that fetestexcept produces
// correct results it seems that sometimes the rust floating point ops set the
// exception flags, but I think rusts optimizer produces code that doesn't
// always set exception flags. better to be consistent here until I know how to
// get floating point exceptions in Rust
extern float  c_f32_add(float a , float b)  { return a + b; }
extern double c_f64_add(double a, double b) { return a + b; }
extern float  c_f32_sub(float a , float b)  { return a - b; }
extern double c_f64_sub(double a, double b) { return a - b; }
extern float  c_f32_mul(float a , float b)  { return a * b; }
extern double c_f64_mul(double a, double b) { return a * b; }
extern float  c_f32_div(float a , float b)  { return a / b; }
extern double c_f64_div(double a, double b) { return a / b; }

// these rint*() functions use the fpu rounding mode, which does not seem availble in rust

// the compiler seems to incorrectly optimize the call to rint, making the rounding mode incorrect
// this inline assembly seems to prevent that optimization
extern double c_rint_f64(double a) { 
#if defined(__GNUC__) || defined(__clang__)
    asm __volatile__(
            "movsd %1,%%xmm0\n\t"
            "call rint\n\t"
            "movsd %%xmm0,%0\n\t"
            : "=m"(a) : "m"(a) : "xmm0", "memory");
#else
    // MSVC seems to work for now. Might need to fix in the future
    a = rint(a);
#endif
    return a;
}

