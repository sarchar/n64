#include <fenv.h>
#include <stdint.h>

int32_t c_fe_upward     = FE_UPWARD;
int32_t c_fe_downward   = FE_DOWNWARD;
int32_t c_fe_tonearest  = FE_TONEAREST;
int32_t c_fe_towardzero = FE_TOWARDZERO;

int32_t c_fe_divbyzero  = FE_DIVBYZERO;
int32_t c_fe_inexact    = FE_INEXACT;
int32_t c_fe_invalid    = FE_INVALID;
int32_t c_fe_overflow   = FE_OVERFLOW;
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

extern float c_f32_add(float a, float b) {
    return a + b;
}
