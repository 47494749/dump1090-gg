#ifndef DUMP1090_CPU_H
#define DUMP1090_CPU_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// x86
int32_t cpu_supports_avx(void);
int32_t cpu_supports_avx2(void);

// ARM
int32_t cpu_supports_armv7_neon_vfpv4(void);

// AARCH64
int32_t cpu_supports_armv8_simd(void);
int32_t cpu_supports_armv8_simd_sve(void);

#ifdef __cplusplus
}
#endif

#endif
