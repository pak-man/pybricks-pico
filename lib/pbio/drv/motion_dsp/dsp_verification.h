// SPDX-License-Identifier: MIT
// DSP instruction verification for RP2350

#ifndef _DSP_VERIFICATION_H_
#define _DSP_VERIFICATION_H_

#include <stdint.h>

typedef struct {
    uint32_t scalar_cycles;
    uint32_t dsp_cycles;
    float speedup;
} dsp_benchmark_result_t;

// Verify DSP instructions are enabled and working
void dsp_verify_instructions(void);

// Benchmark multiply-accumulate operations
dsp_benchmark_result_t dsp_benchmark_multiply_accumulate(uint32_t iterations);

#endif // _DSP_VERIFICATION_H_