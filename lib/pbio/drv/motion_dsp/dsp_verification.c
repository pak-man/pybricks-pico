// SPDX-License-Identifier: MIT
// DSP instruction verification for RP2350
// Confirms that DSP extensions are actually compiled and working

#include <stdio.h>
#include "dsp_verification.h"
#ifdef PICO_2W

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

// Test if DSP instructions are available by using inline assembly
static bool test_dsp_hardware(void) {
    // Try to execute SADD16 (SIMD add for two 16-bit values)
    // This will fault if DSP extensions aren't enabled
    volatile int32_t a = 0x00010002; // Two 16-bit values: 1, 2
    volatile int32_t b = 0x00030004; // Two 16-bit values: 3, 4
    volatile int32_t result;
    
    __asm__ volatile (
        "sadd16 %0, %1, %2"
        : "=r" (result)
        : "r" (a), "r" (b)
    );
    
    // Result should be 0x00040006 (1+3=4, 2+4=6)
    return (result == 0x00040006);
}

// Simple benchmark: scalar vs DSP multiply-accumulate
static void benchmark_mac(uint32_t iterations, uint32_t *scalar_cycles, uint32_t *dsp_cycles) {
    const int N = 64;
    int16_t vec_a[N];
    int16_t vec_b[N];
    
    // Initialize test vectors
    for (int i = 0; i < N; i++) {
        vec_a[i] = i;
        vec_b[i] = N - i;
    }
    
    // Scalar implementation
    uint32_t start = time_us_32();
    for (uint32_t iter = 0; iter < iterations; iter++) {
        int32_t acc = 0;
        for (int i = 0; i < N; i++) {
            acc += (int32_t)vec_a[i] * vec_b[i];
        }
        // Prevent optimization
        __asm__ volatile ("" : : "r" (acc));
    }
    *scalar_cycles = time_us_32() - start;
    
    // DSP implementation using SMLAD (signed multiply accumulate dual)
    start = time_us_32();
    for (uint32_t iter = 0; iter < iterations; iter++) {
        int32_t acc = 0;
        // Process 2 elements at a time using SIMD
        for (int i = 0; i < N; i += 2) {
            int32_t packed_a = *((int32_t*)&vec_a[i]);
            int32_t packed_b = *((int32_t*)&vec_b[i]);
            
            __asm__ volatile (
                "smlad %0, %1, %2, %0"
                : "+r" (acc)
                : "r" (packed_a), "r" (packed_b)
            );
        }
        __asm__ volatile ("" : : "r" (acc));
    }
    *dsp_cycles = time_us_32() - start;
}

// Public API
void dsp_verify_instructions(void) {
    printf("\n=== RP2350 DSP Verification ===\n");
    
    // Check compiler flags
    #ifdef ARM_MATH_CM33
    printf("✓ ARM_MATH_CM33 defined\n");
    #else
    printf("✗ ARM_MATH_CM33 NOT defined - DSP may not be enabled!\n");
    #endif
    
    #ifdef ARM_MATH_DSP
    printf("✓ ARM_MATH_DSP defined\n");
    #else
    printf("✗ ARM_MATH_DSP NOT defined - DSP may not be enabled!\n");
    #endif
    
    #ifdef __FPU_PRESENT
    printf("✓ __FPU_PRESENT=%d\n", __FPU_PRESENT);
    #else
    printf("✗ __FPU_PRESENT NOT defined\n");
    #endif
    
    // Test hardware
    printf("\nTesting DSP hardware instructions...\n");
    if (test_dsp_hardware()) {
        printf("✓ DSP SIMD instructions working (SADD16 test passed)\n");
    } else {
        printf("✗ DSP SIMD instructions FAILED\n");
        return;
    }
    
    // Benchmark
    printf("\nBenchmarking MAC operations...\n");
    uint32_t scalar_us, dsp_us;
    benchmark_mac(1000, &scalar_us, &dsp_us);
    
    printf("  Scalar: %lu µs\n", scalar_us);
    printf("  DSP:    %lu µs\n", dsp_us);
    
    if (dsp_us > 0) {
        float speedup = (float)scalar_us / dsp_us;
        printf("  Speedup: %.2fx\n", speedup);
        
        if (speedup > 1.5f) {
            printf("✓ DSP extensions providing significant acceleration\n");
        } else {
            printf("⚠ Speedup lower than expected - check compiler flags\n");
        }
    }
    
    printf("\n");
}

dsp_benchmark_result_t dsp_benchmark_multiply_accumulate(uint32_t iterations) {
    dsp_benchmark_result_t result = {0};
    benchmark_mac(iterations, &result.scalar_cycles, &result.dsp_cycles);
    
    if (result.dsp_cycles > 0) {
        result.speedup = (float)result.scalar_cycles / result.dsp_cycles;
    }
    
    return result;
}

#else
// Pico W stub - DSP not available on RP2040
void dsp_verify_instructions(void) {
    printf("DSP verification: Not available on RP2040 (Pico W)\n");
}

dsp_benchmark_result_t dsp_benchmark_multiply_accumulate(uint32_t iterations) {
    dsp_benchmark_result_t result = {0};
    printf("DSP benchmark: Not available on RP2040\n");
    return result;
}
#endif // PICO_2W