# Phase 2: DSP-Optimized Encoder Filters - Integration Guide

## Overview
Phase 2 implements biquad IIR filters and Kalman-based state estimation for encoder noise reduction and velocity estimation on the RP2350.

## Files Created
1. **encoder_filter_dsp.h** - Header with filter structures and API
2. **encoder_filter_dsp.c** - Implementation with DSP acceleration
3. **encoder_filter_test.c** - Comprehensive test suite

## Integration Steps

### Step 1: Copy Files to Project
```bash
cd ~/pybricks-micropython/lib/pbio/drv/motion_dsp
cp /path/to/encoder_filter_dsp.h .
cp /path/to/encoder_filter_dsp.c .
cp /path/to/encoder_filter_test.c .
```

### Step 2: Update CMakeLists.txt
Add to `lib/pbio/CMakeLists.txt` (in the PICO_2W section):

```cmake
if(IS_PICO_2W)
    target_sources(pbio PRIVATE
        ${PBIO_TOP}/drv/motion_dsp/dsp_verification.c
        ${PBIO_TOP}/drv/motion_dsp/encoder_filter_dsp.c    # ADD THIS
        ${PBIO_TOP}/drv/motion_dsp/encoder_filter_test.c   # ADD THIS
    )
endif()
```

### Step 3: Update system_test.c
Add these includes at the top:
```c
#ifdef PICO_2W
#include "dsp_verification.h"
#include "encoder_filter_dsp.h"
extern void encoder_filter_run_tests(void);
#endif
```

Add to command parser (in the if/else chain):
```c
} else if (strncmp(cmd, "FILTER TEST", 11) == 0) {
#ifdef PICO_2W
    printf("Running encoder filter test suite...\n");
    encoder_filter_run_tests();
    send_ble_response("FILTER TEST", "Complete - check serial");
#else
    send_ble_response("ERROR", "Requires Pico 2W");
#endif
```

Add to HELP text:
```c
"  FILTER TEST    - Run encoder filter tests (DSP)\n"
```

### Step 4: Build
```bash
cd ~/pybricks-micropython/bricks/picow
make clean
make pico2w
```

### Step 5: Flash and Test
1. Flash `build/pico2w/pybricks_pico2w.uf2` to Pico 2W
2. Connect via serial (115200 baud)
3. Send command: `FILTER TEST`

## Expected Test Results

### Test 1: Biquad Filter Initialization
- ✓ Coefficients calculated correctly
- ✓ DC gain near unity (32768 in Q15)

### Test 2: Kalman Filter Initialization
- ✓ Covariance matrix initialized
- ✓ Process/measurement noise configured

### Test 3: DC Response
- ✓ DC signal passes through with <5% error

### Test 4: Noise Rejection
- ✓ Noise reduction ratio >2:1
- Target: 3-5:1 for typical encoder noise

### Test 5: Step Response
- ✓ Settling time <50ms (at 1 kHz sample rate)
- ✓ No overshoot (Butterworth characteristic)

### Test 6: Velocity Estimation
- ✓ Average error <20% on sinusoidal motion
- ✓ Kalman filter provides smooth velocity estimate

### Performance Benchmark
Expected results on RP2350 @ 150 MHz:
- **Biquad filter**: ~0.5-1.0 µs per update
- **Kalman filter**: ~2-4 µs per update
- **Full pipeline**: ~3-6 µs per update
- **Budget usage**: <1% of 1 kHz control loop (target: <10%)

## DSP Acceleration Details

### ARM Cortex-M33 Instructions Used
1. **SMLABB** - Signed Multiply Accumulate Bottom-Bottom
   - Used in biquad filter for coefficient multiplication
   - Processes 16-bit x 32-bit multiply-accumulate in 1 cycle
   - Speedup: ~1.5-2x vs scalar code

### Fixed-Point Math
- **Biquad coefficients**: Q15 format (15 fractional bits)
- **Kalman state**: Q16.16 format (16 integer, 16 fractional)
- **Position**: millidegrees (mdeg), int32_t
- **Velocity**: millidegrees/second (mdeg/s), int32_t

### Filter Characteristics
- **Biquad**: 2nd-order Butterworth lowpass
  - Default: 50 Hz cutoff @ 1 kHz sample rate
  - Phase delay: ~3-5 ms
  - -3 dB point at cutoff frequency
  
- **Kalman**: Constant velocity model
  - State: [position, velocity]
  - Prediction: position += velocity * dt
  - Update: Correct with encoder measurement
  - Provides optimal noise filtering with minimal lag

## Usage Example in Motor Control

```c
#include "encoder_filter_dsp.h"

// Global filter instance
encoder_dsp_pipeline_t motor_encoder;

// Initialize (once at startup)
void motor_init(void) {
    // 50 Hz cutoff, 1 kHz sample rate (1ms control loop)
    encoder_pipeline_init(&motor_encoder, 50.0f, 1000.0f);
}

// Update in 1 kHz control loop
void motor_control_loop(void) {
    // Read raw encoder count (with overflow handling)
    int32_t raw_count = read_encoder_hw();
    
    // Filter through pipeline
    int32_t filtered_pos_mdeg = encoder_pipeline_update(&motor_encoder, raw_count);
    int32_t velocity_mdeg_s = encoder_pipeline_get_velocity(&motor_encoder);
    
    // Use filtered position and velocity in PID control
    pid_update(filtered_pos_mdeg, velocity_mdeg_s);
}
```

## Tuning Parameters

### Biquad Filter Cutoff Frequency
- **Lower cutoff (20-30 Hz)**: More smoothing, more lag
- **Higher cutoff (80-100 Hz)**: Less smoothing, less lag
- **Default (50 Hz)**: Good balance for most applications

Adjust in encoder_pipeline_init():
```c
encoder_pipeline_init(&pipeline, 30.0f, 1000.0f);  // More smoothing
```

### Kalman Filter Noise Parameters
Located in encoder_kalman_init():
```c
ek->Q_pos = 10 << 16;    // Position process noise (increase if position drifts)
ek->Q_vel = 1000 << 16;  // Velocity process noise (increase for rapid acceleration)
ek->R = 100 << 16;       // Measurement noise (increase if encoder is noisy)
```

Higher Q_vel = faster velocity tracking, more noise
Lower Q_vel = smoother velocity, more lag

### Encoder Resolution
Adjust in encoder_pipeline_update():
```c
const int32_t COUNTS_PER_REV = 8192;  // Change to match your encoder
```

## Next Steps: Phase 3

With validated encoder filtering, Phase 3 will implement:
1. **DSP-optimized PID controller** using filtered encoder data
2. **Cascade control** (position outer loop, velocity inner loop)
3. **Adaptive gain scheduling** based on velocity
4. **Anti-windup** with back-calculation method

Phase 3 builds directly on the filtered encoder signals from Phase 2.

## Troubleshooting

### Build Errors
- **"undefined reference to encoder_filter_init"**: Check CMakeLists.txt includes encoder_filter_dsp.c
- **"implicit declaration"**: Verify all #include statements in test file

### Runtime Issues
- **Tests fail**: Check serial output for specific failure
- **Poor noise rejection**: Decrease cutoff frequency
- **Too much lag**: Increase cutoff frequency
- **Velocity estimate noisy**: Increase Kalman Q_vel or decrease R

### Performance Issues
- **Pipeline takes >10% of loop time**: Profile with timer, check compiler optimization flags
- **Tests don't run**: Verify PICO_2W defined and DSP enabled

## Success Criteria for Phase 2

Before proceeding to Phase 3, verify:
- ✅ All 6 tests pass
- ✅ Benchmark shows <10% of 1 kHz loop budget
- ✅ Noise reduction >2:1 demonstrated
- ✅ Velocity estimation error <20%
- ✅ DSP instructions confirmed working (from Phase 1)

## Contact/Issues
If tests fail or performance is insufficient, review:
1. DSP instruction enablement (Phase 1 verification)
2. Compiler optimization level (-O2 or -O3 recommended)
3. Sample rate matches actual control loop frequency
