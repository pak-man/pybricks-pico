# Phase 3: DSP-Optimized PID Control - Integration Guide

## Overview
Phase 3 implements advanced PID control with DSP acceleration, cascade control, adaptive gain scheduling, and anti-windup on the RP2350.

## Files Created
1. **pid_controller_dsp.h** - Header with PID, cascade, and adaptive controllers
2. **pid_controller_dsp.c** - Implementation with DSP acceleration
3. **pid_controller_test.c** - Comprehensive test suite with plant simulation

## Integration Steps

### Step 1: Copy Files to Project
```bash
cd ~/pybricks-micropython/lib/pbio/drv/motion_dsp
cp /path/to/pid_controller_dsp.h .
cp /path/to/pid_controller_dsp.c .
cp /path/to/pid_controller_test.c .
```

### Step 2: Update CMakeLists.txt
Add to the same location where you added Phase 2 files:

```cmake
# In lib/pbio/CMakeLists.txt or bricks/picow/CMakeLists.txt
# Add these lines alongside Phase 2 files:
${PBIO_TOP}/drv/motion_dsp/pid_controller_dsp.c
${PBIO_TOP}/drv/motion_dsp/pid_controller_test.c
```

### Step 3: Update system_test.c
Add these includes at the top:
```c
#ifdef PICO_2W
#include "pid_controller_dsp.h"
extern void pid_controller_run_tests(void);
#endif
```

Add to command parser:
```c
} else if (strncmp(cmd, "PID TEST", 8) == 0) {
#ifdef PICO_2W
    printf("Running PID controller test suite...\n");
    pid_controller_run_tests();
    send_ble_response("PID TEST", "Complete - check serial");
#else
    send_ble_response("ERROR", "Requires Pico 2W");
#endif
```

Add to HELP text:
```c
"  PID TEST       - Run PID controller tests (DSP)\n"
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
3. Send command: `PID TEST`

## Expected Test Results

### Test 1: PID Initialization
- ✓ Gains converted to Q16.16 format
- ✓ Default limits and options set

### Test 2: Step Response
- ✓ Settles within 500ms
- ✓ Final error <1 degree
- ✓ Reasonable overshoot (<20%)

### Test 3: Sinusoidal Tracking
- ✓ RMS tracking error <5 degrees
- ✓ Smooth following of 1 Hz sine wave

### Test 4: Cascade Controller
- ✓ Faster settling than basic PID
- ✓ Better disturbance rejection
- ✓ Final error <2 degrees

### Test 5: Adaptive Gain Scheduling
- ✓ Gains adjust based on velocity
- ✓ High gains at low speed (precision)
- ✓ Low gains at high speed (stability)

### Test 6: Anti-Windup
- ✓ Back-calculation prevents integral saturation
- ✓ Faster settling than without anti-windup

### Performance Benchmark @ 150 MHz
Expected results:
- **Basic PID**: ~2-4 µs per update
- **Cascade PID**: ~4-8 µs per update  
- **Adaptive PID**: ~3-6 µs per update
- **Budget usage**: <1% of 1 kHz control loop

## Usage Examples

### Example 1: Basic Position Control
```c
#include "pid_controller_dsp.h"
#include "encoder_filter_dsp.h"

pid_controller_t position_pid;
encoder_dsp_pipeline_t encoder;

void motor_init(void) {
    // Initialize encoder filter (Phase 2)
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    
    // Initialize PID: Kp=2.0, Ki=0.5, Kd=0.05, dt=1ms
    pid_init(&position_pid, 2.0f, 0.5f, 0.05f, 0.001f);
    pid_set_limits(&position_pid, -10000, 10000);  // ±10000 motor units
    pid_enable_back_calculation(&position_pid, 1.0f);
}

void motor_control_loop_1khz(void) {
    // Read and filter encoder
    int32_t raw_encoder = read_encoder_hardware();
    int32_t position = encoder_pipeline_update(&encoder, raw_encoder);
    
    // PID control
    int32_t setpoint = 90000;  // 90 degrees
    int32_t motor_output = pid_update(&position_pid, setpoint, position);
    
    // Apply to motor
    set_motor_pwm(motor_output);
}
```

### Example 2: Cascade Control (Position + Velocity)
```c
#include "pid_controller_dsp.h"
#include "encoder_filter_dsp.h"

cascade_pid_t cascade;
encoder_dsp_pipeline_t encoder;

void motor_init(void) {
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    
    // Position loop: Kp=5, Ki=1, Kd=0.1
    // Velocity loop: Kp=2, Ki=0.5, Kd=0.05
    cascade_pid_init(&cascade,
                     5.0f, 1.0f, 0.1f,   // Position gains
                     2.0f, 0.5f, 0.05f,  // Velocity gains
                     0.001f);             // 1 kHz sample rate
    
    // Optional: enable feed-forward
    cascade_pid_set_feedforward(&cascade, 0.8f, 0.1f);
}

void motor_control_loop_1khz(void) {
    int32_t raw_encoder = read_encoder_hardware();
    int32_t position = encoder_pipeline_update(&encoder, raw_encoder);
    int32_t velocity = encoder_pipeline_get_velocity(&encoder);
    
    int32_t setpoint = 180000;  // 180 degrees
    int32_t motor_output = cascade_pid_update(&cascade, setpoint, 
                                               position, velocity);
    
    set_motor_pwm(motor_output);
}
```

### Example 3: Adaptive Gain Scheduling
```c
adaptive_pid_t adaptive;
encoder_dsp_pipeline_t encoder;

void motor_init(void) {
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    
    // Base gains
    adaptive_pid_init(&adaptive, 2.0f, 0.5f, 0.05f, 0.001f);
    
    // Gain schedules for different velocity ranges
    adaptive_pid_add_schedule(&adaptive, 0,      1.0f, 1.0f, 1.0f);  // Low speed
    adaptive_pid_add_schedule(&adaptive, 50000,  0.8f, 0.6f, 0.8f);  // Medium
    adaptive_pid_add_schedule(&adaptive, 100000, 0.5f, 0.3f, 0.5f);  // High speed
}

void motor_control_loop_1khz(void) {
    int32_t raw_encoder = read_encoder_hardware();
    int32_t position = encoder_pipeline_update(&encoder, raw_encoder);
    int32_t velocity = encoder_pipeline_get_velocity(&encoder);
    
    int32_t setpoint = get_target_position();
    
    // Adaptive PID automatically scales gains based on velocity
    int32_t motor_output = adaptive_pid_update(&adaptive, setpoint,
                                                position, velocity);
    
    set_motor_pwm(motor_output);
}
```

## Tuning Guide

### Basic PID Tuning (Ziegler-Nichols Method)
1. **Start with Kp only**: Set Ki=0, Kd=0
   - Increase Kp until system oscillates
   - Note Ku (ultimate gain) and Tu (oscillation period)

2. **Calculate initial gains**:
   - Kp = 0.6 * Ku
   - Ki = 1.2 * Ku / Tu
   - Kd = 0.075 * Ku * Tu

3. **Fine-tune**:
   - Too much overshoot? Decrease Kp, increase Kd
   - Too slow? Increase Kp
   - Steady-state error? Increase Ki
   - Oscillations? Decrease Kp and Ki

### Cascade Tuning
1. **Tune inner loop (velocity) first**:
   - Temporarily disable outer loop (or use very low gains)
   - Apply step velocity commands
   - Tune for fast, stable response

2. **Then tune outer loop (position)**:
   - With inner loop stable, tune position controller
   - Position loop should be ~3-5x slower than velocity loop
   - Increase position Kp until slight overshoot
   - Add position Ki to eliminate steady-state error

### Adaptive Gain Scheduling
- **Low velocity** (precision): Higher gains for tight control
- **Medium velocity** (transition): Slightly reduced gains
- **High velocity** (stability): Lower gains to prevent oscillation

Example thresholds:
- 0-50,000 mdeg/s: 100% gains (precise)
- 50,000-100,000 mdeg/s: 70% gains (balanced)
- >100,000 mdeg/s: 40% gains (stable)

## Integration with Phase 2

Phase 3 works seamlessly with Phase 2 encoder filters:

```c
// Complete control system
encoder_dsp_pipeline_t encoder;
cascade_pid_t controller;

void setup(void) {
    // Phase 2: Encoder filtering
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    
    // Phase 3: PID control
    cascade_pid_init(&controller, 
                     5.0f, 1.0f, 0.1f,
                     2.0f, 0.5f, 0.05f,
                     0.001f);
}

void loop_1khz(void) {
    // Phase 2: Filter encoder
    int32_t pos = encoder_pipeline_update(&encoder, read_encoder());
    int32_t vel = encoder_pipeline_get_velocity(&encoder);
    
    // Phase 3: Control
    int32_t output = cascade_pid_update(&controller, setpoint, pos, vel);
    
    // Apply
    set_motor(output);
}
```

**Total overhead**: Phase 2 (0.45%) + Phase 3 (0.8%) = **1.25% of 1 kHz loop**

## Troubleshooting

### Build Issues
**"undefined reference to pid_init"**
→ Check CMakeLists.txt includes pid_controller_dsp.c

**"implicit declaration"**
→ Add `#include "pid_controller_dsp.h"` to your code

### Runtime Issues
**Tests fail**
→ Check serial output for specific failure
→ Plant simulation may need adjustment for your system

**Oscillations in control**
→ Reduce Kp and Ki gains
→ Increase Kd or derivative filter (lower deriv_lpf_alpha)

**Sluggish response**
→ Increase Kp
→ Check output limits aren't too restrictive

**Integral windup**
→ Enable back-calculation: `pid_enable_back_calculation(&pid, 1.0f)`
→ Set integral limits: `pid_set_integral_limits(&pid, -limit, limit)`

**Cascade oscillates**
→ Ensure velocity loop is stable first
→ Reduce position loop gains
→ Position loop bandwidth should be ~3-5x lower than velocity

### Performance Issues
**PID uses >5% of loop**
→ Check compiler optimization (-O2 or -O3)
→ Verify DSP instructions enabled (Phase 1)
→ Profile with timer

## Next Steps: Phase 4

With validated PID control, Phase 4 will implement:
1. **S-curve trajectory generation** for smooth motion
2. **Polynomial trajectory evaluation** with DSP
3. **Look-ahead trajectory planning**
4. **Jerk-limited motion profiles**

Phase 4 feeds optimized trajectories (position, velocity, acceleration) into Phase 3's `cascade_pid_update_trajectory()`.

## Success Criteria

Before proceeding to Phase 4, verify:
- ✅ All 6 tests pass
- ✅ Benchmark shows <2% of 1 kHz loop
- ✅ Step response settles <500ms
- ✅ Tracking error <5 degrees RMS
- ✅ Integration with Phase 2 working

Run `PID TEST` command to validate all items.
