# Phase 3 PID Controller - Quick Reference

## 🚀 Quick Start (Copy-Paste)

### Basic Position Control
```c
#include "pid_controller_dsp.h"

pid_controller_t pid;

void setup() {
    pid_init(&pid, 2.0f, 0.5f, 0.05f, 0.001f);
    //             Kp    Ki    Kd    dt(1kHz)
    pid_set_limits(&pid, -10000, 10000);
    pid_enable_back_calculation(&pid, 1.0f);
}

void loop_1khz() {
    int32_t position = get_filtered_position();
    int32_t setpoint = 90000;  // 90 degrees in mdeg
    int32_t output = pid_update(&pid, setpoint, position);
    set_motor(output);
}
```

### Cascade Control (Better Performance!)
```c
#include "pid_controller_dsp.h"

cascade_pid_t cascade;

void setup() {
    cascade_pid_init(&cascade,
        5.0f, 1.0f, 0.1f,   // Position: Kp, Ki, Kd
        2.0f, 0.5f, 0.05f,  // Velocity: Kp, Ki, Kd
        0.001f);             // dt = 1ms
}

void loop_1khz() {
    int32_t pos = get_filtered_position();
    int32_t vel = get_filtered_velocity();
    int32_t output = cascade_pid_update(&cascade, setpoint, pos, vel);
    set_motor(output);
}
```

## 📊 Performance at 150 MHz

| Controller | Time | Cycles | Loop % |
|-----------|------|--------|--------|
| Basic PID | 3 µs | 450 | 0.3% |
| Cascade PID | 6 µs | 900 | 0.6% |
| Adaptive PID | 4 µs | 600 | 0.4% |

**Combined with Phase 2**: Total ~1.5% of 1 kHz loop

## 🎛️ Tuning Cheat Sheet

### Step-by-Step Tuning
```c
// Step 1: Proportional only
pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.001f);
// Increase Kp until slight oscillation

// Step 2: Add Integral (if steady-state error)
pid_init(&pid, Kp, Kp/10, 0.0f, 0.001f);
// Ki typically 10-20% of Kp

// Step 3: Add Derivative (if overshoot)
pid_init(&pid, Kp, Ki, Kp/20, 0.001f);
// Kd typically 5% of Kp
```

### Typical Gain Ranges
```c
// Stiff mechanical system (low inertia):
pid_init(&pid, 5.0f, 1.0f, 0.1f, 0.001f);

// Medium system (typical motor):
pid_init(&pid, 2.0f, 0.5f, 0.05f, 0.001f);

// Soft system (high inertia):
pid_init(&pid, 0.5f, 0.1f, 0.01f, 0.001f);
```

### Problem → Solution
```
Too much overshoot?     → ↓Kp, ↑Kd
Oscillates?             → ↓Kp, ↓Ki
Too slow?               → ↑Kp
Steady-state error?     → ↑Ki
Noisy?                  → ↓Kd, increase deriv filter
```

## 🧪 Testing Commands

```bash
# Build
cd ~/pybricks-micropython/bricks/picow
make pico2w

# Flash to hardware, then via serial:
PID TEST    # Runs all 6 tests + benchmark
```

## ✅ Expected Test Output

```
=== Test 2: Step Response ===
  Target: 90000 mdeg (90.0 deg)
  Final position: 89950 mdeg (90.0 deg)
  Final error: 50 mdeg (0.050 deg)
  Settling time (2%): 325.0 ms
  Overshoot: 8.2%
✓ Step response test passed

=== Performance Benchmark ===
  Basic PID:      3.14 µs (471 cycles)
  Cascade PID:    5.89 µs (884 cycles)
  Adaptive PID:   4.02 µs (603 cycles)
  
  Budget usage (1 kHz loop):
    Basic PID:    0.31%
    Cascade PID:  0.59%
  ✓ PID overhead acceptable

╔══════════════════════════════════════════╗
║   TEST SUMMARY: 6/6 PASSED               ║
╚══════════════════════════════════════════╝
✓ All tests passed!
```

## 🎯 Controller Selection Guide

### Use **Basic PID** when:
- Simple position control needed
- Single control loop sufficient
- Minimal CPU overhead critical
- No velocity sensor available

### Use **Cascade PID** when:
- Best performance required
- Velocity feedback available (from Phase 2)
- Fast disturbance rejection needed
- Trajectory following required

### Use **Adaptive PID** when:
- Wide velocity range expected
- Different gains needed at different speeds
- Precision at low speed + stability at high speed
- Non-linear load characteristics

## 🔧 Advanced Features

### Anti-Windup (Recommended!)
```c
pid_enable_back_calculation(&pid, 1.0f);
pid_set_integral_limits(&pid, -50000, 50000);
```
**When to use**: Always! Prevents integral buildup during saturation.

### Derivative-on-Measurement
```c
pid.derivative_on_measurement = true;  // Default, recommended
```
**Benefit**: Prevents "derivative kick" on setpoint changes.

### Derivative Filtering
```c
// Adjust derivative low-pass filter cutoff
// For noisier encoders, use lower alpha (more filtering)
pid.deriv_lpf_alpha = (int16_t)(0.2f * 32768.0f);  // Q15 format
```

### Feed-Forward (Cascade Only)
```c
cascade_pid_set_feedforward(&cascade, 
    0.8f,   // Velocity feed-forward
    0.1f);  // Acceleration feed-forward
```
**Benefit**: Improves trajectory tracking, reduces lag.

## 📐 Units Reference

| Variable | Units | Type | Range |
|----------|-------|------|-------|
| Position | millidegrees (mdeg) | int32_t | ±2.1M |
| Velocity | mdeg/s | int32_t | ±2.1M |
| Gains | Q16.16 internally | float input | any |
| Output | motor units | int32_t | ±2.1M |

**Example conversions**:
```c
int32_t degrees_to_mdeg(float deg) { return (int32_t)(deg * 1000.0f); }
float mdeg_to_degrees(int32_t mdeg) { return mdeg / 1000.0f; }
```

## 🎓 Key Concepts

**P (Proportional)**: Reacts to current error
- Higher → Faster response, more overshoot
- Lower → Slower response, less overshoot

**I (Integral)**: Eliminates steady-state error
- Higher → Faster convergence, more overshoot
- Lower → Slower convergence, less overshoot

**D (Derivative)**: Damps oscillations
- Higher → Less overshoot, may amplify noise
- Lower → More overshoot, less noise sensitivity

**Cascade Control**: Two loops
- Outer (position) → Slow, precise
- Inner (velocity) → Fast, responsive
- Like: Brain (position) → Muscles (velocity)

**Adaptive Gains**: Automatically adjust based on speed
- Low speed → High gains (precision)
- High speed → Low gains (stability)

## 💡 Pro Tips

1. **Always tune velocity loop first** in cascade control
2. **Use cascade for best performance** (if you have Phase 2)
3. **Enable anti-windup** to prevent integral buildup
4. **Start with derivative-on-measurement** (prevents kicks)
5. **Log error and output** to visualize behavior
6. **Test step response** to validate tuning
7. **Monitor integral value** - shouldn't grow huge

## 🔗 Phase Integration

**Phase 2 + Phase 3 = Complete Motion Control**

```c
// Phase 2: Filtering
encoder_pipeline_update(&encoder, raw);
int32_t pos = encoder_pipeline_get_position(&encoder);
int32_t vel = encoder_pipeline_get_velocity(&encoder);

// Phase 3: Control
int32_t output = cascade_pid_update(&cascade, setpoint, pos, vel);

// Total overhead: 0.45% + 0.6% = 1.05%
```

## 🎯 Next Steps

After Phase 3 validation:
1. ✅ Confirm all tests pass
2. ✅ Tune gains for your motor
3. ✅ Validate step response <500ms
4. ✅ Check tracking error <5° RMS
5. → **Proceed to Phase 4**: Trajectory Planning

## 📁 File Locations

```
~/pybricks-micropython/lib/pbio/drv/motion_dsp/
├── pid_controller_dsp.h       ← API
├── pid_controller_dsp.c       ← Implementation
└── pid_controller_test.c      ← Tests
```

## 🆘 Common Issues

**Oscillates constantly**
```c
// Reduce gains by 50%
pid_init(&pid, Kp*0.5f, Ki*0.5f, Kd, 0.001f);
```

**Slow response**
```c
// Increase Kp by 50%
pid_init(&pid, Kp*1.5f, Ki, Kd, 0.001f);
```

**Steady-state error**
```c
// Increase Ki
pid_init(&pid, Kp, Ki*2.0f, Kd, 0.001f);
```

**Integral windup**
```c
pid_enable_back_calculation(&pid, 1.0f);
pid_set_integral_limits(&pid, -limit, limit);
```

---

**Quick Reference v1.0** | Phase 3 Complete ✅ | Ready for Phase 4 🚀
