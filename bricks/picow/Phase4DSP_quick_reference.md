# Phase 4 Trajectory Planning - Quick Reference

## 🚀 Quick Start (Copy-Paste)

### S-Curve Motion (Recommended)
```c
#include "trajectory_planner_dsp.h"

scurve_profile_t traj;

void setup() {
    scurve_init(&traj,
        0,       // Start: 0°
        90000,   // End: 90° (in mdeg)
        200000,  // Max velocity: 200°/s
        500000,  // Max acceleration: 500°/s²
        1000000); // Max jerk: 1000°/s³
    scurve_start(&traj);
}

void loop_1khz() {
    trajectory_point_t point = scurve_get_point(&traj);
    
    // Use point.position, point.velocity, point.acceleration
    // with Phase 3 cascade controller
    
    if (scurve_is_complete(&traj)) {
        // Trajectory finished!
    }
}
```

### Complete Stack (Phase 2+3+4)
```c
scurve_profile_t trajectory;
cascade_pid_t controller;
encoder_dsp_pipeline_t encoder;

void setup() {
    encoder_pipeline_init(&encoder, 50.0f, 1000.0f);
    cascade_pid_init(&controller, 5.0f, 1.0f, 0.1f,
                     2.0f, 0.5f, 0.05f, 0.001f);
    scurve_init(&trajectory, 0, 180000, 200000, 500000, 1000000);
    scurve_start(&trajectory);
}

void loop_1khz() {
    // Phase 2: Filter
    int32_t pos = encoder_pipeline_update(&encoder, read_encoder());
    int32_t vel = encoder_pipeline_get_velocity(&encoder);
    
    // Phase 4: Trajectory
    trajectory_point_t traj = scurve_get_point(&trajectory);
    
    // Phase 3: Control with feed-forward
    int32_t output = cascade_pid_update_trajectory(&controller,
        traj.position, traj.velocity, traj.acceleration,
        pos, vel);
    
    set_motor(output);
}
```

## 📊 Trajectory Type Selection

| Use Case | Best Choice | Why |
|----------|-------------|-----|
| Precision positioning | S-Curve | Jerk-limited, minimal vibration |
| Custom path shapes | Polynomial | Exact boundary conditions |
| Simple point-to-point | Trapezoidal | Fastest calculation |
| Multi-waypoint | Path Planner | Automatic segmentation |

## 🎛️ Tuning Cheat Sheet

### Constraint Selection
```c
// For position (in millidegrees):
start_pos = 0;        // 0°
end_pos = 90000;      // 90°
end_pos = 180000;     // 180°
end_pos = 360000;     // 360° (one revolution)

// For velocity (mdeg/s):
max_vel = 100000;     // 100°/s  (slow, gentle)
max_vel = 200000;     // 200°/s  (typical)
max_vel = 500000;     // 500°/s  (fast, aggressive)

// For acceleration (mdeg/s²):
max_accel = 200000;   // 200°/s²  (gentle)
max_accel = 500000;   // 500°/s²  (typical)
max_accel = 1000000;  // 1000°/s² (aggressive)

// For jerk (mdeg/s³):
max_jerk = 400000;    // 400°/s³  (very smooth)
max_jerk = 1000000;   // 1000°/s³ (typical)
max_jerk = 2000000;   // 2000°/s³ (sporty)
```

### Quick Conversion Table
| Degrees | Millidegrees | Revolutions |
|---------|--------------|-------------|
| 1° | 1,000 | 1/360 |
| 90° | 90,000 | 1/4 |
| 180° | 180,000 | 1/2 |
| 360° | 360,000 | 1 |

## 📐 All Trajectory Types

### 1. S-Curve (7-Phase Jerk-Limited)
```c
scurve_profile_t profile;

// Basic init (start at rest, end at rest):
scurve_init(&profile, start_pos, end_pos, max_vel, max_accel, max_jerk);

// With non-zero velocities:
scurve_init_with_velocity(&profile,
    start_pos, start_vel,
    end_pos, end_vel,
    max_vel, max_accel, max_jerk);

scurve_start(&profile);
trajectory_point_t point = scurve_get_point(&profile);
bool done = scurve_is_complete(&profile);
```

### 2. Polynomial (Quintic Spline)
```c
polynomial_segment_t seg;

// Quintic (5th order) - position, velocity, acceleration:
polynomial_init_quintic(&seg, duration,
    p0, v0, a0,  // Start conditions
    p1, v1, a1); // End conditions

// Cubic (3rd order) - position, velocity only:
polynomial_init_cubic(&seg, duration,
    p0, v0,  // Start
    p1, v1); // End

trajectory_point_t point = polynomial_evaluate(&seg, time);
```

### 3. Trapezoidal (Simple)
```c
trapezoidal_profile_t profile;

trap_init(&profile, start_pos, end_pos, max_vel, max_accel);
trap_start(&profile);
trajectory_point_t point = trap_get_point(&profile);
bool done = trap_is_complete(&profile);
```

### 4. Multi-Waypoint Path
```c
path_planner_t planner;

path_planner_init(&planner, max_vel, max_accel, max_jerk);

// Add waypoints:
waypoint_t wp1 = {.position = 90000, .velocity = 0, .blend_radius = 0};
waypoint_t wp2 = {.position = 180000, .velocity = 0, .blend_radius = 0};
path_planner_add_waypoint(&planner, wp1);
path_planner_add_waypoint(&planner, wp2);

path_planner_start(&planner, current_position);
trajectory_point_t point = path_planner_get_point(&planner);
bool done = path_planner_is_complete(&planner);
```

## 🎯 Trajectory Point Structure

```c
typedef struct {
    int32_t position;      // Position (mdeg)
    int32_t velocity;      // Velocity (mdeg/s)
    int32_t acceleration;  // Acceleration (mdeg/s²)
    int32_t jerk;          // Jerk (mdeg/s³)
    uint32_t timestamp_us; // Timestamp (microseconds)
} trajectory_point_t;
```

**Usage:**
```c
trajectory_point_t traj = scurve_get_point(&profile);
int32_t target_pos = traj.position;
int32_t target_vel = traj.velocity;
int32_t target_acc = traj.acceleration;
```

## 🔧 Common Patterns

### Pattern 1: Simple Move
```c
// Move from current position to target
scurve_init(&traj, current_pos, target_pos, 200000, 500000, 1000000);
scurve_start(&traj);
```

### Pattern 2: Continuous Motion
```c
// When first move completes, start next
if (scurve_is_complete(&traj)) {
    scurve_init(&traj, current_pos, next_target, 200000, 500000, 1000000);
    scurve_start(&traj);
}
```

### Pattern 3: Multi-Segment Path
```c
waypoint_t waypoints[] = {
    {90000, 0, 0},   // Stop at 90°
    {180000, 0, 0},  // Stop at 180°
    {270000, 0, 0},  // Stop at 270°
    {360000, 0, 0}   // Stop at 360°
};

for (int i = 0; i < 4; i++) {
    path_planner_add_waypoint(&planner, waypoints[i]);
}
```

### Pattern 4: Velocity Blending
```c
// Don't stop at waypoint, blend through
waypoint_t wp = {
    .position = 90000,
    .velocity = 50000,    // Pass through at 50°/s
    .blend_radius = 5000  // Blend over 5° radius
};
```

## 💡 Pro Tips

1. **Start with trapezoidal** to verify basic motion
2. **Use S-curve for production** - smoothest motion
3. **Tune jerk first** - affects smoothness most
4. **Log trajectory points** - verify profile shape
5. **Combine with feed-forward** - improves tracking
6. **Pre-calculate total time** - for UI progress bars
7. **Check is_complete()** - before starting new move

## ⚡ Performance Tips

### Fast Calculation
- **S-curve**: ~3-5 µs per point
- **Polynomial**: ~1-3 µs (DSP Horner's method)
- **Trapezoidal**: ~2-4 µs

### Memory Usage
- **S-curve**: 88 bytes
- **Polynomial segment**: 40 bytes
- **Multi-segment (16)**: 650 bytes
- **Path planner (32 waypoints)**: 1.2 KB

## 🐛 Common Issues

**Trajectory doesn't move**
```c
// Did you call start?
scurve_start(&traj);  // Don't forget this!
```

**Motion too jerky**
```c
// Decrease max_jerk for smoother motion
max_jerk = 400000;  // Lower = smoother (but slower)
```

**Can't reach target**
```c
// Check constraints aren't contradictory
// Verify start != end
// Ensure max_vel > 0
```

**Overshoots target**
```c
// Probably PID issue, not trajectory
// Check Phase 3 controller gains
```

## 🎓 Understanding S-Curve Phases

```
7-Phase S-Curve Profile:

Jerk    │     ╱‾‾‾╲           ╱‾‾‾╲
        │    ╱     ╲         ╱     ╲
        │___╱_______╲_______╱_______╲___
        │            ╲     ╱
        │             ╲___╱

Accel   │        ╱‾‾‾‾‾╲
        │       ╱       ╲
        │  ____╱_________╲____
        │                 ╲
        │                  ╲____

Vel     │           ╱‾‾‾‾‾‾‾╲
        │          ╱         ╲
        │      ___╱___________╲___
        │     ╱                 ╲
        │____╱___________________╲____

Phase:  │ 1│2│3│  4  │5│6│7│
        └────────────────────────────→ Time

1. Accel increasing (jerk+)
2. Accel constant
3. Accel decreasing (jerk-)
4. Cruise (constant velocity)
5. Decel increasing (jerk-)
6. Decel constant
7. Decel decreasing (jerk+)
```

## 🔗 Integration Points

### With Phase 3 (PID Control)
```c
// Basic tracking:
int32_t output = cascade_pid_update(&controller, 
                                    traj.position, pos, vel);

// With feed-forward (better!):
int32_t output = cascade_pid_update_trajectory(&controller,
    traj.position, traj.velocity, traj.acceleration,
    pos, vel);
```

### With Phase 2 (Encoder Filtering)
```c
// Trajectory provides target
// Encoder provides actual
trajectory_point_t target = scurve_get_point(&traj);
int32_t actual = encoder_pipeline_get_position(&encoder);
int32_t error = target.position - actual;
```

## 📊 Typical Parameters

| Motor Type | Max Vel | Max Accel | Max Jerk |
|------------|---------|-----------|----------|
| Small servo | 300°/s | 1000°/s² | 3000°/s³ |
| Medium motor | 200°/s | 500°/s² | 1000°/s³ |
| Large gantry | 100°/s | 200°/s² | 400°/s³ |
| Precision stage | 50°/s | 100°/s² | 200°/s³ |

## 🎯 Next Steps

After Phase 4:
1. ✅ Integrate trajectory planner
2. ✅ Test simple S-curve moves
3. ✅ Tune constraints for your system
4. ✅ Combine with Phase 3 feed-forward
5. → **Proceed to Phase 5**: Multi-axis coordination

---

**Quick Reference v1.0** | Phase 4 Complete ✅ | Ready for Phase 5 🚀
