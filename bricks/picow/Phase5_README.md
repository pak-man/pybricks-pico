# Phase 5: Motion Profiling & Multi-Axis Coordination

> **Part of the 7-phase DSP Motion Control roadmap for Pybricks on Raspberry Pi Pico 2W**

## 📦 What's Included

Complete multi-axis coordination and advanced motion profiling:

- **Linear Interpolation (G1)** - Straight-line paths in Cartesian space
- **Circular Interpolation (G2/G3)** - Arc movements in any plane
- **Synchronized Multi-Axis** - Coordinated motion across 1-4 axes
- **Gantry Control** - Dual motor coordination with cross-coupling
- **Electronic Gearing** - Master-slave axis coordination
- **G-Code Motion Planner** - CNC-style coordinated moves

## 🎯 Key Features

✅ **Multi-axis coordination** - Up to 4 axes synchronized  
✅ **Linear interpolation** - Straight-line Cartesian paths  
✅ **Circular interpolation** - Smooth arcs (XY, YZ, XZ planes)  
✅ **Gantry control** - Dual motors, perfect squareness  
✅ **Electronic gearing** - Master-slave with configurable ratio  
✅ **G-Code compatible** - Standard CNC motion planning  

## 📁 Files in This Package

### Core Implementation
- **motion_profiler_dsp.h** (10.5 KB) - Multi-axis API
- **motion_profiler_dsp.c** (16 KB) - Coordinated motion implementation

### Documentation
- **Phase5_README.md** (this file) - Package overview
- **Phase5_INTEGRATION_GUIDE.md** - Step-by-step integration
- **Phase5_QUICK_REFERENCE.md** - Developer cheat sheet

## 🚀 Quick Start

### 1. Download Files
[motion_profiler_dsp.h](computer:///mnt/user-data/outputs/motion_profiler_dsp.h)
[motion_profiler_dsp.c](computer:///mnt/user-data/outputs/motion_profiler_dsp.c)

### 2. Copy to Project
```bash
cd ~/pybricks-micropython/lib/pbio/drv/motion_dsp
# Copy the 2 files here
```

### 3. Update CMakeLists.txt
Add where you added Phase 2, 3, 4 files:
```cmake
${PBIO_TOP}/drv/motion_dsp/motion_profiler_dsp.c
```

### 4. Linear Interpolation Example
```c
#include "motion_profiler_dsp.h"

linear_interpolation_t line_move;

void setup() {
    position_t start = {0, 0, 0, 0};        // Origin
    position_t end = {100000, 50000, 0, 0}; // (100mm, 50mm, 0, 0)
    
    linear_interp_init(&line_move, start, end,
                       200000,  // 200 mm/s max velocity
                       500000,  // 500 mm/s² max acceleration
                       1000000); // 1000 mm/s³ max jerk
    linear_interp_start(&line_move);
}

void loop_1khz() {
    // Get current position for all axes
    position_t pos = linear_interp_get_position(&line_move);
    velocity_t vel = linear_interp_get_velocity(&line_move);
    
    // Control each axis (Phase 3 PID + Phase 2 encoder)
    control_axis_x(pos.x, vel.x);
    control_axis_y(pos.y, vel.y);
    
    if (linear_interp_is_complete(&line_move)) {
        // Move complete!
    }
}
```

## 📖 Motion Types Guide

### Linear Interpolation (G1)
**Use for**: Straight-line moves in Cartesian space

```c
linear_interpolation_t move;
position_t start = {0, 0, 0, 0};
position_t end = {100000, 100000, 0, 0};  // Diagonal line

linear_interp_init(&move, start, end, 200000, 500000, 1000000);
linear_interp_start(&move);
```

**Benefits:**
- All axes arrive simultaneously
- Constant feed rate along path
- Perfect for CNC machining, 3D printing

### Circular Interpolation (G2/G3)
**Use for**: Arc movements, rounded corners

```c
circular_interpolation_t arc;
position_t start = {100000, 0, 0, 0};
position_t end = {0, 100000, 0, 0};
position_t center = {0, 0, 0, 0};  // Arc around origin

circular_interp_init(&arc, start, end, center,
                     CIRCULAR_CCW,  // Counter-clockwise
                     AXIS_X, AXIS_Y,  // XY plane
                     200000, 500000, 1000000);
circular_interp_start(&arc);
```

**Benefits:**
- Smooth corners
- Constant radius
- Standard CNC arcs (G2/G3)

### Gantry Control (Dual Motors)
**Use for**: Large axes with two motors (e.g., dual Y motors)

```c
gantry_controller_t gantry;

gantry_init(&gantry,
            Q16_16(0.5),  // Coupling gain
            1000);        // Max correction (mdeg)

gantry_set_target(&gantry, 0, 100000, 200000, 500000, 1000000);
gantry_start(&gantry);

// In control loop:
int32_t motor1_sp, motor2_sp;
gantry_update(&gantry, motor1_pos, motor2_pos, &motor1_sp, &motor2_sp);
// motor1_sp and motor2_sp include cross-coupling correction
```

**Benefits:**
- Perfect squareness
- Automatic racking correction
- Industry-standard gantry control

### Electronic Gearing
**Use for**: Master-slave coordination (e.g., conveyor + robot)

```c
electronic_gearing_t gearing;

gearing_init(&gearing,
             2.0f,   // Gear ratio (slave moves 2x master)
             0);     // No offset

gearing_set_master(&gearing, 0, 100000, 200000, 500000, 1000000);
gearing_start(&gearing);

// Get coordinated positions:
int32_t master_pos, slave_pos;
gearing_get_positions(&gearing, &master_pos, &slave_pos);
```

**Benefits:**
- Perfect synchronization
- Configurable ratio
- Offset capability

## 🎯 Complete Multi-Axis System

**All phases integrated:**

```c
// Phase 2: Encoder filters (one per axis)
encoder_dsp_pipeline_t encoder_x, encoder_y;

// Phase 3: PID controllers (one per axis)
cascade_pid_t pid_x, pid_y;

// Phase 4 & 5: Multi-axis trajectory
linear_interpolation_t trajectory;

void setup() {
    // Initialize encoders
    encoder_pipeline_init(&encoder_x, 50.0f, 1000.0f);
    encoder_pipeline_init(&encoder_y, 50.0f, 1000.0f);
    
    // Initialize controllers
    cascade_pid_init(&pid_x, 5.0f, 1.0f, 0.1f, 2.0f, 0.5f, 0.05f, 0.001f);
    cascade_pid_init(&pid_y, 5.0f, 1.0f, 0.1f, 2.0f, 0.5f, 0.05f, 0.001f);
    
    // Initialize coordinated move
    position_t start = {0, 0, 0, 0};
    position_t end = {100000, 50000, 0, 0};
    linear_interp_init(&trajectory, start, end, 200000, 500000, 1000000);
    linear_interp_start(&trajectory);
}

void loop_1khz() {
    // Phase 2: Filter both axes
    int32_t pos_x = encoder_pipeline_update(&encoder_x, read_encoder_x());
    int32_t vel_x = encoder_pipeline_get_velocity(&encoder_x);
    int32_t pos_y = encoder_pipeline_update(&encoder_y, read_encoder_y());
    int32_t vel_y = encoder_pipeline_get_velocity(&encoder_y);
    
    // Phase 5: Get coordinated trajectory
    position_t traj_pos = linear_interp_get_position(&trajectory);
    velocity_t traj_vel = linear_interp_get_velocity(&trajectory);
    
    // Phase 3: Control each axis
    int32_t out_x = cascade_pid_update_trajectory(&pid_x,
        traj_pos.x, traj_vel.x, 0, pos_x, vel_x);
    int32_t out_y = cascade_pid_update_trajectory(&pid_y,
        traj_pos.y, traj_vel.y, 0, pos_y, vel_y);
    
    set_motor_x(out_x);
    set_motor_y(out_y);
}
```

## 🗺️ Roadmap Context

```
Phase 1: ✅ DSP Foundation
Phase 2: ✅ Encoder Filters
Phase 3: ✅ Advanced PID
Phase 4: ✅ Trajectory Planning
Phase 5: ✅ Motion Profiling ← YOU ARE HERE
Phase 6: 🔲 Integration & Testing
Phase 7: 🔲 Advanced Features
```

### What Phase 5 Enables
- Multi-axis CNC machines
- 3D printers with coordinated XYZ
- Gantry systems with dual motors
- Pick-and-place robots
- Conveyor coordination
- Complex Cartesian paths

### Next: Phase 6
- Complete system integration
- Real-world testing
- Performance optimization
- Documentation finalization

## 📊 Use Cases

| Application | Motion Type | Axes |
|-------------|-------------|------|
| 3D Printer | Linear + Circular | X, Y, Z |
| CNC Router | Linear + Circular | X, Y, Z |
| Laser Cutter | Linear + Circular | X, Y |
| Pick-and-Place | Linear | X, Y, Z, A |
| Large Gantry | Gantry Control | Y (dual motors) |
| Conveyor Tracking | Electronic Gearing | Master + Slave |

## 🎓 Technical Highlights

- **Euclidean distance** calculation for multi-axis moves
- **Axis scaling** for coordinated motion
- **Cross-coupling** for gantry squareness
- **Gear ratio** in Q16.16 fixed-point
- **Trigonometric** arc calculations
- **CNC-compatible** G-code structure

## 🚀 Next Steps

1. Download files above
2. Integrate into project
3. Test linear moves first
4. Add circular interpolation
5. Implement gantry if needed
6. Proceed to Phase 6!

---

**Phase 5 Complete** ✅ | Multi-Axis Motion Profiling  
**Ready for Phase 6** 🚀 | Integration & Testing

All files ready to download above!
