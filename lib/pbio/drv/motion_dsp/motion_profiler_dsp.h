// SPDX-License-Identifier: MIT
// Multi-axis motion profiling and coordination for RP2350

#ifndef _MOTION_PROFILER_DSP_H_
#define _MOTION_PROFILER_DSP_H_

#include <stdint.h>
#include <stdbool.h>
#include "trajectory_planner_dsp.h"

// ============================================================================
// Multi-Axis Coordinate System
// ============================================================================

#define MAX_AXES 4  // Support up to 4 axes (X, Y, Z, A)

// Axis identifiers
typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
    AXIS_A = 3  // Rotary axis
} axis_id_t;

// Multi-axis position (in millidegrees or millimeters)
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t a;
} position_t;

// Multi-axis velocity
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t a;
} velocity_t;

// ============================================================================
// Linear Interpolation (G1)
// ============================================================================

// Linear interpolation move (straight line in Cartesian space)
typedef struct {
    position_t start;       // Start position
    position_t end;         // End position
    
    int32_t distance;       // Total Euclidean distance (mdeg or mm)
    int32_t max_velocity;   // Maximum feed rate (mm/s or mdeg/s)
    int32_t max_accel;      // Maximum acceleration
    int32_t max_jerk;       // Maximum jerk
    
    // Per-axis scaling factors (Q16.16)
    int32_t axis_scale[MAX_AXES];
    
    // Master trajectory along path
    scurve_profile_t master_profile;
    
    // Current state
    float progress;         // 0.0 to 1.0 along path
    position_t current_pos;
    velocity_t current_vel;
    
    bool initialized;
    bool active;
} linear_interpolation_t;

// Initialize linear interpolation
void linear_interp_init(linear_interpolation_t *interp,
                        position_t start, position_t end,
                        int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start linear move
void linear_interp_start(linear_interpolation_t *interp);

// Get current multi-axis position
position_t linear_interp_get_position(linear_interpolation_t *interp);

// Get current multi-axis velocity
velocity_t linear_interp_get_velocity(linear_interpolation_t *interp);

// Check if complete
bool linear_interp_is_complete(linear_interpolation_t *interp);

// ============================================================================
// Circular Interpolation (G2/G3)
// ============================================================================

typedef enum {
    CIRCULAR_CW,   // Clockwise (G2)
    CIRCULAR_CCW   // Counter-clockwise (G3)
} circular_direction_t;

// Circular interpolation (arc in 2D plane)
typedef struct {
    // Arc definition
    position_t start;           // Start position
    position_t end;             // End position
    position_t center;          // Arc center point
    
    circular_direction_t direction;
    
    // Arc parameters (calculated)
    int32_t radius;             // Arc radius (mdeg or mm)
    float start_angle;          // Start angle (radians)
    float end_angle;            // End angle (radians)
    float total_angle;          // Total angle to sweep (radians)
    int32_t arc_length;         // Total arc length
    
    // Motion constraints
    int32_t max_velocity;       // Max feed rate along arc
    int32_t max_accel;
    int32_t max_jerk;
    
    // Plane selection (which 2 axes define the arc)
    axis_id_t plane_axis1;      // First axis (e.g., X)
    axis_id_t plane_axis2;      // Second axis (e.g., Y)
    
    // Master trajectory along arc
    scurve_profile_t master_profile;
    
    // Current state
    float progress;             // 0.0 to 1.0 along arc
    position_t current_pos;
    velocity_t current_vel;
    
    bool initialized;
    bool active;
} circular_interpolation_t;

// Initialize circular interpolation
// center: arc center point (I, J offsets from start in CNC terms)
void circular_interp_init(circular_interpolation_t *interp,
                          position_t start, position_t end, position_t center,
                          circular_direction_t direction,
                          axis_id_t plane_axis1, axis_id_t plane_axis2,
                          int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start circular move
void circular_interp_start(circular_interpolation_t *interp);

// Get current position
position_t circular_interp_get_position(circular_interpolation_t *interp);

// Get current velocity
velocity_t circular_interp_get_velocity(circular_interpolation_t *interp);

// Check if complete
bool circular_interp_is_complete(circular_interpolation_t *interp);

// ============================================================================
// Synchronized Multi-Axis Motion
// ============================================================================

// Synchronized axis group
typedef struct {
    uint8_t num_axes;                    // Number of axes (1-4)
    scurve_profile_t axis_profile[MAX_AXES]; // Per-axis trajectories
    
    // Synchronization
    bool synchronized;                   // All axes move together
    float master_progress;               // 0.0 to 1.0
    
    // Current positions
    int32_t current_pos[MAX_AXES];
    int32_t current_vel[MAX_AXES];
    
    bool initialized;
    bool active;
} synchronized_motion_t;

// Initialize synchronized motion
void sync_motion_init(synchronized_motion_t *sync, uint8_t num_axes);

// Set target for specific axis
void sync_motion_set_axis_target(synchronized_motion_t *sync,
                                 axis_id_t axis,
                                 int32_t start, int32_t end,
                                 int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start synchronized motion (all axes move together)
void sync_motion_start(synchronized_motion_t *sync);

// Get position for specific axis
int32_t sync_motion_get_position(synchronized_motion_t *sync, axis_id_t axis);

// Get velocity for specific axis
int32_t sync_motion_get_velocity(synchronized_motion_t *sync, axis_id_t axis);

// Check if all axes complete
bool sync_motion_is_complete(synchronized_motion_t *sync);

// ============================================================================
// Gantry Control (Dual Motor Coordination)
// ============================================================================

// Gantry with two motors driving one axis (e.g., dual Y motors)
typedef struct {
    // Two motor positions (both should track same position)
    int32_t motor1_pos;
    int32_t motor2_pos;
    
    // Target trajectory
    scurve_profile_t target_profile;
    
    // Cross-coupling correction
    int32_t coupling_gain;      // Q16.16 cross-coupling gain
    int32_t max_correction;     // Maximum correction allowed
    
    // Position error between motors (for squareness)
    int32_t position_error;
    
    bool initialized;
    bool active;
} gantry_controller_t;

// Initialize gantry controller
void gantry_init(gantry_controller_t *gantry,
                 int32_t coupling_gain,
                 int32_t max_correction);

// Set gantry target
void gantry_set_target(gantry_controller_t *gantry,
                       int32_t start, int32_t end,
                       int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start gantry motion
void gantry_start(gantry_controller_t *gantry);

// Update with current motor positions, returns corrected setpoints
void gantry_update(gantry_controller_t *gantry,
                   int32_t motor1_pos, int32_t motor2_pos,
                   int32_t *motor1_setpoint, int32_t *motor2_setpoint);

// Check if complete
bool gantry_is_complete(gantry_controller_t *gantry);

// ============================================================================
// Velocity Blending (Smooth Corner Transitions)
// ============================================================================

// Blend two trajectories at a waypoint
typedef struct {
    scurve_profile_t segment1;   // First segment
    scurve_profile_t segment2;   // Second segment
    
    position_t waypoint;         // Corner position
    int32_t blend_radius;        // Blend distance (mdeg or mm)
    float blend_velocity;        // Velocity through corner
    
    bool in_blend;               // Currently in blend zone
    float blend_progress;        // 0.0 to 1.0 through blend
    
    bool initialized;
} velocity_blend_t;

// Initialize velocity blending
void blend_init(velocity_blend_t *blend,
                position_t start, position_t waypoint, position_t end,
                int32_t blend_radius,
                int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Get blended position
position_t blend_get_position(velocity_blend_t *blend);

// Check if past waypoint
bool blend_past_waypoint(velocity_blend_t *blend);

// ============================================================================
// Electronic Gearing (Master-Slave Coordination)
// ============================================================================

// Electronic gearing - one axis follows another with ratio
typedef struct {
    // Master axis trajectory
    scurve_profile_t master_profile;
    
    // Slave ratio (Q16.16 fixed-point)
    int32_t gear_ratio;         // e.g., 2.0 = slave moves 2x master
    
    // Slave offset
    int32_t slave_offset;       // Offset from master (mdeg)
    
    // Current positions
    int32_t master_pos;
    int32_t slave_pos;
    
    bool initialized;
    bool active;
} electronic_gearing_t;

// Initialize electronic gearing
void gearing_init(electronic_gearing_t *gearing,
                  float gear_ratio,
                  int32_t slave_offset);

// Set master trajectory
void gearing_set_master(electronic_gearing_t *gearing,
                        int32_t start, int32_t end,
                        int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start geared motion
void gearing_start(electronic_gearing_t *gearing);

// Get master and slave positions
void gearing_get_positions(electronic_gearing_t *gearing,
                           int32_t *master_pos, int32_t *slave_pos);

// Check if complete
bool gearing_is_complete(electronic_gearing_t *gearing);

// ============================================================================
// Coordinated Motion Planner (G-Code Style)
// ============================================================================

typedef enum {
    MOVE_RAPID,       // G0 - Rapid positioning
    MOVE_LINEAR,      // G1 - Linear interpolation
    MOVE_CW_ARC,      // G2 - Circular interpolation CW
    MOVE_CCW_ARC      // G3 - Circular interpolation CCW
} move_type_t;

// G-code style move command
typedef struct {
    move_type_t type;
    position_t target;
    position_t center;  // For arcs (I, J, K offsets)
    int32_t feed_rate;  // F parameter (mm/min or mdeg/s)
} gcode_move_t;

// Coordinated motion planner
typedef struct {
    // Move queue
    gcode_move_t move_queue[16];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
    
    // Current move
    union {
        linear_interpolation_t linear;
        circular_interpolation_t circular;
        synchronized_motion_t rapid;
    } current_move;
    
    move_type_t current_type;
    
    // Current position
    position_t current_position;
    
    // Default parameters
    int32_t default_feed_rate;
    int32_t rapid_feed_rate;
    int32_t max_accel;
    int32_t max_jerk;
    
    bool initialized;
    bool active;
} motion_planner_t;

// Initialize motion planner
void motion_planner_init(motion_planner_t *planner,
                         int32_t default_feed, int32_t rapid_feed,
                         int32_t max_accel, int32_t max_jerk);

// Add move to queue
bool motion_planner_add_move(motion_planner_t *planner, gcode_move_t move);

// Start executing queued moves
void motion_planner_start(motion_planner_t *planner, position_t start_pos);

// Get current position (all axes)
position_t motion_planner_get_position(motion_planner_t *planner);

// Get current velocity (all axes)
velocity_t motion_planner_get_velocity(motion_planner_t *planner);

// Check if queue empty and current move complete
bool motion_planner_is_idle(motion_planner_t *planner);

// ============================================================================
// Path Optimization
// ============================================================================

// Optimize path for minimum time
void optimize_path_time(gcode_move_t *moves, uint8_t num_moves,
                        int32_t max_vel, int32_t max_accel);

// Optimize path for smoothness
void optimize_path_smoothness(gcode_move_t *moves, uint8_t num_moves,
                              int32_t blend_tolerance);

#endif // _MOTION_PROFILER_DSP_H_
