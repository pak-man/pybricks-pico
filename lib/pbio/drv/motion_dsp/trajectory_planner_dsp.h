// SPDX-License-Identifier: MIT
// DSP-accelerated trajectory planning for RP2350

#ifndef _TRAJECTORY_PLANNER_DSP_H_
#define _TRAJECTORY_PLANNER_DSP_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Trajectory Point (Position, Velocity, Acceleration, Jerk)
// ============================================================================

typedef struct {
    int32_t position;      // Position (mdeg)
    int32_t velocity;      // Velocity (mdeg/s)
    int32_t acceleration;  // Acceleration (mdeg/s²)
    int32_t jerk;          // Jerk (mdeg/s³)
    uint32_t timestamp_us; // Timestamp (microseconds)
} trajectory_point_t;

// ============================================================================
// S-Curve Profile (7-Segment Jerk-Limited Motion)
// ============================================================================

// S-curve motion profile phases
typedef enum {
    SCURVE_PHASE_ACCEL_JERK_POS,    // Phase 1: Increasing acceleration
    SCURVE_PHASE_ACCEL_CONST,       // Phase 2: Constant acceleration
    SCURVE_PHASE_ACCEL_JERK_NEG,    // Phase 3: Decreasing acceleration
    SCURVE_PHASE_CRUISE,            // Phase 4: Constant velocity
    SCURVE_PHASE_DECEL_JERK_NEG,    // Phase 5: Increasing deceleration
    SCURVE_PHASE_DECEL_CONST,       // Phase 6: Constant deceleration
    SCURVE_PHASE_DECEL_JERK_POS,    // Phase 7: Decreasing deceleration
    SCURVE_PHASE_COMPLETE           // Motion complete
} scurve_phase_t;

// S-curve trajectory parameters
typedef struct {
    // Motion constraints
    int32_t max_velocity;      // Maximum velocity (mdeg/s)
    int32_t max_acceleration;  // Maximum acceleration (mdeg/s²)
    int32_t max_jerk;          // Maximum jerk (mdeg/s³)
    
    // Start and end conditions
    int32_t start_position;    // Starting position (mdeg)
    int32_t end_position;      // Target position (mdeg)
    int32_t start_velocity;    // Starting velocity (mdeg/s)
    int32_t end_velocity;      // Ending velocity (mdeg/s)
    
    // Phase timing (calculated)
    float t1, t2, t3, t4, t5, t6, t7;  // Phase durations (seconds)
    float total_time;                   // Total move time (seconds)
    
    // Current state
    scurve_phase_t current_phase;
    float elapsed_time;        // Time since start (seconds)
    uint32_t start_time_us;    // Start timestamp
    
    // Intermediate values (pre-calculated for speed)
    int32_t cruise_velocity;   // Actual cruise velocity achieved
    int32_t accel_reached;     // Maximum acceleration reached
    
    bool initialized;
    bool active;
} scurve_profile_t;

// Initialize S-curve profile
// distance: target distance (mdeg)
// max_vel, max_accel, max_jerk: motion constraints
void scurve_init(scurve_profile_t *profile,
                 int32_t start_pos, int32_t end_pos,
                 int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Initialize with non-zero start/end velocities
void scurve_init_with_velocity(scurve_profile_t *profile,
                                int32_t start_pos, int32_t start_vel,
                                int32_t end_pos, int32_t end_vel,
                                int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Start trajectory execution
void scurve_start(scurve_profile_t *profile);

// Get trajectory point at current time
trajectory_point_t scurve_get_point(scurve_profile_t *profile);

// Check if trajectory is complete
bool scurve_is_complete(scurve_profile_t *profile);

// Reset trajectory
void scurve_reset(scurve_profile_t *profile);

// ============================================================================
// Polynomial Trajectory (Quintic/Cubic Splines)
// ============================================================================

// Polynomial trajectory segment
typedef struct {
    // Polynomial coefficients: p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // Using Q16.16 fixed-point for DSP efficiency
    int32_t a0, a1, a2, a3, a4, a5;
    
    // Time range for this segment
    float t_start;   // Segment start time (seconds)
    float t_end;     // Segment end time (seconds)
    
    // Boundary conditions (for validation)
    int32_t pos_start, pos_end;
    int32_t vel_start, vel_end;
    int32_t acc_start, acc_end;
} polynomial_segment_t;

// Multi-segment polynomial trajectory
typedef struct {
    polynomial_segment_t segments[16];  // Up to 16 segments
    uint8_t num_segments;
    
    uint8_t current_segment;
    float elapsed_time;
    uint32_t start_time_us;
    
    bool initialized;
    bool active;
} polynomial_trajectory_t;

// Initialize quintic (5th order) polynomial segment
// Satisfies: position, velocity, acceleration at start and end
void polynomial_init_quintic(polynomial_segment_t *seg,
                              float duration,
                              int32_t p0, int32_t v0, int32_t a0,  // Start conditions
                              int32_t p1, int32_t v1, int32_t a1); // End conditions

// Initialize cubic (3rd order) polynomial segment
// Satisfies: position and velocity at start and end
void polynomial_init_cubic(polynomial_segment_t *seg,
                            float duration,
                            int32_t p0, int32_t v0,  // Start conditions
                            int32_t p1, int32_t v1); // End conditions

// Evaluate polynomial at time t (DSP-accelerated)
trajectory_point_t polynomial_evaluate(polynomial_segment_t *seg, float t);

// Add segment to multi-segment trajectory
void polynomial_trajectory_add_segment(polynomial_trajectory_t *traj, polynomial_segment_t *seg);

// Start multi-segment trajectory
void polynomial_trajectory_start(polynomial_trajectory_t *traj);

// Get current trajectory point
trajectory_point_t polynomial_trajectory_get_point(polynomial_trajectory_t *traj);

// Check if complete
bool polynomial_trajectory_is_complete(polynomial_trajectory_t *traj);

// ============================================================================
// Trapezoidal Profile (Simple Alternative to S-Curve)
// ============================================================================

typedef enum {
    TRAP_PHASE_ACCEL,
    TRAP_PHASE_CRUISE,
    TRAP_PHASE_DECEL,
    TRAP_PHASE_COMPLETE
} trap_phase_t;

typedef struct {
    int32_t max_velocity;
    int32_t max_acceleration;
    
    int32_t start_position;
    int32_t end_position;
    
    float t_accel;   // Acceleration time
    float t_cruise;  // Cruise time
    float t_decel;   // Deceleration time
    float total_time;
    
    int32_t cruise_velocity;
    
    trap_phase_t current_phase;
    float elapsed_time;
    uint32_t start_time_us;
    
    bool initialized;
    bool active;
} trapezoidal_profile_t;

// Initialize trapezoidal profile
void trap_init(trapezoidal_profile_t *profile,
               int32_t start_pos, int32_t end_pos,
               int32_t max_vel, int32_t max_accel);

// Start trajectory
void trap_start(trapezoidal_profile_t *profile);

// Get trajectory point
trajectory_point_t trap_get_point(trapezoidal_profile_t *profile);

// Check if complete
bool trap_is_complete(trapezoidal_profile_t *profile);

// ============================================================================
// Path Planning (Multi-Point Waypoint Following)
// ============================================================================

// Waypoint for path planning
typedef struct {
    int32_t position;      // Target position (mdeg)
    int32_t velocity;      // Desired velocity at waypoint (0 = stop)
    float blend_radius;    // Blending radius for smooth corners (0 = sharp corner)
} waypoint_t;

// Path planner with multiple waypoints
typedef struct {
    waypoint_t waypoints[32];  // Up to 32 waypoints
    uint8_t num_waypoints;
    uint8_t current_waypoint;
    
    // Motion constraints
    int32_t max_velocity;
    int32_t max_acceleration;
    int32_t max_jerk;
    
    // Current trajectory segment
    scurve_profile_t current_segment;
    
    bool initialized;
    bool active;
} path_planner_t;

// Initialize path planner
void path_planner_init(path_planner_t *planner,
                       int32_t max_vel, int32_t max_accel, int32_t max_jerk);

// Add waypoint
void path_planner_add_waypoint(path_planner_t *planner, waypoint_t waypoint);

// Start path execution
void path_planner_start(path_planner_t *planner, int32_t current_position);

// Get current trajectory point
trajectory_point_t path_planner_get_point(path_planner_t *planner);

// Check if path is complete
bool path_planner_is_complete(path_planner_t *planner);

// ============================================================================
// Look-Ahead Trajectory Optimization
// ============================================================================

// Optimized trajectory with velocity profiling
typedef struct {
    scurve_profile_t base_profile;
    
    // Look-ahead optimization
    float lookahead_time;     // How far to look ahead (seconds)
    int32_t corner_velocity;  // Velocity limit for corners
    
    bool use_optimization;
} optimized_trajectory_t;

// Initialize with look-ahead optimization
void optimized_trajectory_init(optimized_trajectory_t *traj,
                               int32_t start_pos, int32_t end_pos,
                               int32_t max_vel, int32_t max_accel, int32_t max_jerk,
                               float lookahead_time);

// Get optimized trajectory point
trajectory_point_t optimized_trajectory_get_point(optimized_trajectory_t *traj);

#endif // _TRAJECTORY_PLANNER_DSP_H_
