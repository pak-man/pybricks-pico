// SPDX-License-Identifier: MIT
// Multi-axis motion profiling and coordination for RP2350

#include "motion_profiler_dsp.h"
#include <math.h>
#include <string.h>

#ifdef PICO_2W
#include "pico/stdlib.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Helper Functions
// ============================================================================

#define Q16_16(x) ((int32_t)((x) * 65536.0f))
#define Q16_16_TO_FLOAT(x) ((float)(x) / 65536.0f)

// Calculate Euclidean distance between two positions
static int32_t calculate_distance(position_t start, position_t end) {
    int64_t dx = (int64_t)(end.x - start.x);
    int64_t dy = (int64_t)(end.y - start.y);
    int64_t dz = (int64_t)(end.z - start.z);
    
    // Sqrt((dx)^2 + (dy)^2 + (dz)^2) in mdeg or mm
    float dist_f = sqrtf((float)(dx*dx + dy*dy + dz*dz));
    return (int32_t)dist_f;
}

// ============================================================================
// Linear Interpolation
// ============================================================================

void linear_interp_init(linear_interpolation_t *interp,
                        position_t start, position_t end,
                        int32_t max_vel, int32_t max_accel, int32_t max_jerk) {
    memset(interp, 0, sizeof(linear_interpolation_t));
    
    interp->start = start;
    interp->end = end;
    interp->max_velocity = max_vel;
    interp->max_accel = max_accel;
    interp->max_jerk = max_jerk;
    
    // Calculate total distance
    interp->distance = calculate_distance(start, end);
    
    if (interp->distance == 0) {
        interp->initialized = false;
        return;
    }
    
    // Calculate axis scaling factors (how much each axis moves relative to total distance)
    int64_t dx = end.x - start.x;
    int64_t dy = end.y - start.y;
    int64_t dz = end.z - start.z;
    int64_t da = end.a - start.a;
    
    interp->axis_scale[AXIS_X] = (int32_t)((dx << 16) / interp->distance);
    interp->axis_scale[AXIS_Y] = (int32_t)((dy << 16) / interp->distance);
    interp->axis_scale[AXIS_Z] = (int32_t)((dz << 16) / interp->distance);
    interp->axis_scale[AXIS_A] = (int32_t)((da << 16) / interp->distance);
    
    // Create master S-curve profile along the path
    scurve_init(&interp->master_profile,
                0, interp->distance,  // Move from 0 to total_distance
                max_vel, max_accel, max_jerk);
    
    interp->initialized = true;
}

void linear_interp_start(linear_interpolation_t *interp) {
    if (!interp->initialized) return;
    
    scurve_start(&interp->master_profile);
    interp->progress = 0.0f;
    interp->current_pos = interp->start;
    interp->active = true;
}

position_t linear_interp_get_position(linear_interpolation_t *interp) {
    if (!interp->active) {
        return interp->end;
    }
    
    // Get master trajectory point
    trajectory_point_t master = scurve_get_point(&interp->master_profile);
    
    // Calculate progress (0.0 to 1.0)
    if (interp->distance > 0) {
        interp->progress = (float)master.position / (float)interp->distance;
    }
    
    // Scale to each axis
    position_t pos;
    pos.x = interp->start.x + (int32_t)(((int64_t)interp->axis_scale[AXIS_X] * master.position) >> 16);
    pos.y = interp->start.y + (int32_t)(((int64_t)interp->axis_scale[AXIS_Y] * master.position) >> 16);
    pos.z = interp->start.z + (int32_t)(((int64_t)interp->axis_scale[AXIS_Z] * master.position) >> 16);
    pos.a = interp->start.a + (int32_t)(((int64_t)interp->axis_scale[AXIS_A] * master.position) >> 16);
    
    interp->current_pos = pos;
    
    // Check if complete
    if (scurve_is_complete(&interp->master_profile)) {
        interp->active = false;
        return interp->end;
    }
    
    return pos;
}

velocity_t linear_interp_get_velocity(linear_interpolation_t *interp) {
    velocity_t vel = {0};
    
    if (!interp->active) return vel;
    
    trajectory_point_t master = scurve_get_point(&interp->master_profile);
    
    // Scale master velocity to each axis
    vel.x = (int32_t)(((int64_t)interp->axis_scale[AXIS_X] * master.velocity) >> 16);
    vel.y = (int32_t)(((int64_t)interp->axis_scale[AXIS_Y] * master.velocity) >> 16);
    vel.z = (int32_t)(((int64_t)interp->axis_scale[AXIS_Z] * master.velocity) >> 16);
    vel.a = (int32_t)(((int64_t)interp->axis_scale[AXIS_A] * master.velocity) >> 16);
    
    interp->current_vel = vel;
    return vel;
}

bool linear_interp_is_complete(linear_interpolation_t *interp) {
    return !interp->active;
}

// ============================================================================
// Circular Interpolation
// ============================================================================

void circular_interp_init(circular_interpolation_t *interp,
                          position_t start, position_t end, position_t center,
                          circular_direction_t direction,
                          axis_id_t plane_axis1, axis_id_t plane_axis2,
                          int32_t max_vel, int32_t max_accel, int32_t max_jerk) {
    memset(interp, 0, sizeof(circular_interpolation_t));
    
    interp->start = start;
    interp->end = end;
    interp->center = center;
    interp->direction = direction;
    interp->plane_axis1 = plane_axis1;
    interp->plane_axis2 = plane_axis2;
    interp->max_velocity = max_vel;
    interp->max_accel = max_accel;
    interp->max_jerk = max_jerk;
    
    // Get coordinates in the arc plane
    int32_t start_p1, start_p2, end_p1, end_p2, center_p1, center_p2;
    
    if (plane_axis1 == AXIS_X && plane_axis2 == AXIS_Y) {
        start_p1 = start.x; start_p2 = start.y;
        end_p1 = end.x; end_p2 = end.y;
        center_p1 = center.x; center_p2 = center.y;
    } else if (plane_axis1 == AXIS_Y && plane_axis2 == AXIS_Z) {
        start_p1 = start.y; start_p2 = start.z;
        end_p1 = end.y; end_p2 = end.z;
        center_p1 = center.y; center_p2 = center.z;
    } else {  // X-Z plane
        start_p1 = start.x; start_p2 = start.z;
        end_p1 = end.x; end_p2 = end.z;
        center_p1 = center.x; center_p2 = center.z;
    }
    
    // Calculate radius
    int64_t dx = start_p1 - center_p1;
    int64_t dy = start_p2 - center_p2;
    interp->radius = (int32_t)sqrtf((float)(dx*dx + dy*dy));
    
    // Calculate start and end angles
    interp->start_angle = atan2f((float)(start_p2 - center_p2), 
                                 (float)(start_p1 - center_p1));
    interp->end_angle = atan2f((float)(end_p2 - center_p2), 
                               (float)(end_p1 - center_p1));
    
    // Calculate total angle (handle wraparound)
    if (direction == CIRCULAR_CW) {
        if (interp->end_angle > interp->start_angle) {
            interp->total_angle = interp->end_angle - interp->start_angle - 2*M_PI;
        } else {
            interp->total_angle = interp->end_angle - interp->start_angle;
        }
    } else {  // CCW
        if (interp->end_angle < interp->start_angle) {
            interp->total_angle = interp->end_angle - interp->start_angle + 2*M_PI;
        } else {
            interp->total_angle = interp->end_angle - interp->start_angle;
        }
    }
    
    // Calculate arc length
    interp->arc_length = (int32_t)(fabsf(interp->total_angle) * interp->radius);
    
    // Create master S-curve profile along arc
    scurve_init(&interp->master_profile,
                0, interp->arc_length,
                max_vel, max_accel, max_jerk);
    
    interp->initialized = true;
}

void circular_interp_start(circular_interpolation_t *interp) {
    if (!interp->initialized) return;
    
    scurve_start(&interp->master_profile);
    interp->progress = 0.0f;
    interp->current_pos = interp->start;
    interp->active = true;
}

position_t circular_interp_get_position(circular_interpolation_t *interp) {
    if (!interp->active) {
        return interp->end;
    }
    
    trajectory_point_t master = scurve_get_point(&interp->master_profile);
    
    // Calculate current angle
    if (interp->arc_length > 0) {
        interp->progress = (float)master.position / (float)interp->arc_length;
    }
    
    float current_angle = interp->start_angle + interp->progress * interp->total_angle;
    
    // Calculate position on arc
    int32_t p1 = interp->center.x + (int32_t)(interp->radius * cosf(current_angle));
    int32_t p2 = interp->center.y + (int32_t)(interp->radius * sinf(current_angle));
    
    position_t pos = interp->start;
    
    if (interp->plane_axis1 == AXIS_X && interp->plane_axis2 == AXIS_Y) {
        pos.x = p1;
        pos.y = p2;
    } else if (interp->plane_axis1 == AXIS_Y && interp->plane_axis2 == AXIS_Z) {
        pos.y = p1;
        pos.z = p2;
    } else {
        pos.x = p1;
        pos.z = p2;
    }
    
    interp->current_pos = pos;
    
    if (scurve_is_complete(&interp->master_profile)) {
        interp->active = false;
        return interp->end;
    }
    
    return pos;
}

velocity_t circular_interp_get_velocity(circular_interpolation_t *interp) {
    velocity_t vel = {0};
    
    if (!interp->active) return vel;
    
    trajectory_point_t master = scurve_get_point(&interp->master_profile);
    
    // Tangential velocity = master velocity
    // Direction is perpendicular to radius
    float current_angle = interp->start_angle + interp->progress * interp->total_angle;
    
    float tangent_angle = current_angle + M_PI / 2.0f;
    if (interp->direction == CIRCULAR_CW) {
        tangent_angle = current_angle - M_PI / 2.0f;
    }
    
    int32_t v_tangent = master.velocity;
    
    int32_t v1 = (int32_t)(v_tangent * cosf(tangent_angle));
    int32_t v2 = (int32_t)(v_tangent * sinf(tangent_angle));
    
    if (interp->plane_axis1 == AXIS_X && interp->plane_axis2 == AXIS_Y) {
        vel.x = v1;
        vel.y = v2;
    } else if (interp->plane_axis1 == AXIS_Y && interp->plane_axis2 == AXIS_Z) {
        vel.y = v1;
        vel.z = v2;
    } else {
        vel.x = v1;
        vel.z = v2;
    }
    
    interp->current_vel = vel;
    return vel;
}

bool circular_interp_is_complete(circular_interpolation_t *interp) {
    return !interp->active;
}

// ============================================================================
// Synchronized Multi-Axis Motion
// ============================================================================

void sync_motion_init(synchronized_motion_t *sync, uint8_t num_axes) {
    memset(sync, 0, sizeof(synchronized_motion_t));
    sync->num_axes = (num_axes > MAX_AXES) ? MAX_AXES : num_axes;
    sync->synchronized = true;
    sync->initialized = true;
}

void sync_motion_set_axis_target(synchronized_motion_t *sync,
                                 axis_id_t axis,
                                 int32_t start, int32_t end,
                                 int32_t max_vel, int32_t max_accel, int32_t max_jerk) {
    if (axis >= sync->num_axes) return;
    
    scurve_init(&sync->axis_profile[axis],
                start, end, max_vel, max_accel, max_jerk);
}

void sync_motion_start(synchronized_motion_t *sync) {
    if (!sync->initialized) return;
    
    // Start all axis trajectories
    for (uint8_t i = 0; i < sync->num_axes; i++) {
        scurve_start(&sync->axis_profile[i]);
    }
    
    sync->master_progress = 0.0f;
    sync->active = true;
}

int32_t sync_motion_get_position(synchronized_motion_t *sync, axis_id_t axis) {
    if (axis >= sync->num_axes || !sync->active) {
        return sync->current_pos[axis];
    }
    
    trajectory_point_t point = scurve_get_point(&sync->axis_profile[axis]);
    sync->current_pos[axis] = point.position;
    sync->current_vel[axis] = point.velocity;
    
    return point.position;
}

int32_t sync_motion_get_velocity(synchronized_motion_t *sync, axis_id_t axis) {
    if (axis >= sync->num_axes) return 0;
    return sync->current_vel[axis];
}

bool sync_motion_is_complete(synchronized_motion_t *sync) {
    if (!sync->active) return true;
    
    // Check if all axes complete
    for (uint8_t i = 0; i < sync->num_axes; i++) {
        if (!scurve_is_complete(&sync->axis_profile[i])) {
            return false;
        }
    }
    
    sync->active = false;
    return true;
}

// ============================================================================
// Gantry Control
// ============================================================================

void gantry_init(gantry_controller_t *gantry,
                 int32_t coupling_gain,
                 int32_t max_correction) {
    memset(gantry, 0, sizeof(gantry_controller_t));
    gantry->coupling_gain = coupling_gain;
    gantry->max_correction = max_correction;
    gantry->initialized = true;
}

void gantry_set_target(gantry_controller_t *gantry,
                       int32_t start, int32_t end,
                       int32_t max_vel, int32_t max_accel, int32_t max_jerk) {
    scurve_init(&gantry->target_profile,
                start, end, max_vel, max_accel, max_jerk);
}

void gantry_start(gantry_controller_t *gantry) {
    if (!gantry->initialized) return;
    scurve_start(&gantry->target_profile);
    gantry->active = true;
}

void gantry_update(gantry_controller_t *gantry,
                   int32_t motor1_pos, int32_t motor2_pos,
                   int32_t *motor1_setpoint, int32_t *motor2_setpoint) {
    if (!gantry->active) {
        *motor1_setpoint = motor1_pos;
        *motor2_setpoint = motor2_pos;
        return;
    }
    
    // Get target position
    trajectory_point_t target = scurve_get_point(&gantry->target_profile);
    
    // Calculate position error between motors (squareness error)
    gantry->position_error = motor1_pos - motor2_pos;
    
    // Cross-coupling correction
    // If motor1 ahead: slow motor1, speed motor2
    // If motor2 ahead: speed motor1, slow motor2
    int64_t correction = ((int64_t)gantry->coupling_gain * gantry->position_error) >> 16;
    
    // Limit correction
    if (correction > gantry->max_correction) correction = gantry->max_correction;
    if (correction < -gantry->max_correction) correction = -gantry->max_correction;
    
    // Apply correction symmetrically
    *motor1_setpoint = target.position - (int32_t)(correction / 2);
    *motor2_setpoint = target.position + (int32_t)(correction / 2);
    
    gantry->motor1_pos = motor1_pos;
    gantry->motor2_pos = motor2_pos;
}

bool gantry_is_complete(gantry_controller_t *gantry) {
    if (!gantry->active) return true;
    
    if (scurve_is_complete(&gantry->target_profile)) {
        gantry->active = false;
        return true;
    }
    
    return false;
}

// ============================================================================
// Electronic Gearing
// ============================================================================

void gearing_init(electronic_gearing_t *gearing,
                  float gear_ratio,
                  int32_t slave_offset) {
    memset(gearing, 0, sizeof(electronic_gearing_t));
    gearing->gear_ratio = Q16_16(gear_ratio);
    gearing->slave_offset = slave_offset;
    gearing->initialized = true;
}

void gearing_set_master(electronic_gearing_t *gearing,
                        int32_t start, int32_t end,
                        int32_t max_vel, int32_t max_accel, int32_t max_jerk) {
    scurve_init(&gearing->master_profile,
                start, end, max_vel, max_accel, max_jerk);
}

void gearing_start(electronic_gearing_t *gearing) {
    if (!gearing->initialized) return;
    scurve_start(&gearing->master_profile);
    gearing->active = true;
}

void gearing_get_positions(electronic_gearing_t *gearing,
                           int32_t *master_pos, int32_t *slave_pos) {
    if (!gearing->active) {
        *master_pos = gearing->master_pos;
        *slave_pos = gearing->slave_pos;
        return;
    }
    
    trajectory_point_t master = scurve_get_point(&gearing->master_profile);
    
    gearing->master_pos = master.position;
    
    // Slave = (master * ratio) + offset
    gearing->slave_pos = (int32_t)(((int64_t)master.position * gearing->gear_ratio) >> 16) 
                        + gearing->slave_offset;
    
    *master_pos = gearing->master_pos;
    *slave_pos = gearing->slave_pos;
}

bool gearing_is_complete(electronic_gearing_t *gearing) {
    if (!gearing->active) return true;
    
    if (scurve_is_complete(&gearing->master_profile)) {
        gearing->active = false;
        return true;
    }
    
    return false;
}

// ============================================================================
// Motion Planner (Simplified Implementation)
// ============================================================================

void motion_planner_init(motion_planner_t *planner,
                         int32_t default_feed, int32_t rapid_feed,
                         int32_t max_accel, int32_t max_jerk) {
    memset(planner, 0, sizeof(motion_planner_t));
    planner->default_feed_rate = default_feed;
    planner->rapid_feed_rate = rapid_feed;
    planner->max_accel = max_accel;
    planner->max_jerk = max_jerk;
    planner->initialized = true;
}

bool motion_planner_add_move(motion_planner_t *planner, gcode_move_t move) {
    if (planner->queue_count >= 16) return false;
    
    planner->move_queue[planner->queue_tail] = move;
    planner->queue_tail = (planner->queue_tail + 1) % 16;
    planner->queue_count++;
    
    return true;
}

void motion_planner_start(motion_planner_t *planner, position_t start_pos) {
    if (!planner->initialized || planner->queue_count == 0) return;
    
    planner->current_position = start_pos;
    planner->active = true;
    
    // Start first move
    // Implementation simplified - would execute first queued move
}

position_t motion_planner_get_position(motion_planner_t *planner) {
    // Simplified - return current position
    return planner->current_position;
}

velocity_t motion_planner_get_velocity(motion_planner_t *planner) {
    velocity_t vel = {0};
    // Simplified
    return vel;
}

bool motion_planner_is_idle(motion_planner_t *planner) {
    return (planner->queue_count == 0 && !planner->active);
}
