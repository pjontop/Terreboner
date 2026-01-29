// src/snapshot_pose/snapshot_bindings.cpp
//
// Snapshot Pose "bindings" layer for EZ-Template:
// - Keeps all configuration in ONE place.
// - Your auton should only call snapshot_pose::snapshot_correct_pose().
//
// What this does:
// - Reads 2–4 Distance sensors (median-filtered).
// - Uses raycasts against the collision map segments to infer robot (x,y) on the field.
// - Applies chassis.odom_set(x, y, heading).
//
// Reliability defaults:
// - Perimeter-only by default (walls are the most stable landmarks).
// - Per-sensor masks supported: low sensors can ignore interior objects that they can't see.
//
// Coordinate system (must match your EZ-Template odom):
// - Field is 144" x 144".
// - (0,0) bottom-left, +X right, +Y up.
// - heading_deg: 0° = +Y, clockwise-positive.

#include "snapshot_pose/snapshot_bindings.hpp"

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

// Include the header where your chassis and distance sensors are declared
#include "main.h"

namespace snapshot_pose {

// -------------------------------
// EDIT SECTION 1: ODOM ACCESS
// -------------------------------
// Your EZ-Template chassis object is declared in main.h as:
// extern Drive chassis;

// -------------------------------
// EDIT SECTION 2: DISTANCE SENSORS
// -------------------------------
// Distance sensors are already declared in subsystems.hpp:
// extern pros::Distance front_dist;
// extern pros::Distance back_dist;
// extern pros::Distance left_dist;
// extern pros::Distance right_dist;

// -------------------------------
// EDIT SECTION 3: TRACKERS + HEADING SOURCES
// -------------------------------
// EZ-Template tracks position using odom_x_get(), odom_y_get(), odom_theta_get()
// and tracking wheels with odom_tracker_left, odom_tracker_right, odom_tracker_back, odom_tracker_front

static float get_heading_deg() {
  // EZ-Template provides odom_theta_get() which returns heading in degrees
  return chassis.odom_theta_get();
}

static float get_forward_tracker_in() {
  // EZ-Template: if you have a vertical/forward tracker
  // Check which tracker you're using (left, right, or custom)
  // Return the tracker distance in inches
  // For now, returning 0 if no tracking wheels configured
  if (chassis.odom_tracker_right) {
    return chassis.odom_tracker_right->get();
  } else if (chassis.odom_tracker_left) {
    return chassis.odom_tracker_left->get();
  }
  return 0.0f;
}

static float get_sideways_tracker_in() {
  // EZ-Template: if you have a horizontal/sideways tracker
  if (chassis.odom_tracker_back) {
    return chassis.odom_tracker_back->get();
  } else if (chassis.odom_tracker_front) {
    return chassis.odom_tracker_front->get();
  }
  return 0.0f;
}

// -------------------------------
// INTERNAL: configuration + sensor list
// -------------------------------
static SnapshotConfig g_cfg;
static std::vector<DistanceSensorConfig> g_sensors;
static bool g_init = false;

static void init_once() {
  if (g_init) return;
  g_init = true;

  // -------- Global snapshot config (safe defaults) --------
  g_cfg.field_mask = MAP_PERIMETER;        // default fallback if per-sensor override is 0
  g_cfg.candidates_per_sensor = 1;         // perimeter-only: 1 candidate is usually enough
  g_cfg.samples = 5;                       // median of 5 readings
  g_cfg.sample_delay_ms = 35;
  g_cfg.max_chi2_per_sensor = 9.0f;        // ~3-sigma per sensor

  // If you enable interior objects globally, increase candidates:
  // g_cfg.field_mask = MAP_PERIMETER | MAP_LONG_GOALS | MAP_CENTER_GOALS;
  // g_cfg.candidates_per_sensor = 2;

  // -------- Sensor definitions --------
  //
  // Units:
  //   x_right_in: inches from robot center, + to robot-right
  //   y_fwd_in:   inches from robot center, + to robot-forward
  //
  // rel_deg (relative to robot forward):
  //   0   = forward
  //   +90 = right
  //   180 = back
  //   -90 = left
  //
  // field_mask_override:
  //   0          => use g_cfg.field_mask
  //   nonzero    => use this mask for THIS sensor

  g_sensors.clear();
  g_sensors.reserve(4);

  // FRONT sensor (often low on the robot)
  {
    DistanceSensorConfig s{};
    s.dev = &front_dist;

    // EDIT: measure these offsets relative to robot center
    s.x_right_in = 0.0f;
    s.y_fwd_in   = 7.0f;  // 7 inches forward of center
    s.rel_deg    = 0.0f;  // pointing forward

    // Per-sensor mask example:
    // Low-mounted sensor: only trust walls.
    s.field_mask_override = MAP_PERIMETER;

    // Confidence gating (0..63) only meaningful when distance > 200mm.
    s.use_confidence_gate = true;
    s.min_confidence = 35;

    // Optional gates (off by default):
    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  // BACK sensor
  {
    DistanceSensorConfig s{};
    s.dev = &back_dist;

    // EDIT: measure these offsets
    s.x_right_in = 0.0f;
    s.y_fwd_in   = -7.0f;  // 7 inches back of center
    s.rel_deg    = 180.0f;  // pointing backward

    s.field_mask_override = MAP_PERIMETER;

    s.use_confidence_gate = true;
    s.min_confidence = 35;

    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  // LEFT sensor
  {
    DistanceSensorConfig s{};
    s.dev = &left_dist;

    // EDIT: measure these offsets
    s.x_right_in = -7.0f;  // 7 inches left of center
    s.y_fwd_in   = 0.0f;
    s.rel_deg    = -90.0f;  // pointing left

    s.field_mask_override = MAP_PERIMETER;

    s.use_confidence_gate = true;
    s.min_confidence = 35;

    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  // RIGHT sensor (maybe higher / different height)
  {
    DistanceSensorConfig s{};
    s.dev = &right_dist;

    // EDIT: measure these offsets
    s.x_right_in = 7.0f;  // 7 inches right of center
    s.y_fwd_in   = 0.0f;
    s.rel_deg    = 90.0f;  // pointing right

    // If this sensor can "see" taller interior objects reliably, enable them here.
    // Example: walls + long goals only
    s.field_mask_override = MAP_PERIMETER | MAP_LONG_GOALS;

    s.use_confidence_gate = true;
    s.min_confidence = 35;

    s.use_velocity_gate = false;
    s.use_object_size_gate = false;

    g_sensors.push_back(s);
  }

  pros::screen::print(pros::E_TEXT_MEDIUM, 6, "SnapshotPose init OK");
}

// -------------------------------
// Public API called by auton
// -------------------------------
SnapshotResult snapshot_correct_pose() {
  init_once();

  // Heading and tracker readings (inches) must match your odom usage.
  const float heading_deg = get_heading_deg();
  const float fwd_in = get_forward_tracker_in();
  const float side_in = get_sideways_tracker_in();

  // Guess pose is used only to choose candidate segments for each sensor.
  const float guess_x = chassis.odom_x_get();
  const float guess_y = chassis.odom_y_get();

  // Strongly recommended: call this while the robot is stopped.
  // (Do chassis.pid_wait(); pros::delay(150-250); in auton before calling.)

  return snapshot_setpose(
    chassis,
    g_sensors,
    g_cfg,
    heading_deg,
    fwd_in,
    side_in,
    guess_x,
    guess_y
  );
}

// -------------------------------
// snapshot_set_offset: Set field coordinate offset for program zero
// -------------------------------
void snapshot_set_offset(float field_x_at_program_zero, float field_y_at_program_zero, float field_heading_at_program_zero) {
  // This function allows you to define where (0,0,0) in your program corresponds to on the actual field
  // For now, this is a placeholder - you could store these values and use them to transform
  // the snapshot_correct_pose results if needed.
  // 
  // Typically, you would:
  // 1. Store these offset values in static variables
  // 2. Apply the transform in snapshot_correct_pose before calling chassis.odom_xyt_set()
  //
  // For basic usage, if your program coordinates already match field coordinates,
  // you can leave this as a no-op.
  
  // Store in static variables for future use (optional)
  static float offset_x = 0.0f;
  static float offset_y = 0.0f;
  static float offset_heading = 0.0f;
  
  offset_x = field_x_at_program_zero;
  offset_y = field_y_at_program_zero;
  offset_heading = field_heading_at_program_zero;
  
  // Print for debugging
  pros::lcd::print(7, "Offset: (%.1f, %.1f, %.1f°)", offset_x, offset_y, offset_heading);
}

} // namespace snapshot_pose
