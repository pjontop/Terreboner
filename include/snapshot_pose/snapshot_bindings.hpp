#pragma once
#include "snapshot_pose/snapshot_pose.hpp"

namespace snapshot_pose {

// Set the coordinate system offset at the start of autonomous
// This tells the distance sensors where (0,0,0) in your program corresponds to on the actual field
// field_coords = program_coords + offset
void snapshot_set_offset(float field_x_at_program_zero, float field_y_at_program_zero, float field_heading_at_program_zero);

// Simple one-call API for auton.
// If ok==false, pose is not modified.
// If ok==true, pose is applied via chassis.odom_xyt_set(...).
SnapshotResult snapshot_correct_pose();

} // namespace snapshot_pose
