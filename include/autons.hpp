#pragma once

extern const int DRIVE_SPEED;
extern const int TURN_SPEED;
extern const int SWING_SPEED;

void default_constants();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();

/* ----- Our own stuf ----- */

void awp_auton();
void skills_auton();
void newskills_auton();
void awp_wo_auton();
void annajiwp();
void right_auton();



float weighted_distance(pros::Distance& sensor);
void weighted_dist_reset();
void simple_dist_reset();
void drive_wall_track(double target_distance_in, int speed, pros::Distance& wall_sensor, float target_wall_distance_in);
void intake_high(int voltage);
void intake_middle(int voltage);
void intake_low(int voltage);
void moveto_matchload(int quadrant, int balls);
