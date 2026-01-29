#include "main.h"

// Distance sensors for pose correction
pros::Distance front_dist(3);  // Front distance sensor on port 3
pros::Distance back_dist(8);    // Back distance sensor on port 8
pros::Distance left_dist(20);   // Left distance sensor on port 20
pros::Distance right_dist(8);  // Right distance sensor on port 18

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(17.0, 0.0, 90);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 70.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(4.5, 0.0, 70.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(4.5, 0.0, 50.0);  // Angular control for boomerang motions

  // Exit conditions (relaxed failsafes)
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 750_ms, 750_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 750_ms, 1000_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 750_ms, 1000_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.75);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.5);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}
/**
 * Drive while tracking a wall to maintain straightness
 * @param target_distance_in - How far to drive forward (inches)
 * @param speed - Drive speed (0-127)
 * @param wall_sensor - Distance sensor facing the wall
 * @param target_wall_distance_in - Desired distance from wall (inches)
 */
void drive_wall_track(double target_distance_in, int speed, pros::Distance& wall_sensor, float target_wall_distance_in) {
    const float kP = 8.0;  // Proportional gain for steering correction
    const float tolerance_in = 0.5;  // Acceptable distance error from wall
    const int update_delay_ms = 20;  // How often to check and correct
    
    // Get starting position
    double start_position = chassis.drive_sensor_right();
    double target_position = start_position + target_distance_in;
    
    pros::lcd::print(5, "Wall tracking started");
    
    while (fabs(chassis.drive_sensor_right() - target_position) > 1.0) {
        // Get current wall distance
        float current_wall_dist = wall_sensor.get() * 0.0394;  // Convert mm to inches
        float wall_error = current_wall_dist - target_wall_distance_in;
        
        // Calculate steering correction (positive error = too far, turn toward wall)
        float steering_correction = wall_error * kP;
        
        // Limit steering correction to reasonable values
        if (steering_correction > 30) steering_correction = 30;
        if (steering_correction < -30) steering_correction = -30;
        
        // Apply drive with steering correction
        // If too far from wall (positive error), increase right side speed (turn left toward wall)
        // If too close to wall (negative error), increase left side speed (turn right away from wall)
        int left_speed = speed + steering_correction;
        int right_speed = speed - steering_correction;
        
        // Clamp speeds
        if (left_speed > 127) left_speed = 127;
        if (left_speed < -127) left_speed = -127;
        if (right_speed > 127) right_speed = 127;
        if (right_speed < -127) right_speed = -127;
        
        // Set motor voltages
        chassis.drive_set(left_speed, right_speed);
        
        // Debug output
        pros::lcd::print(6, "Wall: %.1f\" Err: %.2f Corr: %.1f", current_wall_dist, wall_error, steering_correction);
        
        pros::delay(update_delay_ms);
    }
    
    // Stop motors
    chassis.drive_set(0, 0);
    pros::lcd::print(5, "Wall tracking complete");
    pros::delay(50);
}