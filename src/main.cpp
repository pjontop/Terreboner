#include "main.h"

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-19, -10, -9},     // Left Chassis Ports (negative port will reverse it!)
    {11, 6, 12},  // Right Chassis Ports (negative port will reverse it!)
    13,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);   // Wheel RPM = cartridge * (motor gear / wheel gear)

  ez::tracking_wheel horiz_tracker(8, 2.125, -1.57);  // This tracking wheel is perpendicular to the drive wheels
  ez::tracking_wheel vert_tracker(18, 2.125, 0);   // This tracking wheel is parallel to the drive wheels

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
      {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      {"Combine all 3 movements", combining_movements},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

void disabled() {
  // . . .
}


void competition_initialize() {
  // . . .
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}


void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}


void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);


void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}


void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  // chassis.opcontrol_tank();  // Tank control
  while (true) {
    chassis.opcontrol_tank();
    
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      outake.move(127);
    } else {
      intake.move(0);
      outake.move(0);
    }

    // Outtake Control (R2 = forward while held, reverse briefly on release)
    static bool r2_was_pressed = false;
    static int r2_release_timer = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      lever.move(127);
      r2_was_pressed = true;
      r2_release_timer = 0;
    } else if (r2_was_pressed && r2_release_timer < 15) {
      lever.move(-127);
      r2_release_timer++;
    } else {
      lever.move(0);
      r2_was_pressed = false;
    }

    // Descore wing toggle (R1)
    descore_wing.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1));
    // Pneumatic Controls (toggle - hold state)
    matchload.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP));    // D-Pad Up: Toggle dropper
    forebar.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));      // D-Pad Down: Toggle arm

    // Manual outtake control (LEFT = reverse, RIGHT = forward)
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      outake.move(-127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      outake.move(127);
    }

    // EZ-Template extras (PID tuner, test auton)
    ez_template_extras();

    pros::delay(ez::util::DELAY_TIME);
  }
}

