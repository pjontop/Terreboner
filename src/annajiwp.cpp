#include "main.h"

void annajiwp() {
    /* ---- FIRST SEQUENCE, START @ LEFT SIDE OF PARKZONE FACING MATCHLOAD ---- */
    forebar.set(true);
    chassis.pid_drive_set(38_in, 127); // drive upto matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-90_deg, 90); // turn towards matchload
    intake.move(127); // start intake
    descore_wing.set(true);
    matchload.set(true); // lower matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(23.5_in, 80); // drive into matchload
    pros::delay(380);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-50_in, 127);
    chassis.pid_wait_quick_chain();
    forebar.set(true);
    matchload.set(false);
    intake.move(127);
    lever.move(127);
    pros::delay(1000);
    lever.move(-127);
    descore_wing.set(false);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(6_in, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(0_deg, 90); // turn towards matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(11_in, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(90_deg, 90); // turn towards matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(40_in, 90);
    chassis.pid_wait();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
}
