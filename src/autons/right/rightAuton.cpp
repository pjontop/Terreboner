#include "main.h"

void right_auton() {
    intake.move(127);
    forebar.set(true);
    chassis.pid_drive_set(38_in, 60); // drive upto matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_wait_until(33_in);
    matchload.set(true);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-75_deg, 90); // turn towards matchload
    chassis.pid_wait_quick_chain();
    forebar.set(false);
    chassis.pid_drive_set(20_in, 90); // drive upto matchload
    intake.move(70);
    pros::delay(2000);
    chassis.pid_wait_quick_chain();
}