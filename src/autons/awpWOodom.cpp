#include "main.h"

void awp_wo_auton() {
    chassis.pid_drive_set(20_in, 127);
    chassis.pid_wait_quick_chain();
    intake.move(127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(10_in, 20);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-100_deg, 90);
    chassis.pid_wait();
    matchload.set(false);
    chassis.pid_wait();
    chassis.pid_drive_set(48_in, 110);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-150_deg, 110);
    matchload.set(true);
    intake.move(127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(40_in, 110); // matchload
    chassis.pid_wait_quick_chain();
    pros::delay(2000);
    forebar.set(true);
    chassis.pid_drive_set(-70_in, 110);
    chassis.pid_wait();
    lever.move(127);
    chassis.pid_wait();

}