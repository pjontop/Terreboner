#include "main.h"

void awp_auton() {
    /* ---- FIRST SEQUENCE, THREE BALLS */
    chassis.odom_xyt_set(52.266_in, 17.234_in, 281.1_deg); // start at the autonomous at the right side of the park zone
    chassis.pid_odom_set({{22.692_in, 22.174_in}, rev, 110}); // go to the three blocks in the middle
    chassis.pid_wait_until(28.584);
    intake.move(127); // intake the balls
    matchload.set(true); // deploy pneumatics
    chassis.pid_wait(); // wait
    pros::delay(200); // give time for intake to grab balls
    /* ---- SECOND SEQUENCE, NE MATCHLOAD, THREE BALLS ---- */


}