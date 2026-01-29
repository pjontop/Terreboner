#include "main.h"
#include "snapshot_pose/snapshot_bindings.hpp"

void awp_auton() {
    // Tell distance sensors where (0,0,0) in program corresponds to on actual field
    // Starting position: Field (44.951, -8.998, 237.7°) = Program (0, 0, 0°)
    snapshot_pose::snapshot_set_offset(44.951, -8.998, 237.7);

    /* ---- FIRST SEQUENCE, THREE BALLS */
    chassis.pid_odom_set({{0.0_in, 30.0_in}, fwd, 127}); // go to the three blocks in the middle
    chassis.pid_wait_until(20.0_in); // wait until close to the blocks 
    intake.move(127); // intake the balls
    matchload.set(true); // deploy pneumatics
    chassis.pid_wait(); // wait
    pros::delay(200); // give time for intake to grab balls
    matchload.set(false);
    /* ---- SECOND SEQUENCE, NE MATCHLOAD, THREE BALLS ---- */
    chassis.pid_odom_set({{{-15.252_in, 22.239_in, 250_deg}, fwd, 127},
                          {{-30.794_in, 16.556_in, 200_deg}, fwd, 127},
                          {{-36.537_in, 1.169_in, 200_deg}, fwd, 127}},
                         true);
    chassis.pid_wait_until({-27.91_in, 24.612_in}); // wait until reaching matchload position
    matchload.set(true); // activate matchload
    chassis.pid_wait();
    chassis.pid_wait_until({-36.537_in, 1.169_in, 200_deg}); // wait until reaching last position
    intake.move(127); // intake the balls
    pros::delay(2000); // give time for intake to grab balls
    /* ---- WHILE WE'RE HERE MIGHT AS WELL CALIBRATE ODOM ---- */
        auto ne_result = snapshot_pose::snapshot_correct_pose();
    if (ne_result.ok) {
        printf("NE position corrected: (%.1f, %.1f) using %d sensors\n", 
               ne_result.x_in, ne_result.y_in, ne_result.used_sensors);
    } else {
        printf("NE correction failed - continuing with odometry estimate\n");
    }
    /* ---- THIRD SEQUENCE, N LONGOAL, SCORE 6 BALLS ---- */
    chassis.pid_odom_set({{-17.116_in, 37.571_in}, rev, 127}, // moving to the long goal
                        true);
    chassis.pid_wait_until({-17.116_in, 37.571_in}); // wait until reaching the long goal
    lever.move(127); // score the balls
    pros::delay(1000); // give time for lever to score
    lever.move(-127); // hold the lever down
    /* ---- FORTH SEQUENCE, NW BALLS, PICKUP 3 BALLS ---- */
    chassis.pid_odom_set({{{29.997_in, 4.682_in}, fwd, 127},
                          {{34.4_in, 7.381_in}, fwd, 127},
                          {{42.808_in, 12.342_in}, fwd, 127}},
                         true);
    chassis.pid_wait_until({34.4_in, 7.381_in}); // wait until reaching the first NW ball
    matchload.set(true); // matchload to "grab" balls
    intake.move(127); // intake the balls
    pros::delay(200); // give time for intake to grab balls
    chassis.pid_wait_until({42.808_in, 12.342_in}); // wait until reaching the last NW ball
    pros::delay(200); // give time for intake to grab balls
    matchload.set(false); // retract matchload



}