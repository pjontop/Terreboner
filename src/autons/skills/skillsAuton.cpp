#include "main.h"

void skills_auton() {
/* ---- FIRST SEQUENCE, START @ LEFT SIDE OF PARKZONE FACING MATCHLOAD ---- */
    chassis.pid_drive_set(39_in, 127); // drive upto matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(90_deg, 90); // turn towards matchload
    intake.move(127); // start intake
    descore_wing.set(true);
    matchload.set(true); // lower matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(20_in, 127); // drive into matchload
    chassis.pid_wait_quick_chain();
    /* chassis.pid_drive_set(-1_in, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(1_in, 127); */
    pros::delay(3000); // give time to load
    /* ---- SECOND SEQUENCE, GO TOWARDS OTHER SIDE OF LONGOAL ---- */
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-30_in, 127); // back away from matchload
    matchload.set(false); // lower matchload
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(0_deg, 90); // turn towards alleyway
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(10.5_in, 127); // go line up infront of alleyway
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-90_deg, 90); // turn towards alleyway
    chassis.pid_wait_quick_chain();
    // Use wall tracking to stay straight along the right wall
    // drive_wall_track(130, 80, right_dist, 4.0); // go down alleyway
    chassis.pid_drive_set(130_in, 90); // go down alleyway
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-180_deg, 90); // turn to face the exit alleyway
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(17.5_in, 127); // exit alleyway
    /* ---- SCORE ON LONG GOAL, 6 BALLS ---- */
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-90_deg, 90); // turn towards long goal
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-15_in, 127); // score on long goal
    chassis.pid_wait_quick_chain();
    lever.move(127); // score the balls
    pros::delay(1000); // give time for lever to score
    lever.move(-127); // hold the lever down
    chassis.pid_wait_quick_chain();
    /* ---- GO TO MATCHLOAD + SCORE 6 BLOCKS ---- */
    chassis.pid_drive_set(10_in, 127); // back away from goal
    chassis.pid_wait_quick_chain();
    matchload.set(true); // lower matchload
    intake.move(127); // start intake
    chassis.pid_drive_set(40_in, 127); // complete action towards the goal
    chassis.pid_wait_quick_chain();
    pros::delay(2000); // give time to intake balls
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(-50_in, 127); // go to the long goal
    chassis.pid_wait_quick_chain();
    lever.move(127); // score the balls
    pros::delay(1000); // give time for lever to score
    lever.move(-127); // hold the lever down
    chassis.pid_wait_quick_chain();
    /* ---- GO TO S SIDE, PARK ZONE TO PICKUP BLOCKS ---- */
    chassis.pid_drive_set(30_in, 127);
    chassis.pid_wait_quick_chain();
    chassis.pid_turn_set(-180_deg, 90);
    chassis.pid_wait_quick_chain();
    chassis.pid_drive_set(180_in, 127);
    chassis.pid_wait_quick_chain();



}
