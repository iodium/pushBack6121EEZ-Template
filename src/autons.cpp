#include "main.h"
#include <cmath>
#include <vector>
#include <random>
struct Particle {
  double x;
  double y;
  double theta;
  double weight;
};
std::vector<Particle> particles;
std::default_random_engine randy(static_cast<unsigned>(std::time(nullptr)));
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 80;
const int TURN_SPEED = 75;
const int SWING_SPEED = 110;
const int BOT_LENGTH = 0; //set later
const int BOT_WIDTH = 0; //set later

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(10, 0, 25);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(5, 0.0, 19.0);        // Holds the robot straight while going forward without odom
  //chassis.pid_turn_constants_set(3.7, 0.67, 27.25, 15.0);     // Turn in place constants
  chassis.pid_turn_constants_set(1.5, 0, 8.7, 0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular contrggit pol for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(1500);


  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1500);


  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  pros::delay(3000);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  pros::delay(5000);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  // chassis.pid_turn_set(45_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .


//intake:
//4 = intake, no score
//3 = outtake a bit
//1 = outtake (full)
//2 = intake & score
//0 = off
/**
 * @brief left qual
 */

void leftQual(){
  chassis.drive_angle_set(0_deg); //sets initial heading to facing 0 degrees
  wing.extend();
  tripleStateStore();
  
  chassis.pid_drive_set(32.8, 80); //drive till in front of match load
  pros::delay(1000);
  
  matchLoad.extend();
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(450);

  chassis.pid_drive_set(11.3, 75); //drive into matchload
  fullIntake();
  pros::delay(950); //wait for balls to be intaken

  chassis.pid_drive_set(-33, 80); //move backwards into long goal
  pros::delay(700); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  pros::delay(150);

  tripleStateLongGoal();

  fullIntake();
  pros::delay(1000); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(168, 105); // turn to face next set of balls while at long goal
  pros::delay(800);

  chassis.pid_drive_set(15.5, 60); //move forward into next set of balls
  tripleStateStore();
  fullIntake();
  pros::delay(700);
  
  chassis.pid_turn_set(get_heading(-23.8, 19.729, -9.982,7.602) + 180, 115); //turn back to mid goal
  pros::delay(600);

  chassis.pid_drive_set(-1 * (get_distance(-23.8, 19.729, -9.982, 7.602)), 65); //drive into mid goal backwards
  stopIntake();
  pros::delay(600);

  tripleStateMidGoal(); //prepare to score mid goal
  midGoalIntake(); //intake to score mid goal
  pros::delay(1050);
  stopIntake(); //stop scoring

  // chassis.pid_turn_set(get_heading(-9.882, 10.276,-33.881, 38), 115); //turn to setup wing
  // pros::delay(200);

  // chassis.pid_drive_set(get_distance(-9.882, 10.276,-33.881, 38), 85); //drive to set up wing
  // pros::delay(1000);
  
  chassis.pid_drive_set(40.5, 85); //drive to set up wing
  pros::delay(1000);

  chassis.pid_turn_set(91, 105); //turn to be parrallel to long goal
  wing.retract(); //put wing down
  pros::delay(500);

  chassis.pid_drive_set(33, 127); //drive forward and descore


  //------------------------------------vvv old code vvv --------------------------------------------


  // wing.retract(); //put wing down

  // chassis.pid_drive_set(get_distance(-12.28, 9.194, -34.485, 35.2) + 1.75, 80); //drive out of mid goal
  // chassis.pid_wait();
  
  // chassis.pid_turn_set(87, 115); //set up for wing descore
  // chassis.pid_wait();

  // chassis.pid_drive_set(23, 95); //descore
  


  // old code vvv --------------------------------------------

  // chassis.pid_turn_set(get_heading(-40.021, 46.798, -23.8, 21.729), 115); //turn to face next set of balls
  // chassis.pid_wait();

  // chassis.pid_drive_set(get_distance(-40.021, 46.798, -23.8, 21.729), 55); //drive to next set of balls
  // fullIntake();
  // chassis.pid_wait();

  // chassis.pid_drive_set(-8, 50); //drive backwards slightly
  // chassis.pid_wait();

  // chassis.pid_turn_set(get_heading(-23.3, 21.729, -9.351, 41.17), 115); //turn to balls under long goal
  // chassis.pid_wait();

  // chassis.pid_drive_set(get_distance(-23.3, 21.729, -9.951, 41.17), 55);
  // chassis.pid_wait();

  // chassis.pid_drive_set(get_distance(-23.3, 21.729, -9.951, 41.17) * -1 - 0.5, 115); //drive back to set up for mid goal
  // chassis.pid_wait();
  // stopIntake();

  // stopIntake();

}

void rightQual(){
  chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_turn_set(get_heading(-45.833, -13.946, -28.707, -21), 127, true); //turn to face balls
  pros::delay(500);

  chassis.pid_drive_set(get_distance(-45.833, -13.946, -28.707, -21)+2, 90);
  fullIntake(); //intake balls  
  pros::delay(400);
  matchLoad.extend(); //extend to trap balls
  pros::delay(400);

  chassis.pid_turn_set(get_heading(-28.707, -21, -43.327, -45.5), 110); //turn to matchload setup
  pros::delay(600);
  stopIntake();

  // chassis.pid_drive_set(get_distance(-28.707, -21, -43.327, -45.5), 90); //drive to matchload setup
  // pros::delay(800);

  chassis.pid_drive_set(29, 90); //drive to matchload setup
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face matchload
  matchLoad.extend(); //extend matchload
  pros::delay(400);

  fullIntake(); //intake matchload balls
  chassis.pid_drive_set(20.5, 75); //drive into matchload while intaking
  pros::delay(800);
  chassis.pid_drive_set(-0.25, 40); //drive back slightly
  pros::delay(450);

  chassis.pid_drive_set(-27.69, 110); //drive backwards into long goal
  pros::delay(600);

  slowOuttake(); //outtake a little to free up balls
  pros::delay(250);
  tripleStateLongGoal();
  fullIntake(); //score

  matchLoad.retract(); //retract matchload 
  chassis.pid_drive_set(-20, 50); //keep wheels driving back to help score

  pros::delay(1800); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(315, 127); //turn to face wing setup
  pros::delay(500);
  
  chassis.pid_drive_set(12, 127); //drive to alley lane
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face direction for wing
  pros::delay(400);

  wing.retract(); //put wing down
 
  chassis.pid_drive_set(-23, 127); //drive into alley and wing

  chassis.pid_wait();
}
void rightQualPurple(){
  chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_turn_set(get_heading(-45.833, -13.946, -28.707, -21), 127, true); //turn to face balls
  pros::delay(500);

  chassis.pid_drive_set(get_distance(-45.833, -13.946, -28.707, -21)+2, 90);
  fullIntake(); //intake balls  
  pros::delay(400);
  matchLoad.extend(); //extend to trap balls
  pros::delay(400);

  chassis.pid_turn_set(get_heading(-28.707, -21, -43.327, -45.5), 110); //turn to matchload setup
  pros::delay(600);
  stopIntake();

  // chassis.pid_drive_set(get_distance(-28.707, -21, -43.327, -45.5), 90); //drive to matchload setup
  // pros::delay(800);

  chassis.pid_drive_set(28, 90); //drive to matchload setup
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face matchload
  matchLoad.extend(); //extend matchload
  pros::delay(400);

  fullIntake(); //intake matchload balls
  chassis.pid_drive_set(20.8, 65); //drive into matchload while intaking
  pros::delay(900);
  chassis.pid_drive_set(-0.25, 40); //drive back slightly
  pros::delay(450);

  chassis.pid_drive_set(-27.4, 110); //drive backwards into long goal
  pros::delay(600);

  slowOuttake(); //outtake a little to free up balls
  pros::delay(250);
  tripleStateLongGoal();
  fullIntake(); //score

  matchLoad.retract(); //retract matchload 
  chassis.pid_drive_set(-20, 50); //keep wheels driving back to help score

  pros::delay(1800); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(315, 127); //turn to face wing setup
  pros::delay(500);
  
  chassis.pid_drive_set(12.4, 127); //drive to alley lane
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face direction for wing
  pros::delay(400);

  wing.retract(); //put wing down
 
  chassis.pid_drive_set(-23, 127); //drive into alley and wing

  chassis.pid_wait();
}
void leftElim(){
  chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_turn_set(get_heading(-45.833, 13.946, -28.707, 21), 105, true); //turn to face balls
  pros::delay(500);

  chassis.pid_drive_set(get_distance(-45.833, 13.946, -28.707, 21), 95);
  fullIntake(); //intake balls
  pros::delay(1000);

  chassis.pid_turn_set(get_heading(-28.707, 21, -43.327, 49), 110); //turn to matchload setup
  pros::delay(600);
  stopIntake();

  chassis.pid_drive_set(get_distance(-28.707, 21, -43.327, 49)-2.5, 75); //drive to matchload setup
  pros::delay(1300);
  matchLoad.extend(); //extend matchload


  chassis.pid_turn_set(270, 105); //turn to face matchload
  pros::delay(425);

  fullIntake(); //intake matchload balls
  chassis.pid_drive_set(18, 75); //drive into matchload while intaking
  pros::delay(1100);
  chassis.pid_drive_set(-0.75, 40); //drive out slightly
  chassis.pid_drive_set(19.5, 75); //drive into matchload while intaking
  pros::delay(1000);
  match_load_procedure(2, 1);

  chassis.pid_drive_set(-27.69, 100); //drive backwards into long goal
  pros::delay(550);
  slowOuttake(); //outtake a little to free up balls
  pros::delay(250);
  tripleStateLongGoal();
  fullIntake(); //score

  matchLoad.retract(); //retract matchload 
  chassis.pid_drive_set(-20, 30); //keep wheels driving back to help score

  pros::delay(1800); //wait for balls to be scored
  stopIntake(); //stop scoring

  chassis.pid_turn_set(315, 127); //turn to face wing setup
  pros::delay(600);
  
  
  chassis.pid_drive_set(9, 110); //drive to alley lane
  pros::delay(550);

  chassis.pid_turn_set(265, 105); //turn to face direction for wing
  pros::delay(500);

  wing.retract(); //put wing down
 
  chassis.pid_drive_set(-20, 100); //drive into alley and wing

  chassis.pid_wait();
}

void right4Rush() {
  
  // chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  // wing.extend();

  // chassis.pid_drive_set(10.677, 105); //set up for middle balls
  // pros::delay(350);

  // chassis.pid_turn_set(get_heading(-35.227, -14.105, -21.536, -23.272), 105); //turn to face balls
  // pros::delay(400);
  // intakeState = 4; //start intaking balls
  // setIntakeMotors();

  // chassis.pid_drive_set(get_distance(-35.227, -14.105, -21.536, -23.272), 65); // move into balls to intake
  // pros::delay(900);

  // chassis.pid_turn_set(get_heading(-21.536, -23.272, -43.327, -49.605), 105); //turn to matchload setup
  // pros::delay(600);
  // intakeState = 0;
  // setIntakeMotors(); //stop intaking

  // chassis.pid_drive_set(get_distance(-21.536, -23.272, -43.327, -49.605), 100); //drive to matchload setup
  // pros::delay(1100);

  // chassis.pid_turn_set(270, 105); //turn to face back to long goal
  // hood.extend(); //extend hood
  // pros::delay(425);

  // chassis.pid_drive_set(-20, 60); //drive backwards into long goal and keep driving back to help score
  // pros::delay(600);

  // setIntake(-127, 127); //score balls into long goal
  // pros::delay(1000); //wait for balls to be scored
  // setIntake(0,0); //stop intaking

  // chassis.pid_drive_set(0, 0); //stop driving completely

  // chassis.pid_turn_set(315, 127); //turn to face wing setup
  // pros::delay(600);

  // chassis.pid_drive_set(7, 110); //drive to alley lane
  // pros::delay(550);
  // hood.retract();

  // chassis.pid_turn_set(265, 105); //turn to face direction for wing
  // pros::delay(500);

  // wing.retract(); //put wing down

  // chassis.pid_drive_set(-18, 100); //drive into alley and wing
  // chassis.pid_wait();
}

void right4PushMatchload() {
  // chassis.drive_angle_set(180); //sets initial heading to facing 180 degrees
  // wing.extend();

  // chassis.pid_drive_set(32.161, 85); //drive till in front of match load
  // chassis.pid_wait();

  // chassis.pid_turn_set(270, 115); //turn to face match load
  // pros::delay(500);

  // matchLoad.extend(); //extend match load pneumatics 
  // pros::delay(300); //wait for pneumatics to extend

  // chassis.pid_drive_set(11.4, 55); //drive into matchload

  // intakeState = 4; //intake matchload balls
  // setIntakeMotors();
  // pros::delay(1100); //wait for balls to be intaken

  // chassis.pid_drive_set(-29.961, 115); //move backwards into long goal
  // pros::delay(500); //wait a bit before retracting matchload
  // matchLoad.retract(); //retract match load 
  // chassis.pid_wait();

  // hood.extend(); //extend hood to prepare for scoring

  // setIntake(-127, 127); //score balls into long goal
  // pros::delay(1000); //wait for balls to be scored
  // setIntake(0,0); //stop intaking

  // chassis.pid_turn_set(315, 127); //turn to face wing setup
  // pros::delay(600);

  // chassis.pid_drive_set(7, 110); //drive to alley lane
  // pros::delay(550);
  // hood.retract();

  // chassis.pid_turn_set(265, 105); //turn to face direction for wing
  // pros::delay(500);

  // wing.retract(); //put wing down

  // chassis.pid_drive_set(-24, 100); //drive into alley and wing
  // chassis.pid_wait();
}

/**
 * @brief skills for stage field
 */
void stageSkills() {
  chassis.drive_angle_set(90); //sets initial heading to facing 0 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_drive_set(get_distance(-45.833, 13.946, -34.6, 13.946), 105);
  pros::delay(400);

  chassis.pid_turn_set(get_heading(-34.6, 13.946,-22.8, 24.4), 105, true); //turn to face balls
  pros::delay(500);
 
  chassis.pid_drive_set(get_distance(-34.6, 14.917,-22.8, 24.4), 65);
  fullIntake(); //intake balls
  pros::delay(500);
  
  stopIntake();
  pros::delay(300);
 //intake balls
  
  
  chassis.pid_turn_set(get_heading(-22.8, 24.4,-9.882, 10.276)+180, 105);//turn to mid goal
  pros::delay(500);

  chassis.pid_drive_set(-1*(get_distance(-22.8, 24.4,-9.882, 10.276) ), 75); //drive into mid goal
  chassis.pid_wait();

  tripleStateMidGoal();
  setIntake(70, -70);
  pros::delay(1200);
  // stopIntake(); //STOP intake balls
  // tripleStateStore();

  chassis.pid_drive_set(get_distance(-9.882, 10.276,-46.43, 41), 95); //drive up to 1st matchload
  pros::delay(300);
  stopIntake(); //STOP intake balls
  tripleStateStore();
  pros::delay(800);


  chassis.pid_turn_set(270, 105);
  matchLoad.extend();

  pros::delay(500);
  chassis.pid_drive_set(13.5, 70); //drive into matchload
  fullIntake();
  pros::delay(600);
  pros::delay(1500);
  chassis.pid_drive_set(-13, 105); //drive out of matchload
  pros::delay(500);

  chassis.pid_turn_set(get_heading(-46.43, 41.5, -37.243, 55.834) + 180, 105); //turn to face point beyond long goal
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(-46.43, 41.5, -37.243, 55.834)+2, 120); //drive past long goal
  pros::delay(500);

  chassis.pid_turn_set(270, 105);
  pros::delay(500);
  stopIntake();
  setIntake(80,-80);

  chassis.pid_drive_set(-get_distance(-37.243, 55.834, 38.312, 55.834), 115); //drive to other side of field
  pros::delay(1500);

  matchLoad.retract();

  chassis.pid_turn_set(180, 105); //turn to set up for scoring
  pros::delay(500);
  chassis.pid_drive_set(9.75, 115); // + 1, drive perpendicular to long
  pros::delay(500);
  chassis.pid_turn_set(90, 105); //turn to face long goal
  pros::delay(500);
  chassis.pid_drive_set(-14, 85);
  pros::delay(600);
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_turn_set(90, 105); //turn to face long goal

  
  pros::delay(400);
  chassis.pid_drive_set(-20, 105);
  pros::delay(1800);//score
  tripleStateStore();
  stopIntake();
  chassis.pid_drive_set(get_distance(28.596,46.412, 32, 46.259), 105);
  
  pros::delay(300);
  chassis.pid_turn_set(90, 85); //turn to be straight
  matchLoad.extend();
  pros::delay(300);
  chassis.pid_drive_set(get_distance(32, 46.259,59,46.259), 60); //drive into 2nd matchload
  fullIntake();
  pros::delay(800);
  pros::delay(1900);  
  stopIntake();
  chassis.pid_drive_set(-get_distance(29.5, 46.259,59,46.25), 100);
  pros::delay(650);
  tripleStateLongGoal();
  fullIntake();
  pros::delay(2350);//score
  tripleStateStore();
  stopIntake();

  // removed hood push
  chassis.pid_drive_set(6, 85); // hood push start
  pros::delay(400); // idk hood wait
  chassis.pid_drive_set(-6, 105); // hood push end
  pros::delay(400); // idk hood wait

  matchLoad.retract();
  chassis.pid_drive_set(get_distance(28.36, 46.25,47.451,46.25), 110);
  pros::delay(500);
  chassis.pid_turn_set(get_heading(47.451,46.25,62.698,23.449), 105); //turn to point beyond wall
  pros::delay(450);
  chassis.pid_drive_set(get_distance(47.451,46.25,62.698,23.449)-11, 105); //drive into wall
  pros::delay(600);
  // chassis.pid_swing_set(ez::LEFT_SWING, 180, 90);
  // pros::delay(500);
  chassis.pid_swing_set(ez::LEFT_SWING, 177, 90, 50);

  pros::delay(700);
  chassis.pid_drive_set(8,60); //uhhh
  pros::delay(600);

  chassis.pid_drive_set(66, 65); //barrier cross changed 70->85, 55 -> 48
  //changed, go all the way to matchload
  fullIntake();
  pros::delay(3200);

  //matchLoad.extend();
  // pros::delay(800);
  // chassis.pid_swing_set(ez::LEFT_SWING, 270, 100, 5);

  // pros::delay(250);
  // chassis.pid_drive_set(-10, 80);
  // pros::delay(500);

  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 90, -40); //turn to put back to wall
  pros::delay(1000);

  chassis.pid_drive_set(-10, 55); //keep driving back into wall;
  pros::delay(500);
  matchLoad.extend();
  double distance_reset_y = get_y(1);
  chassis.drive_angle_set(270); //sets current heading to facing 270 degrees

  chassis.pid_swing_set(ez::LEFT_SWING, get_heading(63.372,distance_reset_y,20,-23.015), 100);
  pros::delay(500);
  chassis.pid_drive_set(get_distance(58.372,distance_reset_y,20,-23.015), 100); //clear balls in front of mid goal
  chassis.pid_wait();
  chassis.pid_turn_set(get_heading(20,-23.015,7.75,-8)+176.5, 100); //turn to face back to mid goal
  stopIntake();

  pros::delay(500);
  chassis.pid_drive_set(-(get_distance(20,-23.015,7.75,-8) + 2.6), 100); // drive into mid goal
  chassis.pid_wait();
  

  tripleStateMidGoal();
  midGoalIntake();
  setIntake(70,-70);
  pros::delay(1200);
  setIntake(55, -55);
  pros::delay(1600);
  stopIntake();
  chassis.pid_drive_set(-0.5, 40); //drive ba ckward slightly to nudge blocks in further
  pros::delay(600);

  chassis.pid_turn_set(get_heading(8, -8, 43, -41), 100); //turn to face setup for matchload
  chassis.pid_drive_set(get_distance(8, -8, 43, -41), 110); //drive to matchload setup)
  pros::delay(1000);
  tripleStateStore();

  //3rd matchload:

  chassis.pid_turn_set(90, 105); //turn to face matchload
  matchLoad.extend(); 
  pros::delay(500);

  chassis.pid_drive_set(12, 65); //drive into matchload while intaking
  fullIntake();
  pros::delay(2700);  

  chassis.pid_drive_set(-11, 105); //drive out of matchload while intaking
  pros::delay(500);
  stopIntake();

  chassis.pid_turn_set(get_heading(43, -42, 34.243, -53.834) + 180, 105); //turn to face point beyond long goal
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(43, -42, 34.243, -53.834), 105); //drive past long goal
  pros::delay(550);

  chassis.pid_turn_set(90, 105);
  pros::delay(500);
  stopIntake();
  setIntake(50,-50);

  //**maybe make faster */
  chassis.pid_drive_set(-get_distance(34.243, -53.834, -38.312, -53.834), 115); //drive to other side of field
  pros::delay(1650);

  matchLoad.retract();

  chassis.pid_turn_set(0, 105); //turn to set up for scoring
  pros::delay(400);
  chassis.pid_drive_set(10, 105); // + 1
  pros::delay(450);

  
  chassis.pid_turn_set(270, 105); //turn to face long goal
  pros::delay(400);
  chassis.pid_drive_set(-15, 85);
  pros::delay(550);
  tripleStateLongGoal();
  fullIntake();
  
  pros::delay(300);
  chassis.pid_drive_set(-20, 10);
  pros::delay(1800);//score
  tripleStateStore();
  stopIntake();

  chassis.pid_drive_set(get_distance(-28.596,-46.412, -32, -46.259), 105);
  
  pros::delay(300);
  chassis.pid_turn_set(270, 85); //turn to be straight
  matchLoad.extend();
  pros::delay(300);
  chassis.pid_drive_set(get_distance(-32, -46.259, -59, -46.259), 60); //drive into 4th matchload
  fullIntake();
  pros::delay(800);
  pros::delay(1900);  
  stopIntake();
  chassis.pid_drive_set(-get_distance(-29.5, -46.259, -59, -46.25)-2, 90);
  pros::delay(800);
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_drive_set(-40, 20);

  pros::delay(2700);//score
  tripleStateStore();
  stopIntake();
  matchLoad.retract();
  chassis.pid_drive_set(5, 85); // hood push start
  pros::delay(400);

  chassis.pid_drive_set(-6, 105); // hood push end
  pros::delay(300);
  chassis.drive_angle_set(270);
  chassis.pid_drive_set(get_distance(-28.36, -46.25, -47.451, -46.25), 95);
  pros::delay(600);
  chassis.pid_turn_set(get_heading(-47.451,-46.25,-62.698,-23.449), 75); //turn to point beyond wall
  pros::delay(500);
  chassis.pid_drive_set(get_distance(-47.451,-46.25,-62.698,-23.449)-10, 75); //drive into wall
  pros::delay(600);

 chassis.pid_swing_set(ez::LEFT_SWING, 0, 90, 50);

  pros::delay(750);
  chassis.pid_drive_set(30,60); //uhhh
  pros::delay(600);

  chassis.pid_drive_set(16, 60); //park
  fullIntake();
  pros::delay(1000);
  //matchLoad.extend();
  pros::delay(1000);
}

/**
 * @brief skills code
 *
*/
void skills(){
  chassis.drive_angle_set(90); //sets initial heading to facing 0 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_drive_set(get_distance(-45.833, 13.946, -34.6, 13.946), 105);
  pros::delay(400);

  chassis.pid_turn_set(get_heading(-34.6, 13.946,-22.8, 24.4), 105, true); //turn to face balls
  pros::delay(500);
 
  chassis.pid_drive_set(get_distance(-34.6, 14.917,-22.8, 24.4), 65);
  fullIntake(); //intake balls
  pros::delay(500);
  
  stopIntake();
  pros::delay(300);
 //intake balls
  
  
  chassis.pid_turn_set(get_heading(-22.8, 24.4,-9.882, 10.276)+180, 105);//turn to mid goal
  pros::delay(500);

  chassis.pid_drive_set(-1*(get_distance(-22.8, 24.4,-9.882, 10.276) ), 75); //drive into mid goal
  chassis.pid_wait();

  tripleStateMidGoal();
  setIntake(70, -70);
  pros::delay(1200);
  // stopIntake(); //STOP intake balls
  // tripleStateStore();

  chassis.pid_drive_set(get_distance(-9.882, 10.276,-46.43, 41), 95); //drive up to 1st matchload
  pros::delay(300);
  stopIntake(); //STOP intake balls
  tripleStateStore();
  pros::delay(800);


  chassis.pid_turn_set(270, 105);
  matchLoad.extend();

  pros::delay(500);
  chassis.pid_drive_set(13.5, 70); //drive into matchload
  fullIntake();
  pros::delay(600);
  pros::delay(1500);
  chassis.pid_drive_set(-13, 105); //drive out of matchload
  pros::delay(500);

  chassis.pid_turn_set(get_heading(-46.43, 41.5, -37.243, 55.834) + 180, 105); //turn to face point beyond long goal
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(-46.43, 41.5, -37.243, 55.834)+2, 120); //drive past long goal
  pros::delay(500);

  chassis.pid_turn_set(270, 105);
  pros::delay(500);
  stopIntake();
  setIntake(80,-80);

  chassis.pid_drive_set(-get_distance(-37.243, 55.834, 38.312, 55.834), 115); //drive to other side of field
  pros::delay(1500);

  matchLoad.retract();

  chassis.pid_turn_set(180, 105); //turn to set up for scoring
  pros::delay(500);
  chassis.pid_drive_set(11.5, 115); // + 1, drive perpendicular to long
  pros::delay(500);
  chassis.pid_turn_set(90, 105); //turn to face long goal
  pros::delay(500);
  chassis.pid_drive_set(-14, 85);
  pros::delay(600);
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_turn_set(90, 105); //turn to face long goal

  
  pros::delay(400);
  chassis.pid_drive_set(-20, 105);
  pros::delay(1800);//score
  tripleStateStore();
  stopIntake();
  chassis.pid_drive_set(get_distance(28.596,46.412, 32, 46.259), 105);
  
  pros::delay(300);
  chassis.pid_turn_set(90, 85); //turn to be straight
  matchLoad.extend();
  pros::delay(300);
  chassis.pid_drive_set(get_distance(32, 46.259,59,46.259), 60); //drive into 2nd matchload
  fullIntake();
  pros::delay(800);
  pros::delay(1900);  
  stopIntake();
  chassis.pid_drive_set(-get_distance(29.5, 46.259,59,46.25), 100);
  pros::delay(650);
  tripleStateLongGoal();
  fullIntake();
  pros::delay(2350);//score
  tripleStateStore();
  stopIntake();

  // removed hood push
  chassis.pid_drive_set(6, 85); // hood push start
  pros::delay(400); // idk hood wait
  chassis.pid_drive_set(-6, 105); // hood push end
  pros::delay(400); // idk hood wait

  matchLoad.retract();
  chassis.pid_drive_set(get_distance(28.36, 46.25,47.451,46.25), 110);
  pros::delay(500);
  chassis.pid_turn_set(get_heading(47.451,46.25,62.698,23.449), 105); //turn to point beyond wall
  pros::delay(450);
  chassis.pid_drive_set(get_distance(47.451,46.25,62.698,23.449)-11, 105); //drive into wall
  pros::delay(600);
  // chassis.pid_swing_set(ez::LEFT_SWING, 180, 90);
  // pros::delay(500);
  chassis.pid_swing_set(ez::LEFT_SWING, 177, 90, 50);

  pros::delay(700);
  chassis.pid_drive_set(8,60); //uhhh
  pros::delay(600);

  chassis.pid_drive_set(66, 65); //barrier cross changed 70->85, 55 -> 48
  //changed, go all the way to matchload
  fullIntake();
  pros::delay(3200);

  //matchLoad.extend();
  // pros::delay(800);
  // chassis.pid_swing_set(ez::LEFT_SWING, 270, 100, 5);

  // pros::delay(250);
  // chassis.pid_drive_set(-10, 80);
  // pros::delay(500);

  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 90, -40); //turn to put back to wall
  pros::delay(1000);

  chassis.pid_drive_set(-10, 55); //keep driving back into wall;
  pros::delay(500);
  matchLoad.extend();
  double distance_reset_y = get_y(1);
  chassis.drive_angle_set(270); //sets current heading to facing 270 degrees

  chassis.pid_swing_set(ez::LEFT_SWING, get_heading(63.372,distance_reset_y,20,-23.015), 100);
  pros::delay(500);
  chassis.pid_drive_set(get_distance(58.372,distance_reset_y,20,-23.015), 100); //clear balls in front of mid goal
  chassis.pid_wait();
  chassis.pid_turn_set(get_heading(20,-23.015,8,-8)+176.5, 100); //turn to face back to mid goal
  stopIntake();

  pros::delay(500);
  chassis.pid_drive_set(-(get_distance(20,-23.015,8,-8) + 2.6), 100); // drive into mid goal
  chassis.pid_wait();
  

  tripleStateMidGoal();
  midGoalIntake();
  setIntake(70,-70);
  pros::delay(1200);
  setIntake(55, -55);
  pros::delay(1600);
  stopIntake();
  chassis.pid_drive_set(-0.5, 40); //drive ba ckward slightly to nudge blocks in further
  pros::delay(600);

  chassis.pid_turn_set(get_heading(8, -8, 43, -41), 100); //turn to face setup for matchload
  chassis.pid_drive_set(get_distance(8, -8, 43, -41)+0.5, 110); //drive to matchload setup)
  pros::delay(1000);
  tripleStateStore();

  //3rd matchload:

  chassis.pid_turn_set(90, 105); //turn to face matchload
  matchLoad.extend(); 
  pros::delay(500);

  chassis.pid_drive_set(12.4, 65); //drive into matchload while intaking
  fullIntake();
  pros::delay(2700);  

  chassis.pid_drive_set(-11, 105); //drive out of matchload while intaking
  pros::delay(500);
  stopIntake();

  chassis.pid_turn_set(get_heading(43, -42, 34.243, -53.834) + 180, 105); //turn to face point beyond long goal
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(43, -42, 34.243, -53.834), 105); //drive past long goal
  pros::delay(550);

  chassis.pid_turn_set(90, 105);
  pros::delay(500);
  stopIntake();
  setIntake(50,-50);

  //**maybe make faster */
  chassis.pid_drive_set(-get_distance(34.243, -53.834, -38.312, -53.834), 115); //drive to other side of field
  pros::delay(1650);

  matchLoad.retract();

  chassis.pid_turn_set(0, 105); //turn to set up for scoring
  pros::delay(400);
  chassis.pid_drive_set(10, 105); // + 1
  pros::delay(450);

  
  chassis.pid_turn_set(270, 105); //turn to face long goal
  pros::delay(400);
  chassis.pid_drive_set(-15, 85);
  pros::delay(550);
  tripleStateLongGoal();
  fullIntake();
  
  pros::delay(300);
  chassis.pid_drive_set(-20, 10);
  pros::delay(1800);//score
  tripleStateStore();
  stopIntake();

  chassis.pid_drive_set(get_distance(-28.596,-46.412, -32, -46.259), 105);
  
  pros::delay(300);
  chassis.pid_turn_set(270, 85); //turn to be straight
  matchLoad.extend();
  pros::delay(300);
  chassis.pid_drive_set(get_distance(-32, -46.259, -59, -46.259), 60); //drive into 4th matchload
  fullIntake();
  pros::delay(800);
  pros::delay(1900);  
  stopIntake();
  chassis.pid_drive_set(-get_distance(-29.5, -46.259, -59, -46.25)-2, 90);
  pros::delay(800);
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_drive_set(-40, 20);

  pros::delay(2700);//score
  tripleStateStore();
  stopIntake();
  matchLoad.retract();
  chassis.pid_drive_set(5, 85); // hood push start
  pros::delay(400);

  chassis.pid_drive_set(-6, 105); // hood push end
  pros::delay(300);
  chassis.drive_angle_set(270);
  chassis.pid_drive_set(get_distance(-28.36, -46.25, -47.451, -46.25), 95);
  pros::delay(600);
  chassis.pid_turn_set(get_heading(-47.451,-46.25,-62.698,-23.449), 75); //turn to point beyond wall
  pros::delay(500);
  chassis.pid_drive_set(get_distance(-47.451,-46.25,-62.698,-23.449)-10, 75); //drive into wall
  pros::delay(600);

 chassis.pid_swing_set(ez::LEFT_SWING, 0, 90, 50);

  pros::delay(750);
  chassis.pid_drive_set(30,60); //uhhh
  pros::delay(600);

  chassis.pid_drive_set(16, 60); //park
  fullIntake();
  pros::delay(1000);
  //matchLoad.extend();
  pros::delay(1000);
}

void soloAWP() {
  chassis.drive_angle_set(180_deg); //sets initial heading to facing 180 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_drive_set(32.8, 80); //drive till in front of match load
  pros::delay(1000);
  
  matchLoad.extend();
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(450);

  chassis.pid_drive_set(12, 75); //drive into matchload
  fullIntake();
  pros::delay(950); //wait for balls to be intaken

  chassis.pid_drive_set(-33, 80); //move backwards into long goal
  pros::delay(700); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  pros::delay(150);

  tripleStateLongGoal();

  fullIntake();
  pros::delay(1050); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(10, 105); // turn to face next set of balls while at long goal
  pros::delay(700);

  chassis.pid_drive_set(15, 75); //move forward into next set of balls
  tripleStateStore();
  fullIntake();
  pros::delay(400);

  chassis.pid_turn_set(0, 105); //turn to face next set of balls on left side of field
  pros::delay(100);

  chassis.pid_drive_set(46, 105); //move forward into next set of balls
  pros::delay(900);

  pros::delay(200);
  setIntake(0, 0); //stop intaking
  matchLoad.extend(); //extend match load to trap balls


  chassis.pid_turn_set(get_heading(-22.331, 19.883, -48.43, 50), 115); // turn to set up for left long goal
  pros::delay(350);

  chassis.pid_drive_set(get_distance(-22.331, 19.883, -48.43, 50), 95);
  pros::delay(1150);

  chassis.pid_turn_set(270, 105); //turn to face back to left long goal
  pros::delay(400);

  chassis.pid_drive_set(-19, 95); //drive backwards into long goal
  pros::delay(500);
  
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_drive_set(-20, 20);
  pros::delay(1450);//score
  stopIntake();
  tripleStateStore();

  chassis.pid_drive_set(30.2, 75); //drive into left matchload
  fullIntake();
  pros::delay(1300);

  chassis.pid_drive_set(-11, 90);
  pros::delay(600);

  matchLoad.retract();

  chassis.pid_turn_set(get_heading(-48.164, 45, -9.5, 7.5) + 180, 105);
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(-48.164, 45, -9.5, 7.5), 100);
  pros::delay(1100);

  tripleStateMidGoal();
  midGoalIntake();

  


//   chassis.pid_turn_set(-45, 105); //turn to face back to mid goal
//   pros::delay(500);

//   chassis.pid_drive_set(-16.4, 105); //drive backwards into mid goal
//   slowOuttake();
//   pros::delay(200);
//   setIntake(0, 0);
//   pros::delay(600);

//   matchLoad.retract(); 
//   midGoalIntake();
//   pros::delay(1500);
//   setIntake(0, 0); //stop intaking

//   //chassis.pid_drive_set(get_distance(-12, 12, -41.013, 45.255), 105); //drive to set up long goal
//   chassis.pid_drive_set(get_distance(-12, 12, -41.013, 45.255)+5, 105); //drive to set up long goal

//   //hood.retract();
//   pros::delay(800);

//   chassis.pid_turn_set(270, 105); //turn to face back to long goal
//   pros::delay(500);

//   chassis.pid_drive_set(-14, 90); //drive into long goal
//   //hood.extend(); 
//   pros::delay(500);

//   setIntake(-127, 127); //score balls into long goal
// ______________________________________________________________

  // chassis.drive_angle_set(0_deg); //sets initial heading to facing 0 degrees
  // wing.extend();

  // chassis.pid_drive_set(-5, 105); //drive to push alliance



  // chassis.pid_drive_set(46, 85); //drive till in front of match load
  // pros::delay(1000);
  
  // chassis.pid_turn_set(270_deg, 105); //turn to face match load
  // pros::delay(500);

  // matchLoad.extend(); //extend match load pneumatics to grab match load
  // pros::delay(250); //wait for pneumatics to extend

  // chassis.pid_drive_set(11, 105); //drive into matchload

  
  // intakeState = 4; //intake matchload balls
  // setIntakeMotors();
  // pros::delay(700);
  // chassis.pid_drive_set(-0.25, 4); //drive out slightly to help intake
  // pros::delay(350); //wait for balls to be intaken

  // chassis.pid_drive_set(-27.961, 95); //move backwards into long goal
  // pros::delay(300); //wait a bit before retracting matchload
  // matchLoad.retract(); //retract match load
  // chassis.pid_wait();

  // hood.extend(); //extend hood to prepare for scoring

  // setIntake(-127, 127); //score balls into long goal
  // chassis.pid_drive_set(-30, 95); //keep driving into long goal

  // pros::delay(1350); //wait for balls to be scored
  // setIntake(0,0); //stop intaking
  // chassis.pid_turn_set(165, 105); //turn left to middle balls
  // pros::delay(850);
  
  // hood.retract(); //retract hood after moving out of long goal
  // chassis.pid_drive_set(12.5, 60);
  // setIntake(-127, 30); //intake middle balls
  // pros::delay(500);
  // chassis.pid_turn_set(315, 105); //turn into middle goal
  // pros::delay(500);
  // setIntake(0, 0);
  // chassis.pid_drive_set(-13, 105); //drive into middle goal
  // pros::delay(400);
  // hood.extend();
  // setIntake(-117, -50); // score on middle goal
  // pros::delay(1350);
  // setIntake(0, 0); //stop intaking
  // chassis.pid_drive_set(5, 75);
  // pros::delay(200);
  // chassis.pid_turn_set(195, 105); //turn to other side's balls
  // hood.retract();
  // pros::delay(800);
  // chassis.pid_drive_set(50, 50); //drive into middle balls
  // setIntake(-127, 30); //start intaking
  // pros::delay(1300);
  // chassis.pid_turn_set(215, 105); //turn to set up for scoring
  // pros::delay(600);
  // chassis.pid_drive_set(34, 105);//drive to set up for long goal
  // pros::delay(900);

  // chassis.pid_turn_set(270, 105); //turn into long goal
  // pros::delay(500);
  // setIntake(0, 0); //stop intaking

  // chassis.pid_drive_set(-12, 90);//drive into long goal
  // pros::delay(300);
  // hood.extend();
  // setIntake(-127, 127);
}
void soloAWPYellow() {
  chassis.drive_angle_set(180_deg); //sets initial heading to facing 180 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_drive_set(32.8, 80); //drive till in front of match load
  pros::delay(1000);
  
  matchLoad.extend();
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(450);

  chassis.pid_drive_set(12, 75); //drive into matchload
  fullIntake();
  pros::delay(950); //wait for balls to be intaken

  chassis.pid_drive_set(-33, 80); //move backwards into long goal
  pros::delay(700); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  pros::delay(150);

  tripleStateLongGoal();

  fullIntake();
  pros::delay(1050); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(10, 105); // turn to face next set of balls while at long goal
  pros::delay(700);

  chassis.pid_drive_set(15, 75); //move forward into next set of balls
  tripleStateStore();
  fullIntake();
  pros::delay(400);

  chassis.pid_turn_set(0, 105); //turn to face next set of balls on left side of field
  pros::delay(100);

  chassis.pid_drive_set(46, 105); //move forward into next set of balls
  pros::delay(900);

  pros::delay(200);
  setIntake(0, 0); //stop intaking
  matchLoad.extend(); //extend match load to trap balls


  chassis.pid_turn_set(get_heading(-22.331, 19.883, -48.43, 50), 115); // turn to set up for left long goal
  pros::delay(350);

  chassis.pid_drive_set(get_distance(-22.331, 19.883, -48.43, 50), 95);
  pros::delay(1150);

  chassis.pid_turn_set(270, 105); //turn to face back to left long goal
  pros::delay(400);

  chassis.pid_drive_set(-19, 95); //drive backwards into long goal
  pros::delay(500);
  
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_drive_set(-20, 20);
  pros::delay(1450);//score
  stopIntake();
  tripleStateStore();

  chassis.pid_drive_set(30.2, 75); //drive into left matchload
  fullIntake();
  pros::delay(1300);

  chassis.pid_drive_set(-11, 90);
  pros::delay(600);

  matchLoad.retract();

  chassis.pid_turn_set(get_heading(-48.164, 45, -9.5, 7.5) + 180, 105);
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(-48.164, 45, -9.5, 7.5), 100);
  pros::delay(1100);

  tripleStateMidGoal();
  midGoalIntake();

}
void soloAWPPurple() {
  chassis.drive_angle_set(180_deg); //sets initial heading to facing 180 degrees
  wing.extend();
  tripleStateStore();

  chassis.pid_drive_set(32.8, 80); //drive till in front of match load
  pros::delay(1000);
  
  matchLoad.extend();
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(450);

  chassis.pid_drive_set(12, 75); //drive into matchload
  fullIntake();
  pros::delay(900); //wait for balls to be intaken

  chassis.pid_drive_set(-33, 80); //move backwards into long goal
  pros::delay(700); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  pros::delay(150);

  tripleStateLongGoal();

  fullIntake();
  pros::delay(1050); //wait for balls to be scored
  stopIntake();

  chassis.pid_turn_set(10, 105); // turn to face next set of balls while at long goal
  pros::delay(700);

  chassis.pid_drive_set(15, 75); //move forward into next set of balls
  tripleStateStore();
  fullIntake();
  pros::delay(400);

  chassis.pid_turn_set(0, 105); //turn to face next set of balls on left side of field
  pros::delay(100);

  chassis.pid_drive_set(46, 105); //move forward into next set of balls
  pros::delay(900);

  pros::delay(200);
  setIntake(0, 0); //stop intaking
  matchLoad.extend(); //extend match load to trap balls


  chassis.pid_turn_set(get_heading(-22.331, 19.883, -48.43, 50), 115); // turn to set up for left long goal
  pros::delay(350);

  chassis.pid_drive_set(get_distance(-22.331, 19.883, -48.43, 50), 95);
  pros::delay(1150);

  chassis.pid_turn_set(270, 105); //turn to face back to left long goal
  pros::delay(400);

  chassis.pid_drive_set(-19, 95); //drive backwards into long goal
  pros::delay(500);
  
  tripleStateLongGoal();
  fullIntake();
  chassis.pid_drive_set(-20, 20);
  pros::delay(1450);//score
  stopIntake();
  tripleStateStore();

  chassis.pid_drive_set(30.2, 65); //drive into left matchload
  fullIntake();
  pros::delay(1400);

  chassis.pid_drive_set(-11, 90);
  pros::delay(600);

  matchLoad.retract();

  chassis.pid_turn_set(get_heading(-48.164, 45, -11.5, 7.5) + 180, 105);
  pros::delay(500);

  chassis.pid_drive_set(-get_distance(-48.164, 45, -10.5, 7.5)-1, 100);
  pros::delay(1100);

  tripleStateMidGoal();
  midGoalIntake();

}
/**
 
/**
 * @brief returns heading that bot needs to turn to to face a given point
 * @note dest_x - current_x (or change in x) cannot equal 0
 * @param current_x current x-coord of bot
 * @param current_y current y-coord of bot
 * @param dest_x x coord of point that bot needs to turn to
 * @param dest_y y coord of point that bot needs to turn to
 * @return heading for bot to turn to, based on its current/desired position
 */
float get_heading(float current_x, float current_y, float dest_x, float dest_y) {
  float delta_x = dest_x - current_x;
  float delta_y = dest_y - current_y;

  if (delta_x < 0.000001 && delta_x > -0.000001) {
    throw std::invalid_argument("dest_x - current_x cannot = 0");
  }

  float alpha = atan(delta_y / delta_x) * 180 / acos(-1);
  
  float theta;

  if (delta_x > 0) {
    theta = 90 - alpha;
  }
  if (delta_x < 0) {
    theta = 270 - alpha;
  }
  
  return theta;
}

/**
 * @brief gets distance between current position and desired position of bot
 */
float get_distance(float current_x, float current_y, float dest_x, float dest_y) {
  return sqrt(pow(dest_x - current_x, 2) + pow(dest_y - current_y, 2));
}

void match_load_procedure(int times, double wiggle_amount) {
 
  for (int i = 0; i < times; i++) {
    chassis.pid_drive_set(-(wiggle_amount - 0.3), 40);
    pros::delay(150);
    chassis.pid_drive_set(wiggle_amount, 40);
    pros::delay(150);
  }

  pros::delay(800 - times * 300); //wait for balls to be intaken
}
void match_load_procedure_skills(int times, double wiggle_amount) {
 
  for (int i = 0; i < times; i++) {
    chassis.pid_drive_set(-(wiggle_amount), 40);
    pros::delay(200);
    chassis.pid_drive_set(wiggle_amount, 40);
    pros::delay(200);
  }

  pros::delay(1700 - times * 300); //wait for balls to be intaken
}
/**
 * @brief returns (with double precision) x-coordinate of the center of the bot on the field
 * @brief from the perspective of red side, positive x is forward, positive y is left
 * @details uses distance sensors to calculate position based on current distance to walls
 * @param facing 1 = +x, 2 = +y, 3 = -x, 4 = -y
 */
double get_x(int facing) {
  double x;


  if (facing == 1) {
    double dist = backDistanceSensor.get() / 25.4 + BOT_LENGTH / 2; //distance from wall to center of bot, converted to inches
    x = dist - 70; // distance from center to edge of field is 70 inches
  }
  else if (facing == 3) {
    double dist = backDistanceSensor.get() / 25.4 + BOT_LENGTH / 2; 
    x = 70 - dist;
  }
  else if (facing == 2) {
    double dist = leftDistanceSensor.get() / 25.4 + BOT_WIDTH / 2;
    x = dist - 70;
  }
  else { //facing = 4
    double dist = rightDistanceSensor.get() / 25.4 + BOT_WIDTH / 2;
    x = 70 - dist;
  }

  return x;
}

/**
 * @brief returns (with double precision) y-coordinate of the center of the bot on the field
 * @brief from the perspective of red side, positive x is forward, positive y is left
 * @details uses distance sensors to calculate position based on current distance to walls
 * @param facing 1 = +x, 2 = +y, 3 = -x, 4 = -y
 */
double get_y(int facing) {
  double y;
  double side_dist_to_center; //distance from side distance sensor to center of bot
  double back_dist_to_center; //distance from back distance sensor to center of bot
  //divide values by 25.4 to convert from mm to inches
  if (facing == 1) {
    double dist = leftDistanceSensor.get() / 25.4 + 13.5 / 2; //distance from wall to center of bot, converted to inches
    y = dist - 70; // distance from center to edge of field is 70 inches
  }
  return y;
}

void tripleStateLongGoal() {
  tripleStateDown.extend();
  tripleStateUp.extend();
}

void tripleStateStore() {
  tripleStateDown.retract(); 
  tripleStateUp.extend();
}

void tripleStateMidGoal() {
  tripleStateDown.retract();
  tripleStateUp.retract();
}

void fullIntake() {
  setIntake(127, -127);
}

void fullOuttake() {
  setIntake(-127, 127);
}
void slowOuttake() {
  setIntake(-40, 40);
}
void stopIntake() {
  setIntake(0, 0);
}
void midGoalIntake() {
  setIntake(80,-80);
}

void update(double dx, double dy, double dtheta) {
  for (auto &p : particles) {
    double noise_x = randomGaussian(0, 0.5);  //noise to simulate variance, tune these values
    double noise_y = randomGaussian(0, 0.5);
    double noise_theta = randomGaussian(0, 0.1);

    p.x += (noise_x +dx);
    p.y += (noise_y +dy);
    p.theta += (noise_theta +dtheta); 
    //update all variables for each particle ^^

  }
  
} 

double getMapDistance(double x, double y, double theta) { //approximation, since we dont have ray casting
  double distances[4];
  distances[0] =abs((x-72)/cos(theta));
  distances[1] =abs((x+72)/cos(theta));
  distances[2] =abs((y-72)/sin(theta));

  distances[3] =abs((y+72)/sin(theta));
  double min = distances[0];
  for (int i = 0; i < 4; i++) {
    if (min > distances[i]) min = distances[i];
  }
  return min;
}

void weighting(double actualDistance) {
  double totalWeight = 0;
  for (auto &p: particles) { //for every particle
    double expected = getMapDistance(p.x, p.y, p.theta);
  }
}

double randomGaussian(double mean, double standard_dev) { //normal distribution
  static std::normal_distribution<double> distribution; //initialize a static distribution
  return std::normal_distribution<double>(mean, standard_dev)(randy); //input mean and standard deviation with the engine
}