#include "main.h"
#include <cmath>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 50;
const int TURN_SPEED = 75;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(21.167, 0, 143.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  //chassis.pid_turn_constants_set(3.7, 0.67, 27.25, 15.0);     // Turn in place constants
  chassis.pid_turn_constants_set(1.18, 0, 5.4, 0);
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

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

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

void leftQual(){

  chassis.drive_angle_set(0_deg); //sets initial heading to facing 0 degrees
  wing.extend();

  chassis.pid_drive_set(32.661, 85); //drive till in front of match load
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, 115); //turn to face match load
  pros::delay(500);

  matchLoad.extend(); //extend match load pneumatics to grab match load
  pros::delay(300); //wait for pneumatics to extend

  chassis.pid_drive_set(11, 55); //drive into matchload

  
  intakeState = 4; //intake matchload balls
  setIntakeMotors();
  pros::delay(700);
  chassis.pid_drive_set(-0.25, 4);
  pros::delay(675); //wait for balls to be intaken

  chassis.pid_drive_set(-27.961, 115); //move backwards into long goal
  pros::delay(500); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  chassis.pid_wait();

  hood.extend(); //extend hood to prepare for scoring

  setIntake(-127, 127); //score balls into long goal
  pros::delay(1000); //wait for balls to be scored
  setIntake(0,0); //stop intaking

  chassis.pid_drive_set(10, 115); //move out from long goal
  chassis.pid_wait();
  hood.retract(); //retract hood after moving out of long goal

  chassis.pid_turn_set(get_heading(-40.021, 46.798, -23.3, 21.729), 115); //turn to face next set of balls
  chassis.pid_wait();

  chassis.pid_drive_set(get_distance(-40.021, 46.798, -23.3, 21.729), 55); //drive to next set of balls
  intakeState = 4; //intake balls
  setIntakeMotors();
  chassis.pid_wait();
  setIntake(0, 0); //stop intaking

  // chassis.pid_drive_set(-8, 50); //drive backwards slightly
  // chassis.pid_wait();

  chassis.pid_turn_set(get_heading(-23.3, 21.729, -9.351, 41.17), 115); //turn to balls under long goal
  chassis.pid_wait();

  chassis.pid_drive_set(get_distance(-23.3, 21.729, -9.951, 41.17), 55);
  intakeState = 4;
  setIntakeMotors(); //intake balls
  chassis.pid_wait();
  
  

  chassis.pid_drive_set(get_distance(-23.3, 21.729, -9.951, 41.17) * -1, 115); //drive back to set up for mid goal
  chassis.pid_wait();
  setIntake(0, 0); //stop intaking

  chassis.pid_turn_set(get_heading(-23.3, 21.729, -12.28, 9.194) + 180, 115); //turn back to mid goal
  pros::delay(450);

  chassis.pid_drive_set(-1 * get_distance(-23.3, 21.729, -12.28, 9.194), 115); //drive into mid goal backwards
  hood.extend();
  intakeState = 3;
  setIntakeMotors();
  pros::delay(400);

  intakeState = 1;
  setIntakeMotors(); //score mid goal
  pros::delay(1550);
  setIntake(0, 0); //stop scoring

  wing.retract(); //put wing down

  chassis.pid_drive_set(get_distance(-12.28, 9.194, -34.485, 35.2)+4, 115); //drive out of mid goal
  chassis.pid_wait();
  
  chassis.pid_turn_set(87, 115); //set up for wing descore
  chassis.pid_wait();

  chassis.pid_drive_set(10, 70);
  chassis.pid_wait();
}

void rightQual(){
  chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  wing.extend();

  chassis.pid_drive_set(10.677, 105); //set up for middle balls
  pros::delay(350);

  chassis.pid_turn_set(get_heading(-35.227, -14.105, -21.536, -23.272), 105); //turn to face balls
  pros::delay(450);
  intakeState = 4; //start intaking balls
  setIntakeMotors();

  chassis.pid_drive_set(get_distance(-35.227, -14.105, -21.536, -23.272), 60); // move into balls to intake
  chassis.pid_wait();

  chassis.pid_turn_set(get_heading(-21.536, -23.272, -43.327, -49.605), 105); //turn to matchload setup
  chassis.pid_wait();
  intakeState = 0;
  setIntakeMotors(); //stop intaking

  chassis.pid_drive_set(get_distance(-21.536, -23.272, -43.327, -49.605), 105); //drive to matchload setup
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face matchload
  pros::delay(425);

  matchLoad.extend(); //extend matchload
  intakeState = 4;
  setIntakeMotors();
  chassis.pid_drive_set(15.327, 50); //drive into matchload while intaking
  chassis.pid_wait();
  pros::delay(175); //keep intaking for a bit after driving in

  chassis.pid_drive_set(-30.69, 90); //drive backwards into long goal
  pros::delay(450);
  hood.extend();
  pros::delay(150);
  intakeState = 3; //outtake a little to free up balls
  setIntakeMotors();
  chassis.pid_wait();

  matchLoad.retract(); //retract matchload and raise hood
  chassis.pid_drive_set(-20, 50); //keep wheels driving back to help score

  setIntake(-127, 127); //score balls into long goal
  pros::delay(2400); //wait for balls to be scored
  setIntake(0,0); //stop intaking

  chassis.pid_drive_set(8.6777, 105); //drive out to set up wing
  chassis.pid_wait();
  hood.retract();

  chassis.pid_turn_set(0, 105); //turn to set up wing
  chassis.pid_wait();

  chassis.pid_drive_set(11.7, 105); //drive to alley lane
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face direction for wing
  chassis.pid_wait();

  wing.retract(); //put wing down

  chassis.pid_drive_set(-27, 85); //drive into alley and wing
  chassis.pid_wait();


}

void leftElim(){
  chassis.drive_angle_set(90_deg); //sets initial heading to facing 90 degrees
  wing.extend();

  chassis.pid_drive_set(10.677, 75); //set up for middle balls
  pros::delay(350);

  chassis.pid_turn_set(get_heading(-35.227, 14.105, -21.536, 23.272), 90); //turn to face balls
  pros::delay(450);
  intakeState = 4; //start intaking balls
  setIntakeMotors();

  chassis.pid_drive_set(get_distance(-35.227, 14.105, -21.536, 23.272), 60); // move into balls to intake
  chassis.pid_wait();
  
  chassis.pid_turn_set(get_heading(-21.536, 23.272, -43.327, 49.605), 60); //turn to matchload setup
  chassis.pid_wait();
  intakeState = 0;
  setIntakeMotors(); //stop intaking


  chassis.pid_drive_set(get_distance(-21.536, 23.272, -43.327, 49.605), 75); //drive to matchload setup
  chassis.pid_wait();

  chassis.pid_turn_set(270, 105); //turn to face matchload
  pros::delay(425);

  matchLoad.extend(); //extend matchload
  intakeState = 4;
  setIntakeMotors();
  chassis.pid_drive_set(16.327, 60); //drive into matchload while intaking
  chassis.pid_wait();
  pros::delay(150); //keep intaking for a bit after driving in

  chassis.pid_drive_set(-31.29, 80); //drive backwards into long goal
  pros::delay(450);
  hood.extend();
  pros::delay(150);
  intakeState = 3; //outtake a little to free up balls
  setIntakeMotors();
  chassis.pid_wait();
  
  matchLoad.retract(); //retract matchload and raise hood
  chassis.pid_drive_set(-20, 50); //keep wheels driving back to help score

  setIntake(-127, 127); //score balls into long goal
  pros::delay(2400); //wait for balls to be scored
  setIntake(0,0); //stop intaking

  chassis.pid_drive_set(8.6777, 60);
  chassis.pid_wait();
  hood.retract();

  chassis.pid_turn_set(0, 105);
  chassis.pid_wait();
  chassis.pid_drive_set(11.0, 75); //drive to alley lane
  chassis.pid_wait();
  chassis.pid_turn_set(270, 105);
  chassis.pid_wait();
  wing.retract();
  chassis.pid_drive_set(-27, 85); //drive into alley and wing
}

void rightElim(){

}

void skills(){
  chassis.drive_angle_set(0_deg); //sets initial heading to facing 0 degrees
  wing.extend();

  chassis.pid_drive_set(32.661, 70); //drive till in front of match load
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(500);

  matchLoad.extend(); //extend match load pneumatics to grab match load
  pros::delay(300); //wait for pneumatics to extend

  chassis.pid_drive_set(11.55, 65); //drive into matchload
  matchLoad.extend(); //extend match load mech
  pros::delay(1000); //manuel delay to wait for bot to drive into matchload
  match_load_procedure(3, 0.9); //matchload

  chassis.pid_drive_set(-4, 75); //move backwards out of matchload
  chassis.pid_wait();
  matchLoad.retract(); //retract match load 
  chassis.pid_drive_set(-8, 75);
  chassis.pid_wait();
  
  intakeState = 0;
  setIntakeMotors(); //stop intaking

  chassis.pid_turn_set(get_heading(-46.175, 45.833, -31.371, 58.597) + 180, 60); //turn to face alley to long goal
  pros::delay(500);

  chassis.pid_drive_set(get_distance(-46.175, 45.833, -31.371, 58.597) * -1, 75); //drive into alley)
  pros::delay(1000);

  chassis.pid_turn_set(270, 60); //turn to face parralel to long goal
  pros::delay(500);

  chassis.pid_drive_set(-70.43, 90); //drive to other side of field
  chassis.pid_wait();

  chassis.pid_turn_set(0, 105); //turn perpendicular to long goal
  pros::delay(500);

  chassis.pid_drive_set(-12.366, 60); //drive to setup long goal
  chassis.pid_wait();

  chassis.pid_turn_set(90, 105); //turn to face back to long goal
  chassis.pid_wait();


  // 1st long goal score
  chassis.pid_drive_set(-6.279, 60); //drive into long goal
  hood.extend(); //extend hood to prepare for scoring
  chassis.pid_wait();

  // intakeState = 1; //outtake for a bit to free up balls
  // setIntakeMotors();
  // pros::delay(300);

  chassis.pid_drive_set(-60, 50); //keep driving backwards to help score
  setIntake(-127, 127); //score balls into long goal
  pros::delay(2600); //wait for balls to be scored
  setIntake(0,0); //stop intaking
  hood.retract(); //retract hood too slightly push ball further into long goal
  pros::delay(100);

  chassis.pid_drive_set(27.713, 65); //move out from long goal, into matchload
  pros::delay(100);
  matchLoad.extend(); //extend match load pneumatics to grab match load
  pros::delay(1000); //manuel delay to wait for bot to drive into matchload
  match_load_procedure(3, 0.9); //matchload

  //2nd long goal score

  chassis.pid_drive_set(-28.361, 75); //move backwards into long goal
  pros::delay(500); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  chassis.pid_wait();
  hood.extend(); //extend hood to prepare for scoring
  pros::delay(300);

  // intakeState = 1; //outtake for a bit to free up balls
  // setIntakeMotors();
  // pros::delay(300);

  chassis.pid_drive_set(-60, 50); //keep driving backwards to help score
  setIntake(-127, 80); //score balls into long goal
  pros::delay(2600); //wait for balls to be scored
  setIntake(0,0); //stop intaking

  hood.retract(); //retract hood to slightly push ball further into long goal
  pros::delay(100);
  chassis.pid_drive_set(14.27, 75); //move out from long goal
  pros::delay(600);

  chassis.pid_turn_set(180, 105); //turn to face perpedicular to other long goal
  pros::delay(500);

  chassis.pid_drive_set(97.162, 85); //drive to be in between other long goal & matchload
  chassis.pid_wait();

  chassis.pid_turn_set(90, 105); //turn to face matchload
  pros::delay(500);
  
  chassis.pid_drive_set(14.739, 65); //drive into matchload
  matchLoad.extend(); //extend match load mech
  pros::delay(1000); //manuel delay to wait for bot to drive into matchload
  match_load_procedure(3, 0.9); //matchload

  chassis.pid_drive_set(-12.739, 75); //move backwards out of matchload
  pros::delay(500); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load
  intakeState = 0; 
  setIntakeMotors(); //stop intaking
  chassis.pid_wait();

  chassis.pid_turn_set(get_heading(45.05, -48.831, 31.371, -59.997) + 180, 60); //turn to face alley to long goal
  pros::delay(500);

  chassis.pid_drive_set(get_distance(45.05, -48.831, 31.371, -59.997) * -1, 75); //drive into alley
  pros::delay(1000);

  chassis.pid_turn_set(90, 105); //turn to face parralel to long goal
  pros::delay(500);

  chassis.pid_drive_set(-70.43, 90); //drive to other side of field
  chassis.pid_wait();

  chassis.pid_turn_set(180, 105); //turn perpendicular to long goal
  pros::delay(500);

  chassis.pid_drive_set(-10.566, 70); //drive to setup long goal
  pros::delay(800);

  chassis.pid_turn_set(270, 105); //turn to face back to long goal
  pros::delay(500);

  // 3rd long goal score

  chassis.pid_drive_set(-8.279, 70); //drive into long goal
  chassis.pid_wait();
  hood.extend(); //extend hood to prepare for scoring
  pros::delay(300);

  chassis.pid_drive_set(-60, 50); //keep driving backwards to help score
  setIntake(-127, 127); //score balls into long goal
  pros::delay(2600); //wait for balls to be scored
  setIntake(0,0); //stop intaking
  hood.retract(); //retract hood to slightly push ball further into long goal
  pros::delay(100);

  chassis.pid_drive_set(27.713, 60); //move out from long goal, into matchload
  pros::delay(100);
  matchLoad.extend(); //extend match load pneumatics to grab match load
  pros::delay(1000); //manuel delay to wait for bot to drive into matchload
  match_load_procedure(3, 0.9); //matchload

  //4th long goal score

  chassis.pid_drive_set(-28.361, 75); //move backwards into long goal
  pros::delay(500); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  chassis.pid_wait();
  hood.extend(); //extend hood to prepare for scoring
  pros::delay(300);

  chassis.pid_drive_set(-60, 50); //keep driving backwards to help score
  setIntake(-127, 80); //score balls into long goal
  pros::delay(2600); //wait for balls to be scored
  setIntake(0,0); //stop intaking
  hood.retract(); //retract hood to slightly push ball further into long goal
  pros::delay(100);

  chassis.pid_drive_set(14.27, 75); //move out from long goal
  pros::delay(300);
  hood.retract(); //retract hood after moving out of long goal
  pros::delay(300);

  chassis.pid_turn_set(320, 105); //turn to face towards parking zone
  pros::delay(500);
  chassis.pid_drive_set(40, 75); //drive into wall
  pros::delay(700);
  chassis.pid_turn_set(340, 105); //turn to face towards parking zone
  pros::delay(500);
  chassis.pid_drive_set(10, 75); //drive into wall
  pros::delay(500);
  chassis.pid_turn_set(0, 105); //turn to face towards parking zone
  pros::delay(500);
  chassis.pid_drive_set(15, 75); //drive into park
  pros::delay(500);
  matchLoad.extend();
  chassis.pid_drive_set(-6, 75); //move back a bit to hook onto platform
  pros::delay(300);
  chassis.pid_drive_set(40, 105); //move forward into park
  setIntake(-127, 30);
  pros::delay(1000);
  matchLoad.retract();


  


}
void soloAWP() {
  chassis.drive_angle_set(0_deg); //sets initial heading to facing 0 degrees
  wing.extend();

  chassis.pid_drive_set(-5, 105); //drive to push alliance



  chassis.pid_drive_set(46, 85); //drive till in front of match load
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, 105); //turn to face match load
  pros::delay(500);

  matchLoad.extend(); //extend match load pneumatics to grab match load
  pros::delay(250); //wait for pneumatics to extend

  chassis.pid_drive_set(11, 105); //drive into matchload

  
  intakeState = 4; //intake matchload balls
  setIntakeMotors();
  pros::delay(700);
  chassis.pid_drive_set(-0.25, 4);
  pros::delay(350); //wait for balls to be intaken

  chassis.pid_drive_set(-27.961, 95); //move backwards into long goal
  pros::delay(300); //wait a bit before retracting matchload
  matchLoad.retract(); //retract match load 
  chassis.pid_wait();

  hood.extend(); //extend hood to prepare for scoring

  setIntake(-127, 127); //score balls into long goal
  chassis.pid_drive_set(-30, 95); //keep driving into long goal

  pros::delay(1350); //wait for balls to be scored
  setIntake(0,0); //stop intaking
  chassis.pid_turn_set(165, 105); //turn left to middle balls
  pros::delay(850);
  
  hood.retract(); //retract hood after moving out of long goal
  chassis.pid_drive_set(12, 60);
  setIntake(-127, 30); //intake middle balls
  pros::delay(500);
  chassis.pid_turn_set(315, 105); //turn into middle goal
  pros::delay(500);
  setIntake(0, 0);
  chassis.pid_drive_set(-13, 105); //drive into middle goal
  pros::delay(400);
  hood.extend();
  setIntake(-117, -50); // score on middle goal
  pros::delay(1350);
  setIntake(0, 0); //stop intaking
  chassis.pid_drive_set(5, 75);
  pros::delay(200);
  chassis.pid_turn_set(185, 105); //turn to other side's balls
  hood.retract();
  pros::delay(800);
  chassis.pid_drive_set(50, 50); //drive into middle balls
  setIntake(-127, 30); //start intaking
  pros::delay(1300);
  chassis.pid_turn_set(215, 105); //turn to set up for scoring
  pros::delay(600);
  chassis.pid_drive_set(34, 105);//drive to set up for long goal
  pros::delay(900);

  chassis.pid_turn_set(270, 105); //turn into long goal
  pros::delay(500);
  setIntake(0, 0); //stop intaking

  chassis.pid_drive_set(-12, 90);//drive into long goal
  pros::delay(300);
  hood.extend();
  setIntake(-127, 127);





  


}

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
  intakeState = 4; //intake matchload balls
  setIntakeMotors();
 
  for (int i = 0; i < times; i++) {
    chassis.pid_drive_set(-(wiggle_amount - 0.3), 40);
    pros::delay(300);
    chassis.pid_drive_set(wiggle_amount, 40);
    pros::delay(300);
  }

  pros::delay(2000 - times * 600); //wait for balls to be intaken
}