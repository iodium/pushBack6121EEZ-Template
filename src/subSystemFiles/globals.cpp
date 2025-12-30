#include "main.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/motors.hpp"


//motors
 pros::Motor intake1(20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
 pros::Motor intake2(11, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// controller
extern pros::Controller controller(pros::E_CONTROLLER_MASTER);

//misc. sensors
//pros::Rotation ladyBrownSense(11);
pros::Optical colorSort(14);