#include "main.h"

void setDrive(int leftFront, int leftBack, 
              int leftMid, int rightFront, 
              int rightBack, int rightMid) {
    driveLeftBack.move(leftBack);
    driveLeftFront.move(leftFront);
    driveLeftMiddle.move(leftMid);
    driveRightMiddle.move(rightMid);
    driveRightBack.move(rightBack);
    driveRightFront.move(rightFront);
}

void setDriveMotors() {
    int forwardBackward = throttleCurve(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 127.0, 1.04, 10);
    int turn = steerCurve(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 127.0, 1.04, 10);
    forwardBackward=(abs(forwardBackward)<10)?0:forwardBackward;
    turn=(abs(turn)<10)?0:turn;
    // float ts = 1;
    // turn*=ts;
    int leftFrontSpeed = forwardBackward +turn;
    int leftBackSpeed = forwardBackward +turn;
    int leftMidSpeed = forwardBackward +turn;
    int rightFrontSpeed = forwardBackward- turn;
    int rightBackSpeed = forwardBackward - turn;
    int rightMidSpeed = forwardBackward -turn;

    setDrive(leftFrontSpeed, 
            leftBackSpeed, 
            leftMidSpeed, 
            rightFrontSpeed, 
            rightBackSpeed, 
            rightMidSpeed);
}

/**
 * @brief Exponential function to scale output from controller joysticks
 * @brief Helper function for throttleCurve() and steerCurve()
 * @param input Input value from controller analog stick (out of 127)
 * @param maxOut Maximum output out of 127 (generally equals max input)
 * @param base base = b, where f(x) = a * b^x; changes how "curved" the function is
 * @param base Should generally be between 1-1.1 (requires tuning based on driver preference)
 * @param deadzone Stick's deadzone out of 127 (where output will be 0)
 */
float expoCurve(float input, float maxOut, float base, float deadzone) {
    double a = maxOut / pow(base, maxOut);
    float output;

    if (fabs(input) < deadzone) {
        return 0;
    }
    if (input < 0) {
        return round(-a * pow(base, -input));
    }
    else {
        return round(a * pow(base, input));
    }
}

/**
 * @brief Exponential function to scale output from controller joysticks for throttle
 * @details Uses expoCurve() as a helper function
 * @param input Input value from controller analog stick (out of 127)
 * @param maxOut Maximum output out of 127 (generally equals max input)
 * @param base base = b, where f(x) = a * b^x; changes how "curved" the function is
 * @param base Should generally be between 1-1.1 (tune by preference)
 * @param base Be careful with the base, as output values under .5 will become 0
 * @param deadzone Stick's deadzone out of 127 (where output will be 0)
 */
float throttleCurve(float input, float maxOut, float base, float deadzone) {
    return expoCurve(input, maxOut, base, deadzone);
}

/**
 * @brief Exponential function to scale output from controller joysticks for steering
 * @details Uses expoCurve() as a helper function
 * @param input Input value from controller analog stick (out of 127)
 * @param maxOut Maximum output out of 127 (generally equals max input)
 * @param base base = b, where f(x) = a * b^x; changes how "curved" the function is
 * @param base Should generally be between 1.02-1.1 (tune by preference)
 * @param deadzone Stick's deadzone out of 127 (where output will be 0)
 */
float steerCurve(float input, float maxOut, float base, float deadzone) {
    return expoCurve(input, maxOut, base, deadzone);
}