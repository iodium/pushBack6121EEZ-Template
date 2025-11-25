#include "main.h"

void setDrive();
//DriveControlFunctions

void setDriveMotors();
float expoCurve(float input, float maxOut, float base, float deadzone);
float throttleCurve(float input, float maxOut, float base, float deadzone);
float steerCurve(float input, float maxOut, float base, float deadzone);