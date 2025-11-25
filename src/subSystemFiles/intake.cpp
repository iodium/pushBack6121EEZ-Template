#include "main.h"
#include "pros/misc.h"
bool sortToggle = false;
int intakeState = 0;
void setIntake(int power1, int power2) {
    intake1.move(power1);
    intake2.move(power2);
}
void setIntakeMotors() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
            setIntake(127, -127);            
        } 
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
            setIntake(-127, 127);            
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            setIntake(127,0);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            setIntake(-127,0);
        }
        else{
            setIntake(0,0);            
        }  
}