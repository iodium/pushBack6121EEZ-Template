#include "main.h"
#include "pros/misc.h"
bool sortToggle = false;
int intakeState = 0;
void setIntake(int power1, int power2) {
    intake1.move(power1);
    intake2.move(power2);
}
// void setIntakeMotors() {
//         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
//             setIntake(127, -127);            
//         } 
//         else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { 
//             setIntake(-127, 127);            
//         }
//         else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
//             setIntake(127,0);
//         }
//         else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
//             setIntake(-127,0);
//         }
//         else{
//             setIntake(0,0);            
//         }  
// }

void setIntakeMotors() {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || intakeState == 2) { 
            setIntake(-127, 127);            
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || intakeState == 1){
            setIntake(-127,-50);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
                !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || intakeState == 4){
            setIntake(-127,20);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            setIntake(127,-127);
        }
        else if (intakeState == 3){
            setIntake(0,127);

        }
        else{
            setIntake(0,0);            
        }  
}