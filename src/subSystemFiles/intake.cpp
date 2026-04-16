#include "main.h"
#include "pros/misc.h"

bool sortToggle = false;
int intakeState = 0;

void setIntake(int power1, int power2, int power3) {
    intake1.move(power1); //right bottom
    intake2.move(power2); // right top
    intake3.move(power3); //left motor
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
            
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
            setIntake(127, 127, 127);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || intakeState == 2 ) {
            setIntake(127, 127 , 127);            

        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || intakeState == 1){ //score middle goal
            setIntake(80,-80, 127); 
        }
        
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            setIntake(-127,-127, -127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)|| intakeState == 3){
            setIntake(45,-45, 127);
        }
        
        else{
            setIntake(0,20,0);            
        }  
}
