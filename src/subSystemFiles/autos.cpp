// #include "main.h"
// #include "pros/rtos.hpp"

// void gurt(){
//     pros::delay(1000);
//     chassis.setPose(0, 0, 0, false); //initialize pose
//     chassis.turnToHeading(90,2000); //intermediate point to three in middle square
//     //chassis.turnToHeading(180, 2000);
// }

// void leftSide(){
//     odomRetract.extend();
//     chassis.setPose(-46.705, 7.805, 65);
//     setIntake(-127,0);
//     pros::delay(1000);
//     setIntake(0,0);
//     //chassis.moveToPoint(-28.229, 18.598, 3000);






//     //     // left side auto:
//     // chassis.setPose(-34.15, 15.832, 90, false); //initialize pose
//     // chassis.moveToPoint(-41.147, 15.008, 2000); //intermediate point to three in middle square
    
//     // chassis.turnToPoint(-21.595, 23.447, 1000); //3 in middle
//     // chassis.moveToPoint(-21.595, 23.447, 1000);
//     // setIntake(-127, 0); // activate 1 stage intake
//     // chassis.waitUntilDone();
//     // setIntake(0, 0); //stop intake

//     // chassis.turnToPoint(-5.952, 45.264, 1000); //2 below long goal
//     // chassis.moveToPoint(-5.952, 45.264, 1000);
//     // setIntake(-127, 0); // activate intake
//     // chassis.waitUntilDone();
//     // setIntake(0, 0); 
//     // chassis.turnToPoint(-21.595, 23.447, 1000); //back to middle
//     // chassis.moveToPoint(-21.595, 23.447, 1000);
//     // chassis.turnToPoint(-8.628, 9.04, 1000); //go score middle goal
//     // chassis.moveToPoint(-8.628, 9.04, 1000);
//     // chassis.waitUntilDone();
//     // midGoal.toggle();
//     // setIntake(-127, 127); // scoring middle

//     // pros::delay(1000);
//     // setIntake(0, 0);  
//     // midGoal.toggle();
//     // chassis.turnToPoint(-43.617, 46.705, 1000); // set up for match load
//     // chassis.moveToPoint(-43.617, 46.705, 1000);
    
//     // chassis.turnToPoint(-64.817, 46.705, 1000); // drive into match load
//     // chassis.moveToPoint(-64.817, 46.705, 1000);
//     // matchLoad.toggle();

//     // setIntake(-127, 0); // activate intake
//     // pros::delay(2000);
//     // setIntake(0,0);

//     // chassis.moveToPoint(-26.328, 46.705, 1000, {.forwards = false}); //drive backwards into long goal
//     // setIntake(-127, 127); // scoring
//     // pros::delay(2000);
//     // setIntake(0, 0);
//     // matchLoad.toggle();

//     // chassis.moveToPoint(-34.15, 46.705, 1000);// set up for descore pt 1
    
//     // chassis.turnToPoint(-34.15, 36.208, 1000); // set up for descore pt 2
//     // chassis.moveToPoint(-34.15, 36.208, 1000);
    
//     // chassis.turnToPoint(-40.15, 36.208, 1000); //wings descore
//     // chassis.moveToPoint(-12.15, 36.208, 1000, {.forwards = false});
//     // wing.toggle();
// }
// void rightSide(){
//     //right side auto:
//     chassis.setPose(-46.293, 15.041, 90, false); //initialize pose

//     chassis.turnToPoint(-15.214, -24.921, 1000);
//     chassis.moveToPoint(-15.214, -24.921, 2000); //intermediate point to three in middle square

//     chassis.turnToPoint(-44.543, -47.458, 1000); 
//     chassis.moveToPoint(-44.543, -47.458, 1000); //go set up for match load

//     chassis.turnToPoint(-64.096, -47.458, 1000);
//     chassis.moveToPoint(-64.096, -47.458, 1000); //go into match load 
//     matchLoad.toggle(); //toggle match load piston

//     setIntake(-127, 0); // activate intake
//     pros::delay(2000);
//     setIntake(0,0);

//     chassis.moveToPoint(-26.637, -47.458, 1000, {.forwards=false});
//     setIntake(-127, 127); // scoring
//     pros::delay(2000);
//     setIntake(0, 0);
//     matchLoad.toggle();

//     chassis.moveToPoint(-34.15, -47.458, 1000); //set up descore pt 1

//     chassis.turnToPoint(-34.15, -36.035, 1000); 
//     chassis.moveToPoint(-34.15, -36.035, 1000); //set up descore pt 2

//     chassis.turnToPoint(-14, -36.035, 1000);
//     chassis.moveToPoint(-14, -36.035, 1000); //descore with wing
//     wing.toggle();

// }
