#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

motor frontLeft = motor(PORT20, ratio6_1, true); 
motor MiddleLeft = motor(PORT15, ratio6_1, true);
motor UpsidedownLeft = motor(PORT4, ratio6_1, false);

motor_group chassisLeft = motor_group(frontLeft, MiddleLeft, UpsidedownLeft); 

motor frontRight = motor(PORT5, ratio6_1, false); 
motor MiddleRight = motor(PORT8, ratio6_1, false);
motor UpsidedownRight = motor(PORT2, ratio6_1, true);

motor_group chassisRight = motor_group(frontRight, MiddleRight, UpsidedownRight);

motor bottomIntake = motor(PORT11, ratio6_1, true);
motor middleIntake = motor(PORT6, ratio6_1, true);;
motor hopperIntake = motor(PORT10, ratio6_1, true);

digital_out scraperRight = digital_out(Brain.ThreeWirePort.A);  //not put
digital_out scraperLeft = digital_out(Brain.ThreeWirePort.A);  // not put

digital_out hopperValve = digital_out(Brain.ThreeWirePort.A); // not put 

controller Controller; 
//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}