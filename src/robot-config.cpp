#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);
motor leftFront = motor(PORT3, ratio6_1, true); 
motor leftBack = motor(PORT2, ratio6_1, true);
motor leftFlipped = motor(PORT1, ratio6_1, false);

motor rightFront = motor(PORT8, ratio6_1, false); 
motor rightBack = motor(PORT9, ratio6_1, false);
motor rightFlipped = motor(PORT10, ratio6_1, true);
//Add your devices below, and don't forget to do the same in robot-config.h:

motor bottomIntake = motor(PORT5, ratio6_1, true);
motor topIntake = motor(PORT6, ratio6_1, true);

digital_out scraper = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.C);
digital_out hood = digital_out(Brain.ThreeWirePort.E);

distance rearDistance = distance(PORT20); 

bool isIntaking = true;

controller Controller; 

void vexcodeInit( void ) {
  // nothing to initialize
}