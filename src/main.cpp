#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

    // Pick your drive setup from the list below:
    // ZERO_TRACKER_NO_ODOM
    // ZERO_TRACKER_ODOM
    // TANK_ONE_FORWARD_ENCODER
    // TANK_ONE_FORWARD_ROTATION
    // TANK_ONE_SIDEWAYS_ENCODER
    // TANK_ONE_SIDEWAYS_ROTATION
    // TANK_TWO_ENCODER
    // TANK_TWO_ROTATION
    // HOLONOMIC_TWO_ENCODER
    // HOLONOMIC_TWO_ROTATION
    //
    // Write it here:
    TANK_ONE_FORWARD_ROTATION,

    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

    // Left Motors:
    motor_group(leftBack, leftFront, leftFlipped),

    // Right Motors:
    motor_group(rightBack, rightFront, rightFlipped),

    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
    PORT13,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    2.75,

    // External ratio, must be in decimal, in the format of input teeth/output teeth.
    // If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
    // If the motor drives the wheel directly, this value is 1:
    0.75,

    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    351,

    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE!                                   */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
    /*  If you are not using position tracking, leave the rest of the values as  */
    /*  they are.                                                                */
    /*---------------------------------------------------------------------------*/

    // If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

    // FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
    // LF:      //RF:
    PORT1, -PORT2,

    // LB:      //RB:
    PORT3, -PORT4,

    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
    PORT17,

    // Input the Forward Tracker diameter (reverse it to make the direction switch):
    -2,

    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    0,

    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    8,

    // Sideways tracker diameter (reverse to make the direction switch):
    2,

    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    -5.75,

    vex::distance(PORT19),
    vex::distance(PORT5)

);

// int current_auton_selection = 0;
// bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  vexcodeInit();
  default_constants();
  bool toggleUp = true, toggleDown = true, toggleA = true ; 
  descore = false;

  while(!auto_started){
    // descore = false;
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(1, 1); 
    Controller.Screen.print("Battery: %d", Brain.Battery.capacity());
    Controller.Screen.setCursor(2, 1); 
    Controller.Screen.print("heading: %f",chassis.get_absolute_heading());
    Controller.Screen.setCursor(3,1); 
    Controller.Screen.print("auton: %d",current_auton_selection); 
    Controller.Screen.setCursor(3,10); 
    Controller.Screen.print("%s", isRed ? "red" : "blue"); 

    if(Controller.ButtonUp.pressing()){
      if(toggleUp){
        current_auton_selection = (current_auton_selection + 1) % 9; 
        toggleUp = false; 
      }
    } else{
      toggleUp = true; 
    }

    if(Controller.ButtonDown.pressing()){
      if(toggleDown){
        current_auton_selection = (current_auton_selection - 1) % 9; 
        toggleDown = false; 
      }
    } else{
      toggleDown = true; 
    }

    if(Controller.ButtonA.pressing()){
      if(toggleA){
        isRed = !isRed; 
        toggleA = false; 
      }
    } else{
      toggleA = true; 
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){
    case 0:
      scraper = true;
      bottomIntake.spin(fwd, 12, volt); 
      topIntake.spin(fwd, 12, volt); 
      
      chassis.drive_settle_error = -1;
      chassis.drive_timeout = 2000; 
      chassis.drive_distance_from_front_wall(8.75);
      //   // right7BallFast();
      //   chassis.set_heading(-90);
      //   chassis.drive_distance_timeout(18, 400);
      // chassis.turn_to_angle(27.5,150);
      // chassis.drive_distance(-80);
      // chassis.drive_distance_timeout(20,250);
      // chassis.drive_distance(-50);
      break;
    case 1:
      right7BallFast();
      break;
    case 2:
      leftAWP(); 
      break;
    case 3:
      skills(); 
      break;
    case 4:
      test();
      //chassis.drive_distance(4); 
      break;
    case 5:
      soloAWP();

      // chassis.drive_distance_customFunct(
      //   50,
      //   // 90.0,
      //   {10.0, 30.0},
      //   {[](){ scraper = true;},[](){ scraper = false; }}
      // );
      // chassis.set_heading(-90);  
      // postCrossScore(); 
      break;
    case 6:
      right4BallFast();
      // chassis.set_heading(-90); 
      // skillsCrossover();
      break;
    case 7:
      // chassis.set_heading(90); 
      // RBCrossover(); 
      break;
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  auto_started = false; 
  isIntaking = false; 
  int intakeDirection = 127;
  bool toggleX = true, toggleR2 = true, toggleL2 = true;
  while (1) {
    // bottomIntake.spin(directionType::fwd, (Controller.ButtonR1.pressing() - Controller.ButtonL1.pressing()) * 127, vex::velocityUnits::pct);
    // topIntake.spin(directionType::fwd, (Controller.ButtonR1.pressing() - Controller.ButtonL1.pressing()) * 127, vex::velocityUnits::pct);

    if(Controller.ButtonX.pressing()){
      if(toggleX){
        toggleX = false;
        scraper = !scraper; 
      }
    } else {
      toggleX = true;
    }

    if(Controller.ButtonR1.pressing()){
      bottomIntake.spin(directionType::fwd,127, vex::velocityUnits::pct);
      topIntake.spin(directionType::fwd,intakeDirection, vex::velocityUnits::pct);
    } else if(Controller.ButtonL1.pressing()){
      bottomIntake.spin(directionType::fwd,-127, vex::velocityUnits::pct);
      topIntake.spin(directionType::fwd,0-intakeDirection, vex::velocityUnits::pct);
    }else{
      bottomIntake.spin(directionType::fwd,0, vex::velocityUnits::pct);
      topIntake.spin(directionType::fwd,0, vex::velocityUnits::pct);
    }

    if(Controller.ButtonL2.pressing()){
      if(toggleL2){
        toggleL2 = false;
        descore = false; 
      }
    } else {
      toggleL2 = true;
      descore = true; 
    }

    if(Controller.ButtonR2.pressing()){
      if(toggleR2){
        toggleR2 = false;
        hood = !hood; 
      } 
    } else {
      toggleR2 = true;
    }
    if(!hood){
      topIntake.spin(directionType::fwd, 0, vex::velocityUnits::pct);
    }
 
    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
