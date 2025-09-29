#include "vex.h"
#include <sys/wait.h>

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  // defaults:
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  /* Ours that we used last year:
chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(8, .07, 0, 0, 0);
  chassis.set_swing_constants(8, .3, .001, 2, 15);
*/

  // Each exit condition set is in the form of (settle_error, settle_time,
  // timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants() {
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test() {
  bottomIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  middleIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  chassis.drive_distance(30);
  chassis.drive_distance(-30);
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

int antiJamIntake() {
  while (true)
  {
  if (middleIntake.current() > 0 && middleIntake.efficiency() == 0)
  {
    middleIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
    bottomIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  }else
  {
    middleIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
    bottomIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  }
}

  return 0;
}
void unjam(){
  middleIntake.spin(directionType::fwd, -150, vex::velocityUnits::pct);
  vex::task::sleep(70);
  middleIntake.spin(directionType::fwd, 150, vex::velocityUnits::pct);
  vex::task::sleep(70);
  middleIntake.spin(directionType::fwd, -150, vex::velocityUnits::pct);
}
void blue_right_awp() {
  scraper = false;
  hopperValve = true;
  bottomIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  middleIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  chassis.drive_distance(11);
  chassis.turn_to_angle(45);
  chassis.drive_distance(18, 43, 3, 9);
  chassis.turn_to_angle(-180);
  // chassis.drive_distance(13, -180, 9, 9);
  chassis.drive_distance(4);
  chassis.drive_distance(16, 135, 9, 9);
  chassis.drive_distance(10, 90, 9, 9);
  chassis.turn_to_angle(0);
  chassis.drive_distance(10);
  task intakeTask = task(antiJamIntake);
  hopperIntake.spin(directionType::fwd, -150, vex::velocityUnits::pct);
  chassis.drive_distance(-2);
  hopperValve = false;
  vex::task::sleep(3500);
  intakeTask.stop();
  scraper = true;
  chassis.drive_distance(-5);
  chassis.turn_to_angle(-180);
  chassis.drive_distance_voltage(18,4);
}
void blue_left_awp() {
  scraper = false;
  hopperValve = true;
  bottomIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  middleIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  chassis.drive_distance(12);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(18, -45, 3, 9);
  chassis.drive_distance(16, -180, 9, 9);
  chassis.drive_distance(4);
  chassis.drive_distance(8, -135, 9, 9);
  // scraper = true;
  // vex::task::sleep(500);
  chassis.drive_distance(3.5);
  chassis.turn_to_angle(0);
  chassis.drive_distance(7);
  task intakeTask = task(antiJamIntake);
  hopperIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  chassis.drive_distance(-2);
  // for(int i = 0; i<10; i++){
  //   unjam();
  // }
  // chassis.drive_distance(8, -180, 9, 9);
  // chassis.drive_distance(0);
  hopperValve = false;
  vex::task::sleep(3000);
  intakeTask.stop();
  scraper = true;
  chassis.drive_distance(-5);
  chassis.turn_to_angle(-180);
  chassis.drive_distance_voltage(18,4);
  // chassis.drive_distance(11);
  // chassis.drive_distance_voltage(11, 6);
  // middleIntake.spin(directionType::fwd, -100, vex::velocityUnits::pct);
  // vex::task::sleep(1500);
  // bottomIntake.spin(directionType::fwd, 0, vex::velocityUnits::pct);
  // chassis.drive_distance_voltage(-10, 6);
  // chassis.turn_to_angle(0);
  // scraper = false; 
  // chassis.drive_distance_voltage(20, 6);
  // bottomIntake.spin(directionType::fwd, 200, vex::velocityUnits::pct);
  // middleIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  // hopperIntake.spin(directionType::fwd, -200, vex::velocityUnits::pct);
  
  // chassis.turn_to_angle(-55);
  // chassis.drive_distance(20);
  // chassis.turn_to_angle(-135);
  // chassis.drive_distance(10);
  // chassis.drive_distance(10);

  // chassis.turn_to_angle(5);
  // chassis.turn_to_angle(30);
  // chassis.turn_to_angle(90);
  // chassis.turn_to_angle(225);
  // chassis.turn_to_angle(0);
}

/**
 * The expected behavior is to return to the start angle, after making a
 * complete turn.
 */

void topGoalMacro() {
  chassis.drive_distance(-0.65); 
}

void turn_test() {
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

/**
 * Should swing in a fun S shape.
 */

void swing_test() {
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test() {
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test() {
  chassis.set_coordinates(0, 0, 0);
  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5, 40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5, 60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 80, "ForwardTracker: %f",
                         chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5, 100, "SidewaysTracker: %f",
                         chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test() {
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24, 24);
  chassis.drive_to_point(0, 0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test() {
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}