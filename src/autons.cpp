#include "vex.h"
/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants() {
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

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
  chassis.drive_distance(12);
  chassis.drive_distance(-12);
  chassis.drive_distance(24);
  chassis.drive_distance(-24);
}

/**
 * The expected behavior is to return to the start angle, after making a
 * complete turn.
 */

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

int antiJamIntake()
{
  while (true)
  {
    if(isIntaking){
      if (bottomIntake.current() > 0 && bottomIntake.efficiency() == 0)
      {
        bottomIntake.spin(reverse, 90, pct);
      }
      else
      {
        bottomIntake.spin(fwd, 90, pct);
      }
    }
  }

  return 0;
}
int antiJamTaskFast()
{
  while (true)
  {
    if(isIntaking){
      if (bottomIntake.current() > 0 && bottomIntake.efficiency() == 0)
      {
        bottomIntake.spin(reverse, 127, pct);
      }
      else
      {
        bottomIntake.spin(fwd, 127, pct);
      }
    }
  }

  return 0;
}


void awp_right() {
  task intakeTask = task(antiJamIntake);
  topIntake.spin(directionType::rev, 127, vex::velocityUnits::pct); 
  isIntaking = true; 
  chassis.drive_distance(5);
  chassis.drive_distance(17, 20, 3, 5);
  chassis.drive_distance(-15);
  intakeTask.stop();
  bottomIntake.spin(directionType::fwd, 0, vex::velocityUnits::pct);

  chassis.turn_to_angle(90);
  chassis.drive_distance(17);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-8);

  topIntake.spin(directionType::fwd, 127, vex::velocityUnits::pct);
  task intakeTask2 = task(antiJamTaskFast);

  // bottomIntake.spin(directionType::fwd, 127, vex::velocityUnits::pct);
  // chassis.drive_distance(2);
  task::sleep(4000);
  intakeTask2.stop();
  // bottomIntake.spin(reverse, 127, pct);
  bottomIntake.spin(fwd, 127, pct);
  // intakeTask.stop()
  chassis.turn_to_angle(180);
  scraper = true; 
  task::sleep(1000);
  chassis.drive_distance(15);
  task::sleep(2000);
  // intakeTask.stop();
  // chassis.drive_distance(2);
}

void notAwp_left(){
  task intakeTask = task(antiJamIntake);
  topIntake.spin(directionType::rev, 127, vex::velocityUnits::pct); 
  isIntaking = true; 
  chassis.drive_distance(5);
  chassis.drive_distance(17, -20, 3, 5);
  chassis.drive_distance(-15);
  intakeTask.stop();
  bottomIntake.spin(directionType::fwd, 0, vex::velocityUnits::pct);

  chassis.turn_to_angle(-90);
  chassis.drive_distance(16.5);
  chassis.turn_to_angle(180);
  topIntake.spin(directionType::fwd, 127, vex::velocityUnits::pct);
  chassis.drive_distance(-7);

  // topIntake.spin(directionType::fwd, 127, vex::velocityUnits::pct);
  task intakeTask2 = task(antiJamTaskFast);

  // bottomIntake.spin(directionType::fwd, 127, vex::velocityUnits::pct);
  // chassis.drive_distance(2);
  task::sleep(4000);
  intakeTask2.stop();
  // bottomIntake.spin(reverse, 127, pct);
  bottomIntake.spin(fwd, 127, pct);
  // intakeTask.stop()
  chassis.turn_to_angle(180);
  scraper = true; 
  task::sleep(1000);
  chassis.drive_distance(15);
  task::sleep(2000);
  // intakeTask.stop();
  // chassis.drive_distance(2);
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

int intakeAntiJamTask(){
  while(true){
    double voltage = bottomIntake.voltage(voltageUnits::volt);
    double efficiency = bottomIntake.efficiency(); 

    if(voltage > 0 && efficiency == 0){
      bottomIntake.spin(reverse, 12, volt); 
    } else{
      bottomIntake.spin(fwd, 12, volt);
    }

    wait(0.1, sec); 
  }
}

void right7BallPush(){
  descore.set(false);
  hood.set(false);

  topIntake.spin(reverse, 10, volt);
  bottomIntake.spin(fwd, 12, volt);
  
  chassis.set_drive_constants(7, 1.4, 0, 12, 0);
  chassis.drive_distance(17);
  chassis.turn_to_angle(25);
  chassis.set_drive_constants(4, 1.4, 0, 12, 0);
  chassis.drive_distance(16);
  scraper.set(true);
  chassis.turn_to_angle(135);

  chassis.set_drive_constants(8, 1.4, 0, 12, 0);
  // chassis.drive_distance(33.85);
  chassis.drive_distance(35.00);
  // bottomIntake.spin(reverse, 1, volt);
  chassis.turn_to_angle(180);
  // bottomIntake.spin(reverse, 12, volt);
  chassis.set_drive_constants(4, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(12.5, 1000);
  bottomIntake.spin(fwd, 12, volt);
  wait(1, sec);

  bottomIntake.spin(fwd, 0, volt);
  topIntake.spin(fwd, 0, volt);
  hood = true; 
  // bottomIntake.spin(fwd, 12, volt);
  chassis.set_drive_constants(6, 1.4, 0, 12, 0);
  // chassis.drive_distance(-28, {.timeout = 1100});;
  chassis.drive_distance_timeout(-32, 1100);;
  topIntake.spin(fwd, 8.5, volt);
  task intakeTask = task(intakeAntiJamTask);
  // hood.set(true);
  scraper.set(false); 
  topIntake.spin(fwd, 8.5, volt);
  wait(3.2, sec);
  intakeTask.stop();
  bottomIntake.spin(fwd, 0, volt);

  //push
  // chassis.set_drive_constants(12, 1.4, 0, 12, 0);
  // chassis.drive_distance(10, 245);
  // chassis.drive_distance(-30, 180);
  // bottomIntake.spin(fwd, 12, volt);
  // topIntake.spin(reverse, 10, volt);
}
void skills(){
  scraper = true;
  hood = false;
  descore = true;
  chassis.set_drive_constants(10, 1.4, 0, 12, 0);
  task intakeTask = task(intakeAntiJamTask);
  topIntake.spin(reverse, 12, volt);
  chassis.drive_distance(36);
  chassis.turn_to_angle(90);
  chassis.set_drive_constants(6, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(10.5, 500);
  wait(3, sec); 
  chassis.set_drive_constants(10, 1.4, 0, 12, 0);
  // chassis.turn_to_angle(90);
  topIntake.spin(fwd, 0, volt);
  intakeTask.stop();
  bottomIntake.spin(fwd, 0, volt); 

  //RB Crossover
  RBCrossover(); 

  // chassis.set_heading(-90); 
  postCrossScore();
  skillsCrossover();

  scraper = true;
  hood = false;
  topIntake.spin(reverse, 12, volt);
  intakeTask = task(intakeAntiJamTask); 
  chassis.set_drive_constants(6, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(25,1000);
  wait(3.5, sec);

  chassis.turn_to_angle(-90);
  

  topIntake.spin(fwd, 0, volt);
  intakeTask.stop();
  bottomIntake.spin(fwd, 0, volt);

  chassis.set_heading(90);
  RBCrossover(); 
  postCrossScore();

  

  chassis.drive_distance_timeout(15, 250);
  chassis.turn_to_angle(70,150); 
  chassis.drive_distance(-50, 0);
  chassis.drive_distance_timeout(20,250);
  chassis.drive_distance(-50);
}

void postCrossScore(){
  hood = true;
  chassis.set_drive_constants(12, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(-35, 700);
  topIntake.spin(fwd, 10, volt);
  task intakeTask = task(intakeAntiJamTask);
  scraper.set(true);
  wait(4, sec);
  chassis.turn_to_angle(-90);
  chassis.set_drive_constants(6, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(29, 1400);//take from scraper 
  topIntake.spin(reverse, 8, volt); //store
  hood.set(false);
  wait(3, sec);
  intakeTask.stop();
  topIntake.spin(fwd, 0, volt);
  bottomIntake.spin(fwd, 0, volt); //stop intake motors
  hood = true; 

  chassis.turn_to_angle(-90);
  chassis.set_drive_constants(12, 1.4, 0, 12, 0);
  chassis.drive_distance_timeout(-35, 700);
  intakeTask = task(intakeAntiJamTask);
  topIntake.spin(fwd, 12, volt);
  wait(4, sec);
  intakeTask.stop(); 
  
}
void skillsCrossover(){
  // chassis.turn_to_angle(-90);
  chassis.drive_distance(13,0); 
  // chassis.turn_to_angle(0);  
  chassis.drive_distance(-93.5,0);
  wait(0.2, sec);

  chassis.set_drive_constants(10, 1.4, 0, 12, 0);
  float Distance1 = 16.5 - rearDistance.objectDistance(vex::distanceUnits::in);
  chassis.drive_distance(Distance1);
  chassis.turn_to_angle(-90); 
}

void RBCrossover(){  
  chassis.set_drive_constants(10, 1.4, 0, 12, 0);
  chassis.drive_distance(-13,155); 
  // scraper.set(false);

  // chassis.turn_to_angle(180);
  // chassis.drive_distance(-16);  
  // chassis.set_drive_constants(10, 1.4, 0, 12, 0);
  // chassis.turn_to_angle(90);
  chassis.drive_distance(-92.5,90); 
  chassis.turn_to_angle(180);
  // float a = c;
  hood = true; 

  float Distance1 = 19 - rearDistance.objectDistance(vex::distanceUnits::in); 
  // float Distance2 = 10 -rightDistance.objectDistance(inch); 
  chassis.set_drive_constants(8, 1.4, 0, 12, 0);
  chassis.drive_distance(Distance1);
  chassis.turn_to_angle(-90) ;

}