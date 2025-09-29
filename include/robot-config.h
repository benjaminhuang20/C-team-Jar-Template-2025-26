using namespace vex;

extern brain Brain;

extern motor frontLeft; 
extern motor MiddleLeft;
extern motor UpsidedownLeft;

extern motor_group chassisLeft; 

extern motor frontRight; 
extern motor MiddleRight;
extern motor UpsidedownRight;

extern motor_group chassisRight;

extern motor bottomIntake;
extern motor middleIntake;
extern motor hopperIntake;

extern digital_out scraper;
extern digital_out hopperValve;
// extern digital_out descore; 

extern controller Controller; 
//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:



void  vexcodeInit( void );