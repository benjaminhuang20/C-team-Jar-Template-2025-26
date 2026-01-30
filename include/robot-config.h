using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

extern motor leftFront; 
extern motor leftBack;
extern motor leftFlipped;

extern motor rightFront; 
extern motor rightBack;
extern motor rightFlipped;
//Add your devices below, and don't forget to do the same in robot-config.cpp:

extern motor bottomIntake;
extern motor topIntake;

extern digital_out scraper;
extern digital_out descore;
extern digital_out hood;

extern optical intakeColor;

// extern distance rearDistance;

extern bool isIntaking;

extern bool auto_started; 

extern bool isRed; 

extern int current_auton_selection;

extern controller Controller;

void  vexcodeInit( void );
