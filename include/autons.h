#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void awp_right();
void notAwp_left();
void drive_test();
void turn_test();
void swing_test();
void antiJamTask();

void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
int intakeAntiJamTask(); 
int colorOutakeTask();

void right7BallPush();
void right7BallFast();
void left7BallFast();  
void leftAWP(); 
void soloAWP();
void right4BallFast();

void skills();
void postCrossScore(); 
void skillsCrossover();
void RBCrossover(); 

void colorTest();
void test(); 