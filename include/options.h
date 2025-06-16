// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)

// Main opcontrol function to use
#define CURRENT_OPCONTROL mainControl

//Sensor ports 

// Ports for telemetry
// IMU
#define IMU_PORT 12
// Rotary encoder
#define ROT_DRIVE_PORT 2
// AI Vision
#define AI_VISION_PORT 3
// GPS Port
#define GPS_PORT 4

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON false
#define DO_SKILLS_AUTON true

// Turn on for skills prep/post auton/opcontrol functions to be run on components
#define DO_SKILLS_PREP true
#define DO_POST_AUTON true
#define DO_OP_CONTROL true

// Ports for the drivetrain motors
#define LEFT_DRIVE_PORTS {20, 19, 18}
#define RIGHT_DRIVE_PORTS {-17, -16, -14}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
