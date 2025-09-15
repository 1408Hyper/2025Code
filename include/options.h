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

// MOTORS: BOT, MID, TOP: 10, 17, 11

// (Intake) R1: normal spin BOT and normal spin MID
// (Bot Goal) R2: reverse spin MID and reverse spin BOT
// (Mid Goal) L2: reverse spin MID and normal spin BOT
// (Top Goal) L1: reverse spin MID and normal spin BOT and reverse spin TOP

#define BOT_PORTS {10}
#define MID_PORTS {17}
#define TOP_PORTS {11}

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON false
#define DO_SKILLS_AUTON false

// Turn on for skills prep/post auton/opcontrol functions to be run on components
#define DO_SKILLS_PREP true
#define DO_POST_AUTON true
#define DO_OP_CONTROL true

#define LEFT_DRIVE_PORTS {-15, -14, -13}
#define RIGHT_DRIVE_PORTS {18, 8, 21}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_
