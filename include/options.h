// All the options for the robot

#ifndef _HYPER_OPTIONS_H_
#define _HYPER_OPTIONS_H_

// Variables (u can change these!!)

// Main opcontrol function to use
#define CURRENT_OPCONTROL mainControl

// Analog ports

#define FORK_MECH_PORT 'G'
#define DESCORE_MECH_PORT 'H'
#define BALL_BLOCKER_MECH_PORT 'F'

// Telemetry ports

// IMU
#define IMU_PORT 17
// Rotary encoder (Lateral)
#define LAT_ROT_DRIVE_PORT 12
// AI Vision
#define AI_VISION_PORT 6
// GPS Port
#define GPS_PORT 4

// Optical Sensors for Dynamic Screen
#define SCREEN_TOP_PORT 6

// Primary Disperser MGs

// (Intake) R1: normal spin BOT and normal spin MID
// (Bot Goal) R2: reverse spin MID and reverse spin BOT
// (Mid Goal) L2: reverse spin MID and normal spin BOT
// (Top Goal) L1: reverse spin MID and normal spin BOT and reverse spin TOP

#define DISP_BOT_PORTS {11}
#define DISP_MID_PORTS {17}
#define DISP_TOP_PORTS {9}
#define DISP_SCORING_PORT {9, -11}

// Turn on/off auton and opcontrol
#define DO_MATCH_AUTON true
#define DO_SKILLS_AUTON false

// Turn on for skills prep/post auton/opcontrol functions to be run on components
#define DO_SKILLS_PREP true
#define DO_POST_AUTON true
#define DO_OP_CONTROL true

// Drivetrain Ports

#define LEFT_DRIVE_PORTS {-15, -14, -13}
#define RIGHT_DRIVE_PORTS {20, 8, 21}

// Chassis class to use (default is initDefaultChassis)
#define INIT_CHASSIS initDefaultChassis

#endif // _HYPER_OPTIONS_H_