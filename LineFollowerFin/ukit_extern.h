// ukit_extern.h — extern declarations + full class definitions for globals from ucode.h.
// Include this in .cpp files along with ukit_macros.h.
#ifndef UKIT_EXTERN_H
#define UKIT_EXTERN_H

#include <Arduino.h>
#include "uKitMotor.h"
#include "uKitServo.h"
#include "uKitSensor.h"
#include "uKitId.h"
#include "onBoardHW.h"
#include "TransforRobot.h"
#include "VisionDiscrim.h"
#include "ClickButton.h"

// Extern globals from ucode.h (defined in LineFollowerFin.ino via ucode.h)
extern uKitMotor uKitMotor;
extern uKitServo uKitServo;
extern uKitSensor uKitSensor;
extern uKitId uKitId;
extern OnBoardHW Sensor;
extern TransforRobot transfor_robot;
extern VisionDiscrim mVisionDiscrim;
extern ClickButton button1;

// Global state variables
extern bool protocolRunState;
extern bool flexiTimerFlag;
extern bool butonTimer;
extern String inputString;
extern String strLoopCmd;
extern int timeTimes;
extern int timeFlag;
extern int buttonFlag;

#endif