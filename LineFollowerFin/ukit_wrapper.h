// ukit_wrapper.h — minimal wrapper for uKit library.
// Include this INSTEAD of ucode.h in header files.
// LineFollowerFin.ino still includes ucode.h directly (single definition).
#ifndef UKIT_WRAPPER_H
#define UKIT_WRAPPER_H

#include <Arduino.h>
// Base classes first (dependency order matters)
#include "uKitId.h"
#include "uKitSensor.h"
#include "uKitMotor.h"
#include "uKitServo.h"
#include "onBoardHW.h"
#include "TransforRobot.h"
#include "VisionDiscrim.h"
#include "ClickButton.h"
#include "KalmanMPU6050.h"  // for IMU

// Global objects from ucode.h (defined in .ino via ucode.h)
extern OnBoardHW Sensor;
extern TransforRobot transfor_robot;
extern uKitMotor uKitMotor;
extern uKitServo uKitServo;
extern uKitSensor uKitSensor;
extern uKitId uKitId;
extern VisionDiscrim mVisionDiscrim;
extern ClickButton button1;

// Global state variables from ucode.h
extern bool protocolRunState;
extern bool flexiTimerFlag;
extern bool butonTimer;
extern String inputString;
extern String strLoopCmd;
extern int timeTimes;
extern int timeFlag;
extern int buttonFlag;

// Macros from ucode.h (text substitution, work anywhere)
#define setServoTurn(id,dir,speed) uKitServo.setServoTurn(id,dir,speed)
#define setServoAngles(ids, angles, times, length) uKitServo.setServoAngles(ids, angles, times, length)
#define setServoAnglesWait(ids, angles, times, length, wait) uKitServo.setServoAnglesWait(ids, angles, times, length, wait)
#define setServoAngle(id,angle,times) uKitServo.setServoAngle(id,angle,times)
#define setServoStop(id) uKitServo.setServoStop(id)
#define setServosStop(ids, length) uKitServo.setServosStop(ids, length)
#define readServoAnglePD(id) uKitServo.readServoAnglePD(id)
#define readServoOriginalAnglePD(id) uKitServo.readServoOriginalAnglePD(id)
#define readServoAnglePD_M(read_id,num) uKitServo.readServoAnglePD_M(read_id,num)
#define readServoAngleNPD(id) uKitServo.readServoAngleNPD(id)
#define readServoOriginalAngleNPD(id) uKitServo.readServoOriginalAngleNPD(id)
#define readServoAngleNPD_M(read_id,num) uKitServo.readServoAngleNPD_M(read_id,num)
#define playMotion(id,action,times) uKitServo.playMotion(id,action,times)

#define setEyelightPetalu(id,a,b) uKitSensor.setEyelightPetalu(id,a,b)
#define setEyelightSceneUntil(id,a,b) uKitSensor.setEyelightSceneUntil(id,a,b)
#define setEyelightLookUntil(id,a,b,c,d,e) uKitSensor.setEyelightLookUntil(id,a,b,c,d,e)
#define setUltrasonicRgbled(id,r,g,b) uKitSensor.setUltrasonicRgbled(id,r,g,b)
#define setUltrasonicRgbledOff(id) uKitSensor.setUltrasonicRgbledOff(id)
#define setAllSensorOff() uKitSensor.setAllSensorOff()
#define readLightValue(id) uKitSensor.readLightValue(id)
#define readSoundValue(id) uKitSensor.readSoundValue(id)
#define readInfraredDistance(id) uKitSensor.readInfraredDistance(id)
#define Set_Infrared_Id(id) uKitSensor.Set_Infrared_Id(id)
#define readColorRgb(id) uKitSensor.readColorRgb(id)
#define readColor(id,color) uKitSensor.readColor(id,color)
#define setColorOff(id) uKitSensor.setColorOff(id)
#define setEyelightLook(id,face,times,red,green,blue) uKitSensor.setEyelightLook(id,face,times,red,green,blue)
#define setEyelightScene(id,scene,times) uKitSensor.setEyelightScene(id,scene,times)
#define setEyelightOff(id) uKitSensor.setEyelightOff(id)
#define setEyelightAllPetals(id,red,green,blue) uKitSensor.setEyelightAllPetals(id,red,green,blue)
#define setEyelightPetals(id,petalnum,petals) uKitSensor.setEyelightPetals(id,petalnum,petals)
#define readHumitureValue(id,choice) uKitSensor.readHumitureValue(id,choice)
#define readButtonValue(id) uKitSensor.readButtonValue(id)
#define readUltrasonicDistance(id) uKitSensor.readUltrasonicDistance(id)
#define setNixieTubeFull(id,tpye,method,frequency,times,start,ends) uKitSensor.setNixieTubeFull(id,tpye,method,frequency,times,start,ends)
#define setNixieTube(id,number) uKitSensor.setNixieTube(id,number)
#define redvalue uKitSensor.redvalue
#define greenvalue uKitSensor.greenvalue
#define bluevalue uKitSensor.bluevalue

#define setMotorTurn(id,pwmDuty) uKitMotor.setMotorTurn(id,pwmDuty)
#define setMotorTurnAdj(id,speed,time) uKitMotor.setMotorTurnAdj(id,speed,time)
#define readMotorSpeed(id) uKitMotor.readMotorSpeed(id)
#define setMotorStop(id) uKitMotor.setMotorStop(id)
#define setMotorsStop(ids, length) uKitMotor.setMotorsStop(ids, length)
#define MotorSetID(id_old,id_new) uKitMotor.MotorSetID(id_old,id_new)
#define MotorCheckID(id) uKitMotor.MotorCheckID(id)
#define StopServo() uKitMotor.StopServo()

#define setDeciveId() uKitId.setDeciveId()
#define getDeciveId() uKitId.getDeciveId()
#define getCpuUUID() uKitId.getCpuUUID()

#define Transformer_Car_Forward(a) transfor_robot.forward(a)
#define Transformer_Car_TurnLeft(speed) transfor_robot.turnL(speed)
#define Transformer_Car_TurnRight(speed) transfor_robot.turnR(speed)
#define Transformer_Car_Back(speed) transfor_robot.back(speed)
#define Transformer_Car_Stop() transfor_robot.stops()
#define Transformer_Car_LineTracking_TurnLeft(speed) transfor_robot.turn_L(speed)
#define Transformer_Car_LineTracking_TurnRight(speed) transfor_robot.turn_R(speed)
#define Transformer_Car_LineTracking_TurnAround(speed) transfor_robot.turn_U(speed)
#define Transformer_Car_ForwardWithTime(speed, time) transfor_robot.one_step(speed, time)
#define motion_case(cases, times) transfor_robot.motion_case(cases, times)
#define motion_forward(times) transfor_robot.motion_forward(times)
#define motion_car(times) transfor_robot.motion_car(times)
#define motion_getup(times) transfor_robot.motion_getup(times)
#define motion_sitdown(times) transfor_robot.motion_sitdown(times)
#define motion_back(times) transfor_robot.motion_back(times)
#define motion_left_shift(times) transfor_robot.motion_left_shift(times)
#define motion_right_shift(times) transfor_robot.motion_right_shift(times)
#define motion_turn_left(times) transfor_robot.motion_turn_left(times)
#define motion_turn_right(times) transfor_robot.motion_turn_right(times)
#define motion_omotion(times) transfor_robot.motion_omotion(times)
#define motion_button(times) transfor_robot.motion_button(times)
#define motion_lift(times) transfor_robot.motion_lift(times)
#define motion_lift_hand(times) transfor_robot.motion_lift_hand(times)
#define motion_head(times) transfor_robot.motion_head(times)
#define motion_zero(times) transfor_robot.motion_zero(times)

#define num1 Sensor.num1
#define num2 Sensor.num2
#define num3 Sensor.num3
#define num4 Sensor.num4
#define num5 Sensor.num5
#define getGrayAllValue() Sensor.getGrayAllValue()
#define readGrayValue(num,grayval) Sensor.readGrayValue(num,grayval)
#define setRgbledColor(red,green,blue) Sensor.setRgbledColor(red,green,blue)
#define setcolor(color) Sensor.setcolor(color)
#define readBatteryVoltage() Sensor.readBatteryVoltage()
#define readHcsr04Distance(jp) Sensor.readHcsr04Distance(jp)
#define redPin Sensor.redPin
#define greenPin Sensor.greenPin
#define bluePin Sensor.bluePin
#define Button_pin Sensor.Button_pin
#define GrayscaleNum1 Sensor.GrayscaleNum1
#define GrayscaleNum2 Sensor.GrayscaleNum2
#define GrayscaleNum3 Sensor.GrayscaleNum3
#define GrayscaleNum4 Sensor.GrayscaleNum4
#define GrayscaleNum5 Sensor.GrayscaleNum5
#define IR_S Sensor.IR_S
#define buzzer_pin Sensor.buzzer_pin

#define VisionIdentity(iId, iType, iTimeOut) mVisionDiscrim.VisionIdentity(iId, iType, iTimeOut)
#define VisionEnable(iVal,iType) mVisionDiscrim.VisionEnable(iVal,iType)
#define VisionOffset(iId) mVisionDiscrim.VisionOffset(iId)

// Function declarations (defined in ucode.h, linked from .ino)
void protocol();
void serialEvent();
void Initialization();
void stopDecives();
long ucode_random(long min, long max);
long ucode_random(unsigned long max);
void tone2(uint16_t frequency, unsigned long duration);
void noTone2(int pin);

#endif