#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S3,     SENSOR_GYRO,    sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     SENSOR_IR,      sensorHiTechnicIRSeeker1200)
#pragma config(Motor,  mtr_S1_C1_1,     motorFR,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     motorFL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motorBL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     motorBR,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_1,     motorLR,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_2,     motorLL,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     motorM,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     motorKek,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    sBasketR,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    sBasketL,             tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    sTubeGrabber,         tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "libsModified_14.h";
#include "JoystickDriver.c";
#include "6210_autonomous_functions.h";

/* Auto60PZ.c
* Parking zone autonomous bringing 60cm to parking zone and scoring
* Created by by 6210 Stryke
* 2014-2015 */

task main()
{
	waitForStart();
	openGrabber(); //Open grabber completely
	moveTo(-40, -10000, 0.8, 5000);//move to 60cm rolling goal
	wait1Msec(500);
	closeGrabber(); //Close grabber on tube rim
	moveTo(50, 8900, 0.8, 5000); //move to parking zone
	turn(30, 110, 5000); //turn 180 degrees clockwise
	setBasket(1);
	liftControl(1);
	wait1Msec(2600); //time it takes for the lift to raise to proper height
	liftControl(2);
	setBasket(3);
	wait1Msec(1000); //wait for servos to finish motion
	setBasket(1);
	wait1Msec(500);
}
