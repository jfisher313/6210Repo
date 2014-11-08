//Software designed by John Stegman
//Adapted and revised by 6299 Quad X
//Expanded and documented by 6210 Stryke

//Gyroscope Autonomous Method Library

#include "drivers\hitechnic-gyro.h";
#include "drivers\hitechnic-irseeker-v2.h"

float heading = 0;

//Tests if the the absolute value of the val parameter is less than the threshold and returns a boolean
//Threshold is 1.0
float valInRange(float val, float threshold = 1.0) {
	return (abs(val) <= threshold) ? 0 : val;
}

//
bool isInRange(float heading, float targetHeading, float threshold = 1.0) {
	return abs(heading - targetHeading) <= threshold;
}

int getDriveDir(int power) {
	return (power > 0) ? 1 : (power < 0) ? -1 : 0;
}

//Averages out the two paramters entered and returns a integer
//Failsafe if either left-side motor encoder or right-side motor encoder dies
int getEncoderAverage(int leftMotor, int rightMotor) {
	if (abs(leftMotor) < 3) {
		return rightMotor;
	}
	else if (abs(rightMotor) < 3) {
		return leftMotor;
	}
	return (leftMotor + rightMotor) / 2;
}

//Sets the left-side drive motors to the first parameter entered and the right-side drive motors to the second parameter
void setMotors(int left, int right) {
	motor[motorFL] = left;
	motor[motorBL] = left;
	motor[motorFR] = right;
	motor[motorBR] = right;
}

//Stops all of the drive motors
void stopMotors() {
	motor[motorFL] = 0;
	motor[motorBL] = 0;
	motor[motorFR] = 0;
	motor[motorBR] = 0;
}

//Moves the robot forward or backward the amount of degrees entered in the deg parameter at the power of the power parameter
//In order for the robot to move backward make the power and deg parameters negative
//The robot will correct its direction by changing motor speeds if the robot's drift in degrees gets out of the threshold parameter
//If the method runs for more than the time paratemeter the method will stop as a failsafe in case the encoders fail
void moveTo(int power, int deg, float threshold = 2.0, long time = 5000) {
	heading = 0;
	nMotorEncoder[motorFL] = 0;
	nMotorEncoder[motorFR] = 0;

	wait1Msec(500);
	HTGYROstartCal(SENSOR_GYRO);
	wait1Msec(500);

	clearTimer(T1);

	if (power > 0) {
		while (time1[T1] < time && getEncoderAverage(nMotorEncoder[motorFL], nMotorEncoder[motorBL]) < deg) {
			displayCenteredBigTextLine(3, "%2i", nMotorEncoder[motorBL]);
			// Reads gyros rate of turn, mulitplies it by the time passed (20ms), and adds it to the current heading
			heading += valInRange(HTGYROreadRot(SENSOR_GYRO), threshold) * (float)(20 / 1000.0);

			// Checks if the gyro is outside of the specified threshold (1.0)
			if (isInRange(heading, 0, threshold)) {
				setMotors(power, power);
			}

			// If not, lower the speed of the required side of the robot to adjust back to 0
			else {
				if (heading > 0) {
					setMotors((power / 4) * getDriveDir(power), power);
				}
				if (heading < 0) {
					setMotors(power, (power / 4) * getDriveDir(power));
				}
			}
			wait1Msec(20);
		}
	}

	else {
		while (time1[T1] < time && getEncoderAverage(nMotorEncoder[motorFL], nMotorEncoder[motorFR]) > deg) {
			// Reads gyros rate of turn, mulitplies it by the time passed (20ms), and adds it to the current heading
			heading += valInRange(HTGYROreadRot(SENSOR_GYRO), threshold) * (float)(20 / 1000.0);

			// Checks if the gyro is outside of the specified threshold (1.0)
			if (isInRange(heading, 0, threshold)) {
				setMotors(power, power);
			}

			// If not, lower the speed of the required side of the robot to adjust back to 0
			else {
				if (heading > 0) {
					setMotors(power, (power / 4) * getDriveDir(power));
				}
				if (heading < 0) {
					setMotors((power / 4) * getDriveDir(power), power);
				}
			}

			wait1Msec(20);
		}
	}

	stopMotors();
}

//The robot will turn for the degrees given in the deg parameter on a point at the power of the power parameter
//If the method runs for more than the time parameter the method will stop for a failsafe if the gyro fails
void turn(int power, int deg, int time = 5000) {

	// 90 Degree Modifier
	if (abs(deg) == 90) {
		int modifier = deg * 8/9;
		deg = modifier;
	}

	// 45 Degree Modifier
	else if (abs(deg) == 45) {
		int modifier = deg * 7/9;
		deg = modifier;
	}

	heading = 0;

	wait1Msec(500);
	HTGYROstartCal(SENSOR_GYRO);
	wait1Msec(500);

	clearTimer(T1);

	if (deg > 0) { //if degree of turn is positive
		while (time1[T1] < time && abs(heading) < abs(deg)) { //while robot timer is greater than time constant and relative heading is less than degree turn
			heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0); //add rotation to heading
			setMotors(power, -power);
			wait1Msec(20);
		}
	}

	if (deg < 0) {
		while (time1[T1] < time && abs(heading) < abs(deg)) {
		heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0);
			setMotors(-power, power);
			wait1Msec(20);
		}
	}

	stopMotors();
}

//Turns on one stopped motor for the degrees given in the deg parameter at the power of the power parameter
//If the method runs for more than the time parameter the method will stop for a failsafe in the gyro fails
void arcTurn(int power, int deg, int time = 2000) {

	// 90 Degree Modifier
	if (abs(deg) == 90) {
		int modifier = deg * 8/9;
		deg = modifier;
	}

	// 45 Degree Modifier
	else if (abs(deg) == 45) {
		int modifier = deg * 7/9;
		deg = modifier;
	}

	heading = 0;
	clearTimer(T1);
	HTGYROstartCal(SENSOR_GYRO);

	// Forward arcTurn
	if (power > 0) {
		if (deg > 0) {
			while (time1[T1] < time && abs(heading) < abs(deg)) {
				heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0);
				setMotors(power, 0);
				wait1Msec(20);
			}
		}

		else {
			while (time1[T1] < time && abs(heading) < abs(deg)) {
				heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0);
				setMotors(0, power);
				wait1Msec(20);
			}
		}
	}

	// Backward arcTurn (flips inequalities)
	else {
		if (deg > 0) {
			while (time1[T1] < time && abs(heading) < abs(deg)) {
				heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0);
				setMotors(power, 0);
				wait1Msec(20);
			}
		}

		else {
			while (time1[T1] < time && abs(heading) < abs(deg)) {
				heading += HTGYROreadRot(SENSOR_GYRO) * (float)(20 / 1000.0);
				setMotors(0, power);
				wait1Msec(20);
			}
		}
	}
	stopMotors();
}

/*void latch(bool position) {
	if (!position) {
		servo[servoL] = 225;
		servo[servoR] = 0;
	}
	if (position) {
		servo[servoL] = 160;
		servo[servoR] = 83;
	}
}*/

//Returns IR value
int getIR(){
	return HTIRS2readACDir(SENSOR_IR);
}

//Moves the robot until the IR value becomes greater then the IR parameter
void moveIrUp(int speed, int IR,){
	while(getIR() < IR){
		setMotors(speed, speed);
	}
	stopMotors();
}

//Moves the robot until the IR value becomes less then the IR parameter
void moveIrDown(int speed, int IR){
	while(getIR() < IR){
		setMotors(speed, speed);
	}
	stopMotors();
}

/*
* Additional Functions
* FTC 6210 Stryke
* 10/14/2014
*/

//calculate and return encoder degree values based on radius of wheel and distance of motion desired
int getDegrees(float distance, float radius = 2.0) {
	float circumference = 2 * PI * radius;
	return (distance / circumference) * 1440;
}

//alternate moveTo functionality to incorporate getDegrees automatically without requiring public use of getDegrees
void moveToDistance(int speed, float distance, float radius, float threshold = 2.0, long time = 5000) {
	moveTo(speed, getDegrees(radius, distance), threshold, time);
}

//alternate moveIR functionality to ignore Up/Down direction. May or may not be practical
void moveIR(int speed, int IR) {
	while (getIR() != IR) {
		setMotors(speed, speed);
	}
	stopMotors();
}

//experimental turn function to attempt to fix accuracy issues
void altTurn(int power, int deg, int time = 5000) {

	// 90 Degree Modifier
/*
	if (abs(deg) == 90) {
		int modifier = deg * 8/9;
		deg = modifier;
	}

	// 45 Degree Modifier
	else if (abs(deg) == 45) {
		int modifier = deg * 7/9;
		deg = modifier;
	}
*/
	heading = 0;

	wait1Msec(500);
	HTGYROstartCal(SENSOR_GYRO);
	wait1Msec(500);

	clearTimer(T1);

	if (deg > 0) { //if degree of turn is positive
		while (time1[T1] < time && abs(heading) < abs(deg)) {

			setMotors(power, -power);
			//flipped line above and below
			heading += HTGYROreadRot(SENSOR_GYRO) * (float)(10 / 1000.0); //decreased time to 10 ms accuracy

			wait1Msec(10); //decreased time to 10 ms accuracy
		}
	}

	if (deg < 0) {
		while (time1[T1] < time && abs(heading) < abs(deg)) {

		setMotors(-power, power);

		heading += HTGYROreadRot(SENSOR_GYRO) * (float)(10 / 1000.0);

		wait1Msec(10);
		}
	}
	stopMotors();
}
