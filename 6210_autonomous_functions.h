/* 6210_autonomous_functions.h
* Various Autonomous Control Functions for PH14 robot
* Created by by 6210 Stryke
* 2014-2015 */

//Raises the tube grabber
void openGrabber() {
	servo[sTubeGrabberL] = 210;
	servo[sTubeGrabberR] = 0;
	wait1Msec(200);
}

//Lowers the tube grabber
void closeGrabber() {
	servo[sTubeGrabberL] = 148;
	servo[sTubeGrabberR] = 60;
	wait1Msec(200);
}

//Sets the basket to one of 3 positions
void setBasket(int preset) {
	if (preset == 1) {
		servo[sBasketL] = 240; //set basket to down state
		servo[sBasketR] = 15;
	}
	else if (preset == 2) {
			servo[sBasketL] = 205; //tilt basket so balls don't fall out
			servo[sBasketR] = 50;
	}
	else if (preset == 3) {
		servo[sBasketL] = 120; //dump balls in basket
		servo[sBasketR] = 135;
	}
	wait1Msec(300);
}

//Moves the lift to one of 3 positions
void liftControl(int preset) {
	if (preset == 1) {
		motor[motorLL] = 85; //Lift basket
		motor[motorLR] = 85;
	}
	else if (preset == 2) {
		motor[motorLL] = 0; //Stop basket
		motor[motorLR] = 0;
		wait1Msec(200);
	}
	else if (preset == 3) {
		motor[motorLL] = -45; //Lower basket
		motor[motorLR] = -45;
	}
}

//Rotates the treads in, stops them, or out
void treadControl(int preset) {
	if (preset == 1) {
		motor[motorML] = 50; //treads in
		motor[motorMR] = -50;
	}
	else if (preset == 2) {
		motor[motorML] = 0; //stop treads
		motor[motorMR] = 0;
	}
	else if (preset == 3) {
		motor[motorML] = -50; //treads out
		motor[motorMR] = 50;
	}
}
