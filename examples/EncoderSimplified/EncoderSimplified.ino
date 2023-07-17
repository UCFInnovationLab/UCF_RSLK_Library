/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Simplified Encoder Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) driving forward
 * by a predefined distance utilizing wheel encoders.
 *
 * How to run:
 * 1) Push left button on Launchpad to start the demo
 * 2) Robot will drive forward by a predefined distance
 * 3) Once distance has been reached the robot will stop
 * 4) Push left button again to start demo again
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"

/* Diameter of Romi wheels in inches */
float wheelDiameter = 3.0;

/* Number of encoder (rising) pulses every time the wheel turns completely */
int cntPerRevolution = 360;

/* How far in inches for the robot to travel */
int inchesToTravel = 6;

int wheelSpeed = 15; // Default raw pwm speed for motor.



void setup() {
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
}

void loop() {
	uint16_t totalCount = 0; // Total amount of encoder pulses received

	/* Amount of encoder pulses needed to achieve distance */
	uint16_t x = countForDistance(wheelDiameter, cntPerRevolution, inchesToTravel);
	String btnMsg = "Expected count: ";
	btnMsg += x;

	/* Wait until button is pressed to start robot */
	btnMsg += "\nPush left button on Launchpad to start demo.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

	delay(2000);

	/* Set the encoder pulses count back to zero */
	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	/* Cause the robot to drive forward */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);

	/* "Turn on" the motor */
	enableMotor(BOTH_MOTORS);

	/* Set motor speed */
	setMotorSpeed(BOTH_MOTORS,wheelSpeed);

	/* Drive motor until it has received x pulses */
	while(totalCount < x)
	{
		totalCount = getEncoderLeftCnt();
		Serial.println(totalCount);
	}

	/* Halt motors */
	disableMotor(BOTH_MOTORS);
}
