/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
 * using a basic line following algorithm. This example works on a dark floor with
 * a white line or a light floor with a dark line. The robot first needs to be calibrated
 * Then place the robot on the hit the left button again to begin the line following.
 *
 * How to run:
 * 1) Place the robot center on the line you want it to follow.
 * 2) Push left button again to have the robot begin to follow the line.
 *
 * Parts Info:
 * o Black eletrical tape or white electrical tape. Masking tape does not work well
 *   with IR sensors.
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

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
uint16_t sensorMinVal[LS_NUM_SENSORS] = {805, 1047, 759, 955, 790, 1154, 931, 1245};

#define GOAL 3500       // value of normalized light sensor to follow
#define BASE_SPEED 10   // base robot speed.  range 0-100
#define P .01          // proportional term used in the PID controller

void setup()
{
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
	clearMinMax(sensorMinVal,sensorMaxVal);
  initialSetup();
}

void initialSetup() {
	String btnMsg = "Push left button on Launchpad to begin line following.\n";
	btnMsg += "Make sure the robot is on the line.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
	delay(200);

	enableMotor(BOTH_MOTORS);
}

void loop()
{
	/* Valid values are either:
	 *  DARK_LINE  if your floor is lighter than your line
	 *  LIGHT_LINE if your floor is darker than your line
	 */
	uint8_t lineColor = DARK_LINE;

	readLineSensor(sensorVal);

  /*
   * Take current sensor values and adjust using previous calibration values
   * Output: sensorCalVal
   */
	readCalLineSensor(sensorVal,
					  sensorCalVal,
					  sensorMinVal,
					  sensorMaxVal,
					  lineColor);

	uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

  /* use PID algorithm to calculate speed delta */
  int error = linePos - GOAL;
  int motor_speed_delta = P * error;
    Serial.print(linePos);
    Serial.print(", ");
    Serial.print(error);
    Serial.print(", ");
    Serial.print(motor_speed_delta);
    Serial.println();

  /* add and subtract delta from left and right sides.  Cap motors at 100 */
  int temp = BASE_SPEED + motor_speed_delta;
  int left_motor_speed = (temp > 100) ? 100 : temp; 
  temp = BASE_SPEED - motor_speed_delta;
  int right_motor_speed = (temp < 0) ? 0 : temp; 

  setMotorSpeed(LEFT_MOTOR, left_motor_speed);
  setMotorSpeed(RIGHT_MOTOR, right_motor_speed);
}