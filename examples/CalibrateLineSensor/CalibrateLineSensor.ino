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
 * 1) Push left button on Launchpad to have the robot perform calibration.
 * 2) Robot will drive forwards and backwards by a predefined distance.
 */

#include "SimpleRSLK.h"

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

void setup()
{
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
	clearMinMax(sensorMinVal,sensorMaxVal);
}

void floorCalibration() {
	/* Place Robot On Floor (no line) */
	delay(2000);
	String btnMsg = "Push left button on Launchpad to begin calibration.\n";
	btnMsg += "Make sure the robot is on the floor away from the line.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

	delay(1000);

	Serial.println("Running calibration on floor");
	simpleCalibrate();
	Serial.println("Reading floor values complete");

  Serial.print("Max Values: {");
  Serial.print(sensorMaxVal[0]);
  for (int i=1;i<8;i++) {
    Serial.print(", ");
    Serial.print(sensorMaxVal[i]);
  }
  Serial.println("}");

  Serial.print("Min Values: {");
  Serial.print(sensorMinVal[0]);
  for (int i=1;i<8;i++) {
    Serial.print(", ");
    Serial.print(sensorMinVal[i]);
  }
  Serial.println("}");
}

void simpleCalibrate() {
	/* Set both motors direction forward */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
	/* Enable both motors */
	enableMotor(BOTH_MOTORS);
	/* Set both motors speed 20 */
	setMotorSpeed(BOTH_MOTORS,20);

	for(int x = 0;x<150;x++){
		readLineSensor(sensorVal);
		setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
	}

	/* Disable both motors */
	disableMotor(BOTH_MOTORS);
}

bool isCalibrationComplete = false;
void loop()
{
	uint16_t normalSpeed = 10;
	uint16_t fastSpeed = 20;

	/* Valid values are either:
	 *  DARK_LINE  if your floor is lighter than your line
	 *  LIGHT_LINE if your floor is darker than your line
	 */
	uint8_t lineColor = DARK_LINE;

	/* Run this setup only once */
	if(isCalibrationComplete == false) {
		floorCalibration();
		isCalibrationComplete = true;
	}
}