/*
 * UCF RSLK Library
 * Gyro Drive Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) driving 
 *
 * How to run:
 * 1) Push left button on Launchpad to start the demo
 * 2) Robot will drive forward by a predefined distance
 * 3) Once distance has been reached the robot will stop
 * 4) Push left button again to start demo again
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://github.com/UCFInnovationLab/UCF_RSLK_Library
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * Created in the UCF TI Innovation Lab
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

float P = 1.0;
float wheelDiameter = 2.5;      // Diameter of Romi wheels in inches
int cntPerRevolution = 360;   // Number of encoder (rising) pulses every time the wheel turns completely
int wheelSpeed = 15;            // Default raw pwm speed for motor.

float initialHeading;

enum states {
 START,
 PRE_LEG1,
 LEG1,
 TURN1,
 DONE,
 SLEEP
};

enum states curState = START;

String btnMsg = " ";

unsigned long lastTime = 0;

void setup() {
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);
	Serial.begin(115200);

	setupRSLK();

  /* Set the encoder pulses count back to zero */
	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	setupWaitBtn(LP_LEFT_BTN);  // Left button on Launchpad
	setupLed(RED_LED);  // led red

	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);   // direction: forward
	enableMotor(BOTH_MOTORS);       // enable both motors
	setMotorSpeed(BOTH_MOTORS,0);   // set motor speed

  delay(1000);
  bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure
  initialHeading = float(myEulerData.h) / 16.00;

  Serial.print("Initial Heading(Yaw): ");				//To read out the Heading (Yaw)
  Serial.println(initialHeading);
  
  Serial.println("Starting");
}

void loop() {

  delay(50);

  if ((millis() - lastTime) >= 1000) //To stream at 10Hz without using additional timers
  {
    Serial.println(curState);
    lastTime = millis();
  }

  //Serial.println(curState);
  switch (curState) {
    case START: 
      Serial.println("Start");
	    /* Wait until button is pressed to start robot */
	    waitBtnPressed(LP_LEFT_BTN,"\nPush left button on Launchpad to start demo.\n",RED_LED);
      curState = PRE_LEG1;
    break;

    case PRE_LEG1:
    	resetLeftEncoderCnt();
	    resetRightEncoderCnt();
      curState = LEG1;
    break;

    case LEG1:
      if (driveToDistanceHeading(15,0,15)) {
        curState = TURN1;
      };
    break;

    case TURN1:
      if (turnTo(90,10)) {
        curState = DONE;
      }
    break;

    case DONE:	/* Halt motors */
      Serial.println("DONE");
	    disableMotor(BOTH_MOTORS);
      curState = SLEEP;
    break;

    case SLEEP:
    break;
  }
}

/*
 * Drive distance
 */
boolean driveToDistanceHeading(float inches, int desiredHeading, int speed) {
  uint16_t ticks = distanceToEncoder(wheelDiameter, cntPerRevolution, inches);

  int headingError = calculateDifferenceBetweenAngles(desiredHeading, getCurrentRealtiveHeadingToStart());
  int adjustSpeed = headingError * P;
  adjustSpeed = constrain(adjustSpeed,-5,5);
  /* Set motor speed */
	setMotorSpeed(LEFT_MOTOR,constrain(speed+adjustSpeed,0,100));
  setMotorSpeed(RIGHT_MOTOR,constrain(speed-adjustSpeed,0,100));

  int totalCount = (getEncoderLeftCnt() + getEncoderRightCnt()) / 2;
	if (totalCount > ticks) {
	  setMotorSpeed(BOTH_MOTORS,0);  // Halt motors
    return true;
	}
  return false;
}

/*
 * Turn degrees
 */
boolean turnTo(int degrees, int speed) {
  
  int headingError = calculateDifferenceBetweenAngles(degrees, getCurrentRealtiveHeadingToStart());

  Serial.print("Heading(Yaw): ");				//To read out the Heading (Yaw)
  Serial.println(headingError);		

  if (headingError >= 0) {
      Serial.println("TURN Right");
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
	    if (headingError <= 0) {
	      setMotorSpeed(BOTH_MOTORS,0);  // Halt motors
        return true;
      }
	} else {
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
      	if (headingError >= 0) {
	        setMotorSpeed(BOTH_MOTORS,0);  // Halt motors
          return true;
      }
  }

  return false;
}

int getCurrentRealtiveHeadingToStart(){
  bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure
  float difference = (float(myEulerData.h) / 16.00) - initialHeading;
  return ((int)difference + 360) % 360;
} 

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the difference between two angles in a 0-360 system
 * - returns +-179
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
   int delta;

    delta = (angle1 - angle2 + 360) % 360;
       if (delta > 180) delta = delta - 360;

     return delta;
}
