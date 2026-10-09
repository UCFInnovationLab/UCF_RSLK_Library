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

// Uncomment to use an MPU6050 IMU (via I2Cdevlib DMP) instead of the default BNO055.
// Install "MPU6050" by UCF Innovation Lab: https://github.com/ucfinnovationlab/mpu6050/releases
//#define USE_MPU6050

#include "SimpleRSLK.h"
#include "UCF_RSLK.h"
#ifdef USE_MPU6050
  #include "I2Cdev.h"
  #include "MPU6050_6Axis_MotionApps20.h"
#else
  #include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
                                                //Install "BNO055" by Robert Bosch GMBH via Library Manager.
#endif
#include <Wire.h>

#ifdef USE_MPU6050
  MPU6050 mpu;                  // Default I2C address 0x68
  bool DMPReady = false;
  uint8_t devStatus;
  uint16_t packetSize;
  uint8_t FIFOBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];                 // [yaw, pitch, roll], radians
  float currentYaw = 0;         // last known yaw in degrees, updated as new DMP packets arrive
#else
  //The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
  // /* bno055 I2C Address */
  // #define BNO055_I2C_ADDR1                0x28
  // #define BNO055_I2C_ADDR2                0x29
  // #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

  //This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
  struct bno055_t myBNO;
  struct bno055_euler myEulerData; //Structure to hold the Euler data
#endif

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

#ifdef USE_MPU6050
  #if !defined(__MSP432P401R__) && !defined(__MSP432__)
    Wire.setClock(400000);      // 400kHz I2C clock (skip on MSP432 -- matches GNOR_V4's tested config)
  #endif

  mpu.reset();
  delay(100);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (true);
  }

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    mpu.setIntEnabled(0);       // polling mode, no interrupt pin used
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("MPU6050 DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
#else
  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);
#endif
	Serial.begin(115200);

	setupRSLK();

  /* Set the encoder pulses count back to zero */
	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	setupWaitBtn(LP_LEFT_BTN);  // Left button on Launchpad
	setupLed(RED_LED);  // led red

  /* Initialize motors */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
	enableMotor(BOTH_MOTORS);
	setMotorSpeed(BOTH_MOTORS,0);

  /* Read initial heading */
  delay(1000);
  initialHeading = readHeading();
  Serial.print("Initial Heading(Yaw): ");				//To read out the Heading (Yaw)
  Serial.println(initialHeading);
  
  Serial.println("Starting");
}

void loop() {

  delay(50);

  if ((millis() - lastTime) >= 10) //To stream at 10Hz without using additional timers
  {
    Serial.println(curState);
    lastTime = millis();
  }

  //Serial.println(curState);
  switch (curState) {
    case START: 
      Serial.println("Start");
	    /* Wait until button is pressed to start robot */
	    waitBtnPressedString(LP_LEFT_BTN,"\nPush left button on Launchpad to start demo.\n",RED_LED);
      curState = PRE_LEG1;
    break;

    case PRE_LEG1:
    	resetLeftEncoderCnt();
	    resetRightEncoderCnt();
      curState = LEG1;
    break;

    case LEG1:
      if (driveToDistanceHeading(12,0,15)) {
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
  Serial.print("Drive: Heading Error");
  Serial.println(headingError);
  int adjustSpeed = headingError * P;
  adjustSpeed = constrain(adjustSpeed,-10,10);
  /* Set motor speed */
  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);   
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
	setMotorSpeed(LEFT_MOTOR,constrain(speed+adjustSpeed,0,100));
  setMotorSpeed(RIGHT_MOTOR,constrain(speed-adjustSpeed,0,100));

  Serial.print("Left motor: "); Serial.print(speed+adjustSpeed);
  Serial.print(" Right motor: "); Serial.println(speed-adjustSpeed);

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


  if (abs(headingError) < 2) {
     setMotorSpeed(BOTH_MOTORS,0);  // Halt motors
      return true;
  }

  Serial.print("Turn: Heading Error: ");				//To read out the Heading (Yaw)
  Serial.println(headingError);		

  if (headingError >= 0) {
      Serial.println("TURN Right");
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
	} else {
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
  }

  return false;
}

/*
 * Read current raw heading in degrees from whichever IMU is active.
 */
float readHeading() {
#ifdef USE_MPU6050
  if (DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentYaw = ypr[0] * 180.0 / M_PI;
  }
  return currentYaw;
#else
  bno055_read_euler_hrp(&myEulerData);
  return float(myEulerData.h) / 16.00;
#endif
}

int getCurrentRealtiveHeadingToStart(){
  float difference = readHeading() - initialHeading;
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
