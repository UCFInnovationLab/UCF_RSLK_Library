#include "SimpleRSLK.h"
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

float wheelDiameter = 2.5;      // Diameter of Romi wheels in inches
int cntPerRevolution = 360;   // Number of encoder (rising) pulses every time the wheel turns completely

float initialHeading;
#define HIGH_LEVEL 1800       // BLACK light level
#define LOW_LEVEL  350        // WHITE light level
#define LINE_GOAL 0.0         // Value to track while following line (-1.0 to 1.0
#define BASE_SPEED 15         // Default speed of the robot
#define TURN_SPEED 10         // Default turn speed
#define P_FOLLOW 5.0          // Proportional constant for line following
#define P_GYRO_DRIVE 5.0      // Proportional constant for gyro heading following
#define TURN_COUNT 350        // Counts to turn
#define DESIRED_HEADING 0     // Heading to return on (figure it out)

float crashedCount;
int currentCount;
unsigned long lastTime = 0;
String btnMsg = " ";

typedef enum State
{
  START,
  FOLLOW,
  CRASH,
  TURN_SIMPLE,
  TURN_GYRO,
  RESET_TURN,
  FOLLOW_HOME,
  STRAIGHT_HOME,
  GYRO_HOME,
  DONE,
  SLEEP
} State;

State state = START;     // Initial state

void setup()
{
  // Initialize I2C communication
  Wire.begin();

  // Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  // Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);       // Wait for gyro to settle
  Serial.begin(115200);

  setupRSLK();

  // reset encoders
	resetLeftEncoderCnt();
	resetRightEncoderCnt();

  setupWaitBtn(LP_LEFT_BTN);    // Left botton on the Launchpad
  setupLed(RED_LED);            // Use red led to signal waiting for button

  /* Initialize motors */
	setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
	enableMotor(BOTH_MOTORS);
	setMotorSpeed(BOTH_MOTORS,0);

  // Read initial heading
  delay(1000);
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);			//Update Euler data into the structure
  initialHeading = float(myEulerData.h) / 16.00;
  Serial.print("Initial Heading(Yaw): ");				//To read out the Heading (Yaw)
  Serial.println(initialHeading);

  enableMotor(BOTH_MOTORS);
}

// MAIN LOOP // 

void loop()
{
  delay(50);      // Don't loop too fast

  if ((millis() - lastTime) >= 10) // Print state at 10hz
  {
    Serial.println(state);
    lastTime = millis();
  }

  // Run Current State
  switch (state) {
    case START: 
      Serial.println("Start");
	    /* Wait until button is pressed to start robot */
	    waitBtnPressed(LP_LEFT_BTN,"\nPush left button on Launchpad to start challenge.\n",RED_LED);
      state = FOLLOW;
    break;

    // Follow the line
    case FOLLOW:
      follow(P_FOLLOW,BASE_SPEED);
      if (crashed()) {
         state = CRASH;
      }
    break;

    case CRASH:
      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);
      crashedCount = getEncoderRightCnt();
      state = TURN_SIMPLE;
    break;

    case TURN_SIMPLE:
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(LEFT_MOTOR,BASE_SPEED);
      setMotorSpeed(RIGHT_MOTOR,BASE_SPEED);
      currentCount = getEncoderRightCnt();
      Serial.print("Ticks: ");
      Serial.println(currentCount);
      if ( currentCount > crashedCount + TURN_COUNT) {
        setMotorSpeed(LEFT_MOTOR,0);
        setMotorSpeed(RIGHT_MOTOR,0);
        state = RESET_TURN;
      }
    break;

    case TURN_GYRO:
      if (turnTo(90,TURN_SPEED)) {
        state = RESET_TURN;
      }
    break;

    case RESET_TURN:
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, BASE_SPEED+20);
      setMotorSpeed(RIGHT_MOTOR,BASE_SPEED+20);
      state = STRAIGHT_HOME;
    break;

    case GYRO_HOME:
      if (driveToDistanceHeading(P_GYRO_DRIVE,12,0,BASE_SPEED)) {
        state = DONE;
      };
    break;

    case STRAIGHT_HOME:
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(LEFT_MOTOR, BASE_SPEED+20);
      setMotorSpeed(RIGHT_MOTOR, BASE_SPEED+20);
    break;

    case FOLLOW_HOME:
      follow(P_FOLLOW+1,BASE_SPEED+5);
    break;

    case DONE:	/* Halt motors */
      Serial.println("DONE");
	    disableMotor(BOTH_MOTORS);
      state = SLEEP;
    break;

    case SLEEP:
    break;

  } // end state
}

boolean crashed()
{
 return (isBumpSwitchPressed(2) || isBumpSwitchPressed(3));
}

void follow(float myP, int myBaseSpeed)
{
  // read sensor and normalize between between -1.0 and 1.0
  float sensor = readLineSensor();
  float sensor_normalized = ((sensor - LOW_LEVEL) / (HIGH_LEVEL - LOW_LEVEL)*2) - 1.0;

  float error = sensor_normalized - LINE_GOAL;
  int motor_speed_delta = myP * error;

  int left_motor_speed = constrain(myBaseSpeed + motor_speed_delta, 0, 100);
  int right_motor_speed = constrain(myBaseSpeed - motor_speed_delta, 0, 100);

  setMotorSpeed(LEFT_MOTOR, left_motor_speed);
  setMotorSpeed(RIGHT_MOTOR, right_motor_speed);
}

/*
 * Drive distance
 */
boolean driveToDistanceHeading(float myP, float inches, int desiredHeading, int speed) {
  uint16_t ticks = distanceToEncoder(wheelDiameter, cntPerRevolution, inches);

  int headingError = calculateDifferenceBetweenAngles(desiredHeading, getCurrentRealtiveHeadingToStart());
  Serial.print("Drive: Heading Error");
  Serial.println(headingError);
  int adjustSpeed = headingError * myP;
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
