#include "SimpleRSLK.h"

/*
 * degreesToEncoder
 * 
 * Convert degrees to encoder ticks
 */
uint32_t degreesToEncoder(float turnConstant, uint32_t angle) {
    float temp = turnConstant * angle;
    return int(temp);
}

/*
 * DriveDistance
 * 
 * Drive the given distance in inches
 */
void driveDistance(int inches) {
  /* Diameter of Romi wheels in inches */
  float wheelDiameter = 3.0;

  /* Number of encoder (rising) pulses every time the wheel turns completely */
  int cntPerRevolution = 360;

  int wheelSpeed = 15; // Default raw pwm speed for motor.
  
  uint16_t totalCount = 0; // Total amount of encoder pulses received

  /* Amount of encoder pulses needed to achieve distance */
  uint16_t ticks = distanceToEncoder(wheelDiameter, cntPerRevolution, inches);

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
  while(totalCount < ticks)
  {
    totalCount = getEncoderLeftCnt();
  }

  /* Halt motors */
  disableMotor(BOTH_MOTORS);
}

/*
 * TurnAngle
 * 
 * Turn angle
 */
// Turn the robot angle amount
void turnAngle(int angleToTurn) {
  Serial.print("Turn: ");  Serial.println(angleToTurn);
  /* Turn Constant: constant that needs to be tuned */
  float turnConstant = 3.0;
  int wheelSpeed = 10; // Default raw pwm speed for motor.
    
  int totalCount = 0; // Total amount of encoder pulses received
  boolean sign = angleToTurn<0? false: true;

  angleToTurn = abs(angleToTurn);

  /* Amount of encoder pulses needed to achieve distance */
  int x = degreesToEncoder(turnConstant, angleToTurn);
  
  /* Set the encoder pulses count back to zero */
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  /* Cause the robot to turn */
  if (sign == false) {
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
  } else {
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
  }

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


/*
 * Setup
 * 
 * initialize drivers
 */
void setup() {
	Serial.begin(115200);

	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
}

/*
 * Loop
 * 
 * Main loop
 */
void loop() {
  /* Wait until button is pressed to start robot */
  String btnMsg = "\nPush left button on Launchpad to start demo.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  driveDistance(5);
  turnAngle(90);
  driveDistance(10);
}
