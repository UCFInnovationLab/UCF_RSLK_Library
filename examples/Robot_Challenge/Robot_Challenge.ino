#include "SimpleRSLK.h"

#define HIGH_LEVEL 1800
#define LOW_LEVEL  350
#define GOAL 0
#define BASE_SPEED 15
#define P 5.0
#define TURN_COUNT 350
#define desired_heading (figure it out)

float midpoint;
float normal;
float crashedCount;
int currentCount;

typedef enum State
{
  START,
  FOLLOW,
  CRASH,
  TURN,
  PREPARE_RETURN_HOME,
  FOLLOW_HOME,
  STRAIGHT_HOME,
  HEADING_HOME,
  DONE
} State;

State state = FOLLOW;

void setup()
{
  Serial.begin(115200);

  setupRSLK();

  /*Calculate light sensor midpoint*/
  midpoint = (HIGH_LEVEL + LOW_LEVEL) / 2.0;
  normal = HIGH_LEVEL - midpoint;

  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red Led in rgb led */
  setupLed(RED_LED);

  delay(2000);

  String btnMsg = "Push Left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Waait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);

}

boolean crashed()
{
 if(isBumpSwitchPressed(2)  || isBumpSwitchPressed(3))
 {
  return true;
 }
 else 
 {
  return false;
 }
}

void follow(float myP, int myBaseSpeed)
{
  float value = readLineSensor();
  float value_normalized = (value - midpoint) / normal;

  float error = value_normalized - GOAL;
  int motor_speed_delta = myP * error;

  int temp = myBaseSpeed + motor_speed_delta;
  int left_motor_speed = ( temp > 100) ? 100 : temp;
  temp = myBaseSpeed - motor_speed_delta;
  int right_motor_speed = (temp < 0) ? 0 : temp;

  setMotorSpeed(LEFT_MOTOR, left_motor_speed);
  setMotorSpeed(RIGHT_MOTOR, right_motor_speed);
}

// MAIN LOOP // 

void loop()
{
  // Run Current State
  switch (state) {
    // Follow the line
    case FOLLOW:
      follow(P,BASE_SPEED);
      if (crashed()) {
         state = CRASH;
      }
    break;

    case CRASH:
      setMotorSpeed(LEFT_MOTOR, 0);
      setMotorSpeed(RIGHT_MOTOR, 0);
      crashedCount = getEncoderRightCnt();
      state = TURN;
    break;

    case TURN:
      setMotorDirection(RIGHT_MOTOR,1);
      setMotorSpeed(LEFT_MOTOR,BASE_SPEED);
      setMotorSpeed(RIGHT_MOTOR,BASE_SPEED);
      currentCount = getEncoderRightCnt();
      Serial.print("Ticks: ");
      Serial.println(currentCount);
      if ( currentCount > crashedCount + TURN_COUNT) {
        setMotorSpeed(LEFT_MOTOR,0);
        setMotorSpeed(RIGHT_MOTOR,0);
        state = PREPARE_RETURN_HOME;
      }
    break;

    case PREPARE_RETURN_HOME:
      setMotorDirection(RIGHT_MOTOR,0);
      setMotorSpeed(LEFT_MOTOR, BASE_SPEED+20);
      setMotorSpeed(RIGHT_MOTOR,BASE_SPEED+20);
      state = FOLLOW_HOME;
    break;

    case STRAIGHT_HOME:
      setMotorDirection(RIGHT_MOTOR,0);
      setMotorSpeed(LEFT_MOTOR, BASE_SPEED+20);
      setMotorSpeed(RIGHT_MOTOR, BASE_SPEED+20);
    break;

    case HEADING_HOME:


    break;

    case FOLLOW_HOME:
      follow(P+1,BASE_SPEED+5);
    break;


  } // end state
}