/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 */
#include "SimpleRSLK.h"

#define HIGH_LEVEL 1800 // value return from light sensor when over dark surface
#define LOW_LEVEL 350   // value return from light sensor when over light surface
#define GOAL 0          // value of normalized light sensor to follow
#define BASE_SPEED 10   // base robot speed.  range 0-100
#define P 4.0           // proportional term used in the PID controller

float midpoint;
float normal;

void setup()
{
  Serial.begin(115200);

  setupRSLK();

  /* Calculate light sensor midpoint */
  midpoint = (HIGH_LEVEL + LOW_LEVEL) / 2.0;
  normal = HIGH_LEVEL - midpoint;
  
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);

  delay(2000);

  String btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void loop()
{
  /*  Read line sensor.  The value returned is the number between 0 and 2500.
   *  0 - maxium light reflected.  A white surface.
   *  2500 - minminum light reflected.  A dark surface.
   */
  float value = readLineSensor();
  float value_normalized = (value - midpoint) / normal;

  /* use PID algorithm to calculate speed delta */
  float error = value_normalized - GOAL;
  int motor_speed_delta = P * error;

  /* add and subtract delta from left and right sides.  Cap motors at 100 */
  int temp = BASE_SPEED + motor_speed_delta;
  int left_motor_speed = (temp > 100) ? 100 : temp; 
  temp = BASE_SPEED - motor_speed_delta;
  int right_motor_speed = (temp < 0) ? 0 : temp; 

  setMotorSpeed(LEFT_MOTOR, left_motor_speed);
  setMotorSpeed(RIGHT_MOTOR, right_motor_speed);
}
