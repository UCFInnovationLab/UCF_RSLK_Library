

/*!
* accel.ino
*
* I2C addr:
* 0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
* 0x69: set I2C address by parameter
*
* Through the example, you can get the sensor data by using getSensorData:
* get acell by paremeter onlyAccel;
* get gyro by paremeter onlyGyro;
* get both acell and gyro by paremeter bothaccel.
*
* With the rotation of the sensor, data changes are visible.
*
* Copyright [DFRobot](http://www.dfrobot.com), 2016
* Copyright GNU Lesser General Public License
*
* version V1.0
* date 2017-11-27
* 
*/

#include <Bump_Switch.h>
#include <GP2Y0A21_Sensor.h>
#include <DFRobot_BMI160.h>
#include <Romi_Motor_Power.h>
#include <Encoder.h>
#include <QTRSensors.h>
#include <SimpleRSLK.h>
#include <RSLK_Pins.h>


int16_t accel[6] = {0}; // temp holder for accel(x,y,z);
int16_t accelArray[500][3]={0};
unsigned long accelTime[500] = {0};
unsigned long mainStartTime = 0;
int accelLength = 0;
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

void setup(){
  Serial.begin(115200);
  delay(100);
  setupRSLK();
  
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  
  /* Red led in rgb led */
  setupLed(RED_LED);
  
  /* init the hardware bmin160 */
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("Error: softReset false");
    while(1);
  }
  
  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("Error: I2C init false");
    while(1);
  }
}


//
// ramp()
//
// Ramp motors from 'initialSpeed' to 'targetSpeed' for 'duration seconds'
// Speeds are given from 0-100
//
void ramp(float duration, float initialSpeed, float targetSpeed) {
  unsigned long currentTime = millis();
  unsigned long sampleTime = currentTime;
  unsigned long startTime = currentTime;
  unsigned long stopTime = currentTime + (duration * 1000);
  float rampAccel = (targetSpeed - initialSpeed)/duration;
  float ramp = 0;
  
  /* Cause the robot to drive forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  
  /* "Turn on" the motor */
  enableMotor(BOTH_MOTORS);

  while (currentTime < stopTime) {
    if (currentTime > sampleTime + 50) {
      sampleAccel();
      sampleTime = currentTime;
    }
    
    ramp = initialSpeed + rampAccel * ((currentTime - startTime)/1000.0);
    setMotorSpeed(BOTH_MOTORS,(int)ramp);
    currentTime = millis();
    delay(2);
  }
}

//
// sampleAccel()
//
void sampleAccel() {
  int rslt;
  rslt = bmi160.getAccelGyroData(accel);
  if (rslt==0) {
    accelArray[accelLength][0] = accel[3];
    accelArray[accelLength][1] = accel[4];
    accelArray[accelLength][2] = accel[5];
    accelTime[accelLength] = millis() - mainStartTime;
    accelLength++;
  } else{
    Serial.println("err");
  }
}

//
// printData()
//
void printData(int16_t ag[][3], unsigned long agTime[], int16_t agLength){
  int i, j;
  int sample;
  
  for(i=0; i<agLength; i++){
    Serial.print(agTime[i]);
    Serial.print(", ");
    for(j=0;j<3;j++){
      //the following three data are accel datas
      Serial.print(ag[i][j]/16384.0);
      
      if(j < 3){
        Serial.print(", ");
      }
    }
    Serial.println();
  }
}
  
//
// loop()
//
void loop(){
  unsigned long currentTime = millis();
  unsigned long sampleTime = currentTime;
  unsigned long startTime = currentTime;
  unsigned long stopTime = startTime + 8000; 
  int i = 0;
  uint16_t totalCount = 0; // Total amount of encoder pulses received
  
  /* Wait until button is pressed to start robot */
  String btnMsg = "";
  btnMsg += "\nPush LEFT button on Launchpad to start.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);
  
  /* Cause the robot to drive forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  
  /* "Turn on" the motor */
  enableMotor(BOTH_MOTORS);

  /* Set the encoder pulses count to zero */
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  mainStartTime = millis(); /* save the start time */
  
  setMotorSpeed(BOTH_MOTORS,30);
  bool done=false;
  float lastAccel;
  while ((currentTime < stopTime) && (!done)) {
    if (currentTime > sampleTime + 50) {
      sampleAccel();
      sampleTime = currentTime;
      // Stop the robot deceleration < VALUE
      lastAccel = accelArray[accelLength-1][1]; // last y
      if (lastAccel < -2.0) done=true;
    }
    
    currentTime = millis();
    delay(2);
  }
    
  totalCount = getEncoderLeftCnt();
  //Serial.println(totalCount);
  
  /* Halt motors */
  disableMotor(BOTH_MOTORS);
  
  /* Right button on Launchpad */
  setupWaitBtn(LP_RIGHT_BTN);
  btnMsg = "\nPush right button on Launchpad to print data.\n";
  waitBtnPressed(LP_RIGHT_BTN,btnMsg,RED_LED);
  printData(accelArray, accelTime, accelLength);
  while(1);
} // end loop
