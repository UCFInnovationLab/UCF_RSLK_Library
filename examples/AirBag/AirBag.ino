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
#include <DFRobot_BMI160.h>
#include <Romi_Motor_Power.h>
#include <Encoder.h>
#include <SimpleRSLK.h>
#include <RSLK_Pins.h>

unsigned long startTime;
int16_t accel[6] = {0}; // temp holder for accel(x,y,z);
float accelArray[350][3]={0};
unsigned long accelTime[350] = {0};
int ticks[350] = {0};
int currentLength = 0;
DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

/*
 * Setup
 */
void setup(){
  Serial.begin(115200);
  delay(100);
  Wire.begin();
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

/*
 * sampleAccel
 */
float *sampleAccel() {
  int rslt;
  float x,y,z;
  rslt = bmi160.getAccelGyroData(accel);
  if (rslt==0) {
    x = (float)accel[3] / 16384.0;  
    accelArray[currentLength][0] = x;
    y = (float)accel[4] / 16384.0;
    accelArray[currentLength][1] = y;
    z = (float)accel[5] / 16384.0;
    accelArray[currentLength][2] = z;
    accelTime[currentLength] = millis() - startTime;
  } else{
    Serial.println("err");
  }
  return(accelArray[currentLength]);
}

/*
 * printData
 */
void printData(float ag[][3], unsigned long agTime[], int16_t agLength){
  int i, j;
  int sample;

  Serial.println("Ticks, Time, X, Y, Z");
  for(i=0; i<agLength; i++){
    Serial.print(ticks[i]);
    Serial.print(", ");
    Serial.print(agTime[i]);
    Serial.print(", ");
    for(j=0;j<3;j++){
      //the following three data are accel datas
      Serial.print(ag[i][j]);
      
      if(j < 3){
        Serial.print(", ");
      }
    }
    Serial.println();
  }

  Serial.print("Samples: ");
  Serial.println(agLength);
}
  
/*
 * Loop
 */
void loop(){
  int i = 0;
  uint16_t totalCount = 0; // Total amount of encoder pulses received
  
  /* Wait until button is pressed to start robot */
  String btnMsg = "";
  btnMsg += "\nPush LEFT button on Launchpad to start.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);
  
  /* Enable robot to drive forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);

  /* Set the encoder pulses count to zero */
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  setMotorSpeed(BOTH_MOTORS,30);
  
  float *currentAccel;

  bool done=false;
  int cnt=0;
  startTime = millis();
  while ((cnt < 300) && (!done)) {
    currentAccel = sampleAccel();
    float x_accel = currentAccel[0];
    float y_accel = currentAccel[1];
    float z_accel = currentAccel[2];
    ticks[currentLength] = getEncoderLeftCnt();
    currentLength++;
    cnt++;
    if (fabs(x_accel) > 1) done=true;
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
  printData(accelArray, accelTime, currentLength);
  while(1);
} // end loop
