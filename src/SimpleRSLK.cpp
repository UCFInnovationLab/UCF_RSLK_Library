#include "SimpleRSLK.h"

GP2Y0A21_Sensor		dst_sensor[3];
Romi_Motor_Power	motor[2];
Bump_Switch			bump_sw[6];
uint16_t calMin[LS_NUM_SENSORS], calMax[LS_NUM_SENSORS];

QTRSensorsUCF qtr;

void setupRSLK() {
	motor[0].begin(MOTOR_L_SLP_PIN,MOTOR_L_DIR_PIN,MOTOR_L_PWM_PIN);
	motor[1].begin(MOTOR_R_SLP_PIN,MOTOR_R_DIR_PIN,MOTOR_R_PWM_PIN);

	setupEncoder(ENCODER_ELA_PIN, ENCODER_ELB_PIN, ENCODER_ERA_PIN, ENCODER_ERB_PIN);

	bump_sw[0].begin(BP_SW_PIN_0, INPUT_PULLUP);
	bump_sw[1].begin(BP_SW_PIN_1, INPUT_PULLUP);
	bump_sw[2].begin(BP_SW_PIN_2, INPUT_PULLUP);
	bump_sw[3].begin(BP_SW_PIN_3, INPUT_PULLUP);
	bump_sw[4].begin(BP_SW_PIN_4, INPUT_PULLUP);
	bump_sw[5].begin(BP_SW_PIN_5, INPUT_PULLUP);

	dst_sensor[0].begin(SHRP_DIST_L_PIN,INPUT_PULLDOWN);
	dst_sensor[1].begin(SHRP_DIST_C_PIN,INPUT_PULLDOWN);
	dst_sensor[2].begin(SHRP_DIST_R_PIN,INPUT_PULLDOWN);

	// qtr.setTypeRC();
	// qtr.setSensorPins((const uint8_t[]){QTR_7, QTR_6, QTR_5, QTR_4, QTR_3, QTR_2, QTR_1, QTR_0}, LS_NUM_SENSORS);
	// qtr.setEmitterPins(QTR_EMITTER_PIN_ODD, QTR_EMITTER_PIN_EVEN);
	// disableMotor(BOTH_MOTORS);
	qtr.setSensorPin(QTR_0);
	disableMotor(BOTH_MOTORS);


	// for (uint8_t x = 0; x < LS_NUM_SENSORS; x++)
	// {
	// 	calMin[x] = 5000;
	// 	calMax[x] = 0;
	// }

}

uint16_t readSharpDist(uint8_t num) {
	if(num < 0 || num > 3)
		return 0;

	return dst_sensor[num].read();
}

bool isBumpSwitchPressed(uint8_t num) {
	if(num < 0 || num > 5)
		return false;

	if(bump_sw[num].read() == 0) {
		return true;
	}
	else
		return false;
}
void enableMotor(uint8_t motorNum) {
	if(motorNum == 0 || motorNum == 2)
	{
		motor[0].enableMotor();
	}

	if(motorNum == 1 || motorNum == 2)
	{
		motor[1].enableMotor();
	}
}


void disableMotor(uint8_t motorNum) {
	if(motorNum == 0 || motorNum == 2)
	{
		motor[0].disableMotor();
	}

	if(motorNum == 1 || motorNum == 2)
	{
		motor[1].disableMotor();
	}
}

void pauseMotor(uint8_t motorNum) {
	if(motorNum == 0 || motorNum == 2)
	{
		motor[0].pauseMotor();
	}

	if(motorNum == 1 || motorNum == 2)
	{
		motor[1].pauseMotor();
	}
}

void resumeMotor(uint8_t motorNum) {
	if(motorNum == 0 || motorNum == 2)
	{
		motor[0].resumeMotor();
	}

	if(motorNum == 1 || motorNum == 2)
	{
		motor[1].resumeMotor();
	}
}

void setMotorDirection(uint8_t motorNum,uint8_t direction) {
	if(motorNum == 0 || motorNum == 2)
	{
		if(direction == 0) {
			motor[0].directionForward();
		} else if(direction == 1) {
			motor[0].directionBackward();
		}
	}

	if(motorNum == 1 || motorNum == 2)
	{
		if(direction == 0) {
			motor[1].directionForward();
		} else if(direction == 1) {
			motor[1].directionBackward();
		}
	}
}

void setMotorSpeed(uint8_t motorNum, uint8_t speed) {
	if(motorNum == 0 || motorNum == 2)
	{
		 motor[0].setSpeed(speed);
	}

	if(motorNum == 1 || motorNum == 2)
	{
		 motor[1].setSpeed(speed);
	}
}

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed) {
	if(motorNum == 0 || motorNum == 2)
	{
		motor[0].setRawSpeed(speed);
	}

	if(motorNum == 1 || motorNum == 2)
	{
		motor[1].setRawSpeed(speed);
	}
}

uint16_t readLineSensor() {
	return qtr.read();
}

void setupWaitBtn(uint8_t btn) {
	pinMode(btn, INPUT_PULLUP);
}

void setupLed(uint8_t ledPin) {
	pinMode(ledPin, OUTPUT);
}

void waitBtnPressed(uint8_t btnPin,String msg,int8_t ledPin) {
	uint8_t btnCnt = 0;
	uint8_t pinVal = HIGH;

	/* Turn on led */
	if(ledPin > 0)
		digitalWrite(ledPin,pinVal);
	while(digitalRead(btnPin) == 1) {
		delay(25);
		btnCnt++;
		if(btnCnt==40) {
			digitalWrite(ledPin,pinVal);
			btnCnt = 0;
			pinVal = !pinVal;

			if( msg != "")
				Serial.println(msg);
		}
	}

	pinVal = LOW;
	if(ledPin > 0)
		digitalWrite(ledPin,pinVal);

	/* Wait for a short period to avoid button debounce */
	delay(50);
	while(digitalRead(btnPin) == 0);

	/* Wait for a short period to avoid button debounce */
	delay(50);
}