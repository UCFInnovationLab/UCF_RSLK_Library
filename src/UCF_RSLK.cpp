#include "SimpleRSLK.h"

void waitBtnPressedString(uint8_t btnPin,String msg,int8_t ledPin) {
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

/* The distance the wheel turns per revolution is equal to the diameter * PI.
 * The distance the wheel turns per encoder pulse is equal to the above divided
 * by the number of pulses per revolution.
 */
float distanceTraveled(float wheel_diam, uint16_t cnt_per_rev, uint8_t current_cnt) {
    float temp = (wheel_diam * PI * current_cnt) / cnt_per_rev;
    return temp;
}


uint32_t distanceToEncoder(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance) {
    float temp = (wheel_diam * PI) / cnt_per_rev;
    temp = distance / temp;
    return int(temp);
}

uint32_t angleToEncoder(float wheel_diam, uint16_t cnt_per_deg, uint32_t angle) {
    float temp = (wheel_diam * PI) / cnt_per_deg;
    temp = angle / temp;
    return int(temp);
}