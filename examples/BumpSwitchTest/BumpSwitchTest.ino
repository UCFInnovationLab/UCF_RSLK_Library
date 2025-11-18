#include "SimpleRSLK.h"

int motorSpeed = 10;

void setup() {
	Serial.begin(115200);
	setupRSLK();
	/* Left button on Launchpad */
	setupWaitBtn(LP_LEFT_BTN);
	/* Red led in rgb led */
	setupLed(RED_LED);
}

void loop() {
  int x;

	String btnMsg = "Push left button on Launchpad to start demo.\n";
	/* Wait until button is pressed to start robot */
	waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  Serial.println("Watching Buttons");

  while (1) {
    /* Loop through all bump switchees to see if it has been pressed */
		for(int x = 0;x<TOTAL_BP_SW;x++)
		{
			/* Check if bump switch was pressed */
			if(isBumpSwitchPressed(x) == true) {
				Serial.print("Button Pressed: ");
        Serial.println(x);
				break;
			}
		}
	}
}