/*
 * Print Line Sensor value
 */

#include "SimpleRSLK.h"

void setup()
{
  Serial.begin(115200);
  setupRSLK();
}

void loop()
{
  int value = readLineSensor();
  Serial.print("Line Sensor: ");
  Serial.println(value);
}
