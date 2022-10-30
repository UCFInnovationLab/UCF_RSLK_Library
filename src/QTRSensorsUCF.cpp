#include "QTRSensorsUCF.h"
#include <Arduino.h>

void QTRSensorsUCF::setSensorPin(const uint8_t pin)
{
    _sensorPin = pin;

  // Any previous calibration values are no longer valid, and the calibration
  // arrays might need to be reallocated if the sensor count was changed.
  _calibrationData.initialized = false;
  _calibrationData.initialized = false;
}

void QTRSensorsUCF::setTimeout(uint16_t timeout)
{
  if (timeout > 32767) { timeout = 32767; }
  _timeout = timeout;
  _maxValue = timeout;
}

void QTRSensorsUCF::resetCalibration()
{
    if (_calibrationData.maximum)   { _calibrationData.maximum = 0; }
    if (_calibrationData.minimum)   { _calibrationData.minimum = _maxValue; }
    _calibrationData.initialized = true;
}

void QTRSensorsUCF::calibrate()
{
  uint16_t sensorValue;
  uint16_t maxSensorValue;
  uint16_t minSensorValue;

  for (uint8_t j = 0; j < 10; j++)
  {
    sensorValue = read();

    // set the max we found THIS time
    if ((j == 0) || (sensorValue > maxSensorValue))
    {
      maxSensorValue = sensorValue;
    }

    // set the min we found THIS time
    if ((j == 0) || (sensorValue < minSensorValue))
    {
      minSensorValue = sensorValue;
    }
  }

  // record the min and max calibration values

  // Update maximum only if the min of 10 readings was still higher than it
  // (we got 10 readings in a row higher than the existing maximum).
  if (minSensorValue > _calibrationData.maximum)
  {
    _calibrationData.maximum = minSensorValue;
  }

  // Update minimum only if the max of 10 readings was still lower than it
  // (we got 10 readings in a row lower than the existing minimum).
  if (maxSensorValue < _calibrationData.minimum)
  {
    _calibrationData.minimum = maxSensorValue;
  }
  
}

uint16_t QTRSensorsUCF::readCalibrated()
{
  uint16_t sensorValue;

  if (!_calibrationData.initialized)
  {
    return 0;
  }
  
  // read the needed values
  sensorValue = read();

  uint16_t calmin, calmax;

  calmax = _calibrationData.maximum;
  calmin = _calibrationData.minimum;

  uint16_t denominator = calmax - calmin;
  int16_t value = 0;

  if (denominator != 0)
  {
    value = (((int32_t)sensorValue) - calmin) * 1000 / denominator;
  }

  if (value < 0) { value = 0; }
  else if (value > 1000) { value = 1000; }

  sensorValue = value;

  return sensorValue;
}

uint16_t QTRSensorsUCF::read()
{
  uint16_t sensorValue = _maxValue;

  // make sensor line an output (drives low briefly, but doesn't matter)
  pinMode(_sensorPin, OUTPUT);
  // drive sensor line high
  digitalWrite(_sensorPin, HIGH);

  delayMicroseconds(10); // charge lines for 10 us

  // disable interrupts so we can switch all the pins as close to the same
  // time as possible
  noInterrupts();

  // record start time before the first sensor is switched to input
  // (similarly, time is checked before the first sensor is read in the
  // loop below)
  uint32_t startTime = micros();
  uint16_t time = 0;

  // make sensor line an input (should also ensure pull-up is disabled)
  pinMode(_sensorPin, INPUT);

  interrupts(); // re-enable

  while (time < _maxValue)
  {
    // disable interrupts so we can read all the pins as close to the same
    // time as possible
    noInterrupts();

    time = micros() - startTime;
    if ((digitalRead(_sensorPin) == LOW) && (time < sensorValue))
    {
      // record the first time the line reads low
      sensorValue = time;
    }

    interrupts(); // re-enable
  }
  
  return sensorValue;
}

// the destructor frees up allocated memory
QTRSensorsUCF::~QTRSensorsUCF()
{
}
