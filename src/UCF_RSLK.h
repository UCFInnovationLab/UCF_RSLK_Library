/** @file */

#ifndef UCF_RSLK_h
#define UCF_RSLK_h

/// \brief Configure pin as a wait to release button
///
/// \param[in] btn the Launchpad pin number you want to use.
///
/// Configure pin to be used as a wait until pushed and released button. Useful if you want to halt the robot's
/// operation until the uses pushes and then releases the button.
//void setupWaitBtn(uint8_t btn);

/// \brief Configure pin that is connected to an led
///
/// \param[in] ledPin the Launchpad pin number you want to use.
///
/// Configure pin to be used for as an led.
//void setupLed(uint8_t ledPin);

/// \brief Busy wait until user pushes and releases button
///
/// \param[in] btnPin the Launchpad pin number you want to use.
/// \param[in] msg contains the string to output while waiting for btn to be pressed.
/// \param[in] ledPin represents the pin to toggle high and low while waiting for btn to be pressed.
///
/// Prevent additional code from executing until use has pushed and released
/// specified button.
void waitBtnPressedString(uint8_t btnPin,String msg = "",int8_t ledPin = 0);

uint32_t distanceToEncoder(float wheel_diam, uint16_t cnt_per_rev, uint32_t distance);

uint32_t angleToEncoder(float wheel_diam, uint16_t cnt_per_deg, uint32_t angle);

#endif