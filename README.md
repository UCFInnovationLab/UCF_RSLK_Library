{#mainpage}
=================

# Introduction


This [Energia] (https://energia.nu/) library provides a set of classes and functions to make using the [Texas Instrument's Robotic System Learning Kit] (http://www.ti.com/rslk) simple.

# Arduino IDE 2.x Setup

This project requires additional board package URLs, standard Arduino libraries, and custom ZIP libraries. Follow the steps below to fully configure Arduino IDE 2.x.

# Step 1 - Download and install the latest version of Arduino IDE. https://www.arduino.cc/en/software/

# Step 2 - Open Arduino IDE Settings

1. Open the **Arduino IDE**.
2. On **macOS**, click **Arduino IDE** in the top menu bar, then select **Settings**.
3. On **Windows** or **Linux**, click **File**, then select **Preferences**.
4. The Arduino IDE settings window will open.

# Step 3 - Add the Additional Boards Manager URLs

1. In the settings window, locate the field labeled **Additional Boards Manager URLs**.
2. Copy and paste the following URLs into that field, with **one URL per line**:

       https://raw.githubusercontent.com/Andy4495/TI_Platform_Cores_For_Arduino/main/json/package_energia_optimized_index.json

3. Go to Tools > Boards > Boards Manager and search for "Energia MSP432 EMT RED boards" and install 5.30.0

4. After the install completes, You need to select the board and COM port in Arduino IDE. Go to Tool > Boards and now you should see the "Energia MSP432 Red Boards" > "Red LaunchPad MSP432P401R EMT" and make sure this is selected.

5. You also should select the correct COM port. This step can be done after you plug your LaunchPad to your computer over USB and you install your LaunchPad drivers (done in later step using RSLK debug tool GUI). Go to Tools > COM port to chose from available options. The LaunchPad populates two COM ports. MacOS users will see Port 001 and 004 populated, please use Port 1. Windows users can verify their COM port by going to Device Manager and finding the XDS110 UART.

6. Add the latest release of the RSLK robot library that is downloadable from the releases section of this page. Download the sorces.zip file. Go to Sketch > Include Library > add .ZIP library. Select the file path of the downloaded zip library and click okay. It should say library added and you can check this by going to File > Examples > TI-Robot-Lib and see the example code provided in the library.

7. Go to Tools > Manage Libraries.. and search for and install "BNO055 by Robert Bosch GMBH"

# Using This Library

## Library Documentation
Library documentation is hosted on Github Pages at https://fcooper.github.io/Energia-RSLK-Library/

## SimpleRSLK (Recommended)


The SimpleRSLK.h defines a set of easy to use functions for the RSLK. Internally it calls the classes and functions mentioned below.

## Peripheral Specific Library/Function

Functions and classes have been created for each of the peripherals used on the RSLK Max.

- Bump_Switch provides an easy to use class for a a single bump switch.
- Romi_Motor_Power provides an easy to use class for a single motor.
- QTRSensors provides an easy to use class for the line sensor. Provided by Pololu.
- GP2Y0A21_Sensor provides an easy to use class for the GP2Y0A21 IR distance sensor.
- Encoder.h provides an easy to use set of functions for the two onboard Encoders.

### Pins and Other Important Defines

RSLK_Pins.h provides an important list of pins definitions for the RSLK.

## Useful Documentation
- [TI RSLK] (http://www.ti.com/rslk) homepage to learn more, view user guide, curriculum and a link to order.
- [RSLK Max Schematic] (https://www.pololu.com/file/0J1670/ti-rslk-max-chassis-board-v1.0-schematic.pdf)
- [Full Chassis Board Pin Diagram] (https://www.pololu.com/file/0J1695/ti_rslk_max_chassis_board_pinout.pdf)

### Links to Parts
- [Left Bump Switch Assembly](https://www.pololu.com/product/3673)
- [Right Bump Switch Assembly](https://www.pololu.com/product/3674)
- [Line Sensor](https://www.pololu.com/product/3672)
- [Motor Board] (https://www.pololu.com/product/3671)
- [Gearmotor and Encoder] (https://www.pololu.com/product/3675)

