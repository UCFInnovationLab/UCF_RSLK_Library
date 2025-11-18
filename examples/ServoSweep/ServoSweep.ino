
#include <Servo.h> // Include the Servo library

Servo myservo; // Create a servo object to control the servo motor

void setup() {
  myservo.attach(4); // Attaches the servo on pin 9 to the servo object
                     // Change '9' to the pin you connected your servo to
}

void loop() {
  // Sweep from 0 degrees to 180 degrees
  for (int pos = 0; pos <= 180; pos += 1) { // Goes from 0 to 180 degrees in steps of 1 degree
    myservo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Waits 15ms for the servo to reach the position
  }

  // Sweep from 180 degrees to 0 degrees
  for (int pos = 180; pos >= 0; pos -= 1) { // Goes from 180 to 0 degrees in steps of 1 degree
    myservo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Waits 15ms for the servo to reach the position
  }
}