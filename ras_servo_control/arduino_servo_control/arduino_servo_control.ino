#include <Servo.h>

Servo servos[2];  // create two servo objects to control servos
// twelve servo objects can be created on most boards

const int servo_pins[] = {9, 10};  // pins for servos
const int sleep_time = 20;  // sleeping time until the loop checks again for new targets

int target_angles[] = {0, 0}; // variable to store target position for servos

void setup() {
  Serial.begin(9600); // initialize serial  
  servos[0].attach(servo_pins[0]);  // attaches the servo object to the given pin
  servos[1].attach(servo_pins[1]);  // attaches the servo object to the given pin
  // set both servos to start angles (if you do not do this, it moves to 90 degrees by default)
  servos[0].write(target_angles[0]);
  servos[1].write(target_angles[1]);
}

void read_targets() {
  // the message we expect has the shape "x,y\n" where x and y are decimal string representations of integers
  String num_1_str = Serial.readStringUntil(',');
  String num_2_str = Serial.readStringUntil('\n');
  target_angles[0] = constrain(num_1_str.toInt(), 0, 180);
  target_angles[1] = constrain(num_2_str.toInt(), 0, 180);
}

void loop() {
  if (Serial.available() > 0) {
    // read in new target values
    read_targets();
    // send them to the servos
    servos[0].write(target_angles[0]);
    servos[1].write(target_angles[1]);
  }
  delay(sleep_time);
}
