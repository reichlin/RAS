# ras_servo_control
This package allows you to control the angle of two servos through ROS using an Arduino Micro.

## Requirements
Apart from a ROS installation including rospy, this package requires the pyserial module to be installed for Python 2.7. On Ubuntu you can simply install it using
```bash
sudo apt install python-serial
```

## Installation
Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/KTH-RAS/ras_servo_control.git
```
Next, you need to build the message types:
```bash
catkin_make
```
Don't forget to source your workspace (You do not need to do this if it's already in your bashrc):
```bash
source ../devel/setup.bash
```
## Setting up the Arduino Micro
The sketch (=Arduino program) in the folder arduino_servo_control is already installed on your Arduino Micro.
If you want to know how to install the sketch, check the official resources: https://www.arduino.cc/

The pins for the servos are hardcoded in the sketch. You need to connect the data wire of your servos to pins 9 and 10.
**TODO**: Instructions on how to connect the servos to power etc.

Lastly, connect your Arduino to your NUC/computer using a Micro-USB cable. **Note:** After the Arduino booted it will send a signal to both attached servos to move to angle 0. If you want to change this behavior, you need to update the sketch. The initial angles are set [here](https://github.com/KTH-RAS/ras_servo_control/blob/master/arduino_servo_control/arduino_servo_control.ino#L9).

## Running the node and commanding servo angles
Once everything is connected, run 
```bash
rosrun arduino_servo_control servo_control
```
You can move the servos by calling the service `/arduino_servo_control/set_servo_angles`:
```bash
rosservice call /arduino_servo_control/set_servo_angles "angle_servo_0: 90  
angle_servo_1: 30"
```
The first servo, servo_0, is the servo attached to pin 9, and the second servo, servo_1, the one attached to pin 10.
The angles are specified in degrees and can range from 0 to 180. The return value of the service is `success: True` if
the message was forwarded to the Arduino, i.e. an Arduino is connected, else it is `success: False`.
