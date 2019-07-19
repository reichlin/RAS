#! /usr/bin/python
"""
    This script allows controlling the position of two servos attached to Arduino Micro
    through ROS. For this to work, the Arduino program that is shipped with this script
    needs to be installed on the Arduino.
"""

import rospy
import serial
import serial.tools.list_ports
from arduino_servo_control.srv import *


class ServoBridge(object):
    def __init__(self, arduino_name='Arduino Micro', baud=9600):
        self._arduino_name = arduino_name
        self._baud_rate = baud
        self._serial_con = serial.Serial(baudrate=self._baud_rate)

    def __enter__(self):
        self.check_connection()

    def __exit__(self, exception_type, exception_value, traceback):
        if self._serial_con.isOpen():
            self._serial_con.close()

    def set_target_angles(self, request):
        response = SetServoAnglesResponse()
        if self.check_connection():
            val0 = max(min(request.angle_servo_0, 180), 0)
            val1 = max(min(request.angle_servo_1, 180), 0)
            self._serial_con.write('%i,%i\n' % (val0, val1))
            response.success = True
        else:
            rospy.logerr("No %s connected" % self._arduino_name)
            response.success = False
        return response

    def check_connection(self):
        arduino_ports = [tuple(port) for port in serial.tools.list_ports.grep(self._arduino_name)]
        if len(arduino_ports) == 0:
            if self._serial_con.isOpen():
                self._serial_con.close()
            return False
        if len(arduino_ports) > 1:
            rospy.logwarn("Found more than one %s. Connecting to %s." %
                          (self._arduino_name, arduino_ports[0]))
        port = arduino_ports[0]
        if self._serial_con.isOpen() and self._serial_con.port != port[0]:
            self._serial_con.close()
        if not self._serial_con.isOpen():
            self._serial_con.port = port[0]
            self._serial_con.open()
        return True


if __name__ == '__main__':
    rospy.init_node('arduino_servo_control')
    bridge = ServoBridge()
    # Create service
    set_angle_service = rospy.Service(rospy.get_name() + '/set_servo_angles',
                                    SetServoAngles, bridge.set_target_angles)
    # Spin until node is killed
    with bridge:
        rospy.spin()
    sys.exit(0)

