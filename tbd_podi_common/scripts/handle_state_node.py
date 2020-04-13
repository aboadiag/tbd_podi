#!/usr/bin/env python

from pyftdi.spi import (
    SpiController
)
import time
import rospy
import actionlib
from std_msgs.msg import (
    Int16,
)

from std_srvs.srv import (
    Empty,

)

from sensor_msgs.msg import (
    JointState
)

from tbd_podi_msgs.msg import (
    setHandleHeightAction,
    setHandleHeightResult
)

import alloy.ros

class AMT203():

    def __init__(self):
         # Instanciate a SPI controller
        spi = SpiController()
        spi.configure('ftdi://ftdi:232h:FT0KGOUA/1')
        # The sensor AMT203-V is the slave
        # https://www.cui.com/product/resource/amt20.pdf
        self.slave = spi.get_port(cs=0, freq=spi.frequency/32, mode=0)

    def send_command(self, command_in_hex):
        out = self.slave.exchange([command_in_hex], readlen=1, duplex=True)
        return bytes(out).hex()

    def read_angle(self):
        rtrn = self.send_command(0x10)
        while rtrn != "10":
            rtrn = self.send_command(0x00)
        msb = self.send_command(0x10)
        lsb = self.send_command(0x10)
        angle_in_hex_string = msb + lsb
        return int(angle_in_hex_string, 16)

    def set_zero_point(self):
        rtrn = self.send_command(0x70)
        while rtrn != "80":
            rtrn = self.send_command(0x00)


ROTATION_RESOLUTION = 0.00613592315 # 2 * PI / 1024

class HandlerReader():

    _current_handle_reading: int

    def __init__(self):
       
        self._current_handle_reading = 0

        # get the ROS Parameters
        self._handle_rotation_center = rospy.get_param("~handle_rotation_zero_reading", 310)
        self._handle_height = self._convert_height_setting_to_value(rospy.get_param("~handle_height_setting", 0))


        # create the AMT203 sensor interface
        self._amt = AMT203()
        # create the publishers
        self._reading_pub = rospy.Publisher("handle_reading", Int16, queue_size=1)
        self._joint_state_pub = rospy.Publisher("handle_state", JointState, queue_size=1)

        # create the services
        rospy.Service('set_handle_rotation_zero', Empty, self._set_zero_point) # To set the zero of the handle

        # create the action to set the height
        self._handle_server = actionlib.SimpleActionServer("set_handle_height", setHandleHeightAction, execute_cb=self._set_handle_cb, auto_start = False)
        self._set_handle_height_result = setHandleHeightResult()
        self._handle_server.start()

        # get the publish rate
        self._pub_rate = rospy.get_param("~rate", 100)
        # success!
        rospy.loginfo("HandlerReader initialized")

    def _set_zero_point(self):
        rospy.loginfo("Sending command for set_zero_point ... ")
        self._amt.set_zero_point()
        rospy.loginfo("AMT20 set_zero_point successful. Powercycle for encoder to restart")

    def _convert_height_setting_to_value(self, setting):
        # TODO calculate the convertion from setting to value
        return setting

    def _set_handle_cb(self, goal):
        setting = goal.handle_height_setting
        # remember the setting
        self._set_handle_height_result.success = True
        self._handle_server.set_succeeded(self._set_handle_height_result)

    def _publish_joint_state(self):
        # convert the joint_reading into radion
        handle_rotation = -1 * (self._current_handle_reading - self._handle_rotation_center) * ROTATION_RESOLUTION
        # create joint state message
        joint_state = JointState()
        joint_state.header = alloy.ros.create_ros_header(rospy)
        joint_state.name = ["handle_rotation", "handle_height"]
        joint_state.position = [handle_rotation, self._handle_height]
        # publish joint state
        self._joint_state_pub.publish(joint_state)

    def spin(self):
        r = rospy.Rate(self._pub_rate)
        msg = Int16()
        while not rospy.is_shutdown():
            self._current_handle_reading = self._amt.read_angle()
            # Publish the RAW readings
            msg.data = self._current_handle_reading
            self._reading_pub.publish(msg)
            # Publish the joint state
            self._publish_joint_state()
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("handle_state_node")
    hr = HandlerReader()
    hr.spin()