#!/usr/bin/python3

import rospy
from bottle import Bottle, run, request
from tbd_podi_msgs.msg import(
    PodiFaceInput
)
import alloy.ros


def main():

    rospy.init_node("face_http_relay")
    touch_pub = rospy.Publisher('touch', PodiFaceInput, queue_size=1)
    app = Bottle()

    @app.post('/touch')
    def touch_response():
        resp = request.forms
        # create message
        msg = PodiFaceInput()
        msg.header = alloy.ros.create_ros_header(rospy)
        msg.point_x = float(resp["pos_x"])
        msg.point_y = float(resp["pos_y"])
        msg.raw_point_x = int(float(resp["raw_x"]))
        msg.raw_point_y = int(float(resp["raw_y"]))
        # for now, all touch from Android is a down touch
        msg.input_type = PodiFaceInput.ACTION_DOWN
        # publish message
        touch_pub.publish(msg)
        return
    
    run(app, host="192.168.0.152", port=9000)

if __name__ == "__main__":
    main()

