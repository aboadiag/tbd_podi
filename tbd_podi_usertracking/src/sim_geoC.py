#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np

from shapely.geometry import Polygon as shp_Polygon
from shapely.geometry import LineString as shp_Line
from shapely.affinity import translate as shp_translate
from shapely.affinity import rotate as shp_rotate
from shapely.ops import nearest_points
from shapely.geometry import Point as shp_Point

from alloy import ros as alloy_ros
from alloy import math as alloy_math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from tbd_podi_usertracking.msg import UserPosition

class userModel:
    def __init__(self, initRelPos, centerOffset = 0.241, regionOffset = 0.421, regionSize = 0.269, regionAngle = 0.696, humanRadius = 0.21, beta = 0.802):
        self.centerOffset = centerOffset
        self.regionOffset = regionOffset
        self.regionSize = regionSize
        self.regionAngle = regionAngle
        self.humanRadius = humanRadius
        self.beta = beta

        self.currentMsgOdom = None

        self.rad = initRelPos[0]
        self.phi = initRelPos[1]
        self.positionWorldCart = None
        self.regionShape, self.regionCenter = self.getRegionShape()
        self.circle = np.array([self.humanRadius*np.cos(np.linspace(0, 2*np.pi, 50)), self.humanRadius*np.sin(np.linspace(0, 2*np.pi, 50))])

        # publishes footprint for visualization, position + params of user model
        self.footprintPub = rospy.Publisher('/userFootprint', PolygonStamped, queue_size=10)
        self.positionPub = rospy.Publisher('/userModel', numpy_msg(UserPosition), queue_size=10)
        self.msgFootprint = PolygonStamped()
        self.msgFootprint.header.frame_id = "podi_map"

        self.msgPosition = UserPosition()
        self.msgPosition.header.frame_id = "podi_map"
        self.msgPosition.params = np.array([centerOffset, regionOffset, regionSize, regionAngle, humanRadius, beta])

        # subscribes to robot odometry
        rospy.Subscriber("/podi/odom", Odometry, self.updateMsgOdom)

    def updateMsgOdom(self, msgOdom):
        self.currentMsgOdom = msgOdom
        if(self.positionWorldCart is None):
            self.positionWorldCart = self.paramsToPosition(msgOdom)

    def getRegionShape(self):
        innerArc = np.vstack([(- self.centerOffset) - self.regionOffset*np.cos(np.linspace(-self.regionAngle/2, self.regionAngle/2, 10)),
                     self.regionOffset*np.sin(np.linspace(-self.regionAngle/2, self.regionAngle/2, 10))]).T
        outerArc = np.vstack([(- self.centerOffset) - (self.regionOffset + self.regionSize)*np.cos(np.linspace(self.regionAngle/2, -self.regionAngle/2, 10)),
                     (self.regionOffset + self.regionSize)*np.sin(np.linspace(self.regionAngle/2, -self.regionAngle/2, 10))]).T
        regionShape = shp_Polygon(np.vstack([innerArc, outerArc]))

        regionCenter = shp_Line(np.array([[-self.centerOffset - self.regionOffset - self.regionSize/2, 0], [0, 0]]))

        return regionShape, regionCenter

    def paramsToPosition(self, msgOdom):
        robotPose = np.array([msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y, alloy_math.rot2D_from_quaternion(alloy_ros.pose_to_numpy(msgOdom.pose.pose)[3:])])
        position = np.array([robotPose[0] - self.centerOffset*np.cos(robotPose[2] + self.rad*np.cos(self.phi)), robotPose[1] - self.centerOffset*np.sin(robotPose[2]) + self.rad*np.sin(self.phi)])

        return position

   # position in world frame
    def getHumanPose(self, msgOdom):
        robotPose = np.array([msgOdom.pose.pose.position.x, msgOdom.pose.pose.position.y, alloy_math.rot2D_from_quaternion(alloy_ros.pose_to_numpy(msgOdom.pose.pose)[3:])])

        regionTrans = shp_translate(shp_rotate(self.regionShape, robotPose[2], use_radians=True, origin=(0, 0)), robotPose[0], robotPose[1])
        regionCenterTrans = shp_translate(shp_rotate(self.regionCenter, robotPose[2], use_radians=True, origin=(0, 0)), robotPose[0], robotPose[1])

        regionAxis = np.array([robotPose[0] - self.centerOffset*np.cos(robotPose[2]), robotPose[1] - self.centerOffset*np.sin(robotPose[2])])
        lastHumanPos = shp_Point(self.positionWorldCart)
        pullPose = np.array(nearest_points(regionTrans, lastHumanPos)[0].coords[:])
        compPose = np.array(regionCenterTrans.coords[0])

        newHumanPos = np.squeeze(self.beta*pullPose + (1 - self.beta)*compPose)
        newHumanPhi = np.arctan2(robotPose[1] - self.centerOffset*np.sin(robotPose[2]) - newHumanPos[1], robotPose[0] - self.centerOffset*np.cos(robotPose[2]) - newHumanPos[0]) + np.pi - robotPose[2]

        newHumanRad = np.linalg.norm(newHumanPos - regionAxis)

        return newHumanPos, newHumanRad, newHumanPhi

    def updateAndPublish(self):
        if(self.currentMsgOdom is None or self.positionWorldCart is None):
            return

        (self.positionWorldCart, self.rad, self.phi) = self.getHumanPose(self.currentMsgOdom)

        self.msgPosition.position = self.positionWorldCart
        self.msgPosition.rad = self.rad
        self.msgPosition.phi = self.phi

        footprint = (self.circle + np.atleast_2d(self.positionWorldCart).T).T
        self.msgFootprint.polygon.points = [Point32(x = point[0], y = point[1]) for point in footprint]

        self.msgFootprint.header.stamp = rospy.get_rostime()
        self.msgPosition.header.stamp = rospy.get_rostime()

        self.footprintPub.publish(self.msgFootprint)
        self.positionPub.publish(self.msgPosition)

if __name__ == '__main__':
    rospy.init_node('sim_geoC')

    initRelPos = [0.5555, np.pi]
    node = userModel(initRelPos)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        node.updateAndPublish()
        rate.sleep()