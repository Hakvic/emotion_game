#!/usr/bin/env python
from __future__ import print_function

# import sys
import rospy
import cv2
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo
from std_msgs.msg import Float32MultiArray


class image_converter:
    faces = None
    faces_time = None

    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        self.object_sub= rospy.Subscriber('/find_object/objects', Float32MultiArray, self.image_callback)

    def image_callback(self,msg):
        print(msg.data)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('object_recognition', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
