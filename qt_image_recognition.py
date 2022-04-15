#!/usr/bin/env python
from __future__ import print_function
from select import select
from xml.etree.ElementTree import tostring
from emotionsId import Emotions

import sys
import rospy
import cv2
import threading
import random
from qt_robot_interface.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray


class image_converter:

    def __init__(self,emotionId):

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.object_sub = rospy.Subscriber('/find_object/objects', Float32MultiArray, self.image_callback)
        self.speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        self.emotionId = emotionId

    def speak(self, message_a_dire):
        # On attend la connection
        date_debut = rospy.get_time()
        while (self.speechSay_pub.get_num_connections() == 0):
            rospy.loginfo("en attente d'une réponse du publisher")
            if rospy.get_time() - date_debut > 5.0:
                rospy.logerr("La connexion avec le publisher n'a pas pu être établie ! Abandon...")
                exit()
            rospy.sleep(1)
        print(message_a_dire)
        self.speechSay_pub.publish(message_a_dire)  # On envoie au robot la phrase à dire

    def image_callback(self, msg):

        showImageId = str(msg.data).split(".")[0][1::]

        print(showImageId)
        foundFlag = False
        try:
            if(int(showImageId)==self.emotionId):
                foundFlag = True
                exit()
        except Exception as e:
            print(e)




    def match_image(self, emotionToFind):
        return Emotions[emotionToFind]

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    #randomEmotionId = random.randrange(10, 28)
    randomEmotionId = 5
    print(randomEmotionId)
    rospy.wait_for_service('/qt_robot/speech/say')
    speechSay("Montre moi l'expression suivante" +str(Emotions(randomEmotionId)))
    rospy.init_node('object_recognition', anonymous=True)
    speechSay("Bravoe ")
    ic = image_converter(randomEmotionId)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
