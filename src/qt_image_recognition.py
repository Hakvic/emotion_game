#!/usr/bin/env python
from __future__ import print_function

import time
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
    frames = 0

    def __init__(self, emotion):

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.object_sub = rospy.Subscriber('/find_object/objects', Float32MultiArray, self.image_callback)
        self.speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        self.emotion_id = emotion
        self.foundFlag = False

    def speak(self, message_a_dire):
        # On attend la connexion
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
        self.frames += 1
        secondes = self.frames // 27
        print(secondes)

        if secondes == 15 and self.foundFlag is False:
            self.speak("Je n'arrive pas à detecter l'image. Essaie de la rapprocher!")
            self.frames =0


        showImageId = str(msg.data).split(".")[0][1::]
        print(showImageId)

        if showImageId.isdigit():
            if int(showImageId) == self.emotion_id and self.foundFlag is False:
                self.foundFlag = True

                self.speak("Bravo tu as trouvé l'image!")
                time.sleep(5)
                self.restartGame()
            else:
                self.speak("Ce n'est pas la bonne image.")
                time.sleep(15)
            self.frames = 0

        else:
            print("Pas d'id.")

    def restartGame(self):
        self.lock.acquire()
        self.foundFlag=False

        self.emotion_id = random.randrange(10, 28)
        self.speak("Je vais te montrer une nouvelle émotion. Montre moi l'%s" % Emotions(self.emotion_id ))
        self.lock.release()

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
    # randomEmotionId = random.randrange(10, 28)
    randomEmotionId = 26

    print(randomEmotionId)
    rospy.wait_for_service('/qt_robot/speech/say')
    speechSay("Montre moi l' %s" % Emotions(randomEmotionId))
    rospy.init_node('object_recognition', anonymous=True)
    ic = image_converter(randomEmotionId)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
