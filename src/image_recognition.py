#!/usr/bin/env python
from __future__ import print_function

import time
import sys
import rospy
import cv2
import threading
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from qt_vosk_app.srv import *


class ImageRecognition:
    frames = 0
    tries = 0
    pictrogram_ids = {
        10: "Drôle",
        11: "Exité",
        12: "Heureux",
        13: "Calme",
        14: "Timide",
        15: "Triste",
        16: "Fatigué",
        17: "Surpris",
        18: "Deçu",
        19: "Anxieux",
        20: "Contrarié",
        21: "Apeuré",
        22: "Méprisant",
        23: "Entêté",
        24: "Fâché",
        25: "Frustré",
    }

    def __init__(self):
        self.round = 2
        self.previous_id = 0
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.object_sub = rospy.Subscriber('/find_object/objects', Float32MultiArray, self.image_callback)
        self.speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        self.emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=20)
        self.recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
        self.emotion_id = random.randrange(10, 25)
        self.foundFlag = False

    def start_game(self):
        """
        Start the game.
        :return:
        """
        try:
            time.sleep(5)
            self.speak("Voici les règles du nouveau jeu. Je vais te donner le nom d'une expression et du devras me "
                       "montrer l'image.")
            self.speak("Montre moi l'émotion %s" % self.pictrogram_ids[self.emotion_id])
            rospy.spin()

        except KeyboardInterrupt:
            print("Shutting down")

    def speak(self, message):
        """
        That is an other method to use the SpeechSay with the publisher.
        Connection to the speechSay publisher. To the robot talk.

        :param message: str, what you want the robot says.
        """
        date_debut = rospy.get_time()
        while self.speechSay_pub.get_num_connections() == 0:
            if rospy.get_time() - date_debut > 5.0:
                rospy.logerr("Connection with publisher failed...")
                exit()
            rospy.sleep(1)
        self.speechSay_pub.publish(message)

    def show_image(self, image):
        """
        Connection with the emotionShow publisher. To display an image on the head on the robot

        :param image: str, path to the image.
        """
        date_debut = rospy.get_time()
        while self.emotionShow_pub.get_num_connections() == 0:
            if rospy.get_time() - date_debut > 5.0:
                rospy.logerr("Connection with publisher failed...")
                exit()
            rospy.sleep(1)
        self.emotionShow_pub.publish(image)

    def image_callback(self, msg):
        """
        Callback used to detect the image show by the kid to the robot and to check if the expression match the image.

        :param msg: Float32MultiArray, information on the image show by the kid from the message std_msgs.msg
        """
        self.frames += 1
        secondes = self.frames // 27

        if secondes == 15 and self.foundFlag is False:
            self.speak("Je n'arrive pas à detecter l'image. Essaie de la rapprocher!")
            self.frames = 0

        if self.tries == 3:
            self.round = self.round - 1
            self.tries = 0
            self.speak("Voici l'image qu'il fallait trouver.")
            self.show_image('pictogram_expression/' + self.pictrogram_ids[self.emotion_id])
            time.sleep(10)

            self.restart_game()
            self.tries = 0

        show_image_id = str(msg.data).split(".")[0][1::]
        print(show_image_id)

        if show_image_id.isdigit():
            if int(show_image_id) != self.previous_id:
                if int(show_image_id) == self.emotion_id and self.foundFlag is False:
                    self.foundFlag = True

                    self.speak("Bravo tu as trouvé l'image!")
                    time.sleep(5)
                    self.tries = 0
                    self.restart_game()
                    self.round = self.round - 1

                else:
                    self.speak("Ce n'est pas la bonne image.")
                    self.tries += 1

                self.frames = 0
            self.previous_id = int(show_image_id)

        else:
            print("Pas d'image détectée.")


    def restart_game(self):
        """
        Restart the game
        """
        if self.round > 0:
            time.sleep(10)
            self.lock.acquire()
            self.foundFlag = False

            self.emotion_id = random.randrange(10, 25)
            self.speak("Je vais te donner une nouvelle émotion. Montre moi l'émotion%s" % self.pictrogram_ids[
                self.emotion_id])
            self.lock.release()
        else:
            self.speak("Merci d'avoir joué avec moi. Au revoir !")
            rospy.signal_shutdown("end game")

    def callback(self, data):
        """
        Callback used to get the camera feedback in live.

        :param data: Image, image of the camera from the message sensor_msgs.msg
        :return:
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
