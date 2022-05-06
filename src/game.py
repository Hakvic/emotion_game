#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from qt_robot_interface.srv import *
import threading

class EmotionGame:
    def __int__(self):
        rospy.init_node('node_emotion_game')
        rospy.loginfo("node_emotion_game started!")

    def init_speech(self):
        """
        Initialise the speech service.
        """
        # define a ros service
        self.speech_conf = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)
        self.speech = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
        self.emotion_show = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)

        # block/wait for ros service
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/speech/config')
        rospy.wait_for_service('/qt_robot/speech/recognize')

    def show_emotion(self, emotion):
        pass

    def emotion_reco(self):
        pass

    def voice_reco(self):
        pass

    def check_inattention(self):


        print("orientation: ", face.angles[0])

    def say(self, words, language="fr-FR", pitch=0, speed=0):
        self.speech_conf(language, pitch, speed)
        self.speech(words)

    def give_vlue(self):
        pass

    def scenario(self):
        pass


