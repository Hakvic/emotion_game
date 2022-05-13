#!/usr/bin/env python
import sys
import rospy
import random
import threading
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
from std_msgs.msg import String
from face_recognition import FaceRecognition

emotionDictionnary = {
        "joyeux" : ["joyeux","allègre", "enjoué", "épanoui", "euphorique", "gai", "guilleret", "heureux", "hilare", "jovial", "radieux", "ravi", "réjoui", "riant" ],
        "surpris":["surpris","ahuri", "bouche bée", "coi", "déconcerté", "ébahi", "éberlué", "étonné", "médusé", "stupéfait", "suffoqué" , "épaté", "époustouflé", "estomaqué", "scié", "sidéré"],
        "triste":["triste","abattu", "accablé", "affecté", "affligé","anéanti", "angoissé", "atterré","attristé", "bouleversé", "chagriné",
                    "chagriné", "consterné", "découragé", "défait", "déprimé", "désabusé", "désenchanté", "désespéré", "émouvant",
                    "malheureux", "maussade", "mauvais", "navré", "peiné"],
        "enervé":["enervé","agacé", "agité", "à bout de nerfs", "crispé", "exaspéré", "irrité", "nerveux", "sur les nerfs", "sous pression","colère"]
    }


class EmotionRecognition:
    def __init__(self):

        rospy.init_node('emotion_game')
        rospy.loginfo("emotion_node started!")
        # define a ros service
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
        self.emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
        self.audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)

        # block/wait for ros service
        rospy.wait_for_service('/qt_robot/speech/say')
        rospy.wait_for_service('/qt_robot/speech/recognize')
        rospy.wait_for_service('/qt_robot/audio/play')

    def selectRandomEmotion(self):
        emotions = [
            "animoji_emotions/joyeux", "animoji_emotions/joyeux2",
            "animoji_emotions/surpris", "animoji_emotions/surpris2",
            "animoji_emotions/triste", "animoji_emotions/triste2",
            "animoji_emotions/enerve", "animoji_emotions/enerve2"]
        # "animoji_emotions/peur", "animoji_emotions/peur2"

        currentEmotionIndex = random.randint(0, len(emotions) - 1)

        return emotions[currentEmotionIndex]

    def emotionInDictonnary(self, emotion, transcript):
        for e in emotionDictionnary[emotion]:
            if e in transcript:
                return True
        return False

    def emotionToFrench(self, emotion):

        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return "joyeux"

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return "surpris"

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return "triste"
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return "enervé"
        # elif emotion == "animoji_emotions/peur" or emotion == "animoji_emotions/peur2":
        #   return "peur"

        return False

    def giveHint(self, emotion):
        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return "Je vais partir en vacances demain. Je suis ?"

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return "Je ne m'y attendais pas. Je suis ?"

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return "J'ai perdu mon animal de compagnie. Je suis ?"
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return "Des amis ont parlé en mal derrière mon dos. Je suis ?"
            # elif emotion == "animoji_emotions/peur" or emotion == "animoji_emotions/peur2":
            #   return "Il y a une araigné dans la baignoire. J'ai ?"

        return False

    def emotionFound(self, emotion, transcript):

        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return self.emotionInDictonnary("joyeux", transcript)

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return self.emotionInDictonnary("surpris", transcript)

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return self.emotionInDictonnary("triste", transcript)
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return self.emotionInDictonnary("enervé", transcript)
        # elif emotion == "animoji_emotions/peur" or emotion == "animoji_emotions/peur2":
        #   return "peur"

        return False

    def show_emotion(self, selected_emotion):
        print("show_emotion")
        # face_recongnition = FaceRecognition()
        # face_recongnition.start()
        self.emotionShow(selected_emotion)
        # face_recongnition.stop()


if __name__ == '__main__':
    # call a ros service with text message
    rounds = 3
    er = EmotionRecognition()
    er.speechSay(
        "Nous allons jouer à un jeu. ")#Je vais te montrer une expression. Après le bip, tu devras me donner le nom de cette expression.")
    while (rounds > 0):
        er.speechSay("Quel est cette expression? ")
        selectedEmotion = er.selectRandomEmotion()
        er.show_emotion(selectedEmotion)

        er.audioPlay("beep-01a.wav", "")

        resp = er.recognize("fr_FR", [""], 15)
        rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
        if er.emotionFound(selectedEmotion, resp.transcript) is True:
            er.speechSay("Bien joué tu as trouvé la bonne expression")
        else:
            er.speechSay("Ce n'est pas la bonne expression! Je vais te donner un indice.")
            er.speechSay(er.giveHint(selectedEmotion))
            er.audioPlay("beep-01a.wav", "")
            resp = er.recognize("fr_FR", [""], 15)
            rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
            if er.emotionFound(selectedEmotion, resp.transcript) is True:
                er.speechSay("Bien joué tu as trouvé la bonne expression")
            else:
                er.speechSay(
                    "Ce n'est pas la bonne réponse, le nom de l'expression est %s" % er.emotionToFrench(selectedEmotion))
        rounds = rounds - 1