#!/usr/bin/env python
import sys
import time

import rospy
import random
import threading
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
from std_msgs.msg import String
from face_recognition import FaceRecognition
from image_recognition import ImageRecognition

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
        self.previousEmotion = 0


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
        face_recongnition = FaceRecognition()
        face_recongnition.start()
        self.emotionShow(selected_emotion)
        if not face_recongnition.isFocused:
            self.speechSay("Je vais te remontrer l'expression")
            self.emotionShow(selected_emotion)
        face_recongnition.stop()

    def speech_to_text(self):
        resp = self.recognize("fr_FR", [""], 10)
        while resp.transcript == "#TIMEOUT#":
            rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
            del resp
            er.speechSay("Je n'ai pas très bien entendu, pourrai-tu répéter ?")
            resp = er.recognize("fr_FR", [""], 10)
        return resp.transcript

    def start_game(self):
        rounds = 2
        gameStarted = False
        self.speechSay("Nous allons jouer à un jeu.")
        self.speechSay("Avant de commencer, pourrai-tu te placer à 1 mètre de moi ?")
        time.sleep(5)

        self.speechSay("Est-tu placé?")
        respIsReady = self.recognize("fr_FR", ['oui'], 5)
        rospy.loginfo("Voici ce que j'ai entendu: %s", respIsReady.transcript)

        while not gameStarted:

            if respIsReady.transcript == "oui":
                gameStarted = True
            else:
                rospy.loginfo("I got: %s", respIsReady.transcript)
                self.speechSay("Je n'ai pas bien entendu. Est-tu placé?")
                del respIsReady
                respIsReady = self.recognize("fr_FR", ['oui'], 5)

        self.speechSay("Voici comment fonctionne le jeu,"
                    "je vais te montrer une expression. "
                   "Après le bip, tu devras me donner le nom de cette expression.")

        while (rounds > 0):
            self.game()
            rounds = rounds - 1

        ir = ImageRecognition()
        ir.start_game()



    def game(self):

            self.speechSay("Quel est cette expression?")
            selectedEmotion = self.selectRandomEmotion()
            while self.previousEmotion == selectedEmotion:
                selectedEmotion = self.selectRandomEmotion()

            self.previousEmotion = selectedEmotion

            self.show_emotion(selectedEmotion)

            self.audioPlay("beep-01a.wav", "")
            words_said = self.speech_to_text()


            if self.emotionFound(selectedEmotion, words_said ) is True:
                self.speechSay("Bien joué tu as trouvé la bonne expression")
            else:
                self.speechSay("Ce n'est pas la bonne expression! Je vais te donner un indice.")
                self.speechSay(er.giveHint(selectedEmotion))
                words_said = self.speech_to_text()

                if self.emotionFound(selectedEmotion, words_said) is True:
                    self.speechSay("Bien joué tu as trouvé la bonne expression")
                else:
                    self.speechSay(
                        "Ce n'est pas la bonne réponse, le nom de l'expression est %s" % er.emotionToFrench(
                            selectedEmotion))




if __name__ == '__main__':
    try:
        # call a ros service with text message
        er = EmotionRecognition()

        er.start_game()
    except KeyboardInterrupt:
        pass


