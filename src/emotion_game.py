#!/usr/bin/env python
import sys
import time
import rospy
import random
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
from face_recognition import FaceRecognition
from image_recognition import ImageRecognition

emotionDictionnary = {
    "joyeux": ["joyeux", "allègre", "enjoué", "épanoui", "euphorique", "gai", "guilleret", "heureux", "hilare",
               "jovial", "radieux", "ravi", "réjoui", "riant"],
    "surpris": ["surpris", "ahuri", "bouche bée", "coi", "déconcerté", "ébahi", "éberlué", "étonné", "médusé",
                "stupéfait", "suffoqué", "épaté", "époustouflé", "estomaqué", "scié", "sidéré"],
    "triste": ["triste", "abattu", "accablé", "affecté", "affligé", "anéanti", "angoissé", "atterré", "attristé",
               "bouleversé", "chagriné",
               "chagriné", "consterné", "découragé", "défait", "déprimé", "désabusé", "désenchanté", "désespéré",
               "émouvant",
               "malheureux", "maussade", "mauvais", "navré", "peiné"],
    "enervé": ["enervé", "agacé", "agité", "à bout de nerfs", "crispé", "exaspéré", "irrité", "nerveux",
               "sur les nerfs", "sous pression", "colère"]
}


class EmotionRecognition:
    def __init__(self):
        """
        Initialize the different rospy service
        """
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

    def select_random_emotion(self):
        """
        Select a random expression image

        :rtype: string
        :return: the path where the expression image is stored
        """
        emotions = [
            "animoji_emotions/joyeux", "animoji_emotions/joyeux2",
            "animoji_emotions/surpris", "animoji_emotions/surpris2",
            "animoji_emotions/triste", "animoji_emotions/triste2",
            "animoji_emotions/enerve", "animoji_emotions/enerve2"]

        return random.choice(emotions)

    def emotion_dictionary(self, emotion, transcript):
        """
        Check if the word said is in the dictionary

        :param emotion: str, the emotion showed
        :param transcript: str, what the robot heard
        :rtype: bool
        :return: if it's true or false
        """
        for e in emotionDictionnary[emotion]:
            if e in transcript:
                return True
        return False

    def emotion_to_french(self, emotion):
        """
        Translate the emotion chosen by the robot in french word

        :param emotion: str, the emotion showed
        :rtype: str
        :return: The french word
        """

        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return "joyeux"

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return "surpris"

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return "triste"
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return "enervé"

        return False

    def give_hint(self, emotion):
        """
        Give a hint when the child gives a wrong answer

        :param emotion: emotion: str, the emotion showed
        :rtype: str
        :return: A hint to help the child to find the correct expression
        """
        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return "Je vais partir en vacances demain. Je suis ?"

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return "Je ne m'y attendais pas. Je suis ?"

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return "J'ai perdu mon animal de compagnie. Je suis ?"
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return "Des amis ont parlé en mal derrière mon dos. Je suis ?"

        return False

    def emotion_found(self, emotion, transcript):
        """
        Check if the emotion heard by the robot is right.

        :param emotion: str, the emotion showed
        :param transcript: str, what the robot heard
        :rtype: bool
        :return: if it's true or false
        """
        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return self.emotion_dictionary("joyeux", transcript)

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return self.emotion_dictionary("surpris", transcript)

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return self.emotion_dictionary("triste", transcript)
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return self.emotion_dictionary("enervé", transcript)

        return False

    def show_emotion(self, selected_emotion):
        """
        Display the emotion on the screen of the robot and launch the face_recognition to check if the child is looking
        at the screen.

        :param selected_emotion: str, the path of the emotion chosen by the robot
        """
        face_recongnition = FaceRecognition()
        face_recongnition.start()
        self.emotionShow(selected_emotion)
        if not face_recongnition.isFocused:
            self.speechSay("Je vais te remontrer l'expression")
            self.emotionShow(selected_emotion)
        face_recongnition.stop()

    def speech_to_text(self):
        """
        Used to listen what the child said and return the transcription

        :rtype: str
        :return: The transcription of the voice
        """
        resp = self.recognize("fr_FR", [""], 10)
        while resp.transcript == "#TIMEOUT#":
            rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
            del resp
            self.speechSay("Je n'ai pas très bien entendu, pourrai-tu répéter ?")
            resp = self.recognize("fr_FR", [""], 10)
        return resp.transcript

    def start_game(self):
        """
        The scenario of the game.
            - Launch the first game
        """
        rounds = 2
        next_game = True
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

        time.sleep(5)

        while (next_game):
            self.speechSay("Veux-tu changer de jeu ?")
            respIsReady = self.recognize("fr_FR", ['oui'], 5)
            rospy.loginfo("Voici ce que j'ai entendu: %s", respIsReady.transcript)

            if respIsReady.transcript == "oui":
                ir = ImageRecognition()
                ir.start_game()
                next_game = False
            elif respIsReady.transcript == "non":
                self.speechSay("Merci d'avoir joué avec moi !")
                rospy.signal_shutdown("end game")
                next_game = False

    def game(self):
        """
        The first game, emotion recognition game. The robot show an expression and the child try to guess and say the
        emotion.
        """
        self.speechSay("Quel est cette expression?")
        selectedEmotion = self.select_random_emotion()
        while self.previousEmotion == selectedEmotion:
            selectedEmotion = self.select_random_emotion()

        self.previousEmotion = selectedEmotion

        self.show_emotion(selectedEmotion)

        self.audioPlay("beep-01a.wav", "")
        words_said = self.speech_to_text()

        if self.emotion_found(selectedEmotion, words_said) is True:
            self.speechSay("Bien joué tu as trouvé la bonne expression")
        else:
            self.speechSay("Ce n'est pas la bonne expression! Je vais te donner un indice.")
            self.speechSay(self.give_hint(selectedEmotion))
            words_said = self.speech_to_text()

            if self.emotion_found(selectedEmotion, words_said) is True:
                self.speechSay("Bien joué tu as trouvé la bonne expression")
            else:
                self.speechSay(
                    "Ce n'est pas la bonne réponse, le nom de l'expression est %s" % self.emotion_to_french(
                        selectedEmotion))


if __name__ == '__main__':
    try:
        # call a ros service with text message
        er = EmotionRecognition()

        er.start_game()
    except KeyboardInterrupt:
        pass
