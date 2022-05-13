#!/usr/bin/env python
import sys
import rospy
import random
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *
from std_msgs.msg import String

if __name__ == '__main__':

    rospy.init_node('robot_emotion_recognition')
    rospy.loginfo("robot_emotion_recognition_node started!")

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')
    rospy.wait_for_service('/qt_robot/audio/play')

    emotionDictionnary = {
        "joyeux" : ["joyeux","allègre", "enjoué", "épanoui", "euphorique", "gai", "guilleret", "heureux", "hilare", "jovial", "radieux", "ravi", "réjoui", "riant" ],
        "surpris":["surpris","ahuri", "bouche bée", "coi", "déconcerté", "ébahi", "éberlué", "étonné", "médusé", "stupéfait", "suffoqué" , "épaté", "époustouflé", "estomaqué", "scié", "sidéré"],
        "triste":["triste","abattu", "accablé", "affecté", "affligé","anéanti", "angoissé", "atterré","attristé", "bouleversé", "chagriné",
                    "chagriné", "consterné", "découragé", "défait", "déprimé", "désabusé", "désenchanté", "désespéré", "émouvant",
                    "malheureux", "maussade", "mauvais", "navré", "peiné"],
        "enervé":["enervé","agacé", "agité", "à bout de nerfs", "crispé", "exaspéré", "irrité", "nerveux", "sur les nerfs", "sous pression"]
    }
    
    def selectRandomEmotion(): 
        emotions = [
            "animoji_emotions/joyeux","animoji_emotions/joyeux2",
            "animoji_emotions/surpris", "animoji_emotions/surpris2",
            "animoji_emotions/triste","animoji_emotions/triste2",
            "animoji_emotions/enerve","animoji_emotions/enerve2"]
       # "animoji_emotions/peur", "animoji_emotions/peur2"

        currentEmotionIndex= random.randint(0,len(emotions)-1)
    
        return emotions[currentEmotionIndex]

    def emotionInDictonnary(emotion,transcript):
        for e in emotionDictionnary[emotion]:
            if e in transcript:
                return True
        return False
    
    def emotionToFrench(emotion):

        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return "joyeux"

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return "surpris"

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return "triste"
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return "enervé"
        #elif emotion == "animoji_emotions/peur" or emotion == "animoji_emotions/peur2":
         #   return "peur"

        return False

    def giveHint(emotion):
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

    def emotionFound(emotion,transcript):

        if emotion == "animoji_emotions/joyeux" or emotion == "animoji_emotions/joyeux2":
            return emotionInDictonnary("joyeux",transcript)

        elif emotion == "animoji_emotions/surpris" or emotion == "animoji_emotions/surpris2":
            return emotionInDictonnary("surpris",transcript)

        elif emotion == "animoji_emotions/triste" or emotion == "animoji_emotions/triste2":
            return emotionInDictonnary("triste",transcript)
        elif emotion == "animoji_emotions/enerve" or emotion == "animoji_emotions/enerve2":
            return emotionInDictonnary("enervé",transcript)
        #elif emotion == "animoji_emotions/peur" or emotion == "animoji_emotions/peur2":
         #   return "peur"

        return False

    try:
        # call a ros service with text message

        #speechSay("Nous allons jouer à un jeu. Je vais te montrer une expression. Tu devras me donner le nom de cette expression après le bip.")
        #speechSay("Quel est cette expression? ")
        selectedEmotion = selectRandomEmotion()
        emotionShow(selectedEmotion)
        
        audioPlay("beep-01a.wav","")

        resp = recognize("fr_FR", [""], 15)
        rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
        if emotionFound(selectedEmotion,resp.transcript) is True:
            speechSay("Bien joué tu as trouvé la bonne expression")
        else:
            speechSay("Ce n'est pas la bonne expression! Je vais te donner un indice.")
            speechSay(giveHint(selectedEmotion))
            audioPlay("beep-01a.wav","")
            resp = recognize("fr_FR", [""], 15)
            rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
            if emotionFound(selectedEmotion, resp.transcript) is True:
                speechSay("Bien joué tu as trouvé la bonne expression")
            else:
                speechSay("Ce n'est pas la bonne réponse, le nom de l'expression est %s"%emotionToFrench(selectedEmotion))
        
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")
