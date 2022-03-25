import sys
import rospy
import random
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *

if __name__ == '__main__':

    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')
    
    def selectRandomEmotion(): 
        emotions =["QT/happy","QT/surprise","QT/sad","QT/angry","QT/afraid"]
        currentEmotionIndex= random.randint(0,len(emotions)-1)
    
        return emotions[currentEmotionIndex]
    
    def emotionToFrench(emotion):
        if emotion == "QT/happy":
            return "joyeux"
        elif emotion =="QT/surprise":
            return "surpris"
        elif emotion =="QT/sad":
            return "triste"
        elif emotion =="QT/angry":
            return "énervé"
        elif emotion =="QT/afraid":
            return "peur"

    try:
        # call a ros service with text message
        speechSay("Nous allons jouer à un jeu. Je vais te montrer une expression du devras me donner le nom de cette expression.")
        speechSay("Quel est cette expression? ")
        selectedEmotion = selectRandomEmotion()
        emotionShow(selectedEmotion)
        speechSay('#CAR HORN#')
        resp = recognize("fr_FR", ['joyeux', 'surpris', 'triste','peur','énervé'], 30)
        rospy.loginfo("Voici ce que j'ai entendu: %s", resp.transcript)
        if emotionToFrench(selectedEmotion) in resp.transcript :
            speechSay("Bien joué tu as trouvé la bonne expression")
        else:
            speechSay("Ce n'est pas la bonne réponse, le nom de l'expression est %s",emotionToFrench(selectedEmotion))
        
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")