#!/usr/bin/env python

from __future__ import print_function

import random


import sys
import cv2
import roslib
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo
from qt_robot_interface.srv import *

class image_converter:
    faces = None
    faces_time = None
    frames = 0
    child_name = "Sacha du bour-palette"

    def __init__(self):
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.face_sub = rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_callback)
        
        self.speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
        
        self.bridge = CvBridge()

    def face_callback(self, data):
        # print("face_callback")
        self.lock.acquire()
        self.faces = data.faces
        self.faces_time = rospy.Time.now()
        self.lock.release()
        
    def parler(self, message_a_dire):
        # On attend la connection
        date_debut = rospy.get_time()
        while (self.speechSay_pub.get_num_connections() == 0) :
            rospy.loginfo("en attente d'une réponse du publisher")
            if rospy.get_time() - date_debut > 5.0:
                rospy.logerr("La connexion avec le publisher n'a pas pu être établie ! Abandon...")
                sys.exit()
            rospy.sleep(1)
        print(message_a_dire)
        self.speechSay_pub.publish(message_a_dire)

    def callback(self, data):
        self.frames += 1 #

        secondes = self.frames // 27 # On a environ 27 fps ave la caméra donc on compte combien de fois on peut diviser le nb de frames par 27

        try:
            if secondes % 10 >= 9: #Toutes les 10 secondes les 1 dernieres secondes
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            else: # Toutes les 10 secondes les 1 dernières secondes
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
 
        except CvBridgeError as e:
            print(e)

        if not self.frames % 27: # Si on remarque que le reste de la division du nb de frame par les 27 fps on obtient alors le nb de secondes estimé
            print(secondes)

            
            if secondes % 5 == 0: #Toutes les 10 secondes les 1 dernieres secondes
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
                
            else: # Toutes les 10 secondes les 1 dernières secondes
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                
            print(cv_image.shape)
            
            if secondes == 1:
                self.parler("Hey ! . Est-ce que tu peux me dire comment tu tappelle ?")
            if secondes == 9:
                self.parler("Salut " + self.child_name + ". Comment ça va ?")
                
            if secondes == 15:
                self.parler("Moi ça va. Mais dis-moi. Quel âge as-tu ? ")
                
            if secondes == 22:
                self.parler("Super ! . Est-ce que tu connais Pokémon ? ")
                
            if secondes == 29:
                self.parler("Super ! . Il est temps de travailler maintenant. Est-ce que tu préfère travailler les mathématiques ou bien le français ?")
                
            if secondes == 44:
                self.parler("C'est parti pour faire des mathématiques !")
                
            if secondes == 50:
                self.parler("Combien font 2+2 ?")    

            if secondes == 56:
                self.parler("C'est ça ! Bravo !")    

            if secondes == 62:
                self.parler("Maintenant, combien font 14 exposant 17 fois 45 moins 12 ?" + self.child_name + " ?")    
               
            if secondes == 70:
                self.parler("Oh non ! Ce n'est pas ça ! Tant pis, tu feras mieux la prochaine fois, garde courage " + self.child_name + " !")  
        (rows, cols, channels) = cv_image.shape

        if cols > 60 and rows > 60: # On vérifie la taille de l'écran
            #cv2.circle(cv_image, (250, 250), 150, (158, 108, 253), 0) # On dessine un cercle pour Akhara
            cv2.putText(cv_image, str(secondes), (0, 25), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1, (25, 25, 200), 2, 5)
            #cv2.addWeighted(cv_image, 0.4, cv2.imread("isep.png"), 0.1, 0.0)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
