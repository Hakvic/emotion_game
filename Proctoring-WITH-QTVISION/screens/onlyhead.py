#!/usr/bin/env python

from __future__ import print_function

import random


import sys
import cv2
import roslib
import rospy
import numpy as np
import math

from face_detector import get_face_detector, find_faces
from face_landmarks import get_landmark_model, detect_marks
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo
from qt_robot_interface.srv import *

def get_2d_points(img, rotation_vector, translation_vector, camera_matrix, val):
    """Return the 3D points present as 2D for making annotation box"""
    point_3d = []
    dist_coeffs = np.zeros((4,1))
    rear_size = val[0]
    rear_depth = val[1]
    point_3d.append((-rear_size, -rear_size, rear_depth))
    point_3d.append((-rear_size, rear_size, rear_depth))
    point_3d.append((rear_size, rear_size, rear_depth))
    point_3d.append((rear_size, -rear_size, rear_depth))
    point_3d.append((-rear_size, -rear_size, rear_depth))
    
    front_size = val[2]
    front_depth = val[3]
    point_3d.append((-front_size, -front_size, front_depth))
    point_3d.append((-front_size, front_size, front_depth))
    point_3d.append((front_size, front_size, front_depth))
    point_3d.append((front_size, -front_size, front_depth))
    point_3d.append((-front_size, -front_size, front_depth))
    point_3d = np.array(point_3d, dtype=np.float).reshape(-1, 3)
    
    # Map to 2d img points
    (point_2d, _) = cv2.projectPoints(point_3d,
                                      rotation_vector,
                                      translation_vector,
                                      camera_matrix,
                                      dist_coeffs)
    point_2d = np.int32(point_2d.reshape(-1, 2))
    return point_2d

def draw_annotation_box(img, rotation_vector, translation_vector, camera_matrix,
                        rear_size=300, rear_depth=0, front_size=500, front_depth=400,
                        color=(255, 255, 0), line_width=2):
    """
    Draw a 3D anotation box on the face for head pose estimation

    Parameters
    ----------
    img : np.unit8
        Original Image.
    rotation_vector : Array of float64
        Rotation Vector obtained from cv2.solvePnP
    translation_vector : Array of float64
        Translation Vector obtained from cv2.solvePnP
    camera_matrix : Array of float64
        The camera matrix
    rear_size : int, optional
        Size of rear box. The default is 300.
    rear_depth : int, optional
        The default is 0.
    front_size : int, optional
        Size of front box. The default is 500.
    front_depth : int, optional
        Front depth. The default is 400.
    color : tuple, optional
        The color with which to draw annotation box. The default is (255, 255, 0).
    line_width : int, optional
        line width of lines drawn. The default is 2.

    Returns
    -------
    None.

    """
    
    rear_size = 1
    rear_depth = 0
    front_size = img.shape[1]
    front_depth = front_size*2
    val = [rear_size, rear_depth, front_size, front_depth]
    point_2d = get_2d_points(img, rotation_vector, translation_vector, camera_matrix, val)
    # # Draw all the lines
    cv2.polylines(img, [point_2d], True, color, line_width, cv2.LINE_AA)
    cv2.line(img, tuple(point_2d[1]), tuple(
        point_2d[6]), color, line_width, cv2.LINE_AA)
    cv2.line(img, tuple(point_2d[2]), tuple(
        point_2d[7]), color, line_width, cv2.LINE_AA)
    cv2.line(img, tuple(point_2d[3]), tuple(
        point_2d[8]), color, line_width, cv2.LINE_AA)

def head_pose_points(img, rotation_vector, translation_vector, camera_matrix):
    """
    Get the points to estimate head pose sideways    

    Parameters
    ----------
    img : np.unit8
        Original Image.
    rotation_vector : Array of float64
        Rotation Vector obtained from cv2.solvePnP
    translation_vector : Array of float64
        Translation Vector obtained from cv2.solvePnP
    camera_matrix : Array of float64
        The camera matrix

    Returns
    -------
    (x, y) : tuple
        Coordinates of line to estimate head pose

    """
    rear_size = 1
    rear_depth = 0
    front_size = img.shape[1]
    front_depth = front_size*2
    val = [rear_size, rear_depth, front_size, front_depth]
    point_2d = get_2d_points(img, rotation_vector, translation_vector, camera_matrix, val)
    y = (point_2d[5] + point_2d[8])//2
    x = point_2d[2]
    
    return (x, y)
    
face_model = get_face_detector()
landmark_model = get_landmark_model()
font = cv2.FONT_HERSHEY_SIMPLEX 
# 3D model points.
model_points = np.array([
                            (0.0, 0.0, 0.0),             # Nose tip
                            (0.0, -330.0, -65.0),        # Chin
                            (-225.0, 170.0, -135.0),     # Left eye left corner
                            (225.0, 170.0, -135.0),      # Right eye right corne
                            (-150.0, -150.0, -125.0),    # Left Mouth corner
                            (150.0, -150.0, -125.0)      # Right mouth corner
                        ])

# Camera internals
size = (480,640)
focal_length = size[1]
center = (size[1]/2, size[0]/2)
camera_matrix = np.array(
                         [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype = "double"
                         )
                         
class image_converter:
    faces = None
    faces_time = None
    frames = 0
    child_name = "Tony"

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
       
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        faces = find_faces(img, face_model)
        
        for face in faces:
            marks = detect_marks(img, landmark_model, face)
            # mark_detector.draw_marks(img, marks, color=(0, 255, 0))
            image_points = np.array([
                                    marks[30],     # Nose tip
                                    marks[8],     # Chin
                                    marks[36],     # Left eye left corner
                                    marks[45],     # Right eye right corne
                                    marks[48],     # Left Mouth corner
                                    marks[54]      # Right mouth corner
                                ], dtype="double")
            dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
            (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_UPNP)
            
            
            # Project a 3D point (0, 0, 1000.0) onto the image plane.
            # We use this to draw a line sticking out of the nose
            
            (nose_end_point2D, jacobian) = cv2.projectPoints(np.array([(0.0, 0.0, 1000.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
            
            for p in image_points:
                cv2.circle(img, (int(p[0]), int(p[1])), 3, (0,0,255), -1)
            
            
            p1 = ( int(image_points[0][0]), int(image_points[0][1]))
            p2 = ( int(nose_end_point2D[0][0][0]), int(nose_end_point2D[0][0][1]))
            x1, x2 = head_pose_points(img, rotation_vector, translation_vector, camera_matrix)

            cv2.line(img, p1, p2, (0, 255, 255), 2)
            cv2.line(img, tuple(x1), tuple(x2), (255, 255, 0), 2)
            #for (x, y) in marks:
            #   cv2.circle(img, (x, y), 4, (255, 255, 0), -1)
            #cv2.putText(img, str(p1), p1, font, 1, (0, 255, 255), 1)
            try:
                m = (p2[1] - p1[1])/(p2[0] - p1[0])
                ang1 = int(math.degrees(math.atan(m)))
            except:
                ang1 = 90
                
            try:
                m = (x2[1] - x1[1])/(x2[0] - x1[0])
                ang2 = int(math.degrees(math.atan(-1/m)))
            except:
                ang2 = 90
                
                # print('div by zero error')
            if ang1 >= 60:
                #print('Bas')
                cv2.putText(img, 'Bas', (530, 30), font, 1, (192, 192, 255), 3)
            elif ang1 <= -15:
                #print('Haut')
                cv2.putText(img, 'Haut', (530, 30), font, 1, (192, 192, 255), 3)
             
            if ang2 >= 30:
                #print('Droit')
                cv2.putText(img, 'Droit', (530, 60), font, 1, (64, 64, 255), 3)
            elif ang2 <= -35:
                #print('Gauche')
                cv2.putText(img, 'Gauche', (530, 60), font, 1, (64, 64, 255), 3)
            
            #cv2.putText(img, str(ang1), tuple(p1), font, 2, (128, 255, 255), 3)
            #cv2.putText(img, str(ang2), tuple(x1), font, 2, (255, 255, 128), 3)
            
            cv2.putText(img, str(ang1), (560, 200), font, 1, (128, 255, 255), 3)
            cv2.putText(img, str(ang2), (560, 250), font, 1, (255, 255, 128), 3)
            
            
        if not self.frames % 27: # Si on remarque que le reste de la division du nb de frame par les 27 fps on obtient alors le nb de secondes estimé
            print(secondes)

            if secondes == 1:
                self.parler("Hey ! . Est-ce que tu peux me dire comment tu tappelle ?")
            if secondes == 6:
                self.parler("Salut " + self.child_name + ". Comment ça va ?")
                
            if secondes == 12:
                self.parler("Moi ça va. Mais dis-moi. Quel âge as-tu ? ")
                
            if secondes == 17:
                self.parler("MMoi aussi ! . Est-ce que tu connais l'âne Trotro ? ")
                
            if secondes == 24:
                self.parler("Super ! . Il est temps de travailler maintenant. Est-ce que tu préfère travailler les mathématiques ou bien le français ?")
                
            if secondes == 34:
                self.parler("C'est parti pour faire des mathématiques !")
                
            if secondes == 38:
                self.parler("Combien font 2+2 ?")    

            if secondes == 42:
                self.parler("C'est ça ! Bravo !")    

            if secondes == 45:
                self.parler("Maintenant, combien font douze multiplié par cinq ?")    
               
            if secondes == 50:
                self.parler("Oh non ! Ce n'est pas ça ! Tant pis, tu feras mieux la prochaine fois, garde courage " + self.child_name + " !")  
                
        cv2.imshow('Image window', img)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
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
