#!/usr/bin/env python
from __future__ import print_function

# import sys
import rospy
import cv2
import threading
from qt_robot_interface.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from qt_nuitrack_app.msg import Faces, FaceInfo


class FaceRecognition(threading.Thread):
    faces = None
    faces_time = None

    def __init__(self):
        # self.lock = threading.Lock()
        threading.Thread.__init__(self)
        # rospy.init_node('face', anonymous=True)
        self.bridge = CvBridge()
        self.is_face_detection_running = False
        self.is_focused = True
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.image_pub = rospy.Publisher("/face_recognition/out", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.face_sub = rospy.Subscriber("/qt_nuitrack_app/faces", Faces, self.face_callback)
        self.speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)

    def run(self):
        """
        Start the thread on the image_callback function
        """
        self.is_face_detection_running = True

    def stop(self):
        """
        Stop the thread on the image_callback function
        :return:
        """
        print("stop")
        self.is_face_detection_running = False
        # rospy.signal_shutdown("finish")
        self.is_focused = True

    def face_callback(self, data):
        """
        Callback used to get the face information from the nuitrack camera

        :param data: Faces, from the message qt_nuitrack_app.msg
        """
        # print("face_callback")
        # self.lock.acquire()
        self.faces = data.faces
        self.faces_time = rospy.Time.now()
        # self.lock.release()

    def image_callback(self, data):
        """
        Callback used to get the image from the camera. Then we merge the face information and the image to display
        in live the information.

        :param data: Images, from the message sensor_msgs.msg
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        # self.lock.acquire()
        new_faces = self.faces
        new_faces_time = self.faces_time
        # self.lock.release()

        # if new_faces_time is not None:
        #     print("now: ", rospy.Time.now())
        #     print("face time: ", new_faces_time)
        if self.is_face_detection_running:
            if new_faces and (rospy.Time.now() - new_faces_time) < rospy.Duration(5.0):
                for face in new_faces:
                    rect = face.rectangle
                    cv2.rectangle(cv_image, (int(rect[0] * cols), int(rect[1] * rows)),
                                  (int(rect[0] * cols + rect[2] * cols), int(rect[1] * rows + rect[3] * rows)),
                                  (0, 255, 0), 2)
                    x = int(rect[0] * cols)
                    y = int(rect[1] * rows)
                    w = int(rect[2] * cols)
                    h = int(rect[3] * rows)
                    # cv2.putText(cv_image, "Gender:", (x, y+h+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), lineType=cv2.LINE_AA)
                    cv2.putText(cv_image, "Gender: %s" % face.gender, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 1, lineType=cv2.LINE_AA)
                    cv2.putText(cv_image, "Age: %d" % face.age_years, (x, y + h + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 0), 1, lineType=cv2.LINE_AA)

                    # Neutral
                    cv2.putText(cv_image, "Neutral:", (x, y + h + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                                lineType=cv2.LINE_AA)
                    cv2.rectangle(cv_image, (x + 80, y + h + 50),
                                  (x + 80 + int(face.emotion_neutral * 100), y + h + 10 + 50), (0, 255, 0), cv2.FILLED)
                    cv2.rectangle(cv_image, (x + 80, y + h + 50),
                                  (x + 80 + 100, y + h + 10 + 50), (255, 255, 255), 1)
                    # Angry
                    cv2.putText(cv_image, "Angry:", (x, y + h + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                                lineType=cv2.LINE_AA)
                    cv2.rectangle(cv_image, (x + 80, y + h + 70),
                                  (x + 80 + int(face.emotion_angry * 100), y + h + 10 + 70), (0, 255, 0), cv2.FILLED)
                    cv2.rectangle(cv_image, (x + 80, y + h + 70),
                                  (x + 80 + 100, y + h + 10 + 70), (255, 255, 255), 1)

                    # Happy
                    cv2.putText(cv_image, "Happy:", (x, y + h + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                                lineType=cv2.LINE_AA)
                    cv2.rectangle(cv_image, (x + 80, y + h + 90),
                                  (x + 80 + int(face.emotion_happy * 100), y + h + 10 + 90), (0, 255, 0), cv2.FILLED)
                    cv2.rectangle(cv_image, (x + 80, y + h + 90),
                                  (x + 80 + 100, y + h + 10 + 90), (255, 255, 255), 1)

                    cv2.putText(cv_image, "Surprise:", (x, y + h + 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                                lineType=cv2.LINE_AA)
                    cv2.rectangle(cv_image, (x + 80, y + h + 110),
                                  (x + 80 + int(face.emotion_surprise * 100), y + h + 10 + 110), (0, 255, 0),
                                  cv2.FILLED)
                    cv2.rectangle(cv_image, (x + 80, y + h + 110),
                                  (x + 80 + 100, y + h + 10 + 110), (255, 255, 255), 1)
                    # Face angle
                    print('face orientation: ', face.angles)
                    if 10 > face.angles[0] > -16:
                        print("Regarde tout droit")
                    elif face.angles[0] > 10:
                        self.is_focused = False
                        self.speechSay("Hey, concentres toi!")
                        print("regarde à gauche")

                    elif face.angles[0] < -16:
                        self.is_focused = False
                        self.speechSay("Hey, concentres toi!")
                        print("regarde à droite")
                    else:
                        print("good")
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
