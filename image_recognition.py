#
# To enable it you need to add run_script start_find_object.sh;
# to the ~/robot/autostart/autostart_screens.sh. 
# It needs to access /camera/color/image_raw topic to work.
# 
#Use simple names for your images such as 15.jpg 
# place them in ~/robot/data/images folder on QTPC. 
# Relaunch the find_object_2d or reboot the robot to load your images.

# #


from std_msgs.msg import Float32MultiArray

def image_callback(msg):
    print(msg.data)

rospy.Subscriber('/find_object/objects', Float32MultiArray, image_callback)