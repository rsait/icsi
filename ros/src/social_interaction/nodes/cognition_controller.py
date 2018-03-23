#! /usr/bin/env python

# ROS node to detect faces in the camera image using OpenCV
# 
# Copyright 2018 I.Rodriguez EHU

import rospy

from social_interaction.msg import Face
from social_interaction.srv import SayText, SayTextRequest

count = 0
area_max = 10000
area_min = 3000

# Task: Control how close or far the face is from the camera
# If the face is too close say: "You are getting too close, please move away from the camera"
# On the contrary, if it is too far say: "Please get close to the camera"
def faceDetectionCallback(msg):
    global count
    #print "Count: ", count
    area = msg.area
    if area > area_max:
        count = count + 1
    elif area < area_min:
        count = count + 1
    else:
        count = 0

    if (count%30 == 0 and area>area_max):
        say_text_client(SayTextRequest("You are getting too close, please move away from the camera","en"))
        count = 0
    elif (count%30 == 0 and area<area_min):
        say_text_client(SayTextRequest("Please get close to the camera","en"))
        count = 0


if __name__ == '__main__':

    # Read parameters
    tts_text_topic = rospy.get_param('tts_text_topic', 'speech/tts/text')
    face_topic = rospy.get_param('face_topic','face')
    # ROS Init node
    rospy.init_node('cognitive_controller')
    
    #ROS Subscribers
    face_sub = rospy.Subscriber(face_topic, Face, faceDetectionCallback, queue_size = 1)
    # Services Client
    rospy.wait_for_service('speech/tts/say_text')
    say_text_client = rospy.ServiceProxy('speech/tts/say_text', SayText)
    # ROS spin loop
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
    #rospy.spin()

    rospy.loginfo("Stopping face_detection module ...")
    exit(0)
