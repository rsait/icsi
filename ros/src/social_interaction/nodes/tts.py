#! /usr/bin/env python
# ROS node to detect faces in the camera image using OpenCV
# 
# Copyright 2018 I.Rodriguez EHU

import rospy
import qi
import os
import subprocess
import commands
from gtts import gTTS
from social_interaction.srv import SayText, SayTextResponse

# Task: Define a service server that generates an audio giving the text and the language
def sayText(language, text):
    # Decode utf8
    text = text.decode('utf-8')
    # Generate audio and save it using gTTS
    tts = gTTS(text=text, lang=language)
    filename = audio_path + "say_text.wav"
    tts.save(filename)

    try:
      open(filename, "r")
      cmd_vlc = 'cvlc --play-and-exit ' + filename
      os.system(cmd_vlc)
    except Exception as e:
      rospy.logerr(str(e))
################################## Services ######################################
def sayTextService(req):
    sayText(req.language, req.text)
    return SayTextResponse(True)

if __name__ == '__main__':

    # Read parameters
    audio_path = rospy.get_param('audio_path', "/home/ehu/robotak/icsi/ros/src/social_interaction/audio/")
    # ROS Init node
    rospy.init_node('tts')
    
    # ROS Services Server
    say_text_service = rospy.Service("speech/tts/say_text", SayText, sayTextService)
    # ROS spin loop
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
    #rospy.spin()

    rospy.loginfo("Stopping face_detection module ...")
    exit(0)
