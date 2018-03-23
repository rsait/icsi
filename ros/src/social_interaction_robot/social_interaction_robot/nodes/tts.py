#! /usr/bin/env python
# ROS node to detect faces in the camera image using OpenCV
# 
# Copyright 2018 I.Rodriguez EHU

import rospy
import qi
import sys
import os
import subprocess
import commands
from gtts import gTTS        
from social_interaction.srv import SayText, SayTextResponse

path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../animations'))
sys.path.insert(0, path)
import global_gestures
global_gestures = global_gestures.GlobalGestures()

# Task: Define a service server that generates an audio giving the text and the language
def sayText(lan, text):
    if lan == "en":
        language = "English"
    elif lan == "es":
        language = "Spanish"
    tts.setLanguage(language)
    tts_animated.say(str(text), configuration)
    posture = global_gestures.getMotion("InitPosture")
    motion.angleInterpolation(posture[0], posture[1], posture[2], True, _async=True)
    motion.angleInterpolation(posture[0], posture[1], posture[2], True, _async=True)
    
################################## Services ######################################
def sayTextService(req):
    sayText(req.language, req.text)
    return SayTextResponse(True)

if __name__ == '__main__':

    # Read parameters
    nao_ip = rospy.get_param('nao_ip', '192.168.1.10')
    audio_path = rospy.get_param('audio_path', "/home/ehu/robotak/icsi/ros/src/social_interaction/audio/")
    # ROS Init node
    rospy.init_node('tts')

    # Start app session
    connection_url = "tcp://" + nao_ip + ":9559"
    #rospy.loginfo(connection_url)
    app = qi.Application(["NaoqiSpeechTTS", "--qi-url=" + connection_url])
    app.start()
    session = app.session

    # Get the services ALTextToSpeech and ALAudioPlayer.
    tts = session.service("ALTextToSpeech")
    tts_animated = session.service("ALAnimatedSpeech")
    configuration = {"bodyLanguageMode":"contextual"}
    global motion
    motion = session.service("ALMotion")
    
    # ROS Services Server
    say_text_service = rospy.Service("speech/tts/say_text", SayText, sayTextService)
    # ROS spin loop
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
    #rospy.spin()

    rospy.loginfo("Stopping face_detection module ...")
    exit(0)
