#! /usr/bin/env python

# ROS node to detect faces in the camera image using OpenCV
# 
# Copyright 2018 I.Rodriguez EHU

import rospy
import os 
import cv2
 
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from social_interaction.msg import Face

################################### Topics #######################################
# Task: Detect faces on the image and publish information about the centroid and the area
def cameraCallback(msg):
  try:
    # Get OpenCV Mat image from ROS image
    cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    # Convert the image to gray scale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Get the list of faces detected on the image
    face_list = faceCascade.detectMultiScale(
      gray,
      scaleFactor=1.1,
      minNeighbors=5,
      minSize=(5, 5),
      maxSize=(400, 400),
      flags=0
    )
    # Draw a rectangle around the faces
    for (x, y, w, h) in face_list:
      cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
      centroid_x = w/2 + x
      centroid_y =  h/2 + y
      cv2.circle(cv_image, ( centroid_x, centroid_y), 2, (0, 255, 255), -1)
      area = h * w

    # Publish the face info in a ROS topic
    if len(face_list) > 0:
      face_msg = Face()
      face_msg.centroid.x = centroid_x
      face_msg.centroid.y = centroid_y
      face_msg.area = area
      face_pub.publish(face_msg)

    # Display the resulting frame
    cv2.imshow('Face', cv_image)
    cv2.waitKey(1)
            
  except CvBridgeError as e:
    rospy.logerror("cv_bridge exception: %s", e.what())
    print(e) 
                    
         
if __name__ == '__main__':

  # Get parameters from launch file
  camera_topic = rospy.get_param('camera_topic','image_raw')
  face_topic = rospy.get_param('face_topic','face')

  cv_image = CvBridge()
  # Load Face detector classifier model
  cascPath = os.path.split(os.path.dirname(__file__))[0]+"/cfg/haarcascades/haarcascade_frontalface_alt.xml"
  faceCascade = cv2.CascadeClassifier(cascPath)
 
  # ROS Init node
  rospy.init_node('face_detection')

  #ROS Publishers and Subscribers
  face_pub = rospy.Publisher(face_topic, Face, queue_size=1, latch=True)
  camera_sub = rospy.Subscriber(camera_topic, Image, cameraCallback, queue_size = 10)

  # ROS spin loop
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    r.sleep()
  #rospy.spin()

  rospy.loginfo("Stopping face_detection module ...")
  cv2.destroyAllWindows()
  exit(0)

