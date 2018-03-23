#!/usr/bin/env python

import rospy
import qi
from naoqi import ALProxy, ALModule
from naoqi_sensors.msg import TactileTouch, Bumper

# Publish memory values in corresponding publisher
def publishOnChange(new_values, old_values):
  for i in xrange(0, len(new_values)):
    if new_values[i] != old_values[i]:
      if i<2: 
        bumper_msg = Bumper()
        bumper_msg.button = i #0:LeftFoot; 1:RightFoot
        bumper_msg.state = new_values[i]
        bumper_pub.publish(bumper_msg)
      else:
        tactile_msg = TactileTouch()
        tactile_msg.button = i-2 #0:HeadFront; 1:HeadMiddle; 2:HeadRear
        tactile_msg.state = new_values[i]
        tactile_pub.publish(tactile_msg)

# Main function    
if __name__ == '__main__':

  # Get parameters from launch file
  nao_ip = rospy.get_param('nao_ip', '127.0.0.1')
  bumper_topic = rospy.get_param('bumper_topic', 'bumper')
  tactile_topic = rospy.get_param('tactile_topic', 'tactile_touch')
      
  # ROS initialization:
  rospy.init_node('naoqi_sensors') 
  
  # ROS Publishers definition
  bumper_pub = rospy.Publisher(bumper_topic, Bumper, queue_size=1, latch=True)
  tactile_pub = rospy.Publisher(tactile_topic, TactileTouch, queue_size=1, latch=True)
  
  # Start app session with NAOqi
  connection_url = "tcp://" + nao_ip + ":9559"
  app = qi.Application(["NaoqiSensors", "--qi-url=" + connection_url])
  app.start()
  session = app.session

  # Get the service ALMemory.
  memory = session.service("ALMemory")

  # Get data from memory
  old_values = [0, 0, 0, 0, 0]
  new_values = [0, 0, 0, 0, 0]
  r = rospy.Rate(10) # 10hz
  # ROS spin loop
  while not rospy.is_shutdown():
    old_values = new_values
    left_foot_val = memory.getData("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value")
    right_foot_val = memory.getData("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value")
    head_front_val = memory.getData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value")
    head_middle_val = memory.getData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value")
    head_rear_val = memory.getData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value")
    new_values = [left_foot_val, right_foot_val, head_front_val, head_middle_val, head_rear_val]
    # Publish info only when there is a change
    publishOnChange(new_values, old_values)
    r.sleep()
    
  rospy.loginfo("Stopping naoqi_sensors module ...")
  exit(0)

