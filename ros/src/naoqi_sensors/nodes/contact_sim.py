#!/usr/bin/env python
import rospy
import qi
from naoqi import ALProxy
import random
import time

if __name__ == '__main__':
    try:
        # Get parameters from launch file
        nao_ip = rospy.get_param('nao_ip', '127.0.0.1')

	# ROS initialization:
  	rospy.init_node('naoqi_sensors_simulation') 
        # Start app session with NAOqi
        connection_url = "tcp://" + nao_ip + ":9559"
        app = qi.Application(["NaoqiSensors", "--qi-url=" + connection_url])
        app.start()
        session = app.session

        # Get the service ALMemory.
        memProxy = session.service("ALMemory")
        
        while True:
            #insertData. Value can be int, float, list, string
            value = random.randint(0,1)
            memProxy.insertData("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value", value)
            memProxy.insertData("Device/SubDeviceList/RFoot/Bumper/Right/Sensor/Value", value)
            memProxy.insertData("Device/SubDeviceList/Head/Touch/Front/Sensor/Value", float(value))
            memProxy.insertData("Device/SubDeviceList/Head/Touch/Middle/Sensor/Value", float(value))
            memProxy.insertData("Device/SubDeviceList/Head/Touch/Rear/Sensor/Value", float(value))
            time.sleep(1)
            #getData
            #print "The value:", memProxy.getData("Device/SubDeviceList/LFoot/Bumper/Right/Sensor/Value")

    except RuntimeError,e:
        # catch exception
        print "error insert data", e
