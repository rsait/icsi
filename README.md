# Master ICSI Practical part: Steps to configure Social Interaction practice

# Choregeraphe Installation
- Download choregraphe from: https://developer.softbankrobotics.com/us-en/downloads/pepper
- Give executable permission at the choregraphe-suite-2.5.5.5-linux64-setup.run
- Run it: ./choregraphe-suite-2.5.5.5-linux64-setup.run
- Select trial mode
- First time you open Choregraphe insert hey: 654e-4564-153c-6518-2f44-7562-206e-4c60-5f47-5f45

# NAOqi sdk configuration
- Download pynaoqi (python sdk) and naoqi-sdk (cpp sdk) from: https://developer.softbankrobotics.com/us-en/downloads/pepper
- Create cpp and python folders and move from Downloads each file into corresponding folder:
  - mkdir -p ~/software/naoqi-sdk/cpp
  - mkdir -p ~/software/naoqi-sdk/python
  - mv ~/Descargas/pynaoqi-python2.7-2.5.5.5-linux64.tar.gz ~/software/naoqi-sdk/python
  - mv ~/Descargas/naoqi-sdk-2.5.5.5-linux64.tar.gz ~/software/naoqi-sdk/cpp
  - cd ~/software/naoqi-sdk/python
  - tar -xvzf pynaoqi-python2.7-2.5.5.5-linux64.tar.gz
  - cd ~/software/naoqi-sdk/cpp
  - tar -xvzf ~/software/naoqi-sdk/cpp/naoqi-sdk-2.5.5.5-linux64.tar.gz ~/software/naoqi-sdk/cpp/

- Add in ~/.bashrc pynaoqi to the PYTHONPATH. Add at the end the following line:
    - export PYTHONPATH=~/software/naoqi-sdk/python/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages:${PYTHONPATH}

# Installation of the software and tool dependencies. Ubuntu 16.04 required
- sudo apt-get install git
- sudo apt-get install ros-kinetic-desktop-full
- sudo apt-get install ros-kinetic-octomap-msgs
- sudo apt-get install ros-kinetic-freenect-camera
- sudo apt-get install ros-kinetic-freenect-launch
- sudo apt-get install python-pip
- sudo pip install gTTS
- pip install --upgrade pip
- sudo apt-get install vlc

# Compiliation process of the developed packages 
- Download from github ROS packages required
  - cd ~
  - git clone https://github.com/rsait/icsi
- Compile the packages
  - cd ~/icsi/ros
  - catkin_make
- Add in ~/.bashrc ROS workspace. Add at the end the following line:
  - source ~/icsi/ros/devel/setup.bash

# Package Explanation Part I: A minimal program for reading memory values of the NAO robot (simulated)
- The package that performs Part I task is named "naoqi_sensors"     
- Package structure:
  - naoqi_sensors/nodes/:
    - contact_memory.py: Touch/Bumper sensors driver. It gets devices data using ALMemory service from NAOqi and publish it in bumper and tactile_touch topics
    - contact_sim.py: Simulates changes in Bumper and Tactile Touch sensors. It writes in corresponding memory random values
  - naoqi_sensors/msg/:
    - Bumper.msg: Bumper sensors info message definition (button and state)
    - TactileTouch.msg: Tactile Touch sensors info message definition (button and state)
  - naoqi_sensors/launch:
    - naoqi_sensors_sim.launch: The launch file for running together contact_memory and contact_sim nodes in simulation
    - naoqi_sensors_robot.launch: The launch file for running together contact_memory in real robot

  - CMakeList.txt: Contains the ROS package dependencies and the creation of the messages
  - packages.xml: Contains the ROS package dependencies and the creation of the messages

# Package Explanation Part IIa: A simple social interaction by combining face tracking, and speech without Robot.
- The package that performs Part II task is named "social_interaction"     
- Package structure:
  - social_interaction/nodes/:
    - face_detection.py: The node that makes the face detection using OpenCV Haarscascade classifier and publish the Face info (centroid and area). The image used is obtained from the Kinect.
    - tts.py: The node that makes the PC able to speak using gTTS. It subscribes to speech topic to receive the text and the language and reproduces the audio from PC's speakers
    - cognition_controller.py: The node that controls when the face is to close or to far from the camera. When it is close or far it advices the user to move away or get closer.
  - social_interaction/msg/:
    - Face.msg: Face info message definition (centroid and area)
  - social_interaction/srv/:
    - SayText.srv: SayText service definition. The request is the text to be said and the language; The response is a boolean
  - social_interaction/launch:
    - face_detection.launch: The launch file for running together freenect package and face_detection node
    - tts.launch: The launch file for running speech capabilities.
    - social_interacion.launch: The launch file for running together face detection, tts, and the cognition controller
  - social_interaction/audio/: Folder to save generated audio with gTTS and then used to be reproduced through the speakers
  - social_interaction/cfg/: Contains the classifiers model for face detection
  - CMakeList.txt: Contains the ROS package dependencies and the creation of the messages and services
  - packages.xml: Contains the ROS package dependencies and the creation of the messages and services

# Package Explanation Part IIb: A simple social interaction by combining face tracking, and speech with a Robot.
- The package that performs Part II task is named "social_interaction_robot". This package requires naoqi_bridge and pepper_sensors_py packages. Both packages are already included in the "social_interaction_robot" metapackage (folder with more than one packages)     
- Package structure:
  - social_interaction_robot/nodes/:
    - face_detection.py: The node that makes the face detection using OpenCV Haarscascade classifier and publish the Face info (centroid and area). The image used is obtained from the robot's camera.
    - tts.py: The node that makes the robot able to speak using NAOqi's speech synthesizer. It subscribes to speech topic to receive the text and the language and reproduces the audio in the robot. 
    - cognition_controller.py: The node that controls when the face is to close or to far from the robot top camera. When it is close or far the robot advices the user to move away or get closer.
  - social_interaction_robot/msg/:
    - Face.msg: Face info message definition (centroid and area)
  - social_interaction_robot/srv/:
    - SayText.srv: SayText service definition. The request is the text to be said and the language; The response is a boolean
  - social_interaction_robot/launch:
    - face_detection.launch: The launch file for running together Robot camera driver and face_detection node. Choose the camera driver according to the robot you want to use. NAO or Pepper.
    - tts.launch: The launch file for running the robot speech capabilities.
    - social_interacion.launch: The launch file for running together face detection, tts, and the cognition controller
  - social_interaction_robot/audio/: Folder to save generated audio with gTTS and then used to be reproduced through the speakers
  - social_interaction_robot/cfg/: Contains the classifiers model for face detection
  - social_interaction_robot/animations/: Contains the animation required to make the robot move to initial position after it speaks.
  - CMakeList.txt: Contains the ROS package dependencies and the creation of the messages and services
  - packages.xml: Contains the ROS package dependencies and the creation of the messages and services

# Running Part I: A simple social interaction by combining face tracking, and speech without Robot.
- In simulation:
  - Open a terminal:
    - cd ~/software/naoqi-sdk/cpp/naoqi-sdk-2.5.5.5-linux64
    - ./naoqi
  - Open Choregraphe. Go to Connect (green wifi signal). You should see a robot with localhost, if not click on "Use fixed IP/hostname" and set it to 127.0.0.1
  - Open a new terminal:
    - roslaunch naoqi_sensors naoqi_sensors_sim.launch
    - check in another two terminals that published info changes in the corresponding topics:
      - rostopic echo /nao_robot/sensors/bumper
      - rostopic echo /nao_robot/sensors/tactile_touch

- In NAO robot:
  - Open a terminal:
    - export NAO_IP="your_robot_ip"
    - roslaunch naoqi_sensors naoqi_sensors_robot.launch
    - check in another two terminals that published info changes in the corresponding topics:
      - rostopic echo /nao_robot/sensors/bumper
      - rostopic echo /nao_robot/sensors/tactile_touch 

# Running Part IIa: A simple social interaction by combining face tracking, and speech without Robot.
- Open a terminal:
  - roslaunch social_interaction social_interaction.launch

# Running Part IIb: A simple social interaction by combining face tracking, and speech with Robot.
- Open a terminal:
  - export NAO_IP="your_robot_ip"
  - roslaunch social_interaction_robot social_interaction.launch
















