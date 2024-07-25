# STORM Navigation

This repo contains code for running the navigation stack of STORM, a low-cost mobile robot designed to help students learn the fundamentals of robotics.

![alt text](https://github.com/trinity-robotics/storm-ros/blob/main/Storm_Iteration_2_-_Assembly.png)

## Get Started
The STORM navigation stack utilizes ROS (robot operating system), to orchestrate communication between various peripherals - such as motors, and the onboard computer. 

### Installation


Before diving into the code, this project runs on ROS2 Humble distribution. If not already done so, please follow the official [ROS2 Humble guide](https://docs.ros.org/en/humble/Installation.html) linked to familiarize with the system

first clone this repo. Then follow the below instructions to create a virtual environments - named storm-venv, and install the necessary dependencies for this repo.
```
python -m venv storm-venv
source /storm-venv/bin/activate

pip install -r requirements.txt
```

### Launching Navigation 
To start the navigation stack of STORM, launch the navigation stack using the following. The "stormbot_launch.py" file launches various ROS nodes used in navigation - such as the drivetrain for 
moving the motors, imu for position feedback, localization, lidar and etc. Feel free to dive into any of these files to learn more about how these nodes are brought up. 
```
source /opt/ros/<ros2_distro>/setup.bash

ros2 launch drivetrain stormbot_launch.py
```

### Running Speech Inference
The files in the "ml_inference" folder provides a baseline support for running speech-to-text inference, feel free to add your own functionality for how the robot can react to certain key words!
To run the onboard speech interpretation, use the following commands. 

```
cd ml_inference
python setup_ml.py

python inference.py
```
