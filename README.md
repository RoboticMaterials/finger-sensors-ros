# finger-sensors-ros
Official ROS Package for Robotic Materials Finger Sensors

## Introduction

This package provides a ROS interface for the Robotic Materials [Smart Gripper Pads for Robotiq](https://roboticmaterials.com/rm/product/smart-gripper-pads-for-robotiq/) and [Smart Fingers for Kinova](https://roboticmaterials.com/rm/product/smart-fingers-for-kinova/). This package is useful if you have one or more of the following use cases:

- Grasping a fragile or deformable object
- Grasping object to use as a tool
- Grasping objects and manipulating them at high speeds
- Pregrasp verification
- Grasp verification
- 3D Scanner

## Prerequisites 

- [ROS Indigo or higher](http://wiki.ros.org/kinetic/Installation)

- PySerial

```
$ sudo apt-get install python-serial
```

- Numpy


```
$ sudo apt-get install python-numpy
```

## Installation
Go to your ROS workspace src directory (if you do not have a ROS workspace follow this [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)), i.e
```
$ cd ~/catkin_ws/src
```
and then pull this repository
```
$ https://github.com/RoboticMaterials/finger-sensors-ros.git
```
If you are using [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) perform the following
```
$ cd ..
$ catkin_make
```
Otherwise, if you are using catkin build 
```
$ catkin build
```
Now to restart your terminal or run 
```
$ source ~/.bashrc
```
After that, you should be able to perform
```
$ roscd finger_sensors
```
to see if the package was properly installed.

## Starting the Finger Sensors

Included in this repository is a launch file under the *finger_sensors* package called *finger_sensors.launch*. To launch this file run
```
$ roslaunch finger_sensors finger_sensors.launch
```
Before we go on to how to use the finger sensor data lets take a quick look at *finger_sensors.launch*. You can open it by running 
```
$ rosed finger_sensors finger_sensors.launch
```
and the file should look like this
```
<launch>
  <node name="finger_sensors" pkg="finger_sensors" type="finger_sensors.py">
    <param name="alpha" value="0.3" />
    <param name="touch_tolerance" value="50" />
  </node>
</launch>
```
As you can see there are two parameters **alpha** and **touch_tolerance**. Here is a brief description of them:

#### touch_tolerance

This value determines how sensitive the touch is. The higher the value the less sensitive the sense of touch is and vice-versa. Since these are optical sensors if you see that the fingers are detecting touch prior to actual contact increase this value. If you are working with deformable or breakable objects set the tolerance to a lower value. Lastly, if you are just using this to verify an object is grasped and are not concerned with displacement or deformation go ahead and set it to a higher value. The default value of 50 should be sufficient for most use cases.

#### alpha

Because this is an optical sensor, it is vulnerable to noise. It is an IR based sensor, so you do not have to be concerned with lighting conditions, but if you have IR sources in proximity to the sensor you may want to decrease this value to reduce the effect of noise. This is a concern if you are using this as a [3D scanner](http://www.cs.utexas.edu/~jsinapov/AAAI-SSS-2017/paper/Cox_AAAI_SSS_2017.pdf). Touch and other tactile signals should not be affected. 

To change these values just change the **value="<your_value>"** adjacent to the parameter you want to change, i.e. 
```
    <param name="touch_tolerance" value="100" />
```

# Integrating Finger Sensor Data

If you are not familiar with subscribing to a topic please view one of the following tutorials [Python](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29_) or [C++](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29). In this package, we give the users access to three values: SAI, FAI, and touch. This package publishes two topics for each value, one that stores the values for each sensor on the left finger and one that stores the values for each sensor on the right finger. Here is a brief description of each.

#### Touch ([finger_sensor_msgs/FingerTouch](https://github.com/RoboticMaterials/finger-sensors-ros/blob/master/finger_sensor_msgs/msg/FingerTouch.msg))

##### Default Topic : /left_finger/touch and /right_finger/touch

Gives a boolean value on whether a sensor has made contact with an object.


#### FAI ([finger_sensor_msgs/FingerFAI](https://github.com/RoboticMaterials/finger-sensors-ros/blob/master/finger_sensor_msgs/msg/FingerFAI.msg))

##### Default Topic : /left_finger/fai and /right_finger/fai

This is for more advanced users. It is used to determine if a sensor has made contact with an object. The value is dependent on surface properties, so users can use this value to determine which object has made contact with the sensor and then adjust the grasping strength accordingly. This is useful in manipulating deformable objects that are likely to slip.

#### SAI ([finger_sensor_msgs/FingerSAI](https://github.com/RoboticMaterials/finger-sensors-ros/blob/master/finger_sensor_msgs/msg/FingerSAI.msg))

##### Default Topic : /left_finger/sai and /right_finger/sai

This is for more advanced users. This can be used for two purposes, to correct the pre-grasp pose and/or to detect slip during manipulation. With the former use case, the SAI signal acts as a  [3D scanner](http://www.cs.utexas.edu/~jsinapov/AAAI-SSS-2017/paper/Cox_AAAI_SSS_2017.pdf) before making contact. Use this to verify if an object is within the robot's end-effector, or use it to scan the object and find the correct grasp pose. Once the object is grasped, use the SAI signal to determine if the object is slipping during manipulation to adjust grasp strength accordingly. 

Here is a sample of what the signals look like

<img src="https://github.com/RoboticMaterials/FA-I-sensor/blob/master/screenshot_serialplotter.png" width=500>

The blue signal is the raw proximity reading. After touch, it roughly corresponds to the human SA-I signal, that is slow adapting pressure. You would not notice anything if it slowly changes, but only if the value exceeds a certain threshold. The yellow signal roughly corresponds to the human FA-I signal, that is fast adapting pressure. It helps you notice changes such as the lightest contact, but keeps quiet for constant pressure. Thinking about wearing clothes is a good analogy: your FA-I sensors alert you when you wear them and your SA-I sensors ignore them, but notice of you larger changes.
