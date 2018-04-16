## Term 3: Capstone Project : System Integrations
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we develop a system which integrates multiple components to
drive Carla, the Udacity self-driving car, around a test track.

## Project Team Members of Drive Safely
|  Name                                   | Udacity Account Email Address     |
|:---------------------------------------:|:---------------------------------:|
| Christoph Albrecht                      |   christoph.albrecht@gmail.com    |
| Alexander Lechner                       |   alexanderlechner7@gmail.com     |
| Ian Burris                              |   ian.burris@gmail.com            |
| Chongyou Ma                             |   chongyou.ma@gmail.com           |
| Clifton Pereira                         |   pereiraclifton@gmail.com        |


[//]: # (Image References)

[image1]: ./imgs/final-project-ros-graph-v2.png "Carla's System Architecture"

## System Architecture
Carla's system can be broken down into three main parts:
- The perception subsystem detects traffic lights and obstacles.
- The planning subsystem (node waypoint updater) updates the waypoints and the
associated target velocities.
- The control subsystem actuates the throttle, steering, and brake to navigate
the waypoints with the target velocity.

![alt text][image1]

### Traffic Detection (aka. tl_detector node)
The traffic light detector node which operates at 10Hz has 4 main tasks:
1. Find the closest waypoint to the position of the car
2. Find the closest traffic light to the car's position 
3. Find the state of the traffic light
4. If the closest light is red, publish its waypoint to the /traffic_waypoint topic

From the topic /base_waypoints, on start-up the waypoints are stored into a KD-Tree. This is used to find the closest waypoint to either the car's position or a traffic light in question. Once the waypoints for both the car's position and the traffic lights are known, it is only a matter of iterating over the waypoints to find the closest traffic light waypoint. 

After we find the closest traffic light, we can quantify the lights state using a classifier. See the following link for details on the traffic light classifier and training. https://github.com/alex-lechner/Traffic-Light-Classification

If the closest (in front) traffic light is classified as "RED" three consecutive times it is published to the topic /traffic_waypoint.

#### Waypoint Updater (a.k.a waypoint_updater node)

The waypoint updater determines the nearest waypoints ahead of the vehicle and
the desired speed to be traveling at each waypoint.  At startup a list of known
waypoints is received and stored in a KD-Tree for efficient lookup of the
nearest waypoint to the vehicle.  Every 0.1 seconds the following tasks are
performed.

1.  Find the nearest `n` waypoints ahead of the vehicle where `n` is defined as
a set number of waypoints.

2.  Determine if a red traffic light falls in the range of waypoints ahead of
the traffic light.

3.  Calculate target velocities for each waypoint.

4.  Publish the target waypoints with velocities to the `final_waypoints`
topic.

If a red traffic light does not fall in the look ahead range the velocities
provided by the waypoint loader are used.  When a red traffic light is present,
velocities are calculated to perform a linear deceleration such that the
vehicle will stop at the waypoint closest to the traffic light.

After we completed our initial implementation, we made two changes to improve
the deceleration behavior:

1.  Initially, we calculated the deceleration starting from the vehicle's
current position as soon as a red light was detected.  However, since we
consider 100 waypoints the vehicle was often far away from the light when it
started to decelerate.  We changed to the code to start decelerating only when
the car was less than 15 meters away from the red light.

2. The vehicle would stop short of the target "stopping" waypoint.  This
happened because the waypoint velocities were recalculated every 0.1 seconds
based on the current velocity of the vehicle.  Often the throttle controller
would decelerate a little faster than it should, causing the waypoint updater
to set slower target velocities each time it performed the velocity
calculations causing a positive feedback loop.  To solve this problem we
introduced a state machine.  When the state transitioned from "driving" to
"breaking" a deceleration rate is calculated and used until a transition back
to the "driving" state occurres.

### Initial README

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

