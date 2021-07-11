# SSL Simulator Using Gazebo-ROS
## Temporary README FILE 
### (9 July 2021 Update)

<p>&nbsp;</p>

### 16020322 - Nanda Pramudia Santosa
### 16520034 - Patrick Amadeus Irawan
### 16520125 - Grace Claudia
### 16520373 - Farrel Ahmad

<p>&nbsp;</p>

## Launching World
**Update 12 July 2021 New Branch "testing"**

To launch world and spawn SSL Field and Robots, run this command :
1. Clone the repository
```
$ git clone https://gitlab.com/dagozilla/academy/2021-internship2/group-1/ssl-simulator.git
```

2. cd to `/ssl-simulator`, change branch to `testing`, then cd to `/ssl_ws`
```
$ cd ssl-simulator
$ git checkout testing
$ cd ssl_ws
```

3. run `catkin_make`
```
$ catkin_make
```

4. Source `setup.bash`
```
$ source devel/setup.bash
```

5. Launch the world
```
$ roslaunch sslbot_gazebo sslbot.launch
```

6. The result will look like this

![](https://i.ibb.co/9NCp2HV/Screenshot-from-2021-07-04-15-38-05.png)

7. **(4 July 2021 Update-3)** : Now the SSL ball is spawned at the center of the field 

![](https://i.ibb.co/jgPX5H7/Screenshot-from-2021-07-04-21-18-57.png)

8. **(5 July 2021 Update)** : Now by using the newly created node called `ball_state_pub` which subscribes to a topic `/gazebo/model_states`, the ball's state (position and twist) is published to a topic `ball_state` 

![](https://i.ibb.co/5r1SWfG/Screenshot-from-2021-07-05-23-23-51.png)

rostopic list

<p>&nbsp;</p>

![](https://i.ibb.co/BrQ9yVh/Screenshot-from-2021-07-05-23-22-20.png)

ball_state topic in rostopic echo  

9. **(10 July 2021 Update-1)** : Now the playing field has a barrier so the robot and the ball can't go outside of the playing field. Delete the visual tag of barrier link in ssl_field model to get invisible barrier instead of a colored one.  

![](https://i.ibb.co/pwq540G/Screenshot-from-2021-07-10-00-24-43.png)


10. **(10 July 2021 Update-2)** : changed the ball's color to orange to meet the standard of SSL Ball

![](https://i.ibb.co/HpddYNW/Screenshot-from-2021-07-10-06-51-53.png)

<p>&nbsp;</p>

## Robot Basic Movement

1. **(9 July 2021 Update-1, EXPERIMENTAL)** : turtlebot_1 will move to the ball's location . Just run this command after the program starts
```
rosrun sslbot_gazebo turtlebot_1_pub
```

![](https://i.ibb.co/LgtGtJz/Screenshot-from-2021-07-09-18-15-38.png)

First, turtlebot_1 will rotate to the ball's direction

![](https://i.ibb.co/744P08V/Screenshot-from-2021-07-09-18-15-42.png)

Second, turtlebot_1 will move forward to the ball

<p>&nbsp;</p>

2. **(9 July 2021 Update-2)** : now using turtlebot3 urdf model to control it using ros control. First, get the turtlebot3 package (we're going to put the urdf in our workspace later on) by using this command
```
$ sudo apt-get install ros-noetic-dynamixel-sdk
$ sudo apt-get install ros-noetic-turtlebot3-msgs
$ sudo apt-get install ros-noetic-turtlebot3
```

roslaunch the usual launch file
```
$ roslaunch sslbot_gazebo sslbot.launch
```

run this program in another terminal (don't forget to source your setup.bash)
```
$ rosrun sslbot_gazebo moveto.py
```

![](https://i.ibb.co/0MkdMr7/robot-mutar.png)
at first, the turtlebot will correct its orientation to face the destination

<p>&nbsp;</p>

![](https://i.ibb.co/s9DxJYv/robot-jalan.png)
then, the turtlebot will head to its destination. Note that the program need to have angle tolerance in order for the program to work smoothly (the robot have inertia and acceleration).  

further improvement in tuning/calibration of the robot itself and let the moveto node to subscribe to a topic for its destination argument

this method is good for future update and improvement because the robot itself act as a node and receiving message from command topic like '/cmd_vel' and '/odom' which is useful for navigation (commonly used).  

![](https://i.ibb.co/hV4FDjJ/Screenshot-from-2021-07-09-21-15-36.png)

<p>&nbsp;</p>

3. **(9 July 2021 Update-3)** : Now the turtlebot3 number 1 will follow the ball wherever it is. Even if we change the ball's position manually

![](https://i.ibb.co/0n9djW2/Screenshot-from-2021-07-09-22-37-51.png)
