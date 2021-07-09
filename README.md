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
To launch world and spawn SSL Field and Robots, run this command :
1. Clone the repository
```
$ git clone https://gitlab.com/dagozilla/academy/2021-internship2/group-1/ssl-simulator.git
```

2. cd to `/ssl_ws`
```
$ cd ssl-simulator/ssl_ws
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

7. (4 July 2021 Update-3) : Now the SSL ball is spawned at the center of the field 

![](https://i.ibb.co/jgPX5H7/Screenshot-from-2021-07-04-21-18-57.png)

8. (5 July 2021 Update) : Now by using the newly created node called `ball_state_pub` which subscribes to a topic `/gazebo/model_states`, the ball's state (position and twist) is published to a topic `ball_state` 

![](https://i.ibb.co/5r1SWfG/Screenshot-from-2021-07-05-23-23-51.png)

rostopic list

<p>&nbsp;</p>

![](https://i.ibb.co/BrQ9yVh/Screenshot-from-2021-07-05-23-22-20.png)

ball_state topic in rostopic echo

9. (9 July 2021 Update, EXPERIMENTAL) : turtlebot_1 will move to the ball's location . Just run this command after the program starts
```
rosrun sslbot_gazebo turtlebot_1_pub
```

![](https://i.ibb.co/LgtGtJz/Screenshot-from-2021-07-09-18-15-38.png)

First, turtlebot_1 will rotate to the ball's direction

![](https://i.ibb.co/744P08V/Screenshot-from-2021-07-09-18-15-42.png)

Second, turtlebot_1 will move forward to the ball