# SSL Simulator Using Gazebo-ROS
## Temporary README FILE

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
$ roslaunch sslbot_gazebo sslbot_world
```

6. The result will look like this

![](https://i.ibb.co/9NCp2HV/Screenshot-from-2021-07-04-15-38-05.png)