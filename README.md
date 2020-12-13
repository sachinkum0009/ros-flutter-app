# my_app

Flutter Base Project.



## Task List

- [x] Splash Screen
- [ ] BLoC Architecture
- [x] JoyStick
- [x] ROS send topic messages
- [ ] View Map
- [ ] Visualize URDF
- [ ] Editable IP Address and topic names
- [ ] add files for the web camera and navigation map

## ROS launch files

```bash
roscore

export TURTLEBOT3_MODEL=waffle

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

roslaunch rosbridge_server rosbridge_websocket.launch

rosrun web_video_server web_video_server
```

## Instruction for map navigation

```bash
roscore

export TURTLEBOT3_MODEL=waffle

roslaunch turtlebot3_gazebo turtlebot3_world.launch

roslaunch turtlebot3_navigation turtlebot3_navigation.launch

roslaunch rosbridge_server rosbridge_websocket.launch

python3 -m http.server
```