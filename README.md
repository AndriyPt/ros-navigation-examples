# ros-navigation-examples

Some examples of ROS based navigation

Installation instructions

```bash
bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh) -o AndriyPt -r ros-navigation-examples -b kinetic-devel -v kinetic

```

## Building the map

Start simulation

```bash
roslaunch moon_launch simulation.launch
```

or real robot

```bash
roslaunch moon_launch real_robot.launch
```

Launch gmapping node

```bash
rosrun gmapping slam_gmapping scan:=/laser/scan
```

Launch teleop node and drive robot around
```bash
roslaunch moon_launch teleop.launch
```

Save map to file
```bash
rosrun map_server map_saver [-f mapname]
```