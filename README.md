# ros-navigation-examples

Some examples of ROS based navigation

## Docker
For convinience it is recommended to use Docker containers. 
Please follow these steps to run Docker container on your machine.
 
 1. Install Desktop OS Ubuntu Trusty or Xenial on your machine or in virtual machine
 2. Install Docker-CE using these [instructions](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
 3. For development [the following](https://hub.docker.com/r/shadowrobot/build-tools/) docker container was used.
 4. To pull it please run
```bash
docker pull shadowrobot/build-tools:xenial-kinetic-ide
```
 5. If you want to run everything from docker container use the following command
```bash
docker run -it --name navigation_demo -e DISPLAY  -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/build-tools:xenial-kinetic-ide
```

Back window of [Terminator](https://gnometerminator.blogspot.com/p/introduction.html) UI console will apear after some time.
You can use it's features to split terminal window into smaller terminals and run few commands in parallel. 

In order to relaunch docker container after you closed Terminator window or rebooted machine please run
```bash
docker start navigation_demo
```
After some time Terminator window will reappear.

## Setup

Download Gazebo models for first time

```bash
bash <( curl -Ls https://raw.githubusercontent.com/shadow-robot/sr-build-tools/F_add_gazebo_models_loading/docker/utils/load_gazebo_models.sh)
```

Install ROS and compile source code (it will ask you for sudo password at some point, just press Enter)

```bash
bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh) -o AndriyPt -r ros-navigation-examples -b kinetic-devel -v kinetic

```

Source workspace which is located here `~/workspace/ros-navigation-examples/base/`
```bash
source ~/workspace/ros-navigation-examples/base/devel/setup.bash
```

## Navigating on known map

Start simulation

```bash
roslaunch moon_launch simulation.launch
```

or real robot

```bash
roslaunch moon_launch real_robot.launch
```

Launch navigation stack
```bash
roslaunch moon_launch navigation.launch
```

In RViz which appear after some time select "2D Nav Goal" and robot will travel to it.
Like it is shown in [this video](https://www.youtube.com/watch?v=xSdHlC2ISq8).

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
roslaunch moon_launch gmapping.launch
```

Launch teleop node and drive robot around
```bash
roslaunch moon_launch teleop.launch
```

Drive arround environment to build map.

Save map to file
```bash
rosrun map_server map_saver -f <map_file_name>
```
