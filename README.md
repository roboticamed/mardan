# Mardan robot

## Base container build

```bash
docker build --file src/ros/mardan/docker/mardan_base_noetic.dockerfile --tag "mardan_base_noetic:local" .
```

## Development using base container

```bash
docker run -it --rm --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ mardan_base_noetic:local /bin/bash
```

Inside the container, build the catkin workspace as:

```bash
cd /root/catkin_ws
catkin_make
```

then, configure the terminal session

```bash
source devel/setup.bash
```

## Runtime container

```bash
docker build \
--build-arg="BASE_IMAGE=mardan_base_noetic:local" \
--file src/ros/mardan/docker/mardan_noetic.dockerfile \
--tag "mardan_noetic:local" .
```

To run it:

```bash
docker run -it --rm mardan_noetic:local /bin/bash
```

## Running teleop

ROS already provides a basic keyboard teleop utility that can be used to control Mardan:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=motors/motor_twist
```
