# Mardan robot

## Base container build

```bash
docker build --file src/ros/mardan/docker/mardan_base_noetic.dockerfile --tag "mardan_base_noetic:local" .
```

## Development using base container

```bash
docker run -it --rm --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ mardan_base_noetic:local /bin/bash
```

## Runtime container

```bash
docker build --file src/ros/mardan/docker/mardan_noetic.dockerfile --tag "mardan_noetic:local" .
```

## Build

```bash
cd /root/catkin_ws
catkin_make
```


## Running teleop

ROS already provides a basic keyboard teleop utility that can be used to control Mardan:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=motors/motor_twist
```
