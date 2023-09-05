# Mardan robot

## Base container build

```bash
docker build --file src/ros/mardan/docker/mardan_base_noetic.dockerfile --tag "mardan_base_noetic:local" .
```

## Development using base container

```bash
docker run -it --rm --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix mardan_base_noetic:local /bin/bash
```

more advanced command mounting the display and camera:

```bash
xhost +

docker run -it --rm -e DISPLAY=$DISPLAY \
    --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --device=/dev/video0 \
    mardan_base_noetic:local \
    /bin/bash
```

Inside the container, build the catkin workspace as:

```bash
cd /root/catkin_ws
catkin_make

source devel/setup.bash
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


## Running camera publisher

Start the local development container with access to a camera device and the computer's display:

```bash
xhost +

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://192.168.68.53:11311 \
    -e ROS_IP=192.168.68.69 \
    --add-host host.docker.internal:host-gateway \
    --network="host" \
    --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --device=/dev/video0 \
    -p 8080:8080 \
    mardan_base_noetic:local \
    /bin/bash

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --device=/dev/video0 \
    -p 8080:8080 \
    mardan_base_noetic:local \
    /bin/bash
```

inside the container, run `tmux` to create several terminal sessions. In one of them, build the catkin workspace:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash

# run the camera launcher
roslaunch mardan camera.launch
```

In another tmux terminal (use `CTRL + B + %`), verify the new camera topic is avaiable via `rostopic list`

```bash
rostopic list
```

then, run the `image_view` executable to visualize the camera feed

```bash
rosrun image_view image_view image:=/camera/BGR/raw
```


```bash
rostopic pub -1 /motors/motor_twist geometry_msgs/Twist -- '[0.5, 0.0, 0.0]' '[0.0, 0.0, 0.5]'
```