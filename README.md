# Mardan robot

## Base container build

```bash
docker build --file src/ros/mardan/docker/mardan_base_noetic.dockerfile --tag "mardan_base_noetic:local" .
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

## Development using base container (Host linux)

### Local development

```bash
docker run -it --rm \
    --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ \
    --device=/dev/video0 \
    --network="host" \
    -p 8080:8080 \
    mardan_base_noetic:local \
    /usr/bin/tmux
```

### Connecting to another machine as a ROS worker

Optionally, include extra environment variables to the command above to connect the container above to another machine acting as ROS master:

```
xhost +

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://10.0.0.5:11311 \
    -e ROS_IP=10.0.0.11 \
    --add-host host.docker.internal:host-gateway \
    --network="host" \
    -p 8080:8080 \
    --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix \
    --device=/dev/video0 \
    mardan_base_noetic:local \
    /usr/bin/tmux
```

where `ROS_MASTER_URI` is the URI of the ROS master machine and `ROS_IP` is the IP address of this machine. Notice that in this case, the container runs with the docker `host` network.


### Running commands inside the container

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


## Running teleop

ROS already provides a basic keyboard teleop utility that can be used to control Mardan:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=motors/motor_twist
```

## Camera node

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

### WebRTC client

In a web browser, open http://localhost:8080 and start the WebRTC stream from the camera.
