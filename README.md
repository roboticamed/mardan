# Mardan robot

## Runtime container build

```bash
docker build --file src/ros/mardan/docker/mardan_base_noetic.dockerfile --tag "mardan_base_noetic:local" .
```

## Development using runtime container

```bash
docker run -it --rm --mount type=bind,source="$(pwd)"/src/ros/,target=/root/catkin_ws/src/ mardan_base_noetic:local /bin/bash
```

## Build

```bash
cd /root/catkin_ws
catkin_make
```