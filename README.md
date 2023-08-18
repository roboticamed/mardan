# Mardan robot

## Runtime container build

```bash
docker build --file src/ros/runtime/mardan_runtime.dockerfile --tag "mardan_runtime:local" .
```

## Development using runtime container

```bash
docker run -it --rm --mount type=bind,source="$(pwd)"/src/ros/catkin_ws,target=/root/catkin_ws mardan_runtime:local /bin/bash
```

## Build

```bash
cd /root/catkin_ws
catkin_make
```