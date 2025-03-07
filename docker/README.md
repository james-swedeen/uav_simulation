# Overview
The files in this directory will help to build and run a docker image.

# Building the docker image
## Obtaining the code
This repository is dependent on three other repositories found here: https://github.com/james-swedeen/advanced_path_planning, https://github.com/james-swedeen/matplotlibcpp, and https://github.com/james-swedeen/kalman_filter.

## Building the docker image
From the `src` folder, run the following command
```bash
docker build --no-cache -t uav_sim -f docker/Dockerfile .
```
Note that the `--no-cache` option ensures that the image will have all of the system updates. Removing it will make the build much faster, but may miss many updates due to the cache. Additionally, the `--pull` option can be used to pull the latest `osrf/ros:humble-desktop` image.

# Running the docker image
## Starting an X-server
### Ubuntu
You must first enable x-server. This must be done each time you log in. Open a terminal and run the following command.
```bash
xhost +
```

### Windows
You need to start an x-server. One example is to use `VcXsrv`. When starting, make sure to select `Disable access control` and add the additional parameter `-nowgl`

## Run the container
Inside the `docker` folder, run the command
```bash
docker-compose run --rm uav_sim
```

This will open a terminal to the base directory of the uav simulation where you can interact with the system as described in the `Scenario/Readme.md` file. For example, you could start the sim by running the command
```bash
ros2 launch pd_planner_launch scenario.launch.py
```
### Additional step for windows
Before launching the code, you need to setup the display.
* Find the IP of your windows host (`ipconfig`)
* In the container terminal, run
  ```bash
    export DISPLAY=192.168.3.41:0.0
  ```
  where you replace `192.168.3.41` with the host IP address

Note that the windows x-server solution does not appear to work well for RVIZ. The 3D rendering of the aircraft will occasionally

# Sharing the docker image
To share the docker image, simply export it as a tar file
```bash
docker save uav_sim > uav_sim.tar
```

To load a docker image from the tar file, use the `docker load` command.
```bash
docker load < uav_sim.tar
```

Note that the load command in windows uses a `-i` instead of a `<`
```bash
docker load -i uav_sim.tar
```
