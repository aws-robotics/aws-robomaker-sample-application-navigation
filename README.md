# AWS RoboMaker Sample Application - Navigation

This sample application demonstrates how to setup navigation in a Gazebo world, and how to subscribe and publish metrics to monitor your robots, including
- linear speed
- angular speed
- distance to nearest obstacle (closest lidar scan return)
- distance to planned goal (bookstore only, requires its navigation system)

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://github.com/aws-robotics/aws-robomaker-sample-application-navigation)._


## Installation

The following will be installed

- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application.
- [vcstool](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool) - Used to pull in sample app dependencies that are only available from source, not from apt or pip.
- [rosdep](http://wiki.ros.org/rosdep#Installing_rosdep) - rosdep is a command-line tool for installing system dependencies of ROS packages.
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - Other versions may work, however they have not been tested. If ROS is detected, then ROS Installation will be skipped.

## Build

### Install requirements
Follow links above for instructions on installing required software.

- *Full Setup Including ROS Install*
currently only melodic is supported, ROS Installation will be skipped if its already present.

```bash
# `sudo` is not needed inside a docker environment
sudo ./scripts/setup.sh
```

### Robot

```bash
cd robot_ws
colcon build
```

### Simulation

```bash
cd simulation_ws
colcon build
```

## Run

The `TURTLEBOT3_MODEL` environment variable must be set when running both robot and simulation application. Currently only `waffle_pi` is supported. Set it by

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

Launch the application with the following commands:

- *Running Robot Application on a Robot*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch navigation_robot deploy_rotate.launch
    ```

- *Running Robot Application in a Simulation*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch navigation_robot [command]
    ```
    There are two robot launch commands:
    - `rotate.launch` - The robot starts rotating
    - `await_commands.launch` - The robot is idle waiting movement commands, use this for teleop and navigation


- *Running Simulation Application*
    ```bash
    source simulation_ws/install/local_setup.sh
    roslaunch navigation_simulation [command]
    ```
    There are three simulation launch commands for three different worlds:
    - `empty_world.launch` - Empty world with some balls surrounding the turtlebot at (0,0)
    - `bookstore_turtlebot_navigation.launch` - A retail space where the robot navigates to random goals
    - `small_house_turtlebot_navigation.launch` - A retail space where the robot navigates to random goals

    Alternatively, to run turtlebot navigation to follow dynamic goals,
    ```bash
    roslaunch navigation_simulation [command] follow_route:=false dynamic_route:=true
    ``` 

Note that when running robot applications on a robot, `use_sim_time` should be set to `false` (which is the default value in `deploy_rotate.launch` and `deploy_await_commands.launch`). When running robot applications along with simulation applications, `use_sim_time` should be set to `true` for both applications (which is the default value in `rotate.launch`, `await_commands.launch` and all the launch files in simulation workspace).
   		  
When running simulation applications, run command with `gui:=true` to run gazebo client for visualization.

For navigation, you can generate a map with map generation plugin. See [this](#generate-occupancy-map-via-map-generation-plugin) for instructions.

![CloudWatchMetrics01.png](docs/images/BookstoreRVizPlan01.png)

### Run with a AWS Robomaker WorldForge world

Pre-requisite: Generate a map for your worldforge exported world following these [instructions](#generate-map-for-a-worldforge-world-with-default-config).

Build your workspace to reference the newly generated maps,
```bash
cd simulation_ws
colcon build
```

Launch the navigation application with the following commands:
```bash
export TURTLEBOT3_MODEL=waffle_pi
source simulation_ws/install/local_setup.sh
roslaunch navigation_simulation worldforge_turtlebot_navigation.launch
```

## Using this sample with RoboMaker

You first need to install [Docker](https://docs.docker.com/get-docker/) and [VCS Import Tool](http://wiki.ros.org/vcstool) (if you use VCS Import Tool). Python 3.5 or above is required.

```bash
pip3 install vcstool
```

After Docker and VCS Import Tool is installed you need to build your robot or simulation docker images:

```bash
# Import dependencies defined in .rosinstall to each source directory using vcs import
vcs import robot_ws < robot_ws/.rosinstall
vcs import simulation_ws < simulation_ws/.rosinstall

# Building Robot Application Docker Image
DOCKER_BUILDKIT=1 docker build . \
--build-arg ROS_DISTRO=melodic \
--build-arg LOCAL_WS_DIR=./robot_ws \
--build-arg APP_NAME=navigation-robot-app \
-t robomaker-navigation-robot-app

# Building Simulation Application Docker Image
DOCKER_BUILDKIT=1 docker build . \
--build-arg GAZEBO_VERSION=gazebo-9 \
--build-arg ROS_DISTRO=melodic \
--build-arg LOCAL_WS_DIR=./simulation_ws \
--build-arg APP_NAME=navigation-sim-app \
-t robomaker-navigation-sim-app
```

This produces the Docker Images `robomaker-navigation-robot-app` and `robomaker-navigation-sim-app` respectively which you can view by running:

```bash
# Listing your Docker Images
docker images
```

You'll need to upload these to Amazon ECR, then you can use these files to
[create a robot application](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot-application.html),
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in RoboMaker. Visit the [preparing-ros-application-and-simulation-containers-for-aws-robomaker](https://aws.amazon.com/blogs/robotics/preparing-ros-application-and-simulation-containers-for-aws-robomaker/#:~:text=Bash-,Publish%20docker%20images%20to%20Amazon%20ECR,-Containers%20used%20by) blog post to find the steps to upload these docker images to Amazon ECR.


## Generate Occupancy Map via map generation plugin

Procedurally generate an occupancy map for any gazebo world. This map can then be plugged in to navigate a robot in Worldforge worlds. For other aws-robotics worlds, this procedure is optional for the use-cases mentioned in this README. 


### Generate map for a aws-robotics world with default config

Currently, the following aws-robotics worlds are supported,
- [`bookstore`](https://github.com/aws-robotics/aws-robomaker-bookstore-world)  
- [`small_house`](https://github.com/aws-robotics/aws-robomaker-small-house-world)  
- [`small_warehouse`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)  
- [`no_roof_small_warehouse`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)


To generate a map, simply run
```bash
./scripts/genmap_script.sh <world_name>
```

where `<world_name>` can be any value in the list above.

### Generate map for a WorldForge world with default config

After exporting a world from WorldForge, unzip and move the contents under simulation_ws workspace

```bash
unzip exported_world.zip
mv aws_robomaker_worldforge_pkgs simulation_ws/src/

#For worldforge worlds, set WORLD_ID to the name of your WF exported world (eg: generation_40r67s111n9x_world_3),
export WORLD_ID=<worldforge-world-name>

# Run map generation script
./scripts/genmap_script.sh worldforge
```

### Generate map for a custom world with custom config

```bash
# Install dependencies (Ubuntu >18.04)
sudo apt-get install ruby-dev libxml-xpath-perl libxml2-utils
```

```bash
# Fetch and install ROS dependencies
cd simulation_ws
vcs import < .rosinstall
rosdep install --from-paths src -r -y
cd ..
```

```bash
# Run plugin with custom world/config,
python scripts/add_map_plugin.py custom -c <path-to-config> -w <path-to-world> -o <output-path>
```

```bash
# Build with plugin added
cd simulation_ws
colcon build
source install/local_setup.sh

# Start map service (for custom worlds, relocate your world file with the added plugin to src/navigation_simulation/worlds/map_plugin.world before running this)
roslaunch navigation_simulation start_map_service.launch

# Generate map (start in a different terminal AFTER you see "[INFO] [*] occupancy map plugin started" message in previous terminal)
rosservice call /gazebo_2Dmap_plugin/generate_map

# Save map
rosrun map_server map_saver -f <path-to-file> /map:=/map2d
```

```bash
# Move the generated map file to navigation_simulation simulation workspace map directory
mv <path-to-file> simulation_ws/src/navigation_simulation/maps/map.yaml
```

## Security

See [CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications) for more information.

## License

This library is licensed under the MIT-0 License. See the LICENSE file.
