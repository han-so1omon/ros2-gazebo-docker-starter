# ROS2-Gazebo Starter
Demonstrate ROS2-Gazebo integration and message transport using modern,
containerized development/runtime environments.

This starter does the following:
- Builds a quadrotor model and plugin for Gazebosim
- Builds a Gazebosim/ROS2 transport node

Build and modify from this as you see fit.

[Blog](https://errcsool.com/blog/66)

# Setup
There are a few changes that must be made to configure this for your system

 - `run/dev/run-dev.sh` => (1) Change first volume to point to your directory. (2) Point
 to your dockerfile if not using errcsool/refrain-dev
 - `run/test-*.sh` => (1) Change first volume to point to your directory.

# Usage
## Development
Develop from a text editor in your base operating system. Compile from the
development container. Files will be synced and given the correct runtime
permissions if the development container is mounted as the Gazebo runtime user.
If the development container is not mounted as the Gazebo runtime user, then you may
have problems finding Gazebo plugin libraries.

### From baseline operating environment
```bash
cd run/dev
run-dev.sh
```

### From within development container
Gazebo plugin development
```bash
cd gazebo/.gazebo/<your_plugin>
mkdir -p build && cd build
cmake ..
make
```

ROS2 development
```bash
colcon build
```

## Run
Setup _myquadnet_ subnet
```bash
cd run/test-my_quadrotor
setup-network.sh
```

Run Gazebo server and GUI
```bash
cd run/test-my_quadrotor
./run-my_quadrotor.sh
```

Run Gazebo/ROS2 transport
```bash
cd run/test-my_quadrotor
./run-my_transport.sh
```
