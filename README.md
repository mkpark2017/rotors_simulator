RotorS
===============

RotorS is a MAV gazebo simulator.
It provides some multirotor models such as the [AscTec Hummingbird](http://www.asctec.de/en/uav-uas-drone-products/asctec-hummingbird/), the [AscTec Pelican](http://www.asctec.de/en/uav-uas-drone-products/asctec-pelican/), or the [AscTec Firefly](http://www.asctec.de/en/uav-uas-drone-products/asctec-firefly/), but the simulator is not limited for the use with these multicopters.

There are simulated sensors coming with the simulator such as an IMU, a generic odometry sensor, and the [VI-Sensor](http://wiki.ros.org/vi_sensor), which can be mounted on the multirotor.

This package also contains some example controllers, basic worlds, a joystick interface, and example launch files.


Installation Instructions - Ubuntu 16.04 with ROS Kinetic
---------------------------------------------------------
 1. Download from github and install by using rotors_velodyne.rosinstall 
 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 $ wget https://raw.githubusercontent.com/mkpark2017/rotors_simulator/tree/kinetic/3D_LiDAR/rotors_velodyne.rosinstall
 $ wstool merge rotors_velodyne.rosinstall
 $ wstool update
 ```
 
 2. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 3. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```


Basic Usage
-----------

Launch the simulator with a hex-rotor helicopter model, in our case, the AscTec Firefly in a basic world.

```
$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
```

Launch the simulator with a hex-rotor helicopter model with velodyne lidar.

```
$ roslaunch rotors_gazebo mav_hovering_example_velodyne.launch mav_name:=firefly world_name:=basic
```

> **Note** The first run of gazebo might take considerably long, as it will download some models from an online database.

The simulator starts by default in paused mode. To start it you can either
 - use the Gazebo GUI and press the play button
 - or you can send the following service call.

   ```
   $ rosservice call gazebo/unpause_physics
   ```

There are some basic launch files where you can load the different multicopters with additional sensors. They can all be found in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/launch`.

The `world_name` argument looks for a .world file with a corresponding name in `~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds`. By default, all launch files, with the exception of those that have the world name explicitly included in the file name, use the empty world described in `basic.world`.


#### Usage with a joystick

Connect a USB joystick to your computer and launch the simulation alongside ROS joystick driver and the RotorS joystick node:
```
$ roslaunch rotors_gazebo mav_with_joy.launch mav_name:=firefly world_name:=basic
```

Depending on the type of joystick and the personal preference for operation, you can assign the axis number using the `axis_<roll/pitch/thrust>_` parameter and the axis direction using the `axis_direction_<roll/pitch/thrust>` parameter.

#### Usage with a keyboard

First, perform a one-time setup of virtual keyboard joystick as described here: https://github.com/ethz-asl/rotors_simulator/wiki/Setup-virtual-keyboard-joystick.

Launch the simulation with the keyboard interface using the following launch file:
```
$ roslaunch rotors_gazebo mav_with_keyboard.launch mav_name:=firefly world_name:=basic
```

If everything was setup correctly, an additional GUI should appear with bars indicating the current throttle, roll, pitch, and yaw inputs. While this window is active, the Arrows and W, A, S, D keys will generate virtual joystick inputs, which can then be processed by the RotorS joystick node in the same way as real joystick commands.

Gazebo Version
--------------

At a minimum, Gazebo `v2.x` is required (which is installed by default with ROS Indigo). However, it is **recommended to install at least Gazebo `v5.x`** for full functionlity, as there are the following limitations:

1. `iris.sdf` can only be generated with Gazebo >= `v3.0`, as it requires use of the `gz sdf ...` tool. If this requirement is not met, you will not be able to use the Iris MAV in any of the simulations.
2. The Gazebo plugins `GazeboGeotaggedImagesPlugin`, `LidarPlugin` and the `LiftDragPlugin` all require Gazebo >= `v5.0`, and will not be built if this requirement is not met.
