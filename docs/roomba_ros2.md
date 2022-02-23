# ROS2 Packages for Roomba RPi #

## Roomba RPi ROS2 Packages ##

| Package Name | Description |
| ------------ | ----------- |
| `roomba_ros2` | Roomba RPi ROS2 metapackage |
| `roomba_bringup` | Roomba RPi ROS2 launch files |


## Additional Required ROS Packages ##

| Package Name | Description | URL |
| ------------ | ----------- | --- |
| `ros2_bno055` | ROS2 node for the Bosch BNO055 IMU | [github.com/process1183/ros2_bno055](https://github.com/process1183/ros2_bno055) |
| `create_driver` | ROS driver for iRobot Create 1 and 2. | [github.com/process1183/create_robot/tree/mode_report_workaround](https://github.com/process1183/create_robot/tree/mode_report_workaround) |
| `libcreate` | C++ library for interfacing with iRobot's Create 1 and 2 as well as most models of Roomba. (Dependency of `create_driver`) | [github.com/process1183/libcreate/tree/mode_report_workaround](https://github.com/process1183/libcreate/tree/mode_report_workaround) |
| `diagnostics` | Packages related to gathering, viewing, and analyzing diagnostics data from robots. (Dependency of `create_driver`) | [github.com/ros/diagnostics](https://github.com/ros/diagnostics) |


## Build and Install Steps ##

Cross-compiling ROS2 packages for the RPi ZW involves performing some initial setup and dependency steps on the Pi first, then moving to the cross-compilation environment detailed in `roomba-rpi/docs/ros2_rpizw_build.md`.

__1.__ __(RPi ZW)__ Log into the RPi ZW and set up the ROS2 workspace:

```
sudo mkdir -p /opt/rws/src

sudo chown -R ${USER}:${USER} /opt/rws/
```

__2.__ __(RPi ZW)__ Clone the needed package sources:

```
git clone https://github.com/process1183/roomba-rpi.git ~/roomba-rpi
cp -r ~/roomba-rpi/software/roomba_ros2 /opt/rws/src/

cd /opt/rws/src

git clone https://github.com/process1183/ros2_bno055.git
git clone https://github.com/process1183/create_robot.git --branch mode_report_workaround
git clone https://github.com/process1183/libcreate.git --branch mode_report_workaround
git clone https://github.com/ros/diagnostics.git --branch foxy
```

__3.__ __(RPi ZW)__ Disable building unnecessary packages:

```
touch \
    create_robot/create_bringup/COLCON_IGNORE \
    create_robot/create_description/COLCON_IGNORE \
    create_robot/create_robot/COLCON_IGNORE \
    diagnostics/diagnostic_aggregator/COLCON_IGNORE \
    diagnostics/diagnostic_analysis/COLCON_IGNORE \
    diagnostics/diagnostic_common_diagnostics/COLCON_IGNORE \
    diagnostics/rosdiagnostic/COLCON_IGNORE \
    diagnostics/self_test/COLCON_IGNORE \
    diagnostics/test_diagnostic_aggregator/COLCON_IGNORE
```

__4.__ __(RPi ZW)__ Source the ROS2 setup file:

```
source /opt/ros2_foxy/install/local_setup.bash
```

__5.__ __(RPi ZW)__ Install ROS package dependencies:

```
cd /opt/rws

rosdep install --from-paths src -i
```

*Note*: `rosdep` installs `libgtest-dev`, which caused a linker error during build. It is optional and only needed for `libcreate` unit tests. After the `rosdep` commands finishes, remove it by running `sudo apt purge googletest libgtest-dev`


__6.__ __(Deb10 Build Env)__ Sync the RPi ZW rootfs to the cross-compiler system:

```
rsync -rLR --safe-links <RPI_IP>:/{etc,lib,opt/rws,usr} /opt/rootfs/
```

__7.__ __(Deb10 Build Env)__ Copy the RPi's workspace dir to the same location on the cross-compiler system:

```
mkdir -p /opt/rws/src/

cp -r /opt/rootfs/opt/rws/src /opt/rws/
```

__8.__ __(Deb10 Build Env)__ Add `/opt/rws/install` to the `CMAKE_FIND_ROOT_PATH` var (paths are separated by semicolons) in `/opt/toolchain.cmake`:

(The `CMAKE_FIND_ROOT_PATH` line should now look like the following)

```
set(CMAKE_FIND_ROOT_PATH /opt/ros2_foxy/install;/opt/rws/install)
```

__9.__ __(Deb10 Build Env)__ Source the ROS2 setup file:

```
source /opt/ros2_foxy/install/local_setup.bash
```

__10.__ __(Deb10 Build Env)__ Export the cross-compilation environment variables:

```
export C_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export CPLUS_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
```

__11.__ __(Deb10 Build Env)__ Build the ROS2 packages:

```
cd /opt/rws/

colcon build --cmake-args -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake
```

__12.__ __(Deb10 Build Env)__ Copy the workspace install dir to the RPi:

```
rsync -av --delete /opt/rws/install/ <RPI_IP>:/opt/rws/install/
```

__13.__ __(RPi ZW)__ On the RPi, source the workspace and run the ROS2 package(s) (e.g. run the BNO055 node):

```
cd /opt/rws/

source /opt/rws/install/setup.bash

ros2 run ros2_bno055 bno055
```
