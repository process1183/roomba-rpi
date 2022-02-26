# Building ROS2 Foxy Fitzroy for the RPi Zero W #

This document contains the steps needed to set up a cross-compilation environment
and build [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) for the
Raspberry Pi Zero W.

Please see
[my post on Hackaday.io](https://hackaday.io/project/178565-roomba-rpi/log/195831-building-ros2-for-the-rpi-zw)
for the reasoning behind these instructions.

To build ROS2 for the RPi ZW, you will need a Debian 10 build environment
(I recommend using a schroot, container, or virtual machine), and a Raspberry 
Pi Zero W with the Raspberry Pi OS (Legacy, Debian 10 Buster) installed.


## Building the Cross-Compilation Toolchain ##

Perform these steps in the Debian 10 build environment (schroot/container/vm), 
not on the RPi.

_Note: This section is based on
[Paul Silisteanu's "Building GCC as a cross compiler for Raspberry Pi"](https://solarianprogrammer.com/2018/05/06/building-gcc-cross-compiler-raspberry-pi/)_

__1.__ Install the required software:

```
sudo apt install build-essential git gawk texinfo bison file wget rsync
```

__2.__ Make `/opt` writable by your user in the build environment:

```
sudo chown ${USER}:${USER} /opt/
```

__3.__ Create the build directory and download the GCC sources:

__Note__: The downloaded source versions need to match what is installed on the
RPi. You can see what Raspbian's versions are by running \
`ld --version`, `ldd --version`, and `gcc --version` on your RPi. If the versions
on your RPi are different than what is listed below, please modify these steps 
accordingly.

```
mkdir /opt/gcc_build && cd /opt/gcc_build

wget https://mirrors.kernel.org/gnu/binutils/binutils-2.31.1.tar.gz \
     https://mirrors.kernel.org/gnu/gcc/gcc-8.3.0/gcc-8.3.0.tar.gz \
     https://mirrors.kernel.org/gnu/glibc/glibc-2.28.tar.gz

git clone --depth=1 https://github.com/raspberrypi/linux.git
```

__4.__ Unpack the GCC sources:

```
for tarball in ./*.tar.gz; do tar -xzf ${tarball} && rm ${tarball}; done
```

__5.__ Download GCC prerequisites:

```
cd /opt/gcc_build/gcc-8.3.0/

./contrib/download_prerequisites
```

__6.__ Create the GCC installation target directory:

```
mkdir /opt/gcc-armv6hf
```

__7.__ Add the cross-compilation `bin` directory to the PATH variable:

```
export PATH=/opt/gcc-armv6hf/bin:$PATH
```

Add the following to `~/.bashrc`:

```bash
if [ -d "/opt/gcc-armv6hf/bin" ] ; then
    PATH="/opt/gcc-armv6hf/bin:$PATH"
fi
```

__8.__ Install the RPi Linux headers into the GCC target directory:

```
cd /opt/gcc_build/linux

KERNEL=kernel7

make ARCH=arm INSTALL_HDR_PATH=/opt/gcc-armv6hf/arm-linux-gnueabihf headers_install
```

__9.__ Compile Binutils:

```
cd /opt/gcc_build

mkdir build-binutils && cd build-binutils

../binutils-2.31.1/configure \
    --prefix=/opt/gcc-armv6hf \
    --target=arm-linux-gnueabihf \
    --with-arch=armv6 \
    --with-fpu=vfp \
    --with-float=hard \
    --disable-multilib

make -j $(nproc)

make install
```

__10.__ GCC build, part 1 of 2:

```
cd /opt/gcc_build

mkdir build-gcc && cd build-gcc

../gcc-8.3.0/configure \
    --prefix=/opt/gcc-armv6hf \
    --target=arm-linux-gnueabihf \
    --enable-languages=c,c++,fortran \
    --with-arch=armv6 \
    --with-fpu=vfp \
    --with-float=hard \
    --disable-multilib

make -j $(nproc) all-gcc

make install-gcc
```

__11.__ Glibc build, part 1 of 2:

```
cd /opt/gcc_build

mkdir build-glibc && cd build-glibc

../glibc-2.28/configure \
    --prefix=/opt/gcc-armv6hf/arm-linux-gnueabihf \
    --build=$MACHTYPE \
    --host=arm-linux-gnueabihf \
    --target=arm-linux-gnueabihf \
    --with-arch=armv6 \
    --with-fpu=vfp \
    --with-float=hard \
    --with-headers=/opt/gcc-armv6hf/arm-linux-gnueabihf/include \
    --disable-multilib libc_cv_forced_unwind=yes

make install-bootstrap-headers=yes install-headers

make -j $(nproc) csu/subdir_lib

install csu/crt1.o csu/crti.o csu/crtn.o /opt/gcc-armv6hf/arm-linux-gnueabihf/lib

arm-linux-gnueabihf-gcc \
    -nostdlib \
    -nostartfiles \
    -shared \
    -x c /dev/null \
    -o /opt/gcc-armv6hf/arm-linux-gnueabihf/lib/libc.so

touch /opt/gcc-armv6hf/arm-linux-gnueabihf/include/gnu/stubs.h
```

__12.__ Build libgcc

```
cd ../build-gcc

make -j $(nproc) all-target-libgcc

make install-target-libgcc
```

__13.__ Glibc build, part 2 of 2:

```
cd ../build-glibc

make -j $(nproc)

make install
```

__14.__ GCC build, part 2 of 2:

```
cd ../build-gcc

make -j $(nproc)

make install

cd ../
```

__15.__ Cross-compile a test C program and run it on the RPi

```C
/* test.c */
#include <stdio.h>

int main()
{
    printf("Hello world!\n");
}
```

```
arm-linux-gnueabihf-gcc -Wall -Wextra -pedantic test.c -o test
```

Now copy the `test` binary to the RPi and run it. You should see no errors and
the `Hello world!` output.


## ROS2 Build Setup ##

Now that the cross-compilation toolchain is built, we need to set up the build
environment and Raspberry Pi Zero W for ROS2.

_Note: This section is based on
[Cyberbotics Ltd's "Cross-Compilation"](https://github.com/cyberbotics/epuck_ros2/tree/master/installation/cross_compile)_

__1.__ __(RPiZW)__ Install dependencies:

```
sudo apt install \
    bison \
    cmake \
    curl \
    libasio-dev \
    libbullet-dev \
    libcunit1-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    liblog4cxx-dev \
    libopencv-dev \
    libtinyxml2-dev \
    python3-dev \
    python3-netifaces \
    python3-numpy \
    python3-setuptools \
    python3-yaml
```

__2.__ __(Deb10 Build Env)__ Install dependencies:

```
sudo apt install wget tar python3-pip git cmake qemu-user-static python3-numpy rsync

pip3 install rosinstall_generator colcon-common-extensions vcstool lark-parser
```

__3.__ __(Deb10 Build Env)__ Add `~/.local/bin` to the PATH:

```
export PATH="$HOME/.local/bin/:$PATH"
```

Add to `~/.bashrc`:

```bash
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi
```

__4.__ __(Deb10 Build Env)__ Copy directories from the RPi that are needed to cross-compile ROS:

_Note_: Replace `<RPI_IP>` with your Raspberry Pi Zero W's IP address.

```
mkdir /opt/rootfs

rsync -rLR --safe-links <RPI_IP>:/{etc,lib,usr} /opt/rootfs/
```

__5.__ __(Deb10 Build Env)__ Create `/opt/toolchain.cmake` with the following contents:

```
# https://github.com/cyberbotics/epuck_ros2/blob/master/installation/cross_compile/toolchain.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER /opt/gcc-armv6hf/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-armv6hf/bin/arm-linux-gnueabihf-g++)
set(CMAKE_SYSROOT /opt/rootfs)

# https://github.com/eProsima/Fast-DDS/issues/1262
set(CMAKE_CXX_FLAGS "-latomic")

set(CMAKE_FIND_ROOT_PATH /opt/ros2_foxy/install)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(PYTHON_SOABI cpython-37m-arm-linux-gnueabihf)

# https://github.com/foonathan/memory/pull/60
set(CMAKE_CROSSCOMPILING_EMULATOR /usr/bin/qemu-arm-static)
```

__6.__ __(Deb10 Build Env)__ Download the ROS 2 sources:

```
mkdir -p /opt/ros2_foxy/src && cd /opt/ros2_foxy

wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos

vcs import src < ros2.repos
```

__7.__ __(Deb10 Build Env)__ Exclude unneeded components from the ROS build:

```
touch /opt/ros2_foxy/src/eclipse-cyclonedds/COLCON_IGNORE \
    /opt/ros2_foxy/src/ros-visualization/COLCON_IGNORE \
    /opt/ros2_foxy/src/ros/ros_tutorials/turtlesim/COLCON_IGNORE \
    /opt/ros2_foxy/src/ros2/demos/image_tools/COLCON_IGNORE \
    /opt/ros2_foxy/src/ros2/demos/intra_process_demo/COLCON_IGNORE \
    /opt/ros2_foxy/src/ros2/rviz/COLCON_IGNORE
```

## Build ROS 2 ##

__1.__ Workaround for Cyclone-DDS libssl Build Error:

While building ROS2, Cyclone DDS kept failing to build. I tried _several_
workarounds before settling on building it directly on the RPi, then copying the
build result into the build environment.

_Note_: These steps are based on the
[Cyclone DDS Build Doc](https://dds-demonstrators.readthedocs.io/en/latest/Teams/1.Hurricane/setupCycloneDDS.html)

__1.1__ __(RPiZW)__ Create required directories (these must match the Debian 10 build env layout):

```
sudo mkdir -p /opt/ros2_foxy/src/eclipse-cyclonedds

sudo chown -R ${USER}:${USER} /opt/ros2_foxy
```

__1.2__ __(RPiZW)__ Clone the Cyclone DDS source:

```
cd /opt/ros2_foxy/src/eclipse-cyclonedds

git clone -b releases/0.7.x https://github.com/eclipse-cyclonedds/cyclonedds.git cyclonedds
```

__1.3__ __(RPiZW)__ Create the ROS `install` and `build/cyclonedds` dirs:

```
cd /opt/ros2_foxy/

mkdir -p install build/cyclonedds
```

__1.4__ __(RPiZW)__ Build Cyclone DDS:

```
cd build/cyclonedds/

cmake -DCMAKE_INSTALL_PREFIX=/opt/ros2_foxy/install ../../src/eclipse-cyclonedds/cyclonedds/ -DBUILD_IDLC=OFF

cmake --build .

cmake --build . --target install
```

__1.5__ __(Deb10 Build Env)__ Copy the Cyclone DDS build from the RPiZW to the build environment:

_Note_: Replace `<RPI_IP>` with your Raspberry Pi Zero W's IP address.

```
mkdir /opt/ros2_foxy/install

rsync -r -l -p -t -v <RPI_IP>:/opt/ros2_foxy/install/ /opt/ros2_foxy/install/
```

__1.6__ __(RPiZW)__ Archive the Cyclone DDS build and cleanup `/opt/ros2_foxy/`:

```
cd /opt/ros2_foxy/

tar cf - build install src | gzip > ~/cyclonedds_build.tar.gz

rm -rf build install src
```


__2.__ __(Deb10 Build Env)__ Export the RPi's libraries to the C and C++ include paths:

_Note: Steps 2 - 4 are based on
[Cyberbotics Ltd's "Cross-Compilation"](https://github.com/cyberbotics/epuck_ros2/tree/master/installation/cross_compile)
and the [ROS2 Build Instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)_

```
export C_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
export CPLUS_INCLUDE_PATH="/opt/rootfs/usr/include:/opt/rootfs/usr/include/arm-linux-gnueabihf:/opt/rootfs/usr/lib/arm-linux-gnueabihf"
```

__3.__ __(Deb10 Build Env)__ Build ROS 2!

```
cd /opt/ros2_foxy

colcon build \
    --merge-install \
    --cmake-force-configure \
    --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF
```

__4.__ __(Deb10 Build Env)__ Compress and copy the install dir to the RPiZW:

```
cd /opt/ros2_foxy

tar -cf - install/ | gzip > ros2_foxy_install.tar.gz

scp ros2_foxy_install.tar.gz <RPI_IP>:~/
```

__5.__ __(RPiZW)__ 'Install' ROS 2:

```
sudo mkdir /opt/ros2_foxy

sudo chown -R ${USER}:${USER} /opt/ros2_foxy

mv ~/ros2_foxy_install.tar.gz /opt/ros2_foxy/

cd /opt/ros2_foxy

tar -xzf ros2_foxy_install.tar.gz
```

__6.__ __(RPiZW)__ Test the ROS 2 install:

```
. /opt/ros2_foxy/install/local_setup.bash

ros2 run demo_nodes_cpp talker
```

Leave the talker running in one terminal, then open a second terminal and run:

```
. /opt/ros2_foxy/install/local_setup.bash

ros2 run demo_nodes_py listener
```

"You should see the `talker` saying that it's `Publishing` messages and the
`listener` saying `I heard` those messages. This verifies both the C++ and Python
APIs are working properly. Hooray!"
("[Building ROS 2 on Ubuntu Linux](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)")

__8.__ __(RPiZW)__ (Optional) Add ROS 2 setup to your .bashrc:

_Warning_: This will add a several second delay to every login on the RPiZW!

Append the following lines to `~/.bashrc`:

```
# Source the ROS 2 setup script
if [ -f /opt/ros2_foxy/install/local_setup.bash ]; then
    . /opt/ros2_foxy/install/local_setup.bash
fi
```
