FROM ros:indigo-robot

#dependencies
RUN apt-get update && apt-get install -y \
  python-catkin-tools \
  ros-indigo-camera-info-manager \
  ros-indigo-pcl-ros \
  libc6-armel-cross libc6-dev-armel-cross \
  binutils-arm-linux-gnueabi\
  libncurses5-dev \
  gcc-arm-linux-gnueabihf \
  lib32stdc++6 \
  lib32z1

##run update again, as in wiki
RUN apt-get update
#SDK and compiler
#RUN mkdir -p /opt/odroid-x2/compiler && mkdir -p /opt/odroid-x2/sdk/
COPY sdkOdroid.tar.bz2 /opt/odroid-x2/
RUN tar -xjf /opt/odroid-x2/sdkOdroid.tar.bz2 -C /opt/odroid-x2/
COPY compilerOdroid.tar.bz2 /opt/odroid-x2/
RUN  tar -xjf /opt/odroid-x2/compilerOdroid.tar.bz2 -C /opt/odroid-x2/

#workspace for mount
RUN mkdir -p /root/model_car/catkin_ws && cd /root/model_car/catkin_ws

RUN catkin config --profile odroid -b build -d devel -i install
RUN catkin config --profile odroid --cmake-args  -DCMAKE_TOOLCHAIN_FILE=/root/model_car/catkin_ws/src/Toolchain-arm-linux-gnueabihf.cmake
RUN catkin config --profile odroid --extend /opt/odroid-x2/sdk/opt/ros/indigo/
RUN catkin config --merge-devel --profile odroid
RUN  catkin config --profile odroid --install

VOLUME /root/model_car
