# Toretto
A Fast and Furious software for self driving cars.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

1. Ubuntu 16.04 LTS. It's highly recommendable that you natively install the OS on your development computer. [Install](http://releases.ubuntu.com/16.04/)
2. ROS Kinetic [Installation Guide](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Installation

The installation process will take some time to get through so make sure you have internet connection and your laptop connected to a power supply. 

1. Make a git clone from this [repo](https://github.com/mnegretev/MobileRobotsCourse)

`git clone https://github.com/mnegretev/MobileRobotsCourse`

2. Enter the repo folder and run the `./Setup.sh` file. This process might take an hour.
```
cd MobileRobotsCourse
./Setup.sh
```

3. Compile everything:

```
cd MobileRobotsCourse/catkin_ws
catkin_make
```

4. Run the Justina Software package to verify everything installed correctly


## Deployment

There are various parts of the project that you will be using. In the next couple sections we talk about how to use different modules during the development process and finish by providing comands to execute it on the car.

### Run the car

On Terminal #1 run:

```
roslaunch lane_detection lane_hough.launch

```
Open a new terminal (Terminal #2)  and run:

```
roslaunch surge_et_embula lane_tracking.launch
```


## License

TODO



