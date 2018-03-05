# Docker for cross-compiling

The mini car runs on Ros indigo and Ubuntu 14.04 Trusty, So Compiling from
ros Kinetic is impossible.
To circumvent this the presnt docker file is created. It lets emulate a Indigo machine inside any Docker-capable computer.

## Requirements
1. Pull this repo.
2. Install Docker: https://docs.docker.com/install/linux/docker-ce/ubuntu/
3. Download SDK and compiler from AutoModelCar https://github.com/AutoModelCar/AutoModelCarWiki/wiki/Cross-compile
4. Copy and rename to sdkOdroid.tar.bz2 and compilerOdroid.tar.bz2 in this folder

## Build image
 First you need to build the image. Run the following on /docker:
```bash
  docker buld -t automodel .
```
This can take a few minutes as it pull and updpate relevant data from  server. the -t tag is optional it specified a user provided name for the image.

## Run container
The configuration for the cross compiling is already set via the dockerfile all you need to do is run the container.

```bash
  docker run -it -v  /route_to_repo_on host/PumasToretto/model_car:/root/model_car/  automodel:latest bash
```
This will return a bash session inside the container. However it is important to note the usage of the -v tag. This tells docker to mount a file on the host machine on the container file system. so any changes on the container will be reflected on the host system.

## Compiling
Once inside the container go to /root/model_car it is important to use this directory as, currently, the configuration of the catkin_ws will point to this file. If eveything was succesful you should see catkin_ws inside.
Move to the workspace directory and:
```bash
  cd catkin_ws/
  catkin build --profile odroid
```
This will likely fail as there is a bug with catkin. To continue simpli
```bash
  touch src/Toolchain-arm-linux-gnueabihf.cmake
  catkin build --profile odroid
```
Reperat until everythin is compiled. Alternatively if you want to compile only one package:

```bash
  catkin build --profile odroid package_to_build
```
And repeat previous step in case of errors.
Also if you need to do a clean Build
```bash
  catkin clean -b --profile odroid 
```

## Copying
Now you can exit the container, the docker daemon will stop it so it will not use cpu.
To copy you need to copy the workspace into the model car. And reconfigure the workspace, thankfully the replace.sh file does all that. But this line must be mentioned.
```bash
target=root@192.168.43.102:./model_car/$ws_name/
```
As it is now. The configuration script only works if you copy the workspace into /root/model_car inside the model car. If you want to compile somewhere else you must change the workspace directory on the container to match exactly the path of the workspace in the model car.
