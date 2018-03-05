catkin clean -b --profile odroid
catkin build --profile odroid
touch src/Toolchain-arm-linux-gnueabihf.cmake
catkin build --profile odroid
echo "replace root path"
pwd
cd ..
var=$(pwd)
target=/root/model_car/catkin_user
echo "The catkin_ws_user directory $var."
sed -i 's#'$var'#/root#'g $var/catkin_ws/devel/.catkin $var/catkin_ws/devel/setup.sh $var/catkin_ws/devel/_setup_util.py $var/catkin_ws/devel/.rosinstall
sed -i 's#/opt/odroid-x2/sdk/#/#'g $var/catkin_ws/devel/_setup_util.py
sed -i 's#'$var'#/root#'g $var/catkin_ws/odroid-build/.catkin_tools.yaml
scp -r $var/catkin_ws/src/ root@192.168.43.102:$target
scp -r $var/catkin_ws/odroid-devel/ root@192.168.43.102:$target
echo "finished"
