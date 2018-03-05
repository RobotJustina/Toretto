echo "replace root path"
var=$(pwd)
ws_name=$(basename $var)
tmpfile=/tmp/magicReplaceTempFile
target=root@192.168.43.102:./model_car/$ws_name/
echo "The workspace directory is $var."

rsync -r  $var/src $target
rsync -r $var/devel $target
rsync -r $var/build $target

sed 's#'$var'#/root#'g $var/devel/_setup_util.py > $tmpfile
sed -i 's#/opt/odroid-x2/sdk/#/#'g $tmpfile
sed -i 's#/catkin_ws/#/'$ws_name'/#'g $tmpfile
sed -i 's#/opt/ros/indigo;//opt/ros/indigo#/opt/ros/indigo;/root/catkin_ws/devel/#'g $tmpfile
rsync $tmpfile $target/devel/_setup_util.py

sed 's#'$var'#/root/'$ws_name'#'g $var/build/.catkin_tools.yaml > $tmpfile
rsync $tmpfile $target/build/.catkin_tools.yaml

sed 's#'$var'#/root/'$ws_name'#'g $var/devel/.catkin > $tmpfile
rsync $tmpfile $target/devel/.catkin

sed 's#'$var'#/root/'$ws_name'#'g $var/devel/setup.sh > $tmpfile
rsync $tmpfile $target/devel/setup.sh

sed 's#'$var'#/root/'$ws_name'#'g $var/devel/.rosinstall > $tmpfile
rsync $tmpfile $target/devel/.rosinstall
echo "finished"
