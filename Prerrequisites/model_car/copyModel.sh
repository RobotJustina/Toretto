scp -r catkin_ws root@192.168.1.199:./model_car
scp -r extra_library/glfw root@192.168.1.199:./model_car
scp -r extra_library/librealsense root@192.168.1.199:./model_car
#scp autostart.sh root@192.168.1.199:./
scp .bashrc root@192.168.1.199:./
scp installation_file/set.sh root@192.168.1.199:./model_car
scp installation_file/set2.sh root@192.168.1.199:./model_car

echo "copy finished!"
