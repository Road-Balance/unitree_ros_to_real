# create catkin_ws

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

# Bashrc Edit

```
alias eb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'

alias cma='catkin_make -DCATKIN_WHITELIST_PACKAGES=""'
alias cop='catkin_make --only-pkg-with-deps'
alias sds='source devel/setup.bash'
alias axclient='rosrun actionlib axclient.py'

alias cba='colcon build --symlink-install'
alias cbr='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias cbp='colcon build --symlink-install --packages-select'
alias killg='killall -9 gzserver && killall -9 gzclient && killall -9 rosmaster'

alias rosmelo='source /opt/ros/melodic/setup.bash'
alias roseloq='source /opt/ros/eloquent/setup.bash && source ./install/setup.bash'
alias rosdinstall='rosdep install -y -r -q --from-paths src --ignore-src --rosdistro'
```

# clone pkgs && build

```
cd ~/catkin_ws/src
git clone https://github.com/Road-Balance/unitree_ros_to_real.git
```

## Edit LEGGED_SDK_DIR in unitree_legged_real/CMakeLists.txt

```
cd ~/catkin_ws
cop unitree_legged_msgs
cop unitree_legged_real
```

# Setup the net connection

`ifconfig`에서 `enx00` 존재 확인

```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```

If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.

# 이건 안해도 된다. 이후 자동화용임

In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to be changed to your own.

# Real Robot

```
roslaunch unitree_legged_real real.launch rname:=a1 ctrl_level:=highlevel firmwork:=3_2

rosrun joy joy_node _autorepeat_rate:=100
rosrun unitree_joy_cmd joy_to_cmd_vel_node
rosrun unitree_legged_real ros_control_a1
```