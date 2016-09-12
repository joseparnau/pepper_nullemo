# pepper_nullemo

# Installation (+ catkin workspace creation)

$ cd ~/catkin_ws/src

$ git clone -b pepper_ext_nullemo_2 https://github.com/joseparnau/rosjac.git .

$ catkin_init_workspace

$ git clone https://github.com/ros-naoqi/pepper_meshes.git

$ cd ..

$ catkin_make

$ cd build/pepper_meshes

$ make pepper_meshes_meshes (install meshes in "~/catkin_ws/src/pepper_meshes")

$ source ~/.bashrc

# Execution

$ roscore

$ roslaunch pepper_nullemo pepper_nullemo_full.launch

$ rostopic pub -1 /pepper_generate_poses_trajectory std_msgs/Int32 "data: 2"
