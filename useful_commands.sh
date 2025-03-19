ros2 launch kortex_bringup gen3_lite.launch.py \
robot_ip:=10.18.2.240 \
launch_rviz:=false


ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
robot_ip:=10.18.2.240

ros2 launch kortex_bringup gen3_lite.launch.py \
robot_ip:=10.18.2.239 \
launch_rviz:=false


ros2 launch kinova_gen3_lite_moveit_config robot.launch.py \
robot_ip:=10.18.2.239

ros2 run tf2_ros tf2_echo base_link end_effector_link

fastdds discovery --server-id 0

colcon build --packages-select final

colcon build --packages-select final_code



ssh ubuntu@10.18.3.92


echo "export ROS_DISCOVERY_SERVER=10.19.215.124:11811" >> ~/.bashrc


ros2 launch turtlebot3_bringup robot.launch.py

ros2 run turtlebot3_teleop teleop_keyboard


GIT_SSH_COMMAND="ssh -i ~/.ssh/id_final" git push origin main


GIT_SSH_COMMAND="ssh -i ~/.ssh/id_final_code" git push origin main
