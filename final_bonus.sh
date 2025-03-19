#!/bin/bash

# Run both ROS 2 nodes in parallel
ros2 run final arm_bonus &  
ros2 run final car_bonus &  

# Wait for both processes to complete
wait

echo "Both ROS 2 nodes have finished!"
