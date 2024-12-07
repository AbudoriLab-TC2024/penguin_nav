#!/bin/bash
echo "Please set waypoints before running script"
echo "navigation"

#gnome-terminal --window -- ros2 launch nav2_bringup rviz_launch.py

#sleep 2
#gnome-terminal --window -- ros2 launch pcl_localization_ros2 hesai.py 
#gnome-terminal --window -- ros2 launch penguin_nav nav.launch.xml

#sleep 7

#ros2 run penguin_nav follow_path.py -- /home/yuta/colcon_ws/src/penguin_nav/data/tsukuba_localozation_sparse_part_mod/test.csv

#gnome-terminal --window -- ros2 run penguin_nav follow_path.py -- /home/yuta/colcon_ws/src/penguin_nav/data/cuborex/cuborex_test_space.csv
#ros2 run penguin_nav follow_path.py -- area03-sparse-later.csv
#ros2 run penguin_nav follow_path.py -- area05-sparse.csv
ros2 run penguin_nav follow_path.py --  area01-sparse.csv  area02-sparse.csv area03-sparse.csv area04-sparse.csv area05-sparse.csv area06-sparse.csv area07-sparse.csv
