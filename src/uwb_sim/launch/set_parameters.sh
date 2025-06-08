for i in 1 2 3; do
  ros2 param set /drone_manager${i} formation_side_length 6.0
  ros2 param set /drone_manager${i} system_id_list "[1, 2, 3]"

done
