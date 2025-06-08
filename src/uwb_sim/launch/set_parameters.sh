for i in 1 2 3 4; do
  # ros2 param set --spin-time 2.0 /drone_manager${i} formation_side_length 9.0
  # ros2 param set --spin-time 2.0 /drone_manager${i} mission_zlevel 5.0
  ros2 param set --spin-time 2.0 /drone_manager${i} formation_k_scale 0.0
  ros2 param set --spin-time 2.0 /drone_manager${i} formation_k_pair 2.0
  ros2 param set --spin-time 2.0 /drone_manager${i} formation_k_shape 4.0
  ros2 param set --spin-time 2.0 /drone_manager${i} formation_k_z 2.0
  ros2 param set --spin-time 2.0 /drone_manager${i} formation_tolerance 1.5
  ros2 param set --spin-time 2.0 /drone_manager${i} repulsion_c_rep 5.0
  ros2 param set --spin-time 2.0 /drone_manager${i} repulsion_cutoff 3.0
  ros2 param set --spin-time 2.0 /drone_manager${i} repulsion_sigma 2.0
  ros2 param set --spin-time 2.0 /drone_manager${i} target_k 3.0
  ros2 param set --spin-time 2.0 /drone_manager${i} weight_table "[10,1,1, 10,1,1, 10,1,2]"
  # ros2 param set --spin-time 2.0 /drone_manager${i} num_particles 1000
  # ros2 param set --spin-time 2.0 /drone_manager${i} uwb_threshold 10.0
done
