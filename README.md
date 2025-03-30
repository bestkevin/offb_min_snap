The node generates min_snap trajectory through the set waypoint, and control quadrotor to fly through the trajectory in offboard mode.

The following parameters can be changed: segment time, waypoints coodinates, minimization_weights, max_velocity, max_normalized_thrust, max_roll_pitch_rate

adjust the max_velocity, max_normalized_thrust and max_roll_pitch_rate according to the flight distance otherwise the trajectory is undefined


The node name is min_snap_trajectory.

run the node using commands: rosrun min_snap_trajectory offb_min_snap_node

The launch file was unusedable till now.



need to git clone dependencies: 
mav_comm: https://github.com/bestkevin/mav_comm.git
ethz-asl/rotors_simulator：https://github.com/ethz-asl/rotors_simulator.git
uzh-rpg/rpg_quadrotor_common：https://github.com/uzh-rpg/rpg_quadrotor_common.git
uzh-rpg/rpg_quadrotor_control: https://github.com/uzh-rpg/rpg_quadrotor_control.git
catkin_simple: git@github.com:catkin/catkin_simple.git
eigen_catkin: git@github.com:ethz-asl/eigen_catkin.git
rpg_single_board_io: git@github.com:uzh-rpg/rpg_single_board_io.git

