The node generates min_snap trajectory through the set waypoint, and control quadrotor to fly through the trajectory in offboard mode.

The following parameters can be changed: segment time, waypoints coodinates, minimization_weights, max_velocity, max_normalized_thrust, max_roll_pitch_rate

adjust the max_velocity, max_normalized_thrust and max_roll_pitch_rate according to the flight distance otherwise the trajectory is undefined


The node name is min_snap_trajectory.

run the node using commands: rosrun min_snap_trajectory offb_min_snap_node

The launch file was unusedable till now.
