string robot_id                 #
string robot_model_rel_path     # A path to the template SDF or custom zip
bool is_custom                  # If true, robot_rel_path points to a custom zip (optional)
                                # Otherwise, it points to a template SDF
cle_ros_msgs/Pose pose          # Euler representation of the robot pose (optional)
                                # Defaults to <0,0,0,0,0,0> or first robot's pose
