Short description for testing the user controller plugin.

1. Compile the GazeboRosPackages ROS workspace with catkin_make

2. Create a symlink for the test model in your ~/.gazebo/models folder:
cd ~/.gazebo/models
ln -s <package_path>/sdf test_model

3. start gazebo with your GazeboRosPackages workspace sourced:
cd <GazeboRosPackages>
source devel/setup.bash
roscore && rosrun gazebo_ros gazebo  (--verbose for debug output)

4. Insert the "test user avatar" model from the models list. (see model.sdf in sdf folder).

5. Use ROS topic publisher to control the joints:
rostopic list    (list available topics)

example for setting velocity of body link to 1.0 meter per second in x direction:
rostopic pub /user_avatar_basic/body/cmd_vel geometry_msgs/Vector3 1.0 0.0 0.0

example for setting rotation of model to 0° 45° 0° (euler angles):
rostopic pub /user_avatar_basic/cmd_rot geometry_msgs/Quaternion 0.9238795325112867 0 0 0.3826834323650897

