Summary
-------

The plugins provided by this package provide interfaces to enable dynamic
recording and synchronized playback of Gazebo and ROS messages.

Gazebo logs only contain delta events, wehreas ROS bags contain all events - so
coordinated recording and playback is non-trivial.


Recording Plugin (gazebo_ros_recording_plugin)
----------------------------------------------

The following service calls are provided:

	- /gazebo/recording/start - Start coordinated recording.

	- /gazebo/recording/stop - Stop recording and wait for files to be written.

	- /gazebo/recording/cancel - Cancel current recording and discard files.

	- /gazebo/recording/cleanup - Clean all generated files and recordings
																to be used for reset/shutdown.

	- /gazebo/recording/get_recording - Notifies the caller if recording is
																			started and the output base path for files.

Recordings can be generated on demand with multiple calls to start and stop that
will generate several files. These files are saved in a temp dir in the form:

	- <dir>/gzserver/<num>.log - Gazebo state logs

	- <dir>/ros/<num>.bag - ROS bags
