Files contained in this directory are modified versions of the official Kinetic
rosbag sources. This code really isn't good and there are a number of potential
issues that could cause us to rewrite it, but it works for now.

Changelog:

2017.31.08
- removed a race condition where recorder stop would not work if called before the
  running flag was set, moved to the constructor to avoid conflicts
- modified playback logic to include time limit for smoother and more consistent
  playback than previous "burst" implementation that published all at once

2017.22.08
- do not record topics published by Gazebo/plugins by default
- support white listing specific topic types from Gazebo (generalizable)
- support rate limiting/downsampling recording of topics published by Gazebo
- reduced sleep while paused for more responsive playback resume

2017.10.08
- added playback state to mark completion of loop or gracefully exit playback
- disabled terminal input/output during rosbag playback
- modified playback to run as fast as possible rather than using realtime
- fixed issue where rosbag would not finish recording entire bag before exiting

2017.21.06
- add support for gracefully stopping a programatically started rosbag recorder
- made destruction thread safe by cancelling ros timers and subscriptions
- removed ros spin as that is handled elsewhere in this context
- add support for "path" in RecorderOptions to add destination other than current dir
