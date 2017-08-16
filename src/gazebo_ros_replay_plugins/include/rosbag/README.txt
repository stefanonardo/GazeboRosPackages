Files contained in this directory are modified versions of the official Kinetic
rosbag sources. This code really isn't good and there are a number of potential
issues that could cause us to rewrite it, but it works for now.

Changelog:

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
