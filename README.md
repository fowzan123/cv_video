CV Video
========

CV Video integrates [openCV](http://opencv.org/)'s [video recording and replaying API's](http://docs.opencv.org/2.4/modules/highgui/doc/reading_and_writing_images_and_video.html) to ROS.

Build & Install
---------------

CV Video is built using [catkin](http://wiki.ros.org/catkin). Type the commands below on a terminal window to create a catkin workspace, clone the repository and build the sources:

    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ git clone https://github.com/xperroni/cv_video.git
    $ cd ..
    $ catkin_make

Features
--------

CV Video can be used as a collection of standalone ROS nodes, a library, or an action server with a C++ client API.

### Nodes

CV Video includes two standalone ROS nodes, `record` and `replay`, which can be used respectively to record and replay video files. Moreover, the `camcorder` action server provides video recording services for external nodes.

#### `camcorder`

The `camcorder` node records images from a ROS image topic and writes them to a video file. It can be accessed by external ROS nodes through an Action API.

##### Topics

`image` ([`sensor_msgs/Image`](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
<p style ="margin-left:40px">
  Subscribed topic from which frames are recorded.
</p>

##### Parameters

`~path` (string, default: `"<current working directory>/video.mpg"`)
<p style ="margin-left:40px">
  Default output video file path.
</p>

`~format` (string, default: `MPEG`)
<p style ="margin-left:40px">
  Default output video file encoding format.
</p>

`~framerate` (double, default: `30.0`)
<p style ="margin-left:40px">
  Default frame rate, in Frames Per Second (FPS).
</p>

`~width` (integer, default: `640`)
<p style ="margin-left:40px">
  Default frame width, in pixels.
</p>

`~height` (integer, default: `640`)
<p style ="margin-left:40px">
  Default frame height, in pixels.
</p>

#### `record`

The `record` node records images from a ROS image topic and writes them to a video file.

##### Topics

`image` ([`sensor_msgs/Image`](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
<p style ="margin-left:40px">
  Subscribed topic from which frames are recorded.
</p>

##### Parameters

`~path` (string, default: `"<current working directory>/video.mpg"`)
<p style ="margin-left:40px">
  Path to the output video file.
</p>

`~format` (string, default: `MPEG`)
<p style ="margin-left:40px">
  Output video file's encoding format.
</p>

`~framerate` (double, default: `30.0`)
<p style ="margin-left:40px">
  The recorded video file's frame rate, in Frames Per Second (FPS).
</p>

`~width` (integer, default: `640`)
<p style ="margin-left:40px">
  The width of recorded frames, in pixels.
</p>

`~height` (integer, default: `640`)
<p style ="margin-left:40px">
  The height of recorded frames, in pixels.
</p>

#### `replay`

The `replay` node reads a video file and publishes its frames to a ROS image topic. See the [image_transport](http://wiki.ros.org/image_transport) API for details on how to read from the topic.

##### Topics

`image` ([`sensor_msgs/Image`](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
<p style ="margin-left:40px">
  Topic to which replayed frames are published.
</p>

`playing` ([`std_msgs/Bool`](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
<p style ="margin-left:40px">
  Enables other nodes to pause and resume a replay session. An `std_msgs/Bool` message published to this topic with a `false` value will pause the node, while a `true` value will make it resume.
</p>

##### Parameters

`~path` (string, default: `"<current working directory>/video.mpg"`)
<p style ="margin-left:40px">
  Path to the replayed video file.
</p>

`~playing` (boolean, default: `true`)
<p style ="margin-left:40px">
  Whether the node should start replaying the video immediately. If `false`, replay only starts after a `std_msgs/Bool` with value `true` is sent to the node's `playing` topic.
</p>

`~framerate` (double, default: `30.0`)
<p style ="margin-left:40px">
  The rate at which video frames are published, in frames per second.
</p>

### Library API

The `cv_video::Video` class connects to a ROS image topic and provides several methods to record, replay, and/or manipulate video streams and respective frames. See the header file `include/video.h` for usage instructions.

### Action API

The `cv_video::Camera` class provides a C++ client API to the `camcorder` action server. See the header file `include/camera.h` for usage instructions.