# ROS Visual Odometry

# Contents
- Introduction 
- System architecture
- Preparing the environment
- Calibrating the camera
- Rectifying image
- Getting odometry
- Visualizing pose
# **Introduction**

After this tutorial you will be able to create the system that determines position and orientation of a robot by analyzing the associated camera images. This information can be used in Simultaneous Localisation And Mapping (SLAM) problem that has been at the center of decades
of robotics research. 

![text](https://d2mxuefqeaa7sj.cloudfront.net/s_DCA5203FC2AD27DE713C6C3ED7EF6FA7E9F1E1444E562CD24751CF0A3DE3C00A_1501773149234_quad.jpg)

## What do we need to build this?
- Raspberry PI running Emlid Linux distro with pre-installed ROS 
- Raspberry PI Camera
- Server running Linux

If you have everything, grab a cup of coffee, sit down, and read this tutorial.

----------
# **System architecture**

If you absolutely have no idea what is ROS, nodes and how they communicate with each other, I strongly recommend you to learn it by reading official documentation and completing tutorials for beginners. Alternatively, you can read our guides. explaining its [basic architecture](https://docs.emlid.com/navio2/common/dev/ros/) and teaching how to write simple publisher and subscriber either on [Python](http://github.com/emlid/mavros-navio-python-example) or [C++](http://github.com/emlid/mavros-navio-cpp-example).
So, the graph of our system looks like this:

![text](https://d2mxuefqeaa7sj.cloudfront.net/s_DCA5203FC2AD27DE713C6C3ED7EF6FA7E9F1E1444E562CD24751CF0A3DE3C00A_1501776109887_visual_odometry.png)


As you can see in this picture, we have Raspberry Camera connected and raspicam creating multimedia pipeline and sending video from camera to gst-launch. The latter then transmit the image to our server over UDP. gscam will broadcast the video to /raspicam/image_raw topic. This image should be rectified with image_proc node. And finally, rectified image is taken by mono_odometer, which handles it and computes position and orientation of the robot publishing this data straight to the /vision/pose topic.   

----------
# **Preparing the environment**
## **Connecting the camera**

Firstly, connect your camera to Raspberry. To determine whether it’s working or not, just type:

    $ sudo vcgencmd get_camera

If you got **supported=1 detected=1**, ****then it’s ok and you can follow the next step. Otherwise, you should enable your camera with raspi-config. Furthermore, you can test video streaming with [this tutorial](https://docs.emlid.com/navio2/common/dev/video-streaming/). 

## **Installing ROS on your server**

I used ROS kinetic, but you may use anything you want. Check out the [official guide](http://wiki.ros.org/kinetic/Installation) to get it working. Installation of ROS is quite straightforward and usually doesn’t produce errors. If you stumbled and got any, you can always ask for help on the [forum](https://discourse.ros.org/). 

## **Installing all packages and libraries**

In this section we are going to build our environment with every library we need. Here are the list of what we should install:

- OpenCV
- libviso2
- gscam
- image_common
- image_pipeline
- vision_opencv
## **OpenCV**

The needed packages should be installed using a terminal and the following commands:

    $ sudo apt-get install build-essential
    $ sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
    $ sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

Get the latest stable OpenCV version:

    $ git clone https://github.com/Itseez/opencv.git

Build:

    $ cd /opencv
    $ mkdir build
    $ cd build
    $ cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ make -j5 # runs 5 jobs in parallel

All the following packages should be cloned into ~/odometry/src, so 

    $ cd ~/odometry/src
## **viso2**
    $ git clone https://github.com/srv/viso2
## **gscam**
    $ git clone https://github.com/ros-drivers/gscam
## **image_common**
    $ git clone https://github.com/ros-perception/image_common.git
## **image_pipeline**
    $ git clone https://github.com/ros-perception/image_pipeline.git
## **vision_opencv**
    $ git clone https://github.com/ros-perception/vision_opencv.git

Building everything:

    $ cd ~/odometry
    $ catkin_make

After successful building all packages let’s get our system up and working. Firstly, ssh into Raspberry and start broadcasting video to our server:

    $ raspivid -n -w 640 -h 480 -b 1000000 -fps 40 -t 0 -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=<IP> port=9000

 Where <IP> is IP address of your server. If you don’t know it, type:

     $ ifconfig

and find your network and your IP.

After that change directory to ~/odometry/gscam/examples and create a new launch file called ‘raspicam.launch’:

    $ cd /odometry/src/gscam/examples
    $ vi raspicam.launch

Paste the following and save it:

    <launch>
       <arg name="cam_name" value="raspicam"/>
       <env name="GSCAM_CONFIG" value="udpsrc port=9000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert" />
       <node pkg="gscam" type="gscam" name="$(arg cam_name)">
         <param name="camera_name" value="$(arg cam_name)" />
         <remap from="camera/image_raw" to="$(arg cam_name)/image_raw" />
       </node>
     </launch>

Then launch gscam and see if you can get an image:

    $ roslaunch gscam raspicam.launch
    $ rosrun image_view image_view image:=/raspicam/image_raw 
----------
# **Calibrating the camera**

Before Starting Make sure that you have a large [checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf) with known dimensions.
To calibrate camera we will use cameracalibrator.py node from package image_calibration which is already installed. To run it for a monocular camera using an 8x6 chessboard with 24mm squares  just type:

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/raspicam/image_raw

You will see a new window opened which will highlight the checkerboard:. This will open up the calibration window:

![text](https://d2mxuefqeaa7sj.cloudfront.net/s_DCA5203FC2AD27DE713C6C3ED7EF6FA7E9F1E1444E562CD24751CF0A3DE3C00A_1501849126944_calibrating.png)


In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

- checkerboard on the camera's left, right, top and bottom of field of view
  - X bar - left/right in field of view
  - Y bar - top/bottom in field of view
  - Size bar - toward/away and tilt from the camera
- checkerboard filling the whole field of view
- checkerboard tilted to the left, right, top and bottom

As you move the checkerboard around you will see three bars on the calibration sidebar increase in length. When the **CALIBRATE** button lights, you have enough data for calibration and can click **CALIBRATE** to see the results.

![text](https://d2mxuefqeaa7sj.cloudfront.net/s_DCA5203FC2AD27DE713C6C3ED7EF6FA7E9F1E1444E562CD24751CF0A3DE3C00A_1501849816655_calibrated.png)


Calibration can take about a minute. The windows might be greyed out but just wait, it is working.
After calibration is done, you can save the archive and then extract it. You will need the *.yaml file. Rename it to ‘raspicam.yaml’ and move it to the ‘~/odometry/src/gscam/example’ directory. Then open file ‘raspicam.launch’ that we’ve already created and change it, so that it should looks like this:

    <launch>
       <arg name="cam_name" value="raspicam"/>
       <env name="GSCAM_CONFIG" value="udpsrc port=9000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert" />
       <node pkg="gscam" type="gscam" name="$(arg cam_name)">
         <param name="camera_name" value="$(arg cam_name)" />
         <param name="camera_info_url" value="package://gscam/examples/raspicam.yaml" />
         <remap from="camera/image_raw" to="$(arg cam_name)/image_raw" />
       </node>
     </launch>

After that you have your camera calibrated and can launch gscam by:

    $ roslaunch gscam raspicam.launch


----------
# **Rectifying the image**

The raw image from the camera driver is not what is needed for visual processing, but rather an undistorted and (if necessary) debayered image. This is the job of image_proc. For example, if you have topics /raspicam/image_raw and /raspicam/camera_info you would do:

    $ ROS_NAMESPACE=raspicam rosrun image_proc image_proc

There will appear a new topic /raspicam/image_rect. It’s exactly what we need.


----------
# **Getting odometry**
    $ rosrun viso2_ros mono_odometer image:=/raspicam/image_rect

This will publish /mono_odometer/pose messages and you can echo them:

    $ rostopic echo /mono_odometer/pose


----------
# **Visualizing pose**

If you want to visualize that messages that is published into /mono_odometer/pose, then you should install and build another one package:

    $ cd ~/odometry/src
    $ git clone https://github.com/ros-visualization/rqt_pose_view.git 
    $ cd ~/odometry
    $ catkin_make

The **rqt_pose_view** is a very simple plugin just displaying an OpenGL 3D view showing a colored cube. You can drag and drop a geometry_msgs/Pose topic onto it from the "Topic Introspection" or "Publisher" plugins to make it visualize the orientation specified in the message.

![text](https://d2mxuefqeaa7sj.cloudfront.net/s_DCA5203FC2AD27DE713C6C3ED7EF6FA7E9F1E1444E562CD24751CF0A3DE3C00A_1501848252527_rqt.png)

