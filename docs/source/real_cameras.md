# perls2 with Real Cameras

## Description
perls2 provides interfaces to obtain rgb and depth images from Cameras. 
Currently all supported cameras use ROS drivers. A separate process forwards
frames published to ROS to redis for the main perls2.Env to obtain. This allows perls2 to run in 
a virtual environment separate from ROS. 

## Set up

ROS should be setup and installed on the workstation. The current setup is 
with ros-kinetic, but it should be forward compatible. 

1. Install redis

    ```bash
    sudo apt-get install redis-server
    ```

2. Install python-rospkg

    ```bash
    sudo apt-get install python-rospkg
    ```

3. Install tmux (terminal-multiplexer) which is used to create multiple windows in the same terminal. 
    ```bash
    sudo apt install tmux
    ```

4. Create a python2.7 virtualenv for perls2. This allows perls2 to run proccess compatible with ros-kinetic. In order to use the python-rospkg we just installed, it should use the system-site-packages.

    ```bash
    cd ~
    virtualenv -p python2.7 p27env
    ```

5. Source the virtualenv you just created and install perls2 using the p27requirements.txt
    ```bash
    source ~/p27env/bin/activate
    cd ~/perls2
    pip install -r p27_requirements.txt
    pip install -e .
    ```

6. Install necessary ros packages for your camera. See manufacturer for details. 
    * [Set up for realsense cameras](https://github.com/IntelRealSense/realsense-ros)
    * Set up for kinect2 cameras. 

### Set up local folder
A local folder is used to store install config specific to this machine. Setting it up once enables use of bash scripts with perls2 to start cameras automatically. 

1. Copy the local/perls2_local_ws folder to customize install for perls2. 

    ```bash
    cd ~/perls2
    cp -r local/perls2_local_ws ~/
    cd ~/perls2_local_ws
    ```

2. Open the `local_dirs.sh` file in a text editor and adjust the variables as necessary specific
to your install. Make sure to edit the `SOURCE_ROS_CMD` and `LAUNCH_RS_CAMERAS_CMD` for your workstation and camera setup.

    ```bash
    #!/bin/bash
    # Command to source virtualenv environment with  python 3.6 perls2 installed
    export SOURCE_ENV_CMD="source ~/p2env/bin/activate"

    # Command to source virtualenv environment with python2.7 perls2 installed
    export SOURCE_P27_CMD="source ~/p27env/bin/activate"

    # ROS WS directory
    export ROS_DIR="$HOME/ros_ws"
    # Command to source ROS
    export SOURCE_ROS_CMD="source /opt/ros/kinetic/setup.sh"

    # ROS Launch command for realsense cameras
    LAUNCH_RS_CAMERAS_CMD="roslaunch ${ROS_DIR}/src/realsense-ros/realsense2_camera/launch/rs_multiple_devices.launch"

    # Directory for perls2 repo
    export PERLS2_DIR="$HOME/perls2"
    ```

3. Edit the `ros_sensors.yaml` file in the local folder with the rostopics you want to subscribe to.
    Note 'name' of the camera in the rostopic. You can see what rostopics are being published by launching the camera and using `rostopic list`. Currently only one rgb and depth stream per camera is supported. 

    Example: A Realsense DS435 publishes to the following rostopics: 

        * rgb : /ds435/color/image_raw
        * depth: /ds435/depth/image_rect_raw


    Our ros_sensors.yaml file would look like this: 
    ```yaml
    # Config for ROS Cameras
    # Redis server hosted locally on workstation
    workstation_redis: 
      host: "127.0.0.1"
      port: 6379

    # Config for Cameras that use ROS 
    rgb_topics:
      ['/ds435/color/image_raw']

    depth_topics:
      ['/ds435/depth/image_rect_raw']

    invert: true
    ```

    You can add additional cameras by appending them as a list, like so: 

    ```yaml
    # Config for ROS Cameras

    # Redis server hosted locally on workstation
    workstation_redis: 
      host: "127.0.0.1"
      port: 6379

    # Config for Cameras that use ROS 
    rgb_topics:
      ['/sr300/color/image_raw',
       '/ds435/color/image_raw']

    depth_topics:
      ['/ds435/depth/image_rect_raw',
       '/sr300/depth/image_rect_raw']

    invert: true
    ```

4. Test your real camera setup using the bash script. 
    1. Open a terminal
        ```bash
        cd ~/perls2
        ./scripts/start_realsense_cameras.sh ~/perls2_local_ws
        ```

    2. Open another terminal, and run the ros_camera_interface as a script, supplying the name of the camera you want to display as a positional arguments. The script will display 5 frames from
    each image stream. 
        ```bash
        source ~/p2env/bin/activate
        cd ~/perls2
        python perls2/sensors/ros_camera_interface.py ds435
        ```

    You can also supply multiple camera names. 
    ```bash
    source ~/p2env/bin/activate
    cd ~/perls2
    python sensors/ros_camera_interface.py ds435 sr300
    ```

## Usage

### Starting RealSense cameras:
After setting up the local folder, it is easy to start the cameras using a bash script. Always start
the cameras before running your main perls2 project environment. 

1. Run the `start_realsense_cameras.sh` file and supply the perls2_local_ws directory as a positional argument. 

    ```bash
    cd ~/perls2
    ./scripts/start_realsense_cameras.sh ~/perls2_local_ws
    ```

    This will open a tmux window with the following processes: 

        1) (Top Left) roscore
        2) (Bottom Left) redis-server
        3) (Top Right) Camera ROS Nodes from launch file
        4) (Bottom Right) perls2 RosRedisPublisher

2. You can now run your perls2.Env in a separate terminal. 

3. To kill the script, in the tmux terminal run the tmux command using `[Ctrl+b+:] kill-session`.
If you close out of the terminal, the tmux session will continue to run. If this happens, use the kill script in new terminal: 
    ```bash
    cd ~/perls2
    ./scripts/kill_tmux_session.sh ros_cameras
    ``` 

### Create camera interface in your perls2 project:

To make an interface to a camera that uses ROS, pass the name of the camera in 
the rostopic it publishes to. For example, if we have a RealSense DS435 camera
that publishes to the rostopic "/ds435/color/image_raw", we create a camera
interface as follows: 

```python
from perls2.sensors.ros_camera_interface import ros_camera_interface
camera_interface = RosCameraInterface('ds435')
```

Note that if you have multiples of the same type of cameras, you can create
multiple camera_interfaces, as long as they are publishing to unique rostopics.
See setup for more information. 

Multiple cameras:

```python
from perls2.sensors.ros_camera_interface import ros_camera_interface

sr300_interface = RosCameraInterface('sr300_1')
sr300_2_interface = RosCameraInterface('sr300_2')
ds435_interface = RosCameraInterface('ds435')
```

### Obtain Frames:
You can obtain camera frames similar to simulated cameras using the `frames()` function. 

For example: 

```python
from perls2.sensors.ros_camera_interface import ros_camera_interface

camera_interface = RosCameraInterface('sr300')

#all frames
frames = camera_interface.frames()
rgb = frames['rgb']
depth = frames['depth']
tstamp = frames['rgb_tstamp']

# Only obtain rgb frames
frames_rgb = camera_interface.frames_rgb()

#only obtain depth frame
frame_depth = camera_interface.frames_depth()
```
