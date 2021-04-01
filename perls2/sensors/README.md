# Sensors
 
perls2.sensors module for sensor_interfaces with perls2.

## Description: 
Sensor interfaces in this module are used by perls2 to retrieve information about specific aspects of the environment state. 
Currently the perls2 sensor module only supports RGB and Depth cameras. 

For simulated worlds (e.g. Bullet), camera interfaces are used to render rgb and depth images using specific parameters. 
For real worlds, camera interfaces are used to obtain images from real cameras and depths sensors. 

## Camera Interfaces

### Usage
After being intialized, the frames for any camera interface can be retrieved using the `frames()` function. This returns a dict with the frames for rgb and depth.

```
frames = camera_interface.frames()

# RGB Image as a numpy ndarray
rgb_image = frames['rgb']

# Depth image as numpy ndarray
depth_image = frames['depth']
```

### Real Camera Interfaces: 
Currently perls2 only supports real cameras that publish images to ROS topics. Cameras are connected to the workstation. The following processes run on the workstation (in separate terminals):

1. roscore : collection of nodes and programs to run ROS 
2. camera ROS nodes: processes spawned by `roslaunch` command that enable publishing of camera images to rostopics.
3. redis-server: server used by perls2 for interprocess communication. 
4. RosRedisPublisher: script used to subscribe to rostopics for camera, and publish them to specified redis keys on the redis-server.

The camera ROS nodes specific to the camera publish images to ROS topics. The `RosRedisPublisher` subscribes to topics specified in the `ros_sensors.yaml` config file, and pushes them to the redis-server database on a callback. The `RosCameraInterface` in the main perls2 environment will query these keys whenever the `frames()` function is called. 

#### Set up for real cameras. 

1. Connect cameras to the workstation. 

2. Download and install the roslaunch file to ros workspace: 
	
	For the Kinect2: Use the [iai_kienct2/kinect2_bridge](https://github.com/code-iai/iai_kinect2)    
	
	[For Intel RealSense cameras](https://github.com/IntelRealSense/realsense-ros.git)
		* For real sense cameras you will have to edit a config with the serial numbers for your camera. This config file will also specify the rostopic  your camera will publish to. 

3. Install redis

	```
	sudo apt-get install redis-server
	```

4. Edit the cfg/ros_sensors.yaml file with the rostopics your camera will publish to. The RosRedisPublisher will use this to publish images to redis. For example: 

```
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
```

5. Create a new python2.7 virtual environment and install perls2 to it. 
```
virtualenv -p python2.7 p27env
source ~/p27env/bin/activate
cd ~/perls2
pip install -e . 
```

As shown above, you can add multiple topics for several camera streams. 

#### Running real cameras: 

**Terminal 1: roscore**
	1. Open a new terminal and start the roscore. 
	```
	source /opt/ros/kinetic/setup.sh; roscore
	```

**Terminal 2: redis-server**
	1. Open a new terminal and start a locally-hosted redis server. 
	```
		redis-server
	```

**Terminal 3: Camera ROS Nodes**
	1. Open a new terminal and cd into the ros or catkin workspace. 
	```
		cd ros_ws; source /opt/ros/kinetic/setup.sh;
	```
		
	2. Launch the camera nodes specific to the camera using the launch file. 

		```
			roslaunch <launch_package> <launch_file>
		```

		For the kinect2:
		
		``` 
			roslaunch kinect2_bridge kinect2_bridge.launch
		```

		For real sense cameras:
		```
		roslaunch rs_multiple_devices.launch
		```

**Terminal 4: RosRedisPublisher**
	1. Open a new terminal and source a python2.7 virtual environment
	```
	source ~/p27env/bin/activate
	```
	2. Run the RosRedisPublisher
	```
	python perls2/ros_interfaces/ros_redis_pub.py
	```

#### Getting images from real cameras
To initialize a real camera in the main perls2.Env, pass the name of the camera as an argument. 

The 'name' of the camera is a unique string found at the beginning of the rostopic channel string. This is specified in your launch file, and you can check it using `rostopic list` after launching the camera with `roslaunch`. 

For example, in our set up an RealSense SR300 publishes to rostopic channels:
```
/sr300/color/image_raw
/sr300/depth/image_rect_raw
```

To make a RosCameraInterface that will provide images from this camera: 
```
camera_interface = RosCameraInterface('sr300')

# Get RGB, depth frames
frames = camera_interface.frames()
rgb = frames['rgb']
depth = frames['depth']
```

The name can be set to any unique string provided it matches the rostopic the camera is publishing to. 
