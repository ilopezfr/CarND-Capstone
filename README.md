# Project: Capstone - Driving Carla - Team Speed

In this project we worked together as a team.

| Team member           | Email                      | GitHub repo                                                                                        |
|-----------------------|----------------------------|----------------------------------------------------------------------------------------------------|
| Thomas Henckel (lead) | thomas.henckel51@gmail.com | [https://github.com/ThomasHenckel/CarND-Capstone](https://github.com/ThomasHenckel/CarND-Capstone) |
| Andre Strobel         | andre.strobel@cyberams.de  | [https://github.com/CyberAMS/CarND-Capstone](https://github.com/CyberAMS/CarND-Capstone)           |
| Matt Spinola          | matt.spinola@gmail.com     |                                                                                                    |
| Ignacio López-Francos | iglopezfrancos@gmail.com   | [https://github.com/ilopezfr/CarND-Capstone](https://github.com/ilopezfr/CarND-Capstone)           |

The goal of this project is to program the [Udacity Carla vehicle](docu_images/190303_StAn_Udacity_Carla_01.jpg) so it can detect and respect traffic lights in real time when driving in a simulator environment and a real environment.

The Udacity Carla vehicle provides a [ROS](http://www.ROS.org/) interface and uses the [ADAS](https://en.wikipedia.org/wiki/Advanced_driver-assistance_systems) kit [dbw_mkz_ros](https://bitbucket.org/DataspeedInc/dbw_mkz_ros) from [Dataspeed Inc](https://www.dataspeedinc.com/).

The connection of the different ROS nodes follows the descriptions provided in the [Udacity Self-Driving Car Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) (2 term course). The basic logic of the traffic light classification uses [Convolutional Neutral Networks (CNN)](https://en.wikipedia.org/wiki/Convolutional_neural_network) and follows [Mathias Köhnke's solution](https://github.com/mkoehnke/CarND-Capstone). A high level overview of [TensorFlow’s Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) is provided by [Daniel Stang](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e). Additional traffic light images were gathered from [LISA](https://www.kaggle.com/mbornoe/lisa-traffic-light-dataset) and [Bosch Small Traffic Light Dataset](https://hci.iwr.uni-heidelberg.de/node/6132/).

The following table shows an overview of the most important files:

| File | Description |
|------|-------------|
| README.md | This file |
| data/ | Folder containing waypoint information (maps), camera calibration data as well as training and test images |
| ros/launch/styx.launch | Launch script for simulator environment |
| ros/launch/site.launch | Launch script for real environment |
| src/styx/ | Folder containing scripts to connect ROS to the simulator by providing input/output ROS topics |
| src/tl_detector/tl_detector.py | Traffic light detection script (uses src/tl_detector/light_classification/tl_classifier.py) |
| src/tl_detector/light_classification/tl_classifier.py | Traffic light classification script (leverages TensorFlow CNN model) |
| src/tl_detector/light_classification/model/frozen_inference_graph.pb | TensorFlow CNN model to classify traffic lights in images |
| src/twist_controller/dbw_node.py | Drive-by-wire node that handles all related ROS topics |
| src/twist_controller/dbw_test.py | Script that records drive-by-wire data |
| src/twist_controller/twist_controller.py | Script that determines throttle and braking |
| src/twist_controller/yaw_controller.py | Script that determines steering |
| src/waypoint_follower/ | Folder containing scripts to determine drive-by-wire signals to follow the desired waypoints |
| src/waypoint_loader/ | Folder containing scripts to load the necessary waypoints (map) |
| src/waypoint_updater/waypoint_updater.py | Waypoint node that listens to the traffic light ROS topic and determins the desired speed at each waypoint ahead |
| Traffic_Light_Detection/ | Folder containing everything related to training the TensorFlow CNN model for traffic light classification |

---

## Content

1. Tool chain setup
    1. Considerations for the best simulation and test environment
    1. Ubuntu, ROS and necessary packages
    1. Udacity Simulator
1. ROS nodes and topics
    1. Communication overview
    1. ROS node waypoint_updater.py
    1. ROS node twist_controller.py
    1. ROS node tl_detector.py
    1. TLClassifier object
1. Traffic light detection
1. Execution
    1. Commands to start the simulation
    1. Simulation results
    1. Commands to test the code in the real environment
    1. Test results
1. Discussion
1. Known issues and possible improvements
1. Changes after first project submission
1. Potential future improvements

[//]: # (Image References)

[image1]: ./docu_images/190303_StAn_rosgraph.png

---

## 1. Tool chain setup

### 1. Considerations for the best simulation and test environment

The following hardware and software was available during the development.

| Description |
|-------------|
| Intel i7-8700K Coffee Lake (6-Core, 12-Thread, 3.7 GHz, 12MB Cache, 95W, CPU Benchmark 16822) |
| Asus ROG Strix Z370G |
| 64GB Corsair DDR4 3000 CL15 |
| Gigabyte Quiet GTX 1060 6GB - 1280 CUDA Cores |
| 2x Samsung 960 EVO 1TB M.2 NVMe PCIe 3 x 4 SSD (3200/1900 MBps R/W) |
| 20TB Synology NAS with 2x 1 GB/s Ethernet |
| Windows 10 and Ubuntu 16.04 dual boot |
| VMware Workstation 15 Pro (with own Ubuntu virtual machine) |
| VirtualBox 6.0 (with Udacity provided Lubuntu virtual machine) |
| Udacity Workspace |

This project requires a host for the Udacity Simulator and a host for the Udacity Carla ROS environment. The following options have been tested in conjunction with each other.

| Number | Udacity Simulator | Findings |
|--------|-------------------|----------|
| 1 | Udacity Workspace VNC | Runs slow when camera sends images |
| 2 | Windows with GPU | Runs slow when camera sends images to virtualized ROS environment |
| 3 | Windows / VMware | Runs very slow when camera sends images |
| 4 | Windows / VirtualBox | Runs extremely slow when camera sends images |
| 5 | Ubuntu w/o GPU | Runs slow when camera sends images |
| 6 | Ubuntu with GPU | Runs perfect |

| Udacity Carla ROS environment | Combinations | Findings |
|-------------------------------|--------------|----------|
| Udacity Workspace Terminal | 1 | Some effort to install all packages correctly. Runs too slow due to Udacity Simulator in Udacity Workspace VNC. |
| Windows / VMware | 2, 3 | Very difficult to install all packages correctly. Runs too slow due to slow simulator options. |
| Windows / VMware / Docker | 2, 3 | Some effort to install all packages correctly. Runs too slow due to slow simulator options. |
| Windows / VirtualBox | 2, 4 | Difficult to install all packages correctly. Runs too slow due to slow simulator options. |
| Windows / VirtualBox / Docker | 2, 4 | Some effort to install all packages correctly. Runs too slow due to slow simulator options. |
| Ubuntu with CPU | 5, 6 | Very difficult to install all packages correctly. Runs too slow with option 5 and runs perfect with option 6. |
| Ubuntu with CPU / Docker | 5, 6 | Was not tested as it wasn't needed. |
| Ubuntu with CUDA | 5, 6 | Could not test due to incompatibility of Ubuntu 16.04 with CUDA 8.0 and NVIDIA GTX 1060. |
| Ubuntu with CUDA / Docker | 5, 6 | Could not test due to incompatibility of Ubuntu 16.04 with CUDA 8.0 and NVIDIA GTX 1060. |

The above evaluations led to the only viable solution of running everything native on a dual boot Ubuntu installation - using Ubuntu with GPU for the Udacity Simulator and Ubuntu with CPU for the Udacity Carla ROS environment. This setup is also very close to the actual setup in the Udacity Carla vehicle.

### 2. Ubuntu, ROS and necessary packages

To follow this project exactly you first need to install [Ubuntu 16.04 Xenial Xerus](https://www.ubuntu.com/download/desktop). Next you have to install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

```console
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Then you need to install the Dataspeed ADAS drive-by-wire kit [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default).

```console
sudo apt-get update
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
<make sure you are in the ros subdirectory of this repository>
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
```

After downloading this repository you need to make sure that you have all the necessary packages installed.

```console
git pull https://github.com/CyberAMS/CarND-Capstone
pip install -r requirements.txt
sudo apt-get install python-termcolor
```

### 3. Udacity Simulator

Finally, you need to download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

## 2. ROS nodes and topics

### 1. Communication overview

The Udacity Carla ROS environment for this project uses the following nodes and topics.

![alt text][image1]

The `/waypoint_loader` reads the map or trajectory information and publishes a list of waypoints that the vehicle can follow safely. The `/tl_detector` node takes this information and the camera image from the vehicle - either simulation or real - and publishes the state of the traffic light ahead (`GREEN`, `YELLOW`, `RED` or `UNKNOWN`). It also publishes the topic `/debug/bounding_box_img` (not shown) in which the most relevant detected traffic light is marked up with a bounding box. The above diagram shows the case for the simulated environment in which the `/styx_server` node publishes all the relevant vehicle state and sensor information as well as receives the necessary control signals for steering (not shown), throttle (not shown) and braking. The `/waypoint_updater` node determines the desired speed for the waypoints ahead. The `/pure_pursuit` node calculates the drive-by-wire commands for the `dbw_node` node based on the information about the waypoints ahead.

### 2. ROS node waypoint_updater.py

The `/waypoint_updater` node constantly looks for the next `LOOKAHEAD_WPS = 100` waypoints. If the next traffic light is either `GREEN` or `UNKNOWN` (`self.stopline_wp_idx == -1`), it will not change the desired speed at the waypoints ahead. If the next traffic light is either `RED` or `YELLOW`, it will use decelerating speed for the waypoints ahead. It is important to mention that the vehicle might detect a `RED` or `YELLOW` traffic light, but will only react to it if the stop line of this traffic light is within distance of the next `LOOKAHEAD_WPS` waypoints.

```python
def generate_lane(self):
	lane = Lane()
	closest_idx = self.get_closest_waypoint_id()
	farthest_idx = closest_idx + LOOKAHEAD_WPS
	base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
	if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
		lane.waypoints = base_waypoints
	else:
		lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
	return lane
```

The decelerated speed is calculated as a minimum of the desired speed at the waypoints ahead and a square root function with a given `MAX_DECEL` deceleration that starts backwards with zero speed at the stop line of the traffic light ahead.

```python
def decelerate_waypoints(self, waypoints, closest_idx):
	temp = []
	for i, wp in enumerate(waypoints):
		p = Waypoint()
		p.pose = wp.pose
		stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # the waypoints are meassured from center so substract 2 to have the front of the car stop at the light
		dist = self.distance(waypoints, i, stop_idx)
		vel = math.sqrt(2 * MAX_DECEL * dist)
		if vel < 1.0:
			vel = 0.0
		p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
		temp.append(p)
	return temp
```

### 3. ROS node twist_controller.py

The `/twist_controller` node uses a yaw controller and throttle controller to determine steering, throttle and braking to keep the desired direction and speed. If the speed gets close to zero, the throttle is set to zero and the brake is engaged to hold the vehicle in place (`brake = 700` Nm is enough for the real Udacity Carla vehicle).

The throttle controller is implemented as PID controller. In order to steer smoothly back when returning from manual driving mode to automatic driving mode, the parameter `ki = 0.001` has been set to a really low value. The parameter `mx = 0.3` defines the maximum throttle and hence indirectly also the maximum speed the vehicle can achieve.

### 4. ROS node tl_detector.py

The `/tl_detector` node constantly determines whether or not an incoming image needs to be processed. Only every `LIGHT_PROCESS_THRESHOLD`-th image will be processed. The detected traffic light state is only valid if it has been detected `STATE_COUNT_THRESHOLD` times in a row. The function either returns the waypoint index of the stop line when the traffic light is `RED` or `YELLOW` or it returns `-1` if no traffic light stop state has been detected.

```python
LIGHT_PROCESS_THRESHOLD = 4
STATE_COUNT_THRESHOLD = 3 / LIGHT_PROCESS_THRESHOLD # if we skip images we cannot wait until we see the same light state as often

def image_cb(self, msg):
	self.camera_image = msg
	if ((self.traffic_count % LIGHT_PROCESS_THRESHOLD) == 0):
		# traffic light must be processed
		light_wp, state = self.process_traffic_lights()
	else:
		light_wp = self.last_wp # use previous value
		state = self.last_state # use previous value
	self.traffic_count += 1
	if (self.state != state):
		self.state_count = 0
		self.state = state
	if (self.state_count >= STATE_COUNT_THRESHOLD):
		self.last_state = self.state
		light_wp = light_wp if ((state == TrafficLight.RED) or (state == TrafficLight.YELLOW)) else -1
		self.last_wp = light_wp
		self.upcoming_red_light_pub.publish(Int32(light_wp))
	else:
		self.upcoming_red_light_pub.publish(Int32(self.last_wp))
	self.state_count += 1
```

The following code is used to display debugging information about the actually correct and the detected traffic light states. It works very well if `python-termcolor` has been installed as mentioned in the above section.

```python
classified_state = self.get_light_state(closest_light)
if bDEBUG and (classified_state != 4):
	print("----------------------------------------------------------------------")
	correct_state_str = self.state_to_string("Correct light state    ", closest_light.state)
	detected_state_str = self.state_to_string("Detected light state   ", classified_state)
	print("car_wp_idx: " + str(car_wp_idx) + " stop line position idx: " + str(line_wp_idx))
return line_wp_idx, classified_state
```

### 5. TLClassifier object

The `TLClassifier` class in `tl_classifier.py` offers the method `get_classification` that calls TensorFlow to run the inference for a single image with the method `run_inference_for_single_image`. It determines the traffic light with the largest bounding box and uses it to detect the state. At the end it publishes the image with the marked up bounding box.

```python
def get_classification(self, image):	
	image_np = self.load_image_into_numpy_array(image)
	output_dict = self.run_inference_for_single_image(image_np, self.detection_graph)
	text_string = "Classified light (idx {0}) state : {1} with probability {2}"
	if (output_dict['detection_scores'][0] > 0.5):
		text_string += " > 0.5"
	else:
		text_string += " <= 0.5"
	max_box_idx = -1
	max_box_size = -1.0
	for i in range(output_dict['num_detections']/2):
		if (output_dict['detection_scores'][i] > 0.5):
			x = output_dict['detection_boxes'][i][3] - output_dict['detection_boxes'][i][1]
			y = output_dict['detection_boxes'][i][2] - output_dict['detection_boxes'][i][0]
			box_size = math.sqrt((x * x) + (y * y))
			print("i: " + str(i) + " score: " + str(output_dict['detection_scores'][i]) + " " + self.state_to_string(output_dict['detection_classes'][i]) + " box_size: " + str(box_size) + " " + str(output_dict['detection_boxes'][i]))
			if (box_size > max_box_size):
				max_box_idx = i
				max_box_size = box_size
	ret_val = TrafficLight.UNKNOWN
	if ((max_box_idx >= 0) and (output_dict['detection_scores'][max_box_idx] > 0.5)):
		print(text_string.format(max_box_idx, output_dict['detection_classes'][max_box_idx], output_dict['detection_scores'][max_box_idx]))
		image = self.add_bounding_box_to_image(image, output_dict['detection_boxes'][max_box_idx])
		if (output_dict['detection_classes'][max_box_idx] == 1 ):
			ret_val = TrafficLight.GREEN
		elif (output_dict['detection_classes'][max_box_idx] == 2 ):
			ret_val = TrafficLight.YELLOW
		elif (output_dict['detection_classes'][max_box_idx] == 3 ):
			ret_val = TrafficLight.RED
	img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")
	self.bounding_box_img_pubs.publish(img_msg)
	return ret_val
```

```python
def run_inference_for_single_image(self,image, graph):
	with graph.as_default():
		with tf.Session() as sess:
			ops = tf.get_default_graph().get_operations()
			all_tensor_names = {output.name for op in ops for output in op.outputs}
			tensor_dict = {}
			for key in ['num_detections', 'detection_boxes', 'detection_scores', 'detection_classes', 'detection_masks']:
				tensor_name = key + ':0'
				if tensor_name in all_tensor_names:
					tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
			image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
			output_dict = sess.run(tensor_dict, feed_dict={image_tensor: np.expand_dims(image, 0)})
			output_dict['num_detections'] = int(output_dict['num_detections'][0])
			output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
			output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
			output_dict['detection_scores'] = output_dict['detection_scores'][0]
	return output_dict
```

## 3. Traffic light detection

The traffic light detection is using the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). It is based on the [SSD MobileNet model](https://github.com/tensorflow/models/tree/master/research/object_detection/models) which has been pre-trained on the [COCO dataset](http://cocodataset.org/). Additional traffic light data has been used to fine-tune the model. The model is able to detect traffic lights and classify their color into `GREEN`, `YELLOW` and `RED`.

The additional traffic light data consists of a combination of three datasets from the Udacity Simulator and the Udacity Carla vehicle on the test site. The combination of these three datasets adds up to a total of 2613 images. The model achieves a good balance between accuracy and fast running time. The benefit of this approach is that the same model can be used for the simulator environment and the real environment.

The detailed [description](./Traffic_Light_Detection/README.md) of the traffic light detection model is located in a separate [folder](./Traffic_Light_Detection/).

## 4. Execution

### 1. Commands to start the simulation

```console
<make sure you are in the ros subdirectory of this repository>
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

Start the Udacity Simulator after the Udacity Carla ROS environment of this project is waiting. Then check `Camera` and uncheck `Manual`. The car will follow the center of the lane at the desired speed and stop at the stop line of upcoming traffic lights if the traffic light state is `RED` or `YELLOW`.

### 2. Simulation results

Here is an example of how the car accelerates and stops as required all by itself. On the right you see the debugging information and camera image. The traffic light which is used to determine `RED`, `YELLOW` or `GREEN` is marked up with a bounding box.

<img src="https://raw.githubusercontent.com/CyberAMS/CarND-Capstone/master/docu_images/190309_StAn_CAP_simulator_smallest.gif" width="100%">

### 3. Commands to test the code in the real environment

Download the [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) and [test bags](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view) that were recorded on the Udacity Carla vehicle. Extract the contents.

```console
unzip <your zipped bag files>.zip
```

Start the Udacity Carla ROS environment of this project with the `site.launch` file.

```console
<make sure you are in the ros subdirectory of this repository>
catkin_make
source devel/setup.sh
roslaunch launch/site.launch
```

Open another terminal and open an image viewer to see the camera image.

```console
rosrun rqt_image_view rqt_image_view topic:=/debug/bounding_box_img
```

Open another terminal and run the ROS bag.

```console
rosbag play -l <your bag file>.bag
```

### 4. Test results

The following ROS bags from the Udacity Carla vehicle have been tested with the implemented solution.

The first ROS bag contains a back and forth maneuver around the traffic light. The state of the traffic light is always classified correctly. There is only a short instance at the end when the car stops and the traffic light is `GREEN`. In this case no traffic light is detected with high probability at all. As we only react to `RED` and `YELLOW` traffic lights, this is not a big issue for the given task.

<img src="https://raw.githubusercontent.com/CyberAMS/CarND-Capstone/master/docu_images/190309_StAn_CAP_just_smallest.gif" width="100%">

The second ROS bag contains several full loops around the test site. The state of the traffic light is always classified correctly. There is one repeatable false positive during the loop. A white wall is classified as `GREEN` traffic light. As we only react to `RED` and `YELLOW` traffic lights, this is not a big issue for the given task.

<img src="https://raw.githubusercontent.com/CyberAMS/CarND-Capstone/master/docu_images/190309_StAn_CAP_loop_smallest.gif" width="100%">

The third ROS bag contains several approaches on the traffic light. The state of the traffic light is mostly classified correctly. There are only a few instances where `RED` and `YELLOW` are mixed up for a very short amount of time. As we react to `RED` and `YELLOW` traffic lights in the same way, this is not a big issue for the given task. Also, when the traffic light is very close, it sometimes is not detected at all. As we stop further away from the traffic light, this is not a big issue for the given task.

<img src="https://raw.githubusercontent.com/CyberAMS/CarND-Capstone/master/docu_images/190309_StAn_CAP_train_smallest_short.gif" width="100%">

## 5. Discussion

The first challenge in this project is to set up a pipeline of hardware and software that fulfills the needs of the Udacity Simulator and is compatible with the setup in the Udacity Carla vehicle. The major bottle neck was the Udacity Simulator itself that absolutely requires correctly configured use of the GPU to send the camera images fluently.

The second challenge is to find an approach to detect traffic lights in front of the vehicle as well as their state with the available hardware and software in the most efficient way. We found a way to convert more recent TensorFlow models based on the TensorFlow Object Detection API back to TensorFlow 1.3. We also optimized how many images really need to be processed, i.e. we don't analyze the complete camera stream.

## 6. Known issues and possible improvements

As the provided stop line coordinates for the traffic lights in the Udacity Simulator are not exact and additional inaccuracy is added by selecting a waypoint close to them, the vehicle sometimes overshoots the stop line.

The output to the log file is not necessarily purged in strict order of the individual ROS node calls on all systems.

## 7. Changes after first project submission

1. Changed the sequence of debugging and print statements to finish each `process_traffic_lights` call in `tl_detector` with a series of dashes.
1. Changed the sequence of commands inside the `__init__` method of the `tl_detector` node to ensure all variables are defined and initialized before any callbacks to subscribed topics are called.
1. Added the `/tld_enabled` topic that is published after the `__init__` method of the `tl_detector` node is finished and every time `process_traffic_lights` in `tl_detector` is finished. This topic is read by the `dbw_node` node and sets the `tld_enabled` flag from `False` to `True`. The `tld_enabled` flag is forwarded to the `control` method of the `controller` object inside the `twist_controller` file. The `controller` does not apply throttle and fully applies the brake until `tld_enabled` is `True`.
1. Retrained the traffic light classifier model with augmented images.
1. Reduced the traffic light detection probability threshold in `tl_classifier` from 0.5 to 0.35 and made it a parameter `TL_PROB_THRESHOLD`.
1. Added a semaphore `self.in_process` to `tl_detector` to prevent calling the traffic light classifier again before the previous call finished.
1. Changed `twist_controller.py` to apply full brake at speed below 0.5 (instead of 0.1). Also doubled normal brake torque and limited it to 700.
1. Changed `waypoint_updater.py` to only call `publish_waypoints` if all variables are initialized
1. Increased offset distance from stop line from 2 to 3 meters in `decelerate_waypoints` in `waypoint_updater`.

## 8. Potential future improvements

1. Reduce the `LIGHT_PROCESS_THRESHOLD` (e.g. from 4 to 2) and `STATE_COUNT_THRESHOLD` (from 3 to 1) parameters in `tl_detector` to account for improved hardware resources and process more camera images, i.e. more frequently.
1. Accelerate quicker to respond faster to traffic light state changes. Reset `ki` to zero in `twist_controller.py` when going from brake to throttle. Also use a higher `kp` value - maybe 0.5 - and `mx` value - maybe 0.5.