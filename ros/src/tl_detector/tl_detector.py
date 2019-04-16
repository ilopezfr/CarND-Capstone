#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from scipy.spatial import KDTree

LIGHT_PROCESS_THRESHOLD = 4
STATE_COUNT_THRESHOLD = 3 / LIGHT_PROCESS_THRESHOLD # if we skip images we cannot wait until we see the same light state as often
bDEBUG = True

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.waypoint_tree = None
        self.waypoints_2d = None
        self.camera_image = None
        self.lights = []
        self.in_process = False

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.traffic_count = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.tld_enabled_pub = rospy.Publisher('/tld_enabled', Bool, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
		
        rospy.spin()

        # send initialization done message
        self.tld_enabled_pub.publish(True)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.logwarn("Image callback :")
        self.has_image = True
        self.camera_image = msg
        if (((self.traffic_count % LIGHT_PROCESS_THRESHOLD) == 0) and not (self.in_process)):
            # traffic light must be processed
            #rospy.logwarn("Processing traffic light image")
            light_wp, state = self.process_traffic_lights()
        else:
            #rospy.logwarn("Skipping processing traffic light image.")
            light_wp = self.last_wp # use previous value
            state = self.last_state # use previous value
        if not (self.in_process):
            self.traffic_count += 1

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state

        if self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if ((state == TrafficLight.RED) or (state == TrafficLight.YELLOW)) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def dist_to_point(self, position, wp_position):
        x_sq = pow((position.x - wp_position.x), 2)
        y_sq = pow((position.y - wp_position.y), 2)
        dist = math.sqrt(x_sq + y_sq)
        return dist

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        min_dist = 999999
        closest_wp_idx = -1
        
        if not self.waypoints:
            rospy.logwarn("[TL_DETECTOR] No waypoints given.")
        else:
            # check all the waypoints to see which one is the closest to our current position
            for i, wp in enumerate(self.waypoints.waypoints):
                dist = self.dist_to_point(pose, wp.pose.pose.position)
                if (dist < min_dist):  
                    closest_wp_idx = i
                    min_dist = dist
                    
            return closest_wp_idx 

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # While Testing return simulator light state
        #return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def state_to_string(self, str, state):
        """ Returns the color light associated with the state """
        if (state == 2 ):
            rospy.logdebug(str + ": GREEN")
        elif (state == 1 ):
            rospy.logwarn(str + " : YELLOW")
        elif (state == 0 ):
            rospy.logerr(str + " : RED")
        else:
            rospy.loginfo(str + " : UNKNOWN")
        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        self.in_process = True
        closest_light = None
        line_wp_idx = None
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # get stop line waypoint index
                stop_line_pose = Pose()
                stop_line_pose.position.x = stop_line_positions[i][0]
                stop_line_pose.position.y = stop_line_positions[i][1]
                temp_wp_idx = self.get_closest_waypoint(stop_line_pose.position)
                # Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            classified_state = self.get_light_state(closest_light)
            if bDEBUG and (classified_state != 4):
                correct_state_str = self.state_to_string("Correct light state    ", closest_light.state)
                detected_state_str = self.state_to_string("Detected light state   ", classified_state)
                # rospy.logwarn("car_wp_idx: " + str(car_wp_idx) + " stop line position idx: " + str(line_wp_idx))
                print("car_wp_idx: " + str(car_wp_idx) + " stop line position idx: " + str(line_wp_idx))
            # rospy.logwarn("----------------------------------------------------------------------")
            print("----------------------------------------------------------------------")
            self.tld_enabled_pub.publish(True)
            self.in_process = False
            return line_wp_idx, classified_state
        self.in_process = False
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
