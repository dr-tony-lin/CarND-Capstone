#!/usr/bin/env python
import rospy
import numpy as np
import tf
import cv2
import yaml
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, TrafficLightStatus
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from waypoint_util import Waypoints

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        
        self.loglevel = rospy.get_param('/loglevel', 3)
        self.state_count_threshold = rospy.get_param('~state_count_threshold', 3)
        self.pose = None
        self.car_wpidx = None
        self.waypoints = None
        self.traffic_light_waypoints = []
        self.camera_image = None
        self.lights = []

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

        config_string = rospy.get_param("/traffic_light_config")
        
        self.traffic_light_lookahead_wps = rospy.get_param('/traffic_light_lookahead_wps', 50)
        self.traffic_light_over_waypoints = rospy.get_param('traffic_light_over_waypoints', 5)
        self.traffic_light_detection_interval = rospy.get_param('~traffic_light_detection_interval', 0.05)
        
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLightStatus, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.last_msg = None
        self.state_count = 0
        self.last_detection_time = -1

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.waypoints is None:
            self.waypoints = Waypoints(waypoints.waypoints)
            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']

            i = 0
            for line in stop_line_positions:
                self.traffic_light_waypoints.append([i, self.waypoints.find_closest_waypoint([line[0], line[1]])])
                i = i + 1

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.waypoints is None:
            return
        if time.time() - self.last_detection_time < 0.2:
            return
        self.last_detection_time = time.time()
        if self.loglevel >= 4:
            rospy.loginfo("Got camera image")
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= self.state_count_threshold:
            self.last_state = self.state
            self.last_wp = light_wp
            self.last_msg = state_msg = TrafficLightStatus()
            state_msg.tlwpidx = light_wp
            state_msg.state = state
            self.upcoming_red_light_pub.publish(state_msg)
        elif self.last_msg:
            if self.car_wpidx < self.last_msg.tlwpidx + self.traffic_light_over_waypoints:
                self.upcoming_red_light_pub.publish(self.last_msg)
            else:
                self.last_msg.tlwpidx = -1
                self.last_msg.state = TrafficLight.UNKNOWN
                self.upcoming_red_light_pub.publish(self.last_msg)
                self.last_msg = None

        self.state_count += 1
        if self.loglevel >= 5:
            rospy.logdebug("Curr Light_wp: %d, state: %d, global state: %d, last Light_wp: %d, state count: %d", light_wp, state, self.state, self.last_wp, self.state_count)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        Is_Site = True #self.config['is_site']
        if Is_Site == True:
        	if (not self.has_image):
	            self.prev_light_loc = None
	            return False
	        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
	        #Get classification
        	return self.light_classifier.get_classification(cv_image)
        else:
            return light.state, None, None

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        if(self.pose):
            self.car_wpidx = self.waypoints.find_closest_waypoint([self.pose.pose.position.x, self.pose.pose.position.y])

        for traffic_light_wp in self.traffic_light_waypoints:
#           rospy.loginfo("Traffic Waypoiny: %d", traffic_light_wp[1])
#           rospy.loginfo("Car position: %d", self.car_wpidx)
            if traffic_light_wp[1] > self.car_wpidx:
                light = True
                light_id = traffic_light_wp[0]
                light_wp = traffic_light_wp[1]
                break

        if light and light_wp - self.car_wpidx < self.traffic_light_lookahead_wps:
            state, max_output_score, _ = self.get_light_state(self.lights[light_id])
            if self.loglevel >= 4:
                rospy.loginfo("Traffic state: %d, score: %f", state, max_output_score)
            return light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
