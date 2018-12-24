#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import numpy as np
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.traffic_light_waypoints = []
        self.waypoints_2d = None
        self.waypoints_kd = None
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

        if self.waypoints_2d is None:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
            self.waypoints_kd = KDTree(self.waypoints_2d)

            # List of positions that correspond to the line to stop in front of for a given intersection
            stop_line_positions = self.config['stop_line_positions']

            i = 0
            for line in stop_line_positions:
                self.traffic_light_waypoints.append([i, self.get_closest_waypoint(line[0], line[1])])
                i = i + 1

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
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
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
#        rospy.loginfo("Curr Light_wp: %d", light_wp)
#        rospy.loginfo("Last Light_wp: %d", self.last_wp)
#        rospy.loginfo("Count: %d", self.state_count)
#        rospy.loginfo("global state: true" if self.state == True else "global state: false")
#        rospy.loginfo("state: true" if state == True else "state: false")

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if self.waypoints is None:
            rospy.logerr("No Waypoints provided to tl_detector.py")
            return 0
        else:
            idx = self.waypoints_kd.query([x,y], 1)[1]
            closest_wp = np.array(self.waypoints_2d[idx])
            prev_wp    = np.array(self.waypoints_2d[idx-1])
            v_dir_vect = closest_wp - prev_wp
            v_dist_vect= [x,y] - closest_wp
            if np.dot(v_dir_vect, v_dist_vect) > 0:
                    idx = (idx+1) % len(self.waypoints_2d)
            return idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        Is_Site = self.config['is_site']
        if Is_Site == True:
        	if(not self.has_image):
	            self.prev_light_loc = None
	            return False

	        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

	        #Get classification
        	return self.light_classifier.get_classification(cv_image)
  	else:
		return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

        for traffic_light_wp in self.traffic_light_waypoints:
#           rospy.loginfo("Traffic Waypoiny: %d", traffic_light_wp[1])
#           rospy.loginfo("Car position: %d", car_position)
            if traffic_light_wp[1] > car_position:
                light = True
                light_id = traffic_light_wp[0]
                light_wp = traffic_light_wp[1]
                break

        if light:
            state = self.get_light_state(self.lights[light_id])
            return light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
