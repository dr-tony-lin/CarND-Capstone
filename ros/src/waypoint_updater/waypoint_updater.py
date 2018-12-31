#!/usr/bin/env python

import numpy as np
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightStatus
from waypoint_util import Waypoints

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.loglevel = rospy.get_param('/loglevel', 3)
        self.lookahead_wps = rospy.get_param('~lookahead_wps', 40)
        self.waypoint_update_frequency = rospy.get_param('~waypoint_update_frequency', 50)
        self.yellow_light_full_speed_distance = rospy.get_param('~yellow_light_full_speed_distance', 30)
        self.traffic_light_full_stop_distance = rospy.get_param('~traffic_light_stop_distance', 5)
        self.traffic_light_lookahead_wps = rospy.get_param('/traffic_light_lookahead_wps', 50)
        self.yellow_light_decel_ratio = rospy.get_param('~yellow_light_decel_ratio', 0.6)
        self.acceleration_start_velocity = rospy.get_param('~acceleration_start_velocity', 5)
        self.acceleration_distance = rospy.get_param('~acceleration_distance', 15)
        self.max_deceleration = -rospy.get_param('/decel_limit', -8.5) * 0.55
        self.max_acceleration = -rospy.get_param('/accel_limit', 2.0)

        rospy.loginfo("Log level: %d, max deceleration: %f", self.loglevel, self.max_deceleration)
        self.pose = None
        self.last_pose_time = None
        self.velocity = None
        self.waypoints_header = None
        self.traffic_light_status = None
        self.waypoints = None
        self.last_stop_wp = None
        self.last_lane = None
        self.last_wp = None
        self.start_decel_wp = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', TrafficLightStatus, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(self.waypoint_update_frequency)
        while not rospy.is_shutdown():
            if self.pose and self.waypoints:
                self.last_wp = self.waypoints.find_closest_waypoint([self.pose.position.x, self.pose.position.y])
                if self.velocity is not None and self.last_pose_time is not None :
                    dt = time.time() - self.last_pose_time + 0.1 # assume 0.l second latency
                    if dt > 2.0 / self.waypoint_update_frequency:
                        last = self.last_wp
                        # Estimate the distance that the vehicle has moved
                        d = (self.velocity * 0.3 + self.last_lane.waypoints[-1].twist.twist.linear.x * 0.7) * dt if self.last_lane else self.velocity * dt
                        self.last_wp += int(d / self.waypoints.average_waypoint_length + 0.5) # try to catch up the pose, round to the next integer
                        if self.loglevel >= 4:
                            rospy.loginfo("Catch pose %d -> %d, dist: %f", last, self.last_wp, d)
                self.publish_waypoints(self.last_wp, self.lookahead_wps)
            rate.sleep()
    
    def publish_waypoints(self, idx, pts):
        self.last_lane = self.generate_lane(idx, pts)
        self.final_waypoints_pub.publish(self.last_lane)
        
    def generate_lane(self, idx, pts):
        lane = Lane()
        lane.header = self.waypoints_header
        lookahead = idx + self.traffic_light_lookahead_wps
        if self.traffic_light_status and self.traffic_light_status.tlwpidx >= 0 \
                and self.traffic_light_status.state in (TrafficLight.RED, TrafficLight.YELLOW) \
                and self.waypoints.before(idx, self.traffic_light_status.tlwpidx) \
                and self.waypoints.before(self.traffic_light_status.tlwpidx, lookahead):
            lane.waypoints = self.decelerate_waypoints(idx, pts)
        elif self.last_stop_wp and self.waypoints.distance(self.last_stop_wp, idx) <= self.acceleration_distance:
            lane.waypoints = self.accelerate_waypoints(idx, pts)
            return lane
        else:
            self.start_decel_wp = None
            lane.waypoints = self.waypoints.get_waypoints(slice(idx, idx+pts))
            if self.loglevel >= 4:
                rospy.loginfo("Waypoint update %s: velocity: %f", idx, lane.waypoints[0].twist.twist.linear.x)
                rospy.loginfo("       Waypoint %s: velocity: %f", idx + pts - 1, lane.waypoints[-1].twist.twist.linear.x)
        if self.velocity < 0.1:
            self.last_stop_wp = self.last_wp
        return lane
    
    def decelerate_waypoints(self, idx, pts):
        waypoints = []
        if self.start_decel_wp is None:
            self.start_decel_wp = idx
        stop_dist = self.waypoints.distance(self.start_decel_wp, self.traffic_light_status.tlwpidx) - self.traffic_light_full_stop_distance
        for i in range(idx, idx + pts + 1):
            wp = self.waypoints[i]
            p = Waypoint()
            p.pose = wp.pose
            dist = max(0.0, self.waypoints.distance(i, self.traffic_light_status.tlwpidx) - self.traffic_light_full_stop_distance)
            if self.traffic_light_status.state == TrafficLight.RED: # red light, stop
                v = max(self.velocity, self.max_deceleration) * dist / stop_dist if dist > 0 else 0
            elif dist > self.yellow_light_full_speed_distance: # yellow light, enough distance to stop
                v = max(self.velocity, self.max_deceleration) * self.yellow_light_decel_ratio * dist / stop_dist if dist > 0 else 0
            else: # distance to light is too short, we will keep going
                v = wp.twist.twist.linear.x
            p.twist.twist.linear.x = min(v if v >= 1 else 0, wp.twist.twist.linear.x)
            waypoints.append(p)
            if self.loglevel >= 4 and (i == idx or i == idx + pts):
                rospy.loginfo("Decelerate: %s -> %s, distance: %f, velocity: %f", i, self.traffic_light_status.tlwpidx, dist, v)
        return waypoints

    def accelerate_waypoints(self, idx, pts):
        waypoints = []
        self.start_decel_wp = None
        for i in range(idx, idx + pts + 1):
            wp = self.waypoints[i]
            p = Waypoint()
            p.pose = wp.pose
            dist = max(0.0, min(self.acceleration_distance, self.waypoints.distance(self.last_stop_wp, i)))
            v = wp.twist.twist.linear.x * dist / self.acceleration_distance
            v = v if v > self.acceleration_start_velocity else max(self.velocity, self.acceleration_start_velocity)
            p.twist.twist.linear.x = min(v, wp.twist.twist.linear.x)
            waypoints.append(p)
            if self.loglevel >= 4 and (i == idx or i == idx + pts):
                rospy.loginfo("Accelerate: %s -> %s, distance: %f, velocity: %f", i, self.last_stop_wp, self.waypoints.distance(self.last_stop_wp, i), v)
        return waypoints
        
    def pose_cb(self, msg):
        self.pose = msg.pose
        self.last_pose_time = time.time()
        if self.loglevel >= 4:
            rospy.loginfo("Pose: (%s,%s)", self.pose.position.x, self.pose.position.y)

    def velocity_cb(self, velocity):
        self.velocity = velocity.twist.linear.x
        if self.loglevel >= 4:
            rospy.loginfo("Velocity: %f", self.velocity)
        
    def waypoints_cb(self, waypoints):
        self.waypoints = Waypoints(waypoints.waypoints)
        self.waypoints_header = waypoints.header
            
    def traffic_cb(self, msg):
        self.traffic_light_status = msg
        if self.loglevel >= 4:
            rospy.loginfo("Traffic light: %d, %d", self.traffic_light_status.tlwpidx, self.traffic_light_status.state)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
