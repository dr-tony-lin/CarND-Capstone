#!/usr/bin/env python

import numpy as np
import math
import time
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightStatus, AllTrafficLights
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
        self.traffic_light_full_stop_distance = rospy.get_param('~traffic_light_stop_distance', 5)
        self.traffic_light_lookahead_wps = rospy.get_param('/traffic_light_lookahead_wps', 50)
        self.acceleration_start_velocity = rospy.get_param('~acceleration_start_velocity', 5) * 1000. / 3600.
        self.acceleration_distance = rospy.get_param('~acceleration_distance', 20)
        self.traffic_light_over_distance = rospy.get_param('~traffic_light_over_distance', 3)
        self.max_velocity_near_traffic_light = rospy.get_param('~max_velocity_near_traffic_light', 40) * 1000. / 3600.
        self.near_traffic_light_distance = rospy.get_param('~near_traffic_light_distance', 20)
        self.far_traffic_light_distance = rospy.get_param('~far_traffic_light_distance', 75)
        self.control_latency = rospy.get_param('~control_latency', 0.02)
        self.velocity_timeout = rospy.get_param('~velocity_timeout', 1000000.)
        self.max_deceleration = -rospy.get_param('/decel_limit', -8.5)
        self.max_acceleration = rospy.get_param('/accel_limit', 2.5)

        rospy.loginfo("Log level: %d, max deceleration: %f", self.loglevel, self.max_deceleration)
        self.velocity_in = None
        self.pose = None
        self.start_time = time.time()
        self.pose_time = None
        self.velocity = None
        self.velocity_time = None
        self.waypoints_header = None
        self.traffic_light_status = None
        self.waypoints = None
        self.all_traffic_lights = None
        self.start_accel_wp = None
        self.last_lane = None
        self.pose_wpidx = None
        self.can_stop = True
        self.stop_dist = None
        self.start_decel_wpidx = None
        self.ready = False

        self.config = yaml.load(rospy.get_param("/traffic_light_config"))

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
            if self.ready and self.pose and self.waypoints and self.velocity is not None:
                self.pose_wpidx = self.waypoints.find_closest_waypoint([self.pose.position.x, self.pose.position.y])
                if self.loglevel >= 4:
                    rospy.loginfo("Updating %d, current v: %f", self.pose_wpidx, self.velocity)
                # Try to estimate new pose if we missd few pose updates
                if self.velocity > 5 and self.pose_time is not None and self.last_lane is not None:
                    dt = time.time() - self.pose_time + self.control_latency
                    if dt >= 4.0 / self.waypoint_update_frequency:
                        dv = self.last_lane.waypoints[-1].twist.twist.linear.x - self.last_lane.waypoints[0].twist.twist.linear.x
                        dt2 = time.time() - self.velocity_time
                        self.velocity = self.velocity_in + dv * (self.max_acceleration if dt2 >= 0 else self.max_acceleration) # estimate the speed
                        last = self.pose_wpidx
                        # Estimate the distance that the vehicle has moved
                        d = (self.velocity * 0.3 + self.last_lane.waypoints[-1].twist.twist.linear.x * 0.7) * dt if self.last_lane else self.velocity * dt
                        self.pose_wpidx += int(d / self.waypoints.average_waypoint_length + 0.5) # try to catch up the pose, round to the next integer
                        if self.loglevel >= 4:
                            rospy.loginfo("Catch pose %d -> %d, dist: %f, v: %f", last, self.pose_wpidx, d, self.velocity)
                self.publish_waypoints(self.pose_wpidx, self.lookahead_wps)
            elif self.velocity is None and time.time() - self.start_time >= self.velocity_timeout:
                self.velocity = 0.
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
        elif self.start_accel_wp and self.waypoints.distance(self.start_accel_wp, idx) <= self.acceleration_distance:
            lane.waypoints = self.accelerate_waypoints(idx, pts)
            return lane
        else:
            dv = self.waypoints[idx].twist.twist.linear.x - self.velocity
            if dv < -self.max_deceleration * 3:
                lane.waypoints = self.decelerate_waypoints(idx, pts)
            elif dv > self.max_acceleration * 3:
                self.start_accel_wp = idx
                lane.waypoints = self.accelerate_waypoints(idx, pts)
                return lane
            else:
                self.start_decel_wpidx = None
                lane.waypoints = self.waypoints[idx:idx+pts+1]
            if self.loglevel >= 4:
                rospy.loginfo("Waypoint update %s: velocity: %f", idx, lane.waypoints[0].twist.twist.linear.x)
                rospy.loginfo("       Waypoint %s: velocity: %f", idx + pts - 1, lane.waypoints[-1].twist.twist.linear.x)
        if self.velocity < 0.1:
            self.start_accel_wp = self.pose_wpidx
        return lane
    
    def decelerate_waypoints(self, idx, pts):
        waypoints = []
        if self.start_decel_wpidx is None:
            self.start_decel_wpidx = idx
            dist = self.waypoints.distance(self.start_decel_wpidx, self.traffic_light_status.tlwpidx) + self.traffic_light_over_distance
            self.stop_dist = dist - self.traffic_light_full_stop_distance
            # Can we stop the vehicle?
            t = self.velocity / self.max_deceleration
            min_dist = max(0, self.velocity*t - 0.5*self.max_deceleration*(t**2))
            self.can_stop = self.velocity < 1 or min_dist <= dist
            if not self.can_stop: # we can not stop accelerate to pass
                self.start_accel_wp = idx
                if self.loglevel >= 4:
                    rospy.loginfo("Cannot decelerate: %s -> %s, start %d, distance: %f, velocity: %f, required dist: %f, t: %f", idx, self.traffic_light_status.tlwpidx, self.start_decel_wpidx, dist, self.velocity, min_dist, t)
        if not self.can_stop: # we can not stop, so accelerate to pass
            self.start_decel_wpidx = None
            return self.accelerate_waypoints(idx, pts)
        for i in range(idx, idx + pts + 1):
            wp = self.waypoints[i]
            p = Waypoint()
            p.pose = wp.pose
            dist = max(0.0, (self.waypoints.distance(i, self.traffic_light_status.tlwpidx) if i < self.traffic_light_status.tlwpidx else 0) - self.traffic_light_full_stop_distance)
            v = max(self.velocity, self.max_deceleration) * math.sqrt(dist / self.stop_dist) if dist > 0 else 0
            p.twist.twist.linear.x = min(v if v >= 1 else 0, wp.twist.twist.linear.x)
            waypoints.append(p)
            if self.loglevel >= 4 and (i == idx or i == idx + pts):
                rospy.loginfo("Decelerate: %s -> %s, start %d, distance: %f, velocity: %f->%f", i, self.traffic_light_status.tlwpidx, self.start_decel_wpidx, dist, v, p.twist.twist.linear.x)
        return waypoints

    def accelerate_waypoints(self, idx, pts):
        waypoints = []
        for i in range(idx, idx + pts + 1):
            wp = self.waypoints[i]
            p = Waypoint()
            p.pose = wp.pose
            dist = max(0.0, min(self.acceleration_distance, self.waypoints.distance(self.start_accel_wp, i)))
            dv = wp.twist.twist.linear.x - self.velocity
            v = max(self.acceleration_start_velocity, self.velocity + dv * dist / self.acceleration_distance)
            p.twist.twist.linear.x = min(v, wp.twist.twist.linear.x)
            waypoints.append(p)
            if self.loglevel >= 4 and (i == idx or i == idx + pts):
                rospy.loginfo("Accelerate: %s -> %s, distance: %f, velocity: %f", self.start_accel_wp, i, self.waypoints.distance(self.start_accel_wp, i), v)
        return waypoints

    def set_road_velocity(self):
        if self.waypoints and not self.ready:
            if self.loglevel >= 4:
                rospy.loginfo("Adjusting road velocity for traffic lights ...")
            stop_line_positions = self.config['stop_line_positions']
            self.all_traffic_lights = []
            for line in stop_line_positions:
                idx = self.waypoints.find_closest_waypoint([line[0], line[1]])
                self.all_traffic_lights.append(idx)
            for i in range(0, len(self.all_traffic_lights) + 1):
                start = self.all_traffic_lights[i-1] if i > 0 else 0
                end = self.all_traffic_lights[i] if i < len(self.all_traffic_lights) \
                        else self.all_traffic_lights[0] + len(self.waypoints)
                for j in range(start, min(end, len(self.waypoints))):
                    d = self.waypoints.distance(j, end)
                    v = self.waypoints[j].twist.twist.linear.x
                    if self.near_traffic_light_distance < d < self.far_traffic_light_distance:
                        self.waypoints[j].twist.twist.linear.x = min(self.max_velocity_near_traffic_light, self.waypoints[j].twist.twist.linear.x)
            self.ready = True

    def pose_cb(self, msg):
        self.pose_time = time.time()
        self.pose = msg.pose
        if self.loglevel >= 4:
            rospy.loginfo("Pose: (%s,%s)", self.pose.position.x, self.pose.position.y)

    def velocity_cb(self, velocity):
        self.velocity_time = time.time()
        self.velocity = self.velocity_in = velocity.twist.linear.x
        if self.loglevel >= 4:
            rospy.loginfo("Velocity: %f", self.velocity)
        
    def waypoints_cb(self, waypoints):
        self.waypoints = Waypoints(waypoints.waypoints)
        self.waypoints_header = waypoints.header
        self.set_road_velocity()

    def all_traffic_lights_cb(self, all_traffic_lights):
        self.all_traffic_lights = all_traffic_lights
        self.set_road_velocity()

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
