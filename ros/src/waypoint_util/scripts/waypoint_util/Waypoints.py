import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
import numpy as np

import math

class Point:
    def __init__(self, waypoint,  d):
        self.waypoint = waypoint
        self.d = d
    
    @property
    def x(self):
        return self.waypoint.pose.pose.position.x

    @property
    def y(self):
        return self.waypoint.pose.pose.position.y

    @property
    def z(self):
        return self.waypoint.pose.pose.position.z

    @property
    def pose(self):
        return self.waypoint.pose

    @property
    def twist(self):
        return self.waypoint.twist

class Waypoints(object):
    def __init__(self, waypoints, circular=False, loglevel=3):
        self.loglevel = loglevel
        self.circular = circular
        self.waypoints  = [Point(waypoint, 0) for waypoint in waypoints]
        self.waypoints_kd = KDTree([[waypoint.x, waypoint.y] for waypoint in self.waypoints])
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(1, len(waypoints)):
            self.waypoints[i].d = self.waypoints[i-1].d + dl(self.waypoints[i-1], self.waypoints[i])
        self.total_length = self.waypoints[-1].d + dl(self.waypoints[-1], self.waypoints[0])
        if self.loglevel >= 4:
            rospy.loginfo("Initialized Waypoints.")
        if self.loglevel >= 5:
            for i in range(0, len(self.waypoints)):
                rospy.logdebug("%d (%f,%f,%f,%f, %f", i, self.waypoints[i].x, self.waypoints[i].y, self.waypoints[i].twist.twist.linear.x, self.waypoints[i].twist.twist.angular.z, self.waypoints[i].d)

    def __iter__(self):
        return iter(self.waypoints)

    def __getitem__(self, index):
        if self.circular:
            result = self.waypoints[self.normalize_index(index)]
            if index is slice and index.stop > len(self.waypoints):
                stop = min(slice.stop - len(self.waypoints), len(self.waypoints))
                result.extend(self.waypoints[slice(0, stop, index.step)])
            return result
        else:
            return self.waypoints[index]

    def __len__(self):
        return len(self.waypoints)

    def normalize_index(self, idx, clamp=False):
        '''
        Return the normalized index
        Parameters:
            idx the inde
            clamp True to clamp the idx to the valid range
        '''
        return idx % len(self.waypoints) if self.circular else min(idx, len(self.waypoints) - 1) if clamp else idx

    def _create_waypoint(self, wp):
        p = Waypoint()
        p.pose = wp.pose
        p.twist = wp.twist
        return p

    def get_waypoints(self, index):
        '''
        Get an array of waypoints given the index
        Parameters:
            index the slice of waypoints to get
        '''
        return [self._create_waypoint(wp) for wp in self.__getitem__(index)]

    def find_closest_waypoint(self, pos):
        '''
        Find the infex of the waypoint that is closest to pos
        Parameters:
            pos the position [x, y] to find
        '''
        idx = self.waypoints_kd.query(pos, 1)[1]
        closest_pos = np.array([self.waypoints[idx].x, self.waypoints[idx].y])
        prev_pos = np.array([self.waypoints[idx - 1].x, self.waypoints[idx - 1].y])
        v_way = closest_pos - prev_pos
        v_pos = np.array(pos) - closest_pos
        if np.dot(v_way, v_pos) > 0:
            idx = self.normalize_index(idx + 1, True)
        if self.loglevel >= 5:
            rospy.logdebug("Closest wp: %d, (%f,%f)->(%f,%f)",idx, pos[0], pos[1], self.waypoints[idx].x, self.waypoints[idx].y)
        return idx

    def before(self, wp1, wp2):
        '''
        Return True if wp1 is before wp2
        '''
        d = wp2 - wp1
        if self.circular:
            if d > len(self.waypoints) / 2:
                return False
            elif d < -len(self.waypoints) / 2:
                return True
        if d > 0:
            return True
        else:
            return False
    
    def distance(self, wp1, wp2):
        '''
        return distance between wp1 and wp2
        '''
        dist = self.waypoints[wp2].d - self.waypoints[wp1].d
        if self.circular and dist < 0: # circled back, normalize the distance
            dist += self.total_length
        return dist
