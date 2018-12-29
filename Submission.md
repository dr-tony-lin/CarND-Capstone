# Programming A Real Self-Driving Car

## By Team Autonomous Mobility

### Team Lead: Tony Lin

### Members:

|        Name         |            Email             |
|---------------------|:-----------------------------|
|     James Korge	  |       jkorge@nyu.edu         |
|     Yulong Li	      |      hiyulongl@gmail.com	 |	
|    Islam Sherif     |   eng.islam.sherif@gmail.com |
|   Fuqiang Huang	  |       fuqiang@wustl.edu      |
|     Tony Lin        |     dr.tony.lin@gmail.com	 |

## Credits

* The training data for traffic light detector was from https://github.com/alex-lechner/Traffic-Light-Classification
* For the simulator, the traffic light classifier uses SSD Mobilenet implementation from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md,
commit dc78c085 for Tensorflow 1.3 compatibility
* For the site, the traffic light classifier uses SSD inception V2 pre-trained model from https://github.com/alex-lechner/Traffic-Light-Classification

## Usage

The usage of the project is provided in the [README.md](README.md)

## Configuration Parameters

### Global Parameters

The following global parameters are provided under ros/launch/styx.launch, and ros/launch/site.launch:

* loglevel: we use this parameter to control logging on top of ros's logging framework
* traffic_light_classifier_model: the trained model for traffic light detection

#### Waypoint Update

Waypoint updater takes the following parameters:

* lookahead_wps: the number of waypoints to updates in every cycle
* waypoint_update_frequency: the waypoint update frequency
* traffic_light_stop_distance: the stop distance for traffic light
* traffic_light_lookahead_wps: the number of waypoints that we need to decelerate for red light
* max_deceleration: maximum deceleration to stop for traffic light

#### Waypoint Follower

Waypoint follower's pure pursuit algorithm takes the following parameters:

* publish_frequency: the frequency for publishing twist
* subscriber_queue_length: the subscriber queue length for final_waypoints, current_pose, and current_velocity topics
* const_lookahead_distance: pure pursuit constant lookahead distance
* minimum_lookahead_distance: pure pursuit maximum lookahead distance
* lookahead_distance_ratio: pure pursuit lookahead distance ratio with respect to velocity
* maximum_lookahead_distance_ratio: pure pursuit maximum lookahead distance ratio with respect to velocity

#### Twist Controller

The twist controller takes the following parameters:

control_update_frequency: the frequency that the controller should publish throttle, steering, and brake

#### Vehicle Parameters

The twist controller uses the following parameters for the vehicle:

* vehicle_mass: mass of the vehicle
* fuel_capacity: fuel capacity
* brake_deadband: brake deadband
* decel_limit: deceleration limit
* accel_limit: acceleration limit
* wheel_radius: wheel radius
* wheel_base: wheel base length
* steer_ratio: steering ratio
* max_lat_accel: maximum lateral acceleration
* max_throttle: maximum throttle
* max_steer_angle: maximum steering angle
* full_stop_brake_keep: the amount of brake to keep when the stopping the vehicle
* full_stop_brake_limit: the maximum amount of brake that could be applied to the vehicle
* brake_deceleration_start: to avoid braking the vehicle too frequently, this parameter provides the threshold on when brake should be applied

#### Traffic Light Detecter

The traffic light detector use the global parameter, traffic_light_classifier_model, to refer to the Tensorflow classification model.
In addition, the following parameters are used for the classification:

* min_positive_score: the minimal score for the result to be considered positive

## Traffic Light Detection

### For the Simulator

SSD Mobilenet V1 is used for detection and classification. It is re-trained from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md,
commit dc78c085 for Tensorflow 1.3 compatibility.

### For the Site

The traffic light detection and classification uses a SSD Inception V2 pre-trained model from https://github.com/alex-lechner/Traffic-Light-Classification. It is based on commit f7e99c0 of https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md for Tensowflow 1.4 compatibility. It also works for Tendowflow 1.3.

