from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import os
import rospy
# from util import label_map_util


class TLClassifier(object):
    def __init__(self, loglevel=3):
        # Minimum score for classifier to consider a positive result
        self.min_score = rospy.get_param('~min_positive_score', 0.5)
        # Class of detected light - start with UNKNOWN
        self.light = TrafficLight.UNKNOWN
        # Define whether or not this is running in a simulation
        self.loglevel = loglevel

        # Select model according to sim value
        # ****************************************************************************
        # Please put the .pb files in the same directory as this tl_classifier.py file
        # ****************************************************************************
        self.model_path = rospy.get_param('/traffic_light_classifier_model', 'frozen_inference_graph.pb')

        # Load label values
        #label_file = 'label_map.pbtxt'
        #label_file_path = os.join.path('util', label_file)
        #self.category_index = label_map_util.create_category_index_from_labelmap(label_file)

        # Load graph from frozen model
        self.graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True	# Prevents crashing when avail. GPU memory < max GPU memory

        with self.graph.as_default():
            # Load graph from file
            graph_def = tf.GraphDef()
            with tf.gfile.Open(self.model_path, 'rb') as f:
                data = f.read()
                graph_def.ParseFromString(data)
            tf.import_graph_def(graph_def, name='')
            
            if self.loglevel >= 4:
                rospy.loginfo("Loaded trained model from %s", self.model_path)

            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            self.tensor_dict = {}
            for key in ['num_detections', 'detection_boxes', 'detection_scores','detection_classes']:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
            
            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.sess = tf.Session(graph=self.graph, config=config)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # Run inference
        output_dict = self.sess.run(self.tensor_dict, feed_dict={self.image_tensor: np.expand_dims(image, 0)})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        # output_dict['num_detections'] = int(output_dict['num_detections'][0])
        max_output_class = output_dict['detection_classes'][0][0].astype(np.uint8)
        max_output_score = output_dict['detection_scores'][0][0]
        max_output_box = output_dict['detection_boxes'][0][0]
        
        #self.light = TrafficLight.UNKNOWN
        #max_output_box = []
        detected_class = 0
        
        if max_output_score > self.min_score:
            detected_class = max_output_class
        # detected_class = output_dict['detection_classes'][np.where(output_dict['detection_scores'] > self.min_score)]
        # detected_class = max(set(detected_class), key=detected_class.count)

        #rospy.loginfo("GREEN: %s, YELLOW: %s, RED: %s, UNKNOWN: %s", TrafficLight.GREEN, TrafficLight.YELLOW, TrafficLight.RED, TrafficLight.UNKNOWN)
        
        if detected_class == 1:
            self.light = TrafficLight.GREEN
        elif detected_class == 2:
            self.light = TrafficLight.RED
        elif detected_class == 3:
            self.light = TrafficLight.YELLOW
        elif detected_class == 4:
            self.light = TrafficLight.UNKNOWN
            max_output_box = []
            
        return self.light, max_output_score, max_output_box
