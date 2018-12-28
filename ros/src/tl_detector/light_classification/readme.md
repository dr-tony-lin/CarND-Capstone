# tl_classifier.py  

## TLClassifier.__init__  

self.min_score is the threshold to determine if an object is detected. The range is from 0 to 1. If your are getting too many false positives, this can be considered to be increased.

When self.is_sight is false, the frozen graph for the simmulator environment will be loaded. Please put the .pb graph file in the same path as tl_classifier.py. To change frozen graph, just modify the self.model assignment.  


## TLClassifier.get_classification  

This function will take an RGB file as input. So in tl_detector.py -> TLDetector.get_light_state -> cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8"), "bgr8" needs to be modified as "rgb8".

This function will take the most confident detection (highest score) as the output. The classifier mapping is - 1: green, 2: yellow, 3: red. But the ROS message defines - TrafficLight.RED: 0, TrafficLight.YELLOW: 1, TrafficLight.GREEN: 2, TrafficLight.UNKNOWN: 4.  

TLClassifier.get_classification returns the ROS message definition of light state, highest score, and the bounding box of the traffic light.  

Bounding box format:  
Normalized coordinates: [ymin, xmin, ymax, xmax]  
ymin, xmin is the top left corner. ymax, xmax is the bottom right corner. (0, 0) is at the top left corner of the image.  
The size of the boxes indicate the distance of the vehicle from the object, and that information can be utilized in the vehicle control.  

In Workspace environment, the process speed is around 10 FPS.  

## Known Issue  
1. In Workspace environment, due to unknown reason, the classifier might stop working after running for a while. No error message.  
2. When vehicle is far from the object, the detection is not stable. A filter is suggested to be added in tl_detector.py to smooth out the detection.
