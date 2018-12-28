# Training Documentation  

## Training Data  

### Collect Image and Labeling

Images collected from ROS topic '/image_color':  

![sample](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/images%20and%20labels/1545181711.33.jpg)

Then the images were manually labeled using labelImg (https://github.com/tzutalin/labelImg):   

![labeling](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/img/labeling.JPG)  

[xml](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/images%20and%20labels/1545181711.33.xml) file was generated after labeling.  


### Convert Labels to TFRecord

Each image has a corresponding xml label file. Both of them need to be converted to TensorFLow's TFRecord format to be ready for training.  

The xml files were first converted to csv file and shuffled: [img_labels_2_shuffled.csv](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/Sample%20TFRecord/img_labels_2_shuffled.csv)  

Then the images and labels were combined into [TFRecord](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/Sample%20TFRecord/TFRecord)  

### Creat Label Maps  

[label_map_2.pbtxt](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/Sample%20TFRecord/label_map_2.pbtxt) was created, which maps the class label with integer values.  

## Training

### Creat Training Pipeline  

The [training pipeline](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20data/Sample%20TFRecord/ssd_mobilenet_v1_coco_2.config) includes model configuration, training and evaluation configurations. It takes the paths to model.ckpt, TFRecord and pbtxt file. This pipeline configuration will be loaded in the train.py or model_main.py.  

### Download Model  

Model was downloaded from [model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md).  

ssd_mobilenet_v1_coco was chosen for this project because it is one of the fastest models.  

### Start Trainng  
```  
# From the tensorflow/models/research/ directory
PIPELINE_CONFIG_PATH={path to pipeline config file}
MODEL_DIR={path to model directory}
NUM_TRAIN_STEPS=50000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python object_detection/model_main.py \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --model_dir=${MODEL_DIR} \
    --num_train_steps=${NUM_TRAIN_STEPS} \
    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
    --alsologtostderr
```  

### Export Frozen Graph
Generated model.ckpt was converted to [frozen_inference_graph.pb](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20result/sim_frozen_inference_graph.pb).  

### Tensorboard -- Total Loss  

![tensorboard](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/img/tensorboard.JPG)  

### Visualization  

![visualization](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/img/visualization.JPG)

[capstone_traffic_light_detection_simulator_final.ipynb](https://github.com/dr-tony-lin/CarND-Capstone/blob/master/training%20documentation/Training%20result/capstone_traffic_light_detection_simulator_final.ipynb)  


