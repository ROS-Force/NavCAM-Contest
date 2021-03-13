# Object Detection Sub-Package

This sub-package performs detection of objects using the state of the art You Only Look Once (YOLO) and for tracking we use the Alex Bewley SORT implementation. For more information about YOLO, Darknet and SORT algorithm see the following links: [YOLO: Real-Time Object Detection](http://pjreddie.com/darknet/yolo/), [Alex Bewley SORT implementation](https://github.com/abewley/sort),[SORT paper](https://arxiv.org/abs/1602.00763).

Based on the [COCO](http://cocodataset.org/#home) dataset we can detect 80 classes:

- person
- bicycle, car, motorbike, aeroplane, bus, train, truck, boat
- traffic light, fire hydrant, stop sign, parking meter, bench
- cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard, sports ball, kite, baseball bat, baseball glove, skateboard, surfboard, tennis racket
- bottle, wine glass, cup, fork, knife, spoon, bowl
- banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza, donut, cake
- chair, sofa, pottedplant, bed, diningtable, toilet, tvmonitor, laptop, mouse, remote, keyboard, cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase, scissors, teddy bear, hair drier, toothbrush

## Download Weights and config files

To use the pre-trained YOLO model you have to [download](https://drive.google.com/drive/folders/12ss30brf8-qYFN3tojY-bVHK2ES4xKWN?usp=sharing) the weight and config files, and store them inside object_tracking/cnf.

## Nodes

### Node: yolo_detection.py

This node implements the YOLO model to perform the detection of objects.

#### Subscribed Topics:

- **`image`** ([sensor_msgs/Image])

  The RGB camera image.

#### Published Topics:

- **`output_image`** ([sensor_msgs/Image])

  The RGB image with the bounding_boxes drawn.

- **`bounding_boxes`** ([object_tracking/BoundingBoxes])

  The bounding box for each object detected in the video.

#### Parameters:

- **`yolo_model`**

  The yolo model to load.

- **`model_scale`**

  Multiplier for frame values..

- **`conf_threshold`**

  A threshold used to filter boxes by confidences.

- **`nms_threshold`** ([object_tracking/BoundingBoxes])

  A threshold used in non maximum suppression.

### Node: sort_tracking.py

This node implements the SORT algorithm to track the objects provided by the yolo_detection node.

#### Subscribed Topics:

- **`image`** ([sensor_msgs/Image])

  The RGB camera image.

- **`bounding_boxes`** ([object_tracking/BoundingBoxes])

  The bounding box for each object detected in the video.

- **`camera_info`** ([sensor_msgs/CameraInfo])

- **`depth_image`** ([sensor_msgs/Image])

  The depth camera image.

#### Published Topics:

- **`output_image`** ([sensor_msgs/Image])

  The RGB image with the bounding_boxes drawn, and the objects id.

- **`object`** ([object_tracking/Object])

  The class of the object, contains the id, class, color, shape, bounding box, speed, real coordinates relative to the camera.

#### Parameters:

- **`blue_humans`**

  The option to blur the detected humans.

- **`sort_threshold`**

  Minimum IOU for match.

- **`min_hits`**

  Minimum number of associated detections before track is initialised.

- **`max_age`** ([object_tracking/BoundingBoxes])

  Maximum number of frames to keep alive a track without associated detections.

## Demos

The rosbags were recorded with a RealSense D435i.

![](img/highway_demo.gif)

![](img/2_humans_demo.gif)

![](img/street_demo.gif)

## Basic Usage

To run the demos you have to put the rosbag file inside object_detection/demo and edit the tracking_demo.launch replacing the value with the rosbag you want to run. 

**`<arg name="bag_file_name" value="your_rosbag_name" />`**

You can change the parameters and then just run 

**`$ roslaunch object_detection tracking_demo.launch`**