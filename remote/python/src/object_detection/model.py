# #!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This module ...

__author__ = "Magnus Kvendseth Øye"
__copyright__ = "Copyright 2020, Sparkie Quadruped Robot"
__credits__ = ["Magnus Kvendseth Øye", "Petter Drønnen", "Vegard Solheim"]
__version__ = "1.0.0"
__license__ = "MIT"
__maintainer__ = "Magnus Kvendseth Øye"
__email__ = "magnus.oye@gmail.com"
__status__ = "Development"
"""


# Import packages
import cv2
import numpy as np
import tensorflow as tf

# Import utilites
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

class ObjectDetector(object):
    """Performs object detection through webcamera."""

    def __init__(self, path_to_ckpt, path_to_labels):
        self.PATH_TO_CKPT = path_to_ckpt
        self.PATH_TO_LABELS = path_to_labels
        self.NUM_CLASSES = 2  # Check labelmap

        self.label_map = label_map_util.load_labelmap(self.PATH_TO_LABELS)

        self.categories = label_map_util.convert_label_map_to_categories(
            self.label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)

        self.category_index = label_map_util.create_category_index(
            self.categories)

        # Initialize variables
        self.sess = None
        self.image_tensor = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None
        self.num_detections = None

    def initialize(self):
        """Initializes the tensorflow trained model."""
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=detection_graph)

        # Input tensor is the image
        self.image_tensor = detection_graph.get_tensor_by_name(
            'image_tensor:0')
        # Output tensors are the detection boxes, scores, and classes
        self.detection_boxes = detection_graph.get_tensor_by_name(
            'detection_boxes:0')
        # Each score represents level of confidence for each of the objects.
        self.detection_scores = detection_graph.get_tensor_by_name(
            'detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name(
            'detection_classes:0')
        # Number of objects detected
        self.num_detections = detection_graph.get_tensor_by_name(
            'num_detections:0')

    def run(self, frame, debug=False):
        """Runs the object detection on the assigned capture."""
        frame_expanded = np.expand_dims(frame, axis=0)

        # Perform the detection by running the model with the image as input
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores,
                self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: frame_expanded})

        # Draw the results of the detection
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=3,
            min_score_thresh=0.60)

        if debug:
            cv2.imshow('Frame', frame)

