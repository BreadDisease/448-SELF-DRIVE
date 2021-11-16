import cv2
from utils import (
    CLASSES,
    COLORS,
    CONFIDENCE_THRESHOLD,
    NMS_THRESHOLD,
    draw_bounding_box_with_label,
    yolo_box_to_points,
)

class ObjectDetector:
    def __init__(self, gpu=False):
        self.weights_path = "models/yolov4.weights"
        self.config_path = "models/yolov4.cfg"
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)
        if gpu == True:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.model = cv2.dnn_DetectionModel(self.net)

    def detect(self, image):
        self.model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)
        class_ids, scores, boxes = self.model.detect(
            image, confThreshold=CONFIDENCE_THRESHOLD, nmsThreshold=NMS_THRESHOLD
        )

        return class_ids, scores, boxes

    def drawbox(self, image, class_ids, scores, boxes):
        # Draw bounding box for each object
        for (class_id, score, box) in zip(class_ids, scores, boxes):
            label = "%s: %.2f" % (CLASSES[class_id[0]], score)
            color = COLORS[class_id[0]]
            x1, y1, x2, y2 = yolo_box_to_points(box)
            draw_bounding_box_with_label(image, x1, y1, x2, y2, label, color)

        return image
