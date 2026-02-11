#!/home/yongseung-b850-yamato/yolo_venv/bin/python3
"""
YOLOv11 Real-time Object Detection Node for RaptorLift Webcam.

Subscribes to raw camera images and publishes:
  - Detection2DArray (vision_msgs) with bounding boxes + class labels
  - Annotated image with drawn detections
  - Detection count (Int32)

Trained on Roboflow, deployed with Ultralytics YOLOv11.
"""

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from ultralytics import YOLO
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


class DetectionNode(Node):
    """Real-time YOLOv11 object detection on webcam images."""

    def __init__(self):
        super().__init__("detection_node")

        # -- Declare parameters --
        self.declare_parameter(
            "model_path",
            self._default_model_path(),
        )
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("input_topic", "image_raw")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("max_detections", 20)

        # -- Read parameters --
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self._conf = self.get_parameter("confidence_threshold").get_parameter_value().double_value
        self._iou = self.get_parameter("iou_threshold").get_parameter_value().double_value
        self._device = self.get_parameter("device").get_parameter_value().string_value
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self._publish_annotated = (
            self.get_parameter("publish_annotated_image").get_parameter_value().bool_value
        )
        self._max_det = self.get_parameter("max_detections").get_parameter_value().integer_value

        # -- Load YOLO model --
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self._model = YOLO(model_path)
        self.get_logger().info(
            f"Model loaded | device={self._device} conf={self._conf} iou={self._iou}"
        )

        # -- cv_bridge --
        self._bridge = CvBridge()

        # -- QoS: SensorDataQoS (BEST_EFFORT, allow frame drops) --
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -- Subscriber --
        self._sub = self.create_subscription(
            Image,
            input_topic,
            self._image_callback,
            sensor_qos,
        )

        # -- Publishers --
        self._det_pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._count_pub = self.create_publisher(Int32, "detection_count", 10)

        if self._publish_annotated:
            self._img_pub = self.create_publisher(Image, "detection_image", sensor_qos)

        self.get_logger().info("Detection node ready, waiting for images...")

    @staticmethod
    def _default_model_path() -> str:
        """Resolve default model path from installed package share."""
        import os

        pkg_share = get_package_share_directory("raptorlift_webcam")
        return os.path.join(pkg_share, "models", "weights.pt")

    def _image_callback(self, msg: Image) -> None:
        """Process incoming image: run YOLO, publish results."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # -- Run inference --
        results = self._model.predict(
            frame,
            conf=self._conf,
            iou=self._iou,
            device=self._device,
            max_det=self._max_det,
            verbose=False,
        )

        result = results[0]
        boxes = result.boxes

        # -- Build Detection2DArray --
        det_array = Detection2DArray()
        det_array.header = msg.header

        for box in boxes:
            det = Detection2D()

            # Bounding box center + size
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            # Classification hypothesis
            hyp = ObjectHypothesisWithPose()
            class_id = int(box.cls[0].item())
            hyp.hypothesis.class_id = str(class_id)
            if result.names and class_id in result.names:
                hyp.hypothesis.class_id = result.names[class_id]
            hyp.hypothesis.score = float(box.conf[0].item())
            det.results.append(hyp)

            det_array.detections.append(det)

        self._det_pub.publish(det_array)

        # -- Publish detection count --
        count_msg = Int32()
        count_msg.data = len(det_array.detections)
        self._count_pub.publish(count_msg)

        # -- Publish annotated image --
        if self._publish_annotated:
            annotated = result.plot()
            img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header = msg.header
            self._img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
