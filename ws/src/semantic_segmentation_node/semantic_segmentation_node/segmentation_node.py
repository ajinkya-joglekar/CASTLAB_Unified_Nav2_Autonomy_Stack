#!/usr/bin/env python3
"""ROS 2 node for semantic segmentation inference using ONNX Runtime."""

import traceback
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import LabelInfo, VisionClass


class SegmentationNode(Node):
    """
    ROS 2 semantic segmentation node using ONNX Runtime.

    Features:
      - Runtime-selectable ONNX model path
      - Runtime-selectable ontology YAML path
      - Works with old demo model and new RELLIS model
      - Dynamically reads ONNX input/output names
      - Handles dynamic or fixed ONNX input sizes
      - Resizes prediction back to original camera image size
      - Publishes:
          /segmentation/mask
          /segmentation/confidence
          /segmentation/label_info
          /segmentation/overlay
    """

    def __init__(self):
        super().__init__("segmentation_node")

        package_share = Path(get_package_share_directory("semantic_segmentation_node"))

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        # self.declare_parameter("use_sim_time", False)

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("mask_topic", "/segmentation/mask")
        self.declare_parameter("confidence_topic", "/segmentation/confidence")
        self.declare_parameter("label_info_topic", "/segmentation/label_info")
        self.declare_parameter("overlay_topic", "/segmentation/overlay")
        self.declare_parameter("publish_overlay", True)

        # Default to the old demo model name if present.
        # Override this from launch for your RELLIS model.
        self.declare_parameter(
            "model_path",
            str(package_share / "models" / "model.onnx"),
        )

        self.declare_parameter(
            "config_path",
            str(package_share / "config" / "ontology.yaml"),
        )

        # Used when the model has dynamic input shape.
        self.declare_parameter("input_width", 512)
        self.declare_parameter("input_height", 512)

        # If true, resize camera image to model input size before inference.
        # This is recommended for your RELLIS model.
        self.declare_parameter("resize_input", True)

        # If true, apply ImageNet normalization.
        self.declare_parameter("normalize", True)

        # Device: "cpu" or "cuda".
        # The YAML can also specify model.device; ROS parameter wins if set here.
        self.declare_parameter("device", "auto")

        # Debug
        self.declare_parameter("debug_histogram", True)
        self.declare_parameter("debug_every_n_frames", 30)

        # Read parameters
        self.input_topic = self.get_parameter("input_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.confidence_topic = self.get_parameter("confidence_topic").value
        self.label_info_topic = self.get_parameter("label_info_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.publish_overlay = self.get_parameter("publish_overlay").value

        self.model_path = Path(self.get_parameter("model_path").value)
        self.config_path = Path(self.get_parameter("config_path").value)

        self.input_width = int(self.get_parameter("input_width").value)
        self.input_height = int(self.get_parameter("input_height").value)
        self.resize_input = bool(self.get_parameter("resize_input").value)
        self.normalize = bool(self.get_parameter("normalize").value)

        self.device_param = str(self.get_parameter("device").value).lower()
        self.debug_histogram = bool(self.get_parameter("debug_histogram").value)
        self.debug_every_n_frames = int(self.get_parameter("debug_every_n_frames").value)

        self.get_logger().info(f"Package share: {package_share}")
        self.get_logger().info(f"Model path: {self.model_path}")
        self.get_logger().info(f"Config path: {self.config_path}")
        self.get_logger().info(f"Input topic: {self.input_topic}")
        self.get_logger().info(f"Resize input: {self.resize_input}")
        self.get_logger().info(f"Requested input size: {self.input_width}x{self.input_height}")
        self.get_logger().info(f"Normalize: {self.normalize}")

        if not self.model_path.exists():
            raise FileNotFoundError(f"ONNX model not found: {self.model_path}")

        if not self.config_path.exists():
            raise FileNotFoundError(f"Ontology YAML not found: {self.config_path}")

        # ---------------------------------------------------------------------
        # Load ontology config
        # ---------------------------------------------------------------------
        with open(self.config_path, "r") as f:
            config = yaml.safe_load(f)

        self.class_names = [cls["name"] for cls in config["ontology"]["classes"]]
        self.class_colors = [cls["color"] for cls in config["ontology"]["classes"]]

        # Background is class 0. YAML contains foreground classes only.
        self.num_classes = len(self.class_names) + 1

        # Device setting:
        # 1. ROS param device if not auto
        # 2. YAML model.device
        # 3. CPU fallback
        yaml_device = config.get("model", {}).get("device", "cpu").lower()

        if self.device_param == "auto":
            device = yaml_device
        else:
            device = self.device_param

        self.get_logger().info(f"Number of classes: {self.num_classes}")
        self.get_logger().info(f"Device setting: {device}")

        # ---------------------------------------------------------------------
        # ONNX Runtime session
        # ---------------------------------------------------------------------
        available_providers = ort.get_available_providers()
        self.get_logger().info(f"Available ONNX Runtime providers: {available_providers}")

        if device == "cuda" and "CUDAExecutionProvider" in available_providers:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        else:
            if device == "cuda":
                self.get_logger().warn(
                    "CUDA requested, but CUDAExecutionProvider is not available. "
                    "Falling back to CPUExecutionProvider."
                )
            providers = ["CPUExecutionProvider"]

        self.get_logger().info(f"Creating ONNX Runtime session with providers: {providers}")
        self.session = ort.InferenceSession(str(self.model_path), providers=providers)

        active_providers = self.session.get_providers()
        self.get_logger().info(f"Active ONNX Runtime providers: {active_providers}")

        # Dynamically read input/output names
        input_meta = self.session.get_inputs()[0]
        output_meta = self.session.get_outputs()[0]

        self.input_name = input_meta.name
        self.output_name = output_meta.name
        self.model_input_shape = input_meta.shape
        self.model_output_shape = output_meta.shape

        self.get_logger().info(f"ONNX input name: {self.input_name}")
        self.get_logger().info(f"ONNX output name: {self.output_name}")
        self.get_logger().info(f"ONNX input shape: {self.model_input_shape}")
        self.get_logger().info(f"ONNX output shape: {self.model_output_shape}")
        self.get_logger().info(f"ONNX input type: {input_meta.type}")

        # Detect model input dtype
        self.input_dtype = input_meta.type
        self.use_fp16 = "float16" in str(self.input_dtype).lower()

        # If model has fixed NCHW spatial input shape, use that automatically.
        self._maybe_update_input_size_from_model_shape()

        dtype = np.float16 if self.use_fp16 else np.float32

        # ImageNet normalization, NCHW-compatible broadcasting
        self.mean = np.array([0.485, 0.456, 0.406], dtype=dtype).reshape(3, 1, 1)
        self.std = np.array([0.229, 0.224, 0.225], dtype=dtype).reshape(3, 1, 1)

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self.bridge = CvBridge()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10,
        )

        self.mask_publisher = self.create_publisher(
            Image,
            self.mask_topic,
            10,
        )

        self.confidence_publisher = self.create_publisher(
            Image,
            self.confidence_topic,
            10,
        )

        # Create overlay publisher unconditionally so topic appears in node info.
        self.overlay_publisher = self.create_publisher(
            Image,
            self.overlay_topic,
            10,
        )

        label_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.label_info_publisher = self.create_publisher(
            LabelInfo,
            self.label_info_topic,
            label_info_qos,
        )

        self.publish_label_info()

        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing mask to: {self.mask_topic}")
        self.get_logger().info(f"Publishing confidence to: {self.confidence_topic}")
        self.get_logger().info(f"Publishing label info to: {self.label_info_topic}")
        self.get_logger().info(f"Publishing overlay to: {self.overlay_topic}")
        self.get_logger().info("Segmentation node initialized successfully.")

    def _maybe_update_input_size_from_model_shape(self):
        """
        If ONNX model has fixed input H/W, use those.
        If input shape is dynamic, keep the ROS parameter values.
        Expected ONNX shape: [N, C, H, W]
        """
        try:
            if len(self.model_input_shape) != 4:
                self.get_logger().warn(
                    f"Unexpected ONNX input shape length: {self.model_input_shape}. "
                    "Keeping parameter input size."
                )
                return

            h = self.model_input_shape[2]
            w = self.model_input_shape[3]

            if isinstance(h, int) and isinstance(w, int):
                self.input_height = h
                self.input_width = w
                self.resize_input = True
                self.get_logger().info(
                    f"Using fixed ONNX input size from model: "
                    f"{self.input_width}x{self.input_height}"
                )
            else:
                self.get_logger().info(
                    "ONNX input shape is dynamic. "
                    f"Using parameter input size: {self.input_width}x{self.input_height}"
                )

        except Exception as exc:
            self.get_logger().warn(
                f"Could not parse ONNX input shape: {exc}. "
                "Keeping parameter input size."
            )

    def publish_label_info(self):
        """Publish LabelInfo message with class mappings."""
        label_info = LabelInfo()
        label_info.header.stamp = self.get_clock().now().to_msg()
        label_info.header.frame_id = ""

        class_map = []

        bg_class = VisionClass()
        bg_class.class_id = 0
        bg_class.class_name = "background"
        class_map.append(bg_class)

        for idx, class_name in enumerate(self.class_names, start=1):
            vc = VisionClass()
            vc.class_id = idx
            vc.class_name = class_name
            class_map.append(vc)

        label_info.class_map = class_map
        label_info.threshold = 0.5

        self.label_info_publisher.publish(label_info)
        self.get_logger().info(f"Published LabelInfo with {len(class_map)} classes")

    def create_colored_mask(self, mask: np.ndarray) -> np.ndarray:
        """
        Convert class ID mask to colored visualization.

        Args:
            mask: [H, W] uint8 class-ID image.

        Returns:
            [H, W, 3] uint8 BGR colored mask.
        """
        h, w = mask.shape
        colored = np.zeros((h, w, 3), dtype=np.uint8)

        # Class 0 background remains black.
        for class_id in range(1, self.num_classes):
            if class_id <= len(self.class_colors):
                color = self.class_colors[class_id - 1]

                # Ontology colors are assumed BGR because this node uses OpenCV
                # and publishes overlay as bgr8.
                colored[mask == class_id] = color

        return colored

    def preprocess_image(self, cv_image: np.ndarray) -> tuple[np.ndarray, int, int]:
        """
        Convert BGR OpenCV image to ONNX input tensor.

        Returns:
            input_tensor: [1, 3, H, W]
            original_h
            original_w
        """
        original_h, original_w = cv_image.shape[:2]

        if self.resize_input:
            model_bgr = cv2.resize(
                cv_image,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_LINEAR,
            )
        else:
            model_bgr = cv_image

        rgb_image = cv2.cvtColor(model_bgr, cv2.COLOR_BGR2RGB)

        dtype = np.float16 if self.use_fp16 else np.float32

        # HWC -> CHW, scale 0-255 to 0-1
        input_tensor = rgb_image.transpose(2, 0, 1).astype(dtype) / 255.0

        if self.normalize:
            input_tensor = (input_tensor - self.mean) / self.std

        # CHW -> NCHW
        input_tensor = np.expand_dims(input_tensor, axis=0)

        return input_tensor, original_h, original_w

    @staticmethod
    def softmax_channelwise(logits: np.ndarray) -> np.ndarray:
        """
        Stable softmax over channel dimension.

        Args:
            logits: [1, C, H, W]

        Returns:
            probabilities: [1, C, H, W]
        """
        # Convert to float32 because OpenCV and NumPy ops are safer here.
        logits = logits.astype(np.float32)

        logits_max = np.max(logits, axis=1, keepdims=True)
        exp_logits = np.exp(logits - logits_max)
        return exp_logits / np.sum(exp_logits, axis=1, keepdims=True)

    def image_callback(self, msg: Image):
        """Process incoming image and publish segmentation results."""
        try:
            self.frame_count += 1

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            input_tensor, original_h, original_w = self.preprocess_image(cv_image)

            outputs = self.session.run(
                [self.output_name],
                {self.input_name: input_tensor},
            )

            output = outputs[0]

            # Some models return list-like outputs, but ONNX Runtime output is usually ndarray.
            # Expected: [1, C, H, W]
            if output.ndim != 4:
                raise RuntimeError(
                    f"Expected ONNX output with shape [1, C, H, W], got {output.shape}"
                )

            # Class prediction
            prediction = np.argmax(output, axis=1).squeeze(0).astype(np.uint8)

            # Resize prediction back to original camera size if needed.
            if prediction.shape[0] != original_h or prediction.shape[1] != original_w:
                prediction = cv2.resize(
                    prediction,
                    (original_w, original_h),
                    interpolation=cv2.INTER_NEAREST,
                )

            # Confidence image
            probabilities = self.softmax_channelwise(output)
            confidence = np.max(probabilities, axis=1).squeeze(0)

            # OpenCV resize does not support float16, so force float32.
            confidence = confidence.astype(np.float32)

            if confidence.shape[0] != original_h or confidence.shape[1] != original_w:
                confidence = cv2.resize(
                    confidence,
                    (original_w, original_h),
                    interpolation=cv2.INTER_LINEAR,
                )

            confidence_uint8 = np.clip(confidence * 255.0, 0, 255).astype(np.uint8)

            # Publish mask
            mask_msg = self.bridge.cv2_to_imgmsg(prediction, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_publisher.publish(mask_msg)

            # Publish confidence
            confidence_msg = self.bridge.cv2_to_imgmsg(confidence_uint8, encoding="mono8")
            confidence_msg.header = msg.header
            self.confidence_publisher.publish(confidence_msg)

            # Publish overlay
            if self.publish_overlay:
                pred_colored = self.create_colored_mask(prediction)

                if pred_colored.shape[:2] != cv_image.shape[:2]:
                    pred_colored = cv2.resize(
                        pred_colored,
                        (original_w, original_h),
                        interpolation=cv2.INTER_NEAREST,
                    )

                overlay = cv2.addWeighted(
                    cv_image,
                    0.7,
                    pred_colored,
                    0.3,
                    0.0,
                )

                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_msg.header = msg.header
                self.overlay_publisher.publish(overlay_msg)

            # Debug histogram
            if self.debug_histogram and self.frame_count % self.debug_every_n_frames == 1:
                unique, counts = np.unique(prediction, return_counts=True)
                hist = ", ".join(
                    [f"{int(u)}:{int(c)}" for u, c in zip(unique, counts)]
                )

                self.get_logger().info(
                    f"Prediction histogram: {hist}; "
                    f"ONNX output shape: {output.shape}; "
                    f"input image: {original_w}x{original_h}; "
                    f"model input: {self.input_width}x{self.input_height}; "
                    f"resize_input={self.resize_input}"
                )

        except Exception as exc:
            self.get_logger().error(f"Segmentation callback failed: {exc}")
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()