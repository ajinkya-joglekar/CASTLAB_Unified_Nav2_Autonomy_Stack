#!/usr/bin/env python3

import math
import struct
import re
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import LabelInfo

import tf2_ros


class TerrainObstacleGridNode(Node):
    """
    Standalone semantic terrain / obstacle grid generator.

    Inputs:
      /camera/depth/points    sensor_msgs/msg/PointCloud2
      /segmentation/mask      sensor_msgs/msg/Image
      /segmentation/label_info vision_msgs/msg/LabelInfo, optional but recommended

    Outputs:
      /terrain/obstacle_grid             nav_msgs/msg/OccupancyGrid
      /terrain/semantic_grid_markers     visualization_msgs/msg/MarkerArray
      /terrain/obstacle_markers          visualization_msgs/msg/MarkerArray

    Canonical semantic class convention used internally:

      -1 = unobserved grid cell
       0 = background_unknown
       1 = hard_traversable
       2 = grass
       3 = mud
       4 = water
       5 = vegetation
       6 = obstacle

    RELLIS 7-class model expected mapping:

       0 = background_unknown
       1 = hard_traversable
       2 = grass
       3 = mud
       4 = water
       5 = vegetation
       6 = obstacle

    Nav2 demo 3-class model mapping:

       0 = background      -> background_unknown
       1 = sidewalk        -> hard_traversable
       2 = grass           -> grass

    Geometry obstacle detection overrides semantic class and sets:

       canonical class = 6 obstacle
       cost = obstacle_cost
    """

    CANONICAL_UNKNOWN = -1
    CANONICAL_BACKGROUND = 0
    CANONICAL_HARD_TRAVERSABLE = 1
    CANONICAL_GRASS = 2
    CANONICAL_MUD = 3
    CANONICAL_WATER = 4
    CANONICAL_VEGETATION = 5
    CANONICAL_OBSTACLE = 6

    def __init__(self):
        super().__init__("terrain_obstacle_grid_node")

        # ---------------------------------------------------------------------
        # Topics
        # ---------------------------------------------------------------------
        self.declare_parameter("points_topic", "/camera/depth/points")
        self.declare_parameter("mask_topic", "/segmentation/mask")
        self.declare_parameter("label_info_topic", "/segmentation/label_info")
        self.declare_parameter("grid_topic", "/terrain/obstacle_grid")
        self.declare_parameter("semantic_marker_topic", "/terrain/semantic_grid_markers")
        self.declare_parameter("cost_marker_topic", "/terrain/obstacle_markers")

        # ---------------------------------------------------------------------
        # Frames
        # ---------------------------------------------------------------------
        self.declare_parameter("grid_frame", "base_link")

        # ---------------------------------------------------------------------
        # Grid geometry
        # Local-grid limits in grid_frame.
        # In base_link convention:
        #   x = forward
        #   y = left
        #   z = up
        # ---------------------------------------------------------------------
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("x_min", -2.0)
        self.declare_parameter("x_max", 18.0)
        self.declare_parameter("y_min", -8.0)
        self.declare_parameter("y_max", 8.0)

        # ---------------------------------------------------------------------
        # Height / obstacle thresholds in grid_frame
        # ---------------------------------------------------------------------
        self.declare_parameter("ground_z_min", -0.80)
        self.declare_parameter("ground_z_max", 0.30)
        self.declare_parameter("obstacle_z_threshold", 0.35)
        self.declare_parameter("min_points_per_cell", 2)

        # ---------------------------------------------------------------------
        # Old Nav2 demo compatibility parameters
        # Keep these so your old launch files still work.
        # Current expected demo model:
        #   0 = background
        #   1 = sidewalk
        #   2 = grass
        # ---------------------------------------------------------------------
        self.declare_parameter("background_label", 0)
        self.declare_parameter("sidewalk_label", 1)
        self.declare_parameter("grass_label", 2)

        # ---------------------------------------------------------------------
        # Semantic model profile
        #   auto:
        #       Prefer /segmentation/label_info if available.
        #       Otherwise assume canonical 0..6 mapping.
        #
        #   rellis7:
        #       Raw mask labels are already canonical 0..6.
        #
        #   nav2_demo3:
        #       Use background_label, sidewalk_label, grass_label params.
        # ---------------------------------------------------------------------
        self.declare_parameter("semantic_profile", "auto")

        # ---------------------------------------------------------------------
        # OccupancyGrid costs
        # OccupancyGrid convention:
        #   -1  = unknown
        #   0   = free
        #   100 = occupied
        #
        # hard_traversable_cost defaults to sidewalk_cost for backwards
        # compatibility if hard_traversable_cost is negative.
        # ---------------------------------------------------------------------
        self.declare_parameter("sidewalk_cost", 20)  # backwards compatibility
        self.declare_parameter("hard_traversable_cost", -1)

        self.declare_parameter("grass_cost", 40)
        self.declare_parameter("background_cost", 60)
        self.declare_parameter("mud_cost", 75)
        self.declare_parameter("water_cost", 95)
        self.declare_parameter("vegetation_cost", 85)
        self.declare_parameter("obstacle_cost", 100)

        # Default cost if geometry observes a cell but no valid semantic vote exists.
        self.declare_parameter("default_observed_cost", 60)
        self.declare_parameter("default_observed_class", 0)

        # If true, geometry obstacle detection overrides semantic label.
        self.declare_parameter("geometry_obstacle_override", True)

        # If mask is old, ignore semantic information.
        # For sim debugging, this can be set very large from launch.
        self.declare_parameter("max_mask_age_sec", 999999.0)

        # Debug logging
        self.declare_parameter("debug_print_grid_stats", False)

        # Marker alpha
        self.declare_parameter("semantic_marker_alpha", 0.55)
        self.declare_parameter("cost_marker_alpha", 0.50)

        # Machine readable semantic grid
        self.declare_parameter("semantic_grid_topic", "/terrain/semantic_grid")

        # ---------------------------------------------------------------------
        # Read parameters
        # ---------------------------------------------------------------------
        self.points_topic = self.get_parameter("points_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.label_info_topic = self.get_parameter("label_info_topic").value
        self.grid_topic = self.get_parameter("grid_topic").value
        self.semantic_marker_topic = self.get_parameter("semantic_marker_topic").value
        self.cost_marker_topic = self.get_parameter("cost_marker_topic").value
        self.grid_frame = self.get_parameter("grid_frame").value
        self.semantic_grid_topic = self.get_parameter("semantic_grid_topic").value

        self.resolution = float(self.get_parameter("resolution").value)
        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)

        self.width = int(math.ceil((self.x_max - self.x_min) / self.resolution))
        self.height = int(math.ceil((self.y_max - self.y_min) / self.resolution))

        self.latest_mask = None
        self.latest_mask_stamp = None

        # This maps raw segmentation labels to canonical terrain classes.
        # It is initialized from params and then updated from LabelInfo if available.
        self.raw_label_to_canonical = self.build_fallback_label_mapping()

        # Canonical class costs and colors.
        self.refresh_canonical_class_tables()

        # ---------------------------------------------------------------------
        # TF
        # ---------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------------------
        self.mask_sub = self.create_subscription(
            Image,
            self.mask_topic,
            self.mask_callback,
            10,
        )

        self.points_sub = self.create_subscription(
            PointCloud2,
            self.points_topic,
            self.points_callback,
            10,
        )

        # LabelInfo is transient-local in your segmentation node, so use
        # transient-local QoS here too.
        label_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.label_info_sub = self.create_subscription(
            LabelInfo,
            self.label_info_topic,
            self.label_info_callback,
            label_info_qos,
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.grid_topic,
            10,
        )

        self.semantic_marker_pub = self.create_publisher(
            MarkerArray,
            self.semantic_marker_topic,
            10,
        )

        self.cost_marker_pub = self.create_publisher(
            MarkerArray,
            self.cost_marker_topic,
            10,
        )

        self.semantic_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.semantic_grid_topic,
            10,
        )

        self.get_logger().info(f"Subscribing to points:             {self.points_topic}")
        self.get_logger().info(f"Subscribing to mask:               {self.mask_topic}")
        self.get_logger().info(f"Subscribing to label info:         {self.label_info_topic}")
        self.get_logger().info(f"Publishing occupancy grid:         {self.grid_topic}")
        self.get_logger().info(f"Publishing semantic markers:       {self.semantic_marker_topic}")
        self.get_logger().info(f"Publishing cost markers:           {self.cost_marker_topic}")
        self.get_logger().info(f"Grid frame:                        {self.grid_frame}")
        self.get_logger().info(
            f"Grid size: {self.width} x {self.height}, "
            f"resolution {self.resolution:.3f} m"
        )
        self.get_logger().info(f"Initial label mapping: {self.raw_label_to_canonical}")
        self.get_logger().info(f"Publishing semantic grid:          {self.semantic_grid_topic}")
        self.print_canonical_tables()

    # -------------------------------------------------------------------------
    # Canonical semantic setup
    # -------------------------------------------------------------------------
    def refresh_canonical_class_tables(self):
        sidewalk_cost = int(self.get_parameter("sidewalk_cost").value)
        hard_cost_param = int(self.get_parameter("hard_traversable_cost").value)

        if hard_cost_param < 0:
            hard_cost = sidewalk_cost
        else:
            hard_cost = hard_cost_param

        self.canonical_cost = {
            self.CANONICAL_BACKGROUND: int(self.get_parameter("background_cost").value),
            self.CANONICAL_HARD_TRAVERSABLE: hard_cost,
            self.CANONICAL_GRASS: int(self.get_parameter("grass_cost").value),
            self.CANONICAL_MUD: int(self.get_parameter("mud_cost").value),
            self.CANONICAL_WATER: int(self.get_parameter("water_cost").value),
            self.CANONICAL_VEGETATION: int(self.get_parameter("vegetation_cost").value),
            self.CANONICAL_OBSTACLE: int(self.get_parameter("obstacle_cost").value),
        }

        self.canonical_name = {
            self.CANONICAL_UNKNOWN: "unobserved",
            self.CANONICAL_BACKGROUND: "background_unknown",
            self.CANONICAL_HARD_TRAVERSABLE: "hard_traversable",
            self.CANONICAL_GRASS: "grass",
            self.CANONICAL_MUD: "mud",
            self.CANONICAL_WATER: "water",
            self.CANONICAL_VEGETATION: "vegetation",
            self.CANONICAL_OBSTACLE: "obstacle",
        }

        # BGR colors matched to your segmentation overlay convention.
        # The marker publisher converts these BGR values to ROS RGB ColorRGBA.
        self.canonical_bgr = {
            self.CANONICAL_BACKGROUND: (0, 0, 0),          # black
            self.CANONICAL_HARD_TRAVERSABLE: (160, 160, 160),  # gray
            self.CANONICAL_GRASS: (0, 200, 0),            # green
            self.CANONICAL_MUD: (0, 255, 255),            # yellow in BGR
            self.CANONICAL_WATER: (255, 0, 0),            # blue in BGR
            self.CANONICAL_VEGETATION: (0, 90, 0),        # dark green
            self.CANONICAL_OBSTACLE: (0, 0, 255),         # red in BGR
        }

    def print_canonical_tables(self):
        self.get_logger().info("Canonical class table:")
        for class_id in range(0, 7):
            name = self.canonical_name[class_id]
            cost = self.canonical_cost[class_id]
            bgr = self.canonical_bgr[class_id]
            self.get_logger().info(
                f"  {class_id}: {name:22s} cost={cost:3d} bgr={bgr}"
            )

    def build_fallback_label_mapping(self):
        profile = str(self.get_parameter("semantic_profile").value).lower().strip()

        background_label = int(self.get_parameter("background_label").value)
        sidewalk_label = int(self.get_parameter("sidewalk_label").value)
        grass_label = int(self.get_parameter("grass_label").value)

        if profile == "nav2_demo3":
            return {
                background_label: self.CANONICAL_BACKGROUND,
                sidewalk_label: self.CANONICAL_HARD_TRAVERSABLE,
                grass_label: self.CANONICAL_GRASS,
            }

        # RELLIS 7-class model is already canonical.
        # This also works for the Nav2 demo labels if they are:
        #   0 = background, 1 = sidewalk, 2 = grass.
        return {
            0: self.CANONICAL_BACKGROUND,
            1: self.CANONICAL_HARD_TRAVERSABLE,
            2: self.CANONICAL_GRASS,
            3: self.CANONICAL_MUD,
            4: self.CANONICAL_WATER,
            5: self.CANONICAL_VEGETATION,
            6: self.CANONICAL_OBSTACLE,
        }

    @staticmethod
    def normalize_class_name(name: str) -> str:
        name = name.strip().lower()
        name = re.sub(r"[^a-z0-9]+", "_", name)
        name = re.sub(r"_+", "_", name)
        return name.strip("_")

    def class_name_to_canonical(self, class_name: str):
        """
        Map LabelInfo class names to canonical 7-class terrain IDs.

        This is what lets the same node work with:
          - RELLIS 7-class ontology
          - Nav2 demo 3-class ontology
          - similar future ontologies
        """
        n = self.normalize_class_name(class_name)

        background_names = {
            "background",
            "background_unknown",
            "unknown",
            "void",
            "sky",
            "unlabeled",
        }

        hard_traversable_names = {
            "hard_traversable",
            "sidewalk",
            "road",
            "trail",
            "dirt",
            "asphalt",
            "concrete",
            "free_space",
            "traversable",
            "traversible",
        }

        grass_names = {
            "grass",
            "field",
            "lawn",
        }

        mud_names = {
            "mud",
            "wet_soil",
        }

        water_names = {
            "water",
            "puddle",
            "deep_water",
            "standing_water",
        }

        vegetation_names = {
            "vegetation",
            "tree",
            "bush",
            "shrub",
        }

        obstacle_names = {
            "obstacle",
            "object",
            "vehicle",
            "person",
            "pole",
            "fence",
            "barrier",
            "building",
            "log",
            "rubble",
        }

        if n in background_names:
            return self.CANONICAL_BACKGROUND

        if n in hard_traversable_names:
            return self.CANONICAL_HARD_TRAVERSABLE

        if n in grass_names:
            return self.CANONICAL_GRASS

        if n in mud_names:
            return self.CANONICAL_MUD

        if n in water_names:
            return self.CANONICAL_WATER

        if n in vegetation_names:
            return self.CANONICAL_VEGETATION

        if n in obstacle_names:
            return self.CANONICAL_OBSTACLE

        self.get_logger().warn(
            f"Unknown class name in LabelInfo: '{class_name}'. "
            "Mapping to background_unknown."
        )
        return self.CANONICAL_BACKGROUND

    def label_info_callback(self, msg: LabelInfo):
        """
        Update raw label -> canonical label mapping using /segmentation/label_info.

        Example RELLIS:
          0 background -> 0 background_unknown
          1 hard_traversable -> 1 hard_traversable
          2 grass -> 2 grass
          3 mud -> 3 mud
          4 water -> 4 water
          5 vegetation -> 5 vegetation
          6 obstacle -> 6 obstacle

        Example Nav2 demo:
          0 background -> 0 background_unknown
          1 sidewalk -> 1 hard_traversable
          2 grass -> 2 grass
        """
        new_mapping = {}

        for vc in msg.class_map:
            raw_id = int(vc.class_id)
            canonical_id = self.class_name_to_canonical(vc.class_name)
            new_mapping[raw_id] = canonical_id

        if new_mapping:
            self.raw_label_to_canonical = new_mapping
            self.get_logger().info(
                f"Updated raw label -> canonical mapping from LabelInfo: "
                f"{self.raw_label_to_canonical}"
            )

    def map_raw_label_to_canonical(self, raw_label: int):
        return self.raw_label_to_canonical.get(
            int(raw_label),
            self.CANONICAL_BACKGROUND,
        )

    # -------------------------------------------------------------------------
    # Mask handling
    # -------------------------------------------------------------------------
    def mask_callback(self, msg: Image):
        try:
            mask = self.image_to_numpy(msg)

            if mask.ndim == 3:
                self.get_logger().warn(
                    "Segmentation mask topic appears to be RGB/BGR. "
                    "Use /segmentation/mask, not /segmentation/overlay."
                )
                mask = mask[:, :, 0]

            self.latest_mask = mask.astype(np.int32)
            self.latest_mask_stamp = msg.header.stamp

        except Exception as e:
            self.get_logger().warn(f"Failed to parse segmentation mask: {e}")

    def image_to_numpy(self, msg: Image):
        h = msg.height
        w = msg.width
        enc = msg.encoding.lower()

        if enc in ["mono8", "8uc1"]:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return arr.reshape((h, w))

        if enc in ["mono16", "16uc1"]:
            arr = np.frombuffer(msg.data, dtype=np.uint16)
            return arr.reshape((h, w))

        if enc in ["rgb8", "bgr8"]:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return arr.reshape((h, w, 3))

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    # -------------------------------------------------------------------------
    # Point cloud callback
    # -------------------------------------------------------------------------
    def points_callback(self, cloud_msg: PointCloud2):
        now = self.get_clock().now()

        # ---------------------------------------------------------------------
        # Get latest semantic mask if valid
        # ---------------------------------------------------------------------
        mask = None
        if self.latest_mask is not None and self.latest_mask_stamp is not None:
            mask_time = rclpy.time.Time.from_msg(self.latest_mask_stamp)
            age = (now - mask_time).nanoseconds * 1e-9
            max_age = float(self.get_parameter("max_mask_age_sec").value)

            if age <= max_age:
                mask = self.latest_mask
            else:
                self.get_logger().warn(
                    f"Segmentation mask too old: {age:.2f} sec. "
                    "Using geometry only."
                )

        # ---------------------------------------------------------------------
        # TF: point cloud frame -> grid frame
        # ---------------------------------------------------------------------
        try:
            transform = self.tf_buffer.lookup_transform(
                self.grid_frame,
                cloud_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Could not transform {cloud_msg.header.frame_id} "
                f"to {self.grid_frame}: {e}"
            )
            return

        # ---------------------------------------------------------------------
        # Convert PointCloud2 to xyz array
        # ---------------------------------------------------------------------
        try:
            points_xyz = self.pointcloud2_to_xyz_array(cloud_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse point cloud: {e}")
            return

        if points_xyz is None or points_xyz.size == 0:
            return

        # ---------------------------------------------------------------------
        # Align mask with organized point cloud
        # ---------------------------------------------------------------------
        semantic_labels = None
        if mask is not None:
            if points_xyz.ndim == 3:
                if (
                    mask.shape[0] == points_xyz.shape[0]
                    and mask.shape[1] == points_xyz.shape[1]
                ):
                    semantic_labels = mask
                else:
                    self.get_logger().warn(
                        f"Mask shape {mask.shape} does not match point cloud "
                        f"shape {points_xyz.shape[:2]}"
                    )
            else:
                self.get_logger().warn(
                    "Point cloud is not organized, so mask cannot be aligned pixel-wise."
                )

        # ---------------------------------------------------------------------
        # Flatten cloud and labels
        # ---------------------------------------------------------------------
        if points_xyz.ndim == 3:
            points_flat = points_xyz.reshape((-1, 3))

            if semantic_labels is not None:
                labels_flat = semantic_labels.reshape((-1,))
            else:
                labels_flat = None
        else:
            points_flat = points_xyz
            labels_flat = None

        # ---------------------------------------------------------------------
        # Filter invalid points
        # ---------------------------------------------------------------------
        valid = (
            np.isfinite(points_flat[:, 0])
            & np.isfinite(points_flat[:, 1])
            & np.isfinite(points_flat[:, 2])
        )

        points_flat = points_flat[valid]

        if labels_flat is not None:
            labels_flat = labels_flat[valid]

        if points_flat.shape[0] == 0:
            return

        # ---------------------------------------------------------------------
        # Transform points into grid frame
        # ---------------------------------------------------------------------
        points_grid = self.transform_points(points_flat, transform)

        # ---------------------------------------------------------------------
        # Build grid
        # ---------------------------------------------------------------------
        grid_data, semantic_grid = self.build_occupancy_grid(points_grid, labels_flat)

        # ---------------------------------------------------------------------
        # Publish OccupancyGrid
        # ---------------------------------------------------------------------
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.grid_frame

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        grid_msg.info.origin.position.x = self.x_min
        grid_msg.info.origin.position.y = self.y_min
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = grid_data.tolist()

        self.grid_pub.publish(grid_msg)

        # ---------------------------------------------------------------------
        # Publish semantic grid markers
        # ---------------------------------------------------------------------
        semantic_grid_msg = OccupancyGrid()
        semantic_grid_msg.header = grid_msg.header
        semantic_grid_msg.info = grid_msg.info

        # semantic_grid contains:
        # -1 = unobserved
        #  0 = background_unknown
        #  1 = hard_traversable
        #  2 = grass
        #  3 = mud
        #  4 = water
        #  5 = vegetation
        #  6 = obstacle
        semantic_grid_msg.data = semantic_grid.astype(np.int8).tolist()


        sem_unique, sem_counts = np.unique(semantic_grid, return_counts=True)
        self.get_logger().info(
            f"Publishing SemanticGrid topic stats: "
            f"{dict(zip(sem_unique.tolist(), sem_counts.tolist()))}"
        )

        self.semantic_grid_pub.publish(semantic_grid_msg)

        # ---------------------------------------------------------------------
        # Publish colored debug markers
        # ---------------------------------------------------------------------
        self.publish_semantic_markers(semantic_grid, grid_msg.header.stamp)
        self.publish_colored_cost_markers(grid_data, grid_msg.header.stamp)

        # if bool(self.get_parameter("debug_print_grid_stats").value):
        #     self.print_grid_stats(grid_data, semantic_grid)

    # -------------------------------------------------------------------------
    # PointCloud2 parsing
    # -------------------------------------------------------------------------
    def pointcloud2_to_xyz_array(self, cloud_msg: PointCloud2):
        field_names = [f.name for f in cloud_msg.fields]
        if not all(name in field_names for name in ["x", "y", "z"]):
            raise ValueError(f"PointCloud2 missing x/y/z fields. Fields: {field_names}")

        x_offset = next(f.offset for f in cloud_msg.fields if f.name == "x")
        y_offset = next(f.offset for f in cloud_msg.fields if f.name == "y")
        z_offset = next(f.offset for f in cloud_msg.fields if f.name == "z")

        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step

        h = cloud_msg.height
        w = cloud_msg.width

        points = np.empty((h, w, 3), dtype=np.float32)
        data = cloud_msg.data

        for v in range(h):
            row_base = v * row_step
            for u in range(w):
                base = row_base + u * point_step
                x = struct.unpack_from("f", data, base + x_offset)[0]
                y = struct.unpack_from("f", data, base + y_offset)[0]
                z = struct.unpack_from("f", data, base + z_offset)[0]

                points[v, u, 0] = x
                points[v, u, 1] = y
                points[v, u, 2] = z

        if h == 1:
            return points.reshape((-1, 3))

        return points

    # -------------------------------------------------------------------------
    # TF transform helpers
    # -------------------------------------------------------------------------
    def transform_points(self, points, transform: TransformStamped):
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        rotation = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

        transformed = points @ rotation.T
        transformed[:, 0] += tx
        transformed[:, 1] += ty
        transformed[:, 2] += tz

        return transformed

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n == 0.0:
            return np.eye(3, dtype=np.float32)

        x /= n
        y /= n
        z /= n
        w /= n

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )

    # -------------------------------------------------------------------------
    # Main grid generation
    # -------------------------------------------------------------------------
    def build_occupancy_grid(self, points, labels):
        ground_z_min = float(self.get_parameter("ground_z_min").value)
        obstacle_z_threshold = float(self.get_parameter("obstacle_z_threshold").value)
        min_points_per_cell = int(self.get_parameter("min_points_per_cell").value)
        geometry_obstacle_override = bool(
            self.get_parameter("geometry_obstacle_override").value
        )

        default_observed_cost = int(self.get_parameter("default_observed_cost").value)
        default_observed_class = int(self.get_parameter("default_observed_class").value)

        obstacle_cost = int(self.get_parameter("obstacle_cost").value)

        num_cells = self.width * self.height

        # Output grids
        grid = np.full(num_cells, -1, dtype=np.int8)
        semantic_grid = np.full(num_cells, self.CANONICAL_UNKNOWN, dtype=np.int16)

        # Per-cell geometry stats
        min_z = np.full(num_cells, np.inf, dtype=np.float32)
        max_z = np.full(num_cells, -np.inf, dtype=np.float32)
        counts = np.zeros(num_cells, dtype=np.int32)

        # Per-cell semantic votes over canonical classes 0..6
        semantic_vote_counts = np.zeros((num_cells, 7), dtype=np.int32)

        # ---------------------------------------------------------------------
        # ROI filter
        # ---------------------------------------------------------------------
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        roi = (
            (x >= self.x_min)
            & (x < self.x_max)
            & (y >= self.y_min)
            & (y < self.y_max)
            & (z >= ground_z_min)
            & (z <= 2.0)
        )

        points = points[roi]

        if labels is not None:
            labels = labels[roi]

        if points.shape[0] == 0:
            return grid, semantic_grid

        # ---------------------------------------------------------------------
        # Point -> cell index
        # ---------------------------------------------------------------------
        ix = ((points[:, 0] - self.x_min) / self.resolution).astype(np.int32)
        iy = ((points[:, 1] - self.y_min) / self.resolution).astype(np.int32)

        valid = (
            (ix >= 0)
            & (ix < self.width)
            & (iy >= 0)
            & (iy < self.height)
        )

        ix = ix[valid]
        iy = iy[valid]
        pz = points[:, 2][valid]

        if labels is not None:
            labels = labels[valid]

        if ix.shape[0] == 0:
            return grid, semantic_grid

        cell_indices = iy * self.width + ix

        # ---------------------------------------------------------------------
        # Accumulate cell stats and semantic votes
        # ---------------------------------------------------------------------
        for i, cell in enumerate(cell_indices):
            z_val = pz[i]

            counts[cell] += 1

            if z_val < min_z[cell]:
                min_z[cell] = z_val

            if z_val > max_z[cell]:
                max_z[cell] = z_val

            if labels is not None:
                raw_label = int(labels[i])
                canonical_label = self.map_raw_label_to_canonical(raw_label)

                if 0 <= canonical_label <= 6:
                    semantic_vote_counts[cell, canonical_label] += 1

        observed = counts >= min_points_per_cell

        # Default observed cells if semantic data is absent or no class vote exists.
        grid[observed] = np.clip(default_observed_cost, -1, 100)
        semantic_grid[observed] = default_observed_class

        # ---------------------------------------------------------------------
        # Semantic terrain voting
        # semantic_grid convention:
        #   -1 = unobserved
        #    0 = background_unknown
        #    1 = hard_traversable
        #    2 = grass
        #    3 = mud
        #    4 = water
        #    5 = vegetation
        #    6 = obstacle
        # ---------------------------------------------------------------------
        if labels is not None:
            observed_cells = np.where(observed)[0]

            for cell in observed_cells:
                votes = semantic_vote_counts[cell]
                total_votes = int(np.sum(votes))

                if total_votes == 0:
                    continue

                winning_class = int(np.argmax(votes))
                semantic_grid[cell] = winning_class
                grid[cell] = np.clip(self.canonical_cost[winning_class], -1, 100)

        # ---------------------------------------------------------------------
        # Geometry obstacle override
        # Obstacle should override semantic traversability if height says no-go.
        # ---------------------------------------------------------------------
        if geometry_obstacle_override:
            height_span = max_z - min_z
            high_point = max_z > obstacle_z_threshold
            tall_cell = height_span > obstacle_z_threshold

            obstacle = observed & (high_point | tall_cell)

            grid[obstacle] = np.clip(obstacle_cost, -1, 100)
            semantic_grid[obstacle] = self.CANONICAL_OBSTACLE

        return grid, semantic_grid

    # -------------------------------------------------------------------------
    # Semantic colored MarkerArray
    # Uses the same canonical colors as the semantic overlay.
    # -------------------------------------------------------------------------
    def bgr_to_rgba(self, bgr, alpha):
        b, g, r = bgr

        color = ColorRGBA()
        color.r = float(r) / 255.0
        color.g = float(g) / 255.0
        color.b = float(b) / 255.0
        color.a = float(alpha)

        return color

    def publish_semantic_markers(self, semantic_grid, stamp):
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self.grid_frame
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker = Marker()
        marker.header.frame_id = self.grid_frame
        marker.header.stamp = stamp
        marker.ns = "semantic_terrain_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.04
        marker.lifetime.sec = 0

        alpha = float(self.get_parameter("semantic_marker_alpha").value)

        for iy in range(self.height):
            for ix in range(self.width):
                idx = iy * self.width + ix
                semantic_class = int(semantic_grid[idx])

                # Skip unobserved cells.
                if semantic_class < 0:
                    continue

                p = Point()
                p.x = float(self.x_min + (ix + 0.5) * self.resolution)
                p.y = float(self.y_min + (iy + 0.5) * self.resolution)
                p.z = -0.03
                marker.points.append(p)

                bgr = self.canonical_bgr.get(
                    semantic_class,
                    self.canonical_bgr[self.CANONICAL_BACKGROUND],
                )

                # Make obstacle slightly more opaque.
                class_alpha = alpha
                if semantic_class == self.CANONICAL_OBSTACLE:
                    class_alpha = max(alpha, 0.80)

                marker.colors.append(self.bgr_to_rgba(bgr, class_alpha))

        marker_array.markers.append(marker)
        self.semantic_marker_pub.publish(marker_array)

    # -------------------------------------------------------------------------
    # Cost colored MarkerArray
    # This colors cells based on numeric cost, not semantic class.
    # -------------------------------------------------------------------------
    def publish_colored_cost_markers(self, grid_data, stamp):
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self.grid_frame
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker = Marker()
        marker.header.frame_id = self.grid_frame
        marker.header.stamp = stamp
        marker.ns = "terrain_cost_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.03
        marker.lifetime.sec = 0

        alpha = float(self.get_parameter("cost_marker_alpha").value)

        for iy in range(self.height):
            for ix in range(self.width):
                idx = iy * self.width + ix
                cost = int(grid_data[idx])

                # Skip unknown cells.
                if cost < 0:
                    continue

                p = Point()
                p.x = float(self.x_min + (ix + 0.5) * self.resolution)
                p.y = float(self.y_min + (iy + 0.5) * self.resolution)
                p.z = -0.02
                marker.points.append(p)

                color = ColorRGBA()

                if cost >= 100:
                    # occupied obstacle: black
                    color.r = 0.0
                    color.g = 0.0
                    color.b = 0.0
                    color.a = 0.90

                elif cost >= 90:
                    # very high cost: red
                    color.r = 1.0
                    color.g = 0.0
                    color.b = 0.0
                    color.a = max(alpha, 0.70)

                elif cost >= 75:
                    # high cost: orange
                    color.r = 1.0
                    color.g = 0.45
                    color.b = 0.0
                    color.a = alpha

                elif cost >= 50:
                    # uncertain / medium: yellow
                    color.r = 1.0
                    color.g = 1.0
                    color.b = 0.0
                    color.a = alpha

                elif cost >= 25:
                    # soft traversable: green-yellow
                    color.r = 0.5
                    color.g = 1.0
                    color.b = 0.0
                    color.a = alpha

                else:
                    # low cost / free: green
                    color.r = 0.0
                    color.g = 1.0
                    color.b = 0.0
                    color.a = max(0.30, alpha * 0.8)

                marker.colors.append(color)

        marker_array.markers.append(marker)
        self.cost_marker_pub.publish(marker_array)

    # -------------------------------------------------------------------------
    # Debug
    # -------------------------------------------------------------------------
    def named_stats(self, values, counts, is_semantic=False):
        stats = {}

        for value, count in zip(values.tolist(), counts.tolist()):
            value = int(value)
            count = int(count)

            if is_semantic:
                name = self.canonical_name.get(value, f"class_{value}")
                stats[f"{value}:{name}"] = count
            else:
                stats[value] = count

        return stats

    def print_grid_stats(self, grid_data, semantic_grid):
        grid_unique, grid_counts = np.unique(grid_data, return_counts=True)
        semantic_unique, semantic_counts = np.unique(semantic_grid, return_counts=True)

        grid_stats = self.named_stats(grid_unique, grid_counts, is_semantic=False)
        semantic_stats = self.named_stats(
            semantic_unique,
            semantic_counts,
            is_semantic=True,
        )

        self.get_logger().info(f"OccupancyGrid cost stats: {grid_stats}")
        self.get_logger().info(f"SemanticGrid class stats: {semantic_stats}")


def main(args=None):
    rclpy.init(args=args)
    node = TerrainObstacleGridNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()