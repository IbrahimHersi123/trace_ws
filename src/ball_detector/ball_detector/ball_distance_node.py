#!/usr/bin/env python3

import math

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class BallDistanceNode(Node):
    def __init__(self):
        super().__init__('ball_distance_node')

        self.bridge = CvBridge()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('show_window', True)

        # HSV — tuned for yellow-green tennis ball on Kinect v1
        self.declare_parameter('hsv_h_min', 25)
        self.declare_parameter('hsv_h_max', 75)
        self.declare_parameter('hsv_s_min', 50)
        self.declare_parameter('hsv_s_max', 255)
        self.declare_parameter('hsv_v_min', 50)
        self.declare_parameter('hsv_v_max', 255)

        # Detection thresholds
        self.declare_parameter('min_area', 2000)
        self.declare_parameter('min_circularity', 0.55)
        self.declare_parameter('min_radius', 10)

        # Minimum number of valid depth pixels required for a reliable reading
        # If fewer valid pixels than this are found, distance is reported as -1.0
        self.declare_parameter('min_valid_depth_pixels', 10)

        # Read once-only params
        self.show_window = self.get_parameter('show_window').value

        # ── Synchronised subscribers ───────────────────────────────────────────
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/kinect/image_raw')
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/kinect/depth/image_raw')

        # slop=0.1 means frames within 100ms of each other are considered a match
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.sync_callback)

        # ── Publishers ────────────────────────────────────────────────────────
        # Bool — true if at least one ball detected
        self.detected_pub = self.create_publisher(Bool, '/ball_detected', 10)

        # PoseArray — one Pose per detected ball
        # pose.position.x = image center x (pixels)
        # pose.position.y = image center y (pixels)
        # pose.position.z = distance in metres (-1.0 if depth unavailable)
        self.pose_pub = self.create_publisher(PoseArray, '/ball_poses', 10)

        self.get_logger().info('BallDistanceNode started.')
        self.get_logger().info('Subscribing to: /kinect/image_raw + /kinect/depth/image_raw')
        self.get_logger().info('Publishing on: /ball_detected, /ball_poses')

    # ── Read all tunable params fresh every frame ──────────────────────────────
    def _read_params(self):
        self.lower_hsv = np.array([
            self.get_parameter('hsv_h_min').value,
            self.get_parameter('hsv_s_min').value,
            self.get_parameter('hsv_v_min').value,
        ], dtype=np.uint8)
        self.upper_hsv = np.array([
            self.get_parameter('hsv_h_max').value,
            self.get_parameter('hsv_s_max').value,
            self.get_parameter('hsv_v_max').value,
        ], dtype=np.uint8)
        self.min_area        = self.get_parameter('min_area').value
        self.min_circ        = self.get_parameter('min_circularity').value
        self.min_radius      = self.get_parameter('min_radius').value
        self.min_valid_depth = self.get_parameter('min_valid_depth_pixels').value

    # ── Ball detection on RGB frame (same logic as ball_detector_node) ─────────
    def _detect_balls(self, frame):
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            hull           = cv2.convexHull(contour)
            hull_area      = cv2.contourArea(hull)
            hull_perimeter = cv2.arcLength(hull, True)

            if hull_perimeter == 0 or hull_area == 0:
                continue

            circularity = 4 * math.pi * hull_area / (hull_perimeter ** 2)
            if circularity < self.min_circ:
                continue

            convexity = area / hull_area
            if convexity < 0.7:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(hull)
            if radius < self.min_radius:
                continue

            x, y, w, h = cv2.boundingRect(hull)
            balls.append({
                'center':      (int(cx), int(cy)),
                'radius':      int(radius),
                'bbox':        (x, y, w, h),
                'circularity': circularity,
                'convexity':   convexity,
            })

        return balls, mask

    # ── Sample depth at ball location ──────────────────────────────────────────
    def _estimate_distance(self, depth_frame, cx, cy, radius):
        # Create a circular mask at the ball's location in the depth image
        # Shrink radius by 20% to avoid sampling background pixels at the edge
        sample_radius = max(int(radius * 0.8), 5)
        circle_mask   = np.zeros(depth_frame.shape, dtype=np.uint8)
        cv2.circle(circle_mask, (cx, cy), sample_radius, 255, -1)

        # Extract depth values within the circle
        depth_values = depth_frame[circle_mask == 255]

        # Filter out zero values (no reading) and implausible values
        # Kinect v1 reliable range is 500mm–4500mm
        valid_depths = depth_values[
            (depth_values > 500) & (depth_values < 4500)
        ]

        if len(valid_depths) < self.min_valid_depth:
            return -1.0  # Not enough valid readings

        # Median is robust against edge noise and the ball's seam/text holes
        distance_mm = float(np.median(valid_depths))
        return distance_mm / 1000.0  # Convert to metres

    # ── Synchronised callback ──────────────────────────────────────────────────
    def sync_callback(self, rgb_msg: Image, depth_msg: Image):
        self._read_params()

        try:
            rgb_frame   = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            # 16UC1 — each pixel is distance in mm as uint16
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # ── Detect balls in RGB ───────────────────────────────────────────────
        balls, mask = self._detect_balls(rgb_frame)

        # ── Estimate distance for each ball ───────────────────────────────────
        for ball in balls:
            cx, cy = ball['center']
            r      = ball['radius']
            ball['distance_m'] = self._estimate_distance(depth_frame, cx, cy, r)

        # ── Publish /ball_detected ────────────────────────────────────────────
        detected_msg      = Bool()
        detected_msg.data = len(balls) > 0
        self.detected_pub.publish(detected_msg)

        # ── Publish /ball_poses ───────────────────────────────────────────────
        pose_array              = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'kinect_rgb'

        for ball in balls:
            pose = Pose()
            pose.position.x = float(ball['center'][0])   # pixel x
            pose.position.y = float(ball['center'][1])   # pixel y
            pose.position.z = ball['distance_m']          # metres, -1 if unavailable
            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)

        # ── Draw overlays ─────────────────────────────────────────────────────
        display = rgb_frame.copy()

        for i, ball in enumerate(balls):
            x, y, w, h = ball['bbox']
            cx, cy     = ball['center']
            r          = ball['radius']
            dist       = ball['distance_m']

            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), r,  (0, 200, 255), 2)
            cv2.circle(display, (cx, cy), 4,  (0, 0, 255),   -1)

            # Show distance on the bounding box — -1.0 means no depth reading
            if dist > 0:
                dist_text = f'{dist:.2f}m'
                dist_color = (0, 255, 0)
            else:
                dist_text  = 'no depth'
                dist_color = (0, 165, 255)

            cv2.putText(
                display,
                f'Ball {i+1} | {dist_text}  circ={ball["circularity"]:.2f}',
                (x, max(y - 8, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, dist_color, 2,
            )

        # Status banner
        if balls:
            status_text  = f'DETECTED  ({len(balls)} ball{"s" if len(balls) > 1 else ""})'
            status_color = (0, 255, 0)
        else:
            status_text  = 'NO BALL DETECTED'
            status_color = (0, 0, 255)

        cv2.rectangle(display, (0, 0), (400, 36), (0, 0, 0), -1)
        cv2.putText(display, status_text, (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        if self.show_window:
            cv2.imshow('Ball Distance', display)
            cv2.imshow('HSV Mask',      mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit key pressed — shutting down.')
                rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()