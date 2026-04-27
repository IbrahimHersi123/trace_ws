#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.bridge = CvBridge()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/kinect/image_raw')
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
        self.declare_parameter('min_circularity', 0.55)  # Applied to convex hull, not raw contour
        self.declare_parameter('min_radius', 10)

        # Read once-only params
        self.image_topic = self.get_parameter('image_topic').value
        self.show_window  = self.get_parameter('show_window').value

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.detection_pub = self.create_publisher(Bool, '/ball_detected', 10)

        self.get_logger().info('BallDetectorNode started.')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info('Publishing detections on: /ball_detected')

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
        self.min_area   = self.get_parameter('min_area').value
        self.min_circ   = self.get_parameter('min_circularity').value
        self.min_radius = self.get_parameter('min_radius').value

    # ── Main callback ──────────────────────────────────────────────────────────
    def image_callback(self, msg: Image):
        self._read_params()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # ── HSV mask ──────────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = np.ones((9, 9), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.GaussianBlur(mask, (7, 7), 0)

        # ── Contour detection ─────────────────────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        balls_found = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue

            # ── Use convex hull instead of raw contour for shape checks ───────
            # This fills in the dents and holes caused by the ball's text/seam
            # so circularity is measured on the overall shape, not the jagged edge
            hull = cv2.convexHull(contour)

            hull_area      = cv2.contourArea(hull)
            hull_perimeter = cv2.arcLength(hull, True)

            if hull_perimeter == 0:
                continue

            circularity = 4 * math.pi * hull_area / (hull_perimeter ** 2)
            if circularity < self.min_circ:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(hull)
            if radius < self.min_radius:
                continue

            # Convexity defect ratio — how much of the hull is filled by the contour
            # Low values mean the contour has large chunks missing (not a ball)
            convexity = area / hull_area if hull_area > 0 else 0
            if convexity < 0.7:
                continue

            x, y, w, h = cv2.boundingRect(hull)
            balls_found.append({
                'bbox':        (x, y, w, h),
                'center':      (int(cx), int(cy)),
                'radius':      int(radius),
                'area':        int(area),
                'circularity': circularity,
                'convexity':   convexity,
            })

        # ── Publish ───────────────────────────────────────────────────────────
        msg_out      = Bool()
        msg_out.data = len(balls_found) > 0
        self.detection_pub.publish(msg_out)

        # ── Draw overlays ─────────────────────────────────────────────────────
        display = frame.copy()

        for i, ball in enumerate(balls_found):
            x, y, w, h = ball['bbox']
            cx, cy     = ball['center']
            r          = ball['radius']

            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), r, (0, 200, 255), 2)
            cv2.circle(display, (cx, cy), 4, (0, 0, 255), -1)

            label = (
                f'Ball {i+1} | r={r}px '
                f'circ={ball["circularity"]:.2f} '
                f'cvx={ball["convexity"]:.2f}'
            )
            cv2.putText(
                display, label,
                (x, max(y - 8, 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2,
            )

        # Status banner
        if balls_found:
            status_text  = f'DETECTED  ({len(balls_found)} ball{"s" if len(balls_found) > 1 else ""})'
            status_color = (0, 255, 0)
        else:
            status_text  = 'NO BALL DETECTED'
            status_color = (0, 0, 255)

        cv2.rectangle(display, (0, 0), (400, 36), (0, 0, 0), -1)
        cv2.putText(display, status_text, (8, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

        # Active param readout at bottom
        hsv_text = (
            f'H:[{self.lower_hsv[0]}-{self.upper_hsv[0]}] '
            f'S:[{self.lower_hsv[1]}-{self.upper_hsv[1]}] '
            f'V:[{self.lower_hsv[2]}-{self.upper_hsv[2]}] '
            f'circ>={self.min_circ:.2f} area>={self.min_area}'
        )
        cv2.rectangle(display, (0, display.shape[0] - 30),
                      (620, display.shape[0]), (0, 0, 0), -1)
        cv2.putText(display, hsv_text, (8, display.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        if self.show_window:
            cv2.imshow('Ball Detector', display)
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
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()