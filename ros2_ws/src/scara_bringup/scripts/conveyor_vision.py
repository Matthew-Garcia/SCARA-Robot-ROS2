#!/usr/bin/env python3
"""OpenCV color segmentation for the conveyor sorting demonstration."""
import json

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ConveyorVision(Node):
    def __init__(self):
        super().__init__('conveyor_vision')
        self.bridge = CvBridge()
        self.detection = self.create_publisher(String, '/conveyor/vision/detection', 10)
        self.debug = self.create_publisher(Image, '/conveyor/vision/debug', 10)
        self.create_subscription(Image, '/conveyor/camera/image_raw', self.process, 10)
        self.get_logger().info('OpenCV conveyor detector ready for red, green and blue cubes.')

    def process(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = {
            'red': cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 110, 70]), np.array([10, 255, 255])),
                cv2.inRange(hsv, np.array([170, 110, 70]), np.array([180, 255, 255]))),
            'green': cv2.inRange(hsv, np.array([38, 80, 55]), np.array([88, 255, 255])),
            'blue': cv2.inRange(hsv, np.array([92, 90, 55]), np.array([132, 255, 255])),
        }
        best = None
        for color, mask in masks.items():
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < 120:
                continue
            moments = cv2.moments(contour)
            if moments['m00'] == 0:
                continue
            center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
            if best is None or area > best[0]:
                best = (area, color, center, contour)
        debug = frame.copy()
        if best:
            area, color, (cx, cy), contour = best
            cv2.drawContours(debug, [contour], -1, (255, 255, 255), 2)
            cv2.circle(debug, (cx, cy), 6, (0, 255, 255), -1)
            cv2.putText(debug, f'{color}: {area:.0f}px', (cx+10, cy-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 2)
            output = String()
            output.data = json.dumps({'color': color, 'cx': cx, 'cy': cy, 'area': area})
            self.detection.publish(output)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_message.header = message.header
        self.debug.publish(debug_message)


def main():
    rclpy.init()
    node = ConveyorVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
