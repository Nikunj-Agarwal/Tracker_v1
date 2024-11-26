import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallTracker(Node):
    def __init__(self):
        super().__init__('ball_tracker')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',  # Adjust the topic name if different
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Point, 'ball_position', 10)
        self.bridge = CvBridge()
        self.blueLower = (100, 150, 0)
        self.blueUpper = (140, 255, 255)
        self.circle_threshold = 0.75  # Tunable parameter for area coverage threshold
        self.PI = 3.141592653589793
        cv2.namedWindow("Ball Tracker", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("BallTracker node has been started.")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame, center, mask = self.detect_and_track(frame)
        cv2.imshow("Ball Tracker", frame)
        cv2.waitKey(1)
        if center is not None:
            point_msg = Point()
            point_msg.x = float(center[0])
            point_msg.y = float(center[1])
            point_msg.z = 0.0
            self.publisher.publish(point_msg)
            self.get_logger().info(f"Ball detected at: {center}")

    def detect_and_track(self, frame):
        frame = cv2.resize(frame, (640, 480))
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blueLower, self.blueUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        center = None
        circles = []
        max_radius = 0
        max_center = None

        for cnt in cnts:
            # Calculate the area of the contour
            contour_area = cv2.contourArea(cnt)

            # Get the minimum enclosing circle and compute its area
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            circle_area = self.PI * radius * radius

            # Ignore very small circles
            if radius < 10:
                continue

            # If the area of the contour makes up for at least the threshold of the enclosing circle,
            # then the contour resembles a circle and we include it
            if contour_area / circle_area > self.circle_threshold:
                center = (int(x), int(y))
                circles.append((center, int(radius)))

                # Did we find a new biggest circle?
                if radius > max_radius:
                    max_radius = radius
                    max_center = center

        # Mark all identified circles
        for (center, radius) in circles:
            cv2.circle(frame, center, radius, (0, 255, 255), 2)
            cv2.circle(frame, center, 3, (0, 255, 255), -1)

        return frame, max_center, mask

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ball_tracker = BallTracker()
    try:
        rclpy.spin(ball_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        ball_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
