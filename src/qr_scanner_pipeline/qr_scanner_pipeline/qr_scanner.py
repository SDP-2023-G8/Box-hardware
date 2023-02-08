# ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# OpenCV
import cv2


class QRCodeNode(Node):
    def __init__(self):
        super().__init__("QRCodeNode")
        self.get_logger().info("*** Initializing QR scanner node. ***")

        # Camera
        self.cap_ = cv2.VideoCapture(0)
        self.detector_ = cv2.QRCodeDetector()
        self.get_logger().debug("Camera initialized.")

        # QR message publisher
        self.qr_dec_pub_ = self.create_publisher(String, "~/qr_decoded", 10)
        timer_period = 0.02 # Seconds
        self.pub_timer = self.create_timer(timer_period, self.publish_qr_decoded_message)
        self.get_logger().info("Publishing decoded QR to %s" % self.qr_dec_pub_.topic_name)
        self.get_logger().info("*** QR scanner node initialized successfully. ***")
    
    def publish_qr_decoded_message(self):
        self.get_logger().debug("QR message publisher called.")
        if self.qr_dec_pub_.get_subscription_count() == 0:
            self.get_logger().debug("No subscribers, no message will be published.")
            return
        
        # Detect and decode QR message
        ret, img = self.cap_.read()
        if not ret:
            self.get_logger().warn("Could not receive camera image.")
            return
        try:
            data, bbox, _ = self.detector_.detectAndDecode(img)
        except Exception as e:
            self.get_logger().warn("Exception while detecting QR: %s." % e.__str__())
            return

        cv2.imshow("QR", img)
        cv2.waitKey(1)
        # Publish 
        if data:
            self.get_logger().debug("QR code detected and decoded %s." % data)
            msg = String()
            msg.data = data
            self.qr_dec_pub_.publish(msg)
        self.get_logger().debug("No QR messages detected.")


def main(args=None):
    rclpy.init(args=args)

    qr_node = QRCodeNode()

    rclpy.spin(qr_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
