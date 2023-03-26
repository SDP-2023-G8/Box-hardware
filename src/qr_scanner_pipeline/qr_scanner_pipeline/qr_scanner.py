# ROS
import rclpy
import socketio
import subprocess
import time
import psutil
from rclpy.node import Node
from std_msgs.msg import String

# OpenCV
import cv2

sio = socketio.Client()
streaming = False
listening = True
s = None
pa = None
p = None
pid = None
qr_node = None

class QRCodeNode(Node):
    def __init__(self):
        super().__init__("qr_code_node")
        self.get_logger().info("*** Initializing QR scanner node. ***")
        self.init_params()

        # Camera
        self.cap_ = cv2.VideoCapture(0)
        self.detector_ = cv2.QRCodeDetector()
        self.get_logger().info("Camera initialized.")       

        # QR message publisher
        self.qr_dec_pub_ = self.create_publisher(String, "~/qr_decoded", 10)
        timer_period = 1. / self.cam_fps_.value # Seconds
        self.pub_timer = self.create_timer(timer_period, self.publish_qr_decoded_message)
        self.get_logger().info("Publishing decoded QR to %s" % self.qr_dec_pub_.topic_name)
        self.get_logger().info("*** QR scanner node initialized successfully. ***")

    def init_params(self):
        self.get_logger().info("*** Initializing parameters. ***")

        self.display_ = self.declare_parameter("display", False)
        self.get_logger().info("display: {0}".format(self.display_.value))

        self.cam_fps_ = self.declare_parameter("cam_fps", 24)
        self.get_logger().info("cam_fps: {0}".format(self.cam_fps_.value))

        self.get_logger().info("*** Parameters initialized sucessfully ***")
    
    def publish_qr_decoded_message(self):
        self.get_logger().debug("QR message publisher called.")
        if self.qr_dec_pub_.get_subscription_count() == 0:
            self.get_logger().debug("No subscribers, no message will be published.")
            return
        
        # Detect and decode QR message
        ret, img = self.cap_.read()
        if not ret and not streaming:
            self.get_logger().debug("Could not receive camera image.")
            return
        
        try:
            data, bbox, _ = self.detector_.detectAndDecode(img)
        except Exception as e:
            if not streaming:
                self.get_logger().warn("Exception while detecting QR: %s." % e.__str__())
            return
        
        if self.display_.value:
            cv2.imshow("QR", img)
            cv2.waitKey(1)

        # Publish 
        if data:
            self.get_logger().debug("QR code detected and decoded %s." % data)
            msg = String()
            msg.data = data
            self.qr_dec_pub_.publish(msg)

        if not streaming:
            self.get_logger().debug("No QR messages detected.")

def kill(proc_id):
    process = psutil.Process(proc_id)
    for proc in process.children(recursive=True):
        proc.kill()

    process.kill()

@sio.on("startVideo")
def start_video():
    global qr_node, p, pid, streaming
    qr_node.get_logger().debug("Started the video")

    p = subprocess.Popen("/home/sdp8/start_stream.sh")
    pid = p.pid

    qr_node.cap_ = cv2.VideoCapture('http://localhost:8090/?action=stream')
    streaming = True

@sio.on("stopVideo")
def stop_video():
    global qr_node, p, pid, streaming
    qr_node.get_logger().debug("Stopped the video")

    kill(pid)
    time.sleep(1)
    qr_node.cap_ = cv2.VideoCapture(0)
    streaming = False

def listen():
    global s, q, listening
    while listening:
        if not q.empty():
            frame = q.get()
            s.write(frame)

    return  # Kill thread if no longer listening

def main(args=None):
    global qr_node, sio

    rclpy.init(args=args)

    qr_node = QRCodeNode()

    # Set up socket using static IP
    sio.connect("http://192.168.43.181:5000")

    rclpy.spin(qr_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
