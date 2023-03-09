# Python
from enum import Enum
import time

# ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_srvs.srv import SetBool
from std_msgs.msg import String

from qr_verify.qr_verify import QRVerify


class State(Enum):
    INIT = 0
    DOOR_CLOSED = 1
    VERIFYING_QR = 2
    DOOR_OPENED = 3


class StateMachine(Node):
    STATE_PUB_TOPIC_NAME_ = "~/current_state"
    STATE_PUB_TYPE = String
    
    QR_TOPIC_NAME = "/qr_msg"
    QR_MSG_TYPE = String

    LOCK_SERVICE_NAME = "/lock_srv"
    LOCK_SERVICE_TYPE = SetBool

    LED_SERVICE_NAME = "/led_srv"
    LED_SERVICE_TYPE = SetBool

    current_state_ = State.INIT
    door_open_time_ = 5
 
    def __init__(self):
        super().__init__("state_machine")
        self.init_params()

        # Current state publisher
        self.state_pub_ = self.create_publisher(self.STATE_PUB_TYPE, self.STATE_PUB_TOPIC_NAME_, 10)
        self.get_logger().info("Current state publisher topic name: {0}".format(self.state_pub_.topic_name))
        self.pub_timer = self.create_timer(1. / self.state_pub_freq_.value, self.publish_state)

        # Locking / unlocking the door
        self.door_lock_client_ = self.create_client(self.LOCK_SERVICE_TYPE, self.LOCK_SERVICE_NAME)
        self.door_lock_client_.wait_for_service(30)
        self.get_logger().info("Door lock service server name: {0}; service type: {1}".format(
            self.door_lock_client_.srv_name,
            self.door_lock_client_.srv_type))
        
        # Lock the door
        self.send_door_request(True)

        # LED 
        self.led_client_ = self.create_client(self.LED_SERVICE_TYPE, self.LED_SERVICE_NAME)
        self.door_lock_client_.wait_for_service(30)
        self.send_led_request(False)

        self.init_waiting_state()

        self.get_logger().info("Created a new subscription to {0}".format(self.qr_msg_subscription.topic_name))
        self.get_logger().info("State machine initialized")

    def init_params(self):
        self.get_logger().info("*** Initializing params ***")

        self.state_pub_freq_ = self.declare_parameter("state_pub_freq", 15)
        self.get_logger().info("State publisher frequency: {0}Hz".format(self.state_pub_freq_.value))

        self.get_logger().info("*** Parameters initialized successfully ***")

    def init_waiting_state(self):
        self.get_logger().info("Door locked.")
        self.current_state_ = State.DOOR_CLOSED
        # Start QR subscription
        self.qr_msg_subscription= self.create_subscription(
            self.QR_MSG_TYPE, 
            self.QR_TOPIC_NAME, 
            self.qr_msg_callback,
            1)
        
    def verify_qr(self, msg):
        verifier = QRVerify(hostname="joflesan-ubuntu.local", host_port=5000)
        try:
            return verifier.verify(msg)
        except ConnectionError as e:
            return None

    def send_door_request(self, data):
        door_req = self.LOCK_SERVICE_TYPE.Request()
        door_req.data = data
        door_fut = self.door_lock_client_.call_async(door_req)
        self.get_logger().info("Sending a request to the door lock service")
        return door_fut
    
    def send_led_request(self, data):
        led_req = self.LED_SERVICE_TYPE.Request()
        led_req.data = data
        led_fut = self.led_client_.call_async(led_req)
        self.get_logger().info("Sending a request to the led service")
        return led_fut
    
    # Locks the door and changes the state
    def close_door_callback(self):
        self.send_door_request(True)
        self.send_led_request(False)
        self.current_state_ = State.DOOR_CLOSED

        # Execute only once
        self.destroy_timer(self.close_door_timer_)

    def verify_qr_callback(self, msg):
        self.get_logger().info("Verifying {0}".format(msg))
        result = self.verify_qr(msg)
        self.get_logger().info("Verification result: {0}".format(result))
        if result:
            self.send_door_request(False)
            self.send_led_request(True)
            self.current_state_ = State.DOOR_OPENED
            print("Opening the door for {0} seconds".format(self.door_open_time_))
            self.close_door_timer_ = self.create_timer(self.door_open_time_, self.close_door_callback)
        else:
            self.current_state_ = State.DOOR_CLOSED

        # Execute only once
        self.destroy_timer(self.verify_timer_)
    
    # Listens for QR codes when in DOOR_CLOSED state
    def qr_msg_callback(self, msg):
        if self.current_state_ != State.DOOR_CLOSED:
            return
        msg_content = msg.data
        self.get_logger().info("Received a QR message: {0}".format(msg_content))
        self.current_state_ = State.VERIFYING_QR
        self.verify_timer_ = self.create_timer(0.1, lambda: self.verify_qr_callback(msg.data))

    def publish_state(self):
        if self.state_pub_.get_subscription_count() > 0:
            self.get_logger().debug("Publishing current state.")
            msg = String()
            msg.data = str(self.current_state_)
            self.state_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = StateMachine()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
