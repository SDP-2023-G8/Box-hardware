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
    WAITING_FOR_QR = 1
    DOOR_OPENED = 2


class StateMachine(Node):
    STATE_PUB_TOPIC_NAME_ = "~/current_state"
    STATE_PUB_TYPE = String
    
    QR_TOPIC_NAME = "/qr_msg"
    QR_MSG_TYPE = String

    LOCK_SERVICE_NAME = "/lock_srv"
    LOCK_SERVICE_TYPE = SetBool

    CURRENT_STATE_ = State.INIT
 
    def __init__(self):
        super().__init__("state_machine")
        self.init_params()
        self.submitted_qrs_ = set()

        # Current state publisher
        self.state_pub_ = self.create_publisher(self.STATE_PUB_TYPE, self.STATE_PUB_TOPIC_NAME_, 10)
        self.get_logger().info("Current state publisher topic name: {0}".format(self.state_pub_.topic_name))
        self.pub_timer = self.create_timer(1. / self.state_pub_freq_.value, self.publish_state)

        # Locking / unlocking the door
        self.door_lock_client_ = self.create_client(self.LOCK_SERVICE_TYPE, self.LOCK_SERVICE_NAME)
        self.door_lock_client_.wait_for_service(3600)
        self.get_logger().info("Door lock service server name: {0}; service type: {1}".format(
            self.door_lock_client_.srv_name,
            self.door_lock_client_.srv_type))
        self.set_state_(State.WAITING_FOR_QR)


    def init_params(self):
        self.get_logger().info("*** Initializing params ***")

        self.state_pub_freq_ = self.declare_parameter("state_pub_freq", 15)
        self.get_logger().info("State publisher frequency: {0}Hz".format(self.state_pub_freq_.value))

        self.get_logger().info("*** Parameters initialized successfully ***")

    def set_state_(self, new_state, **kwargs):
        self.get_logger().info("State {0} requested.".format(str(new_state)))
        if self.CURRENT_STATE_ == new_state:
            return

        if new_state == State.WAITING_FOR_QR:
            # Lock the door and start listening
            # Asynchronous programming prevents deadlocks
            self.door_req_future = self.send_door_request(True)
            self.init_waiting_state()
            self.get_logger().info("Created a new subscription to {0}".format(self.qr_msg_subscription.topic_name))

        elif new_state == State.DOOR_OPENED:
            qr_msg = kwargs['qr_msg']
            self.get_logger().info("Veryfing QR: {0}".format(qr_msg))
            if self.verifyQR(qr_msg):
                self.get_logger().info("QR code authorized.")
                self.door_req_future = self.send_door_request(False)
                self.init_open_state()
                self.get_logger().info("Door opened for 5s")
                time.sleep(5)
                self.door_req_future = self.send_door_request(True)
                self.set_state_(State.WAITING_FOR_QR)
            else:
                self.get_logger().info("QR code not authorized.")

    def init_waiting_state(self):
        self.get_logger().info("Door locked.")
        self.CURRENT_STATE_ = State.WAITING_FOR_QR
        # Start QR subscription
        self.qr_msg_subscription= self.create_subscription(
            self.QR_MSG_TYPE, 
            self.QR_TOPIC_NAME, 
            self.qr_msg_callback,
            1)

    def init_open_state(self):
        self.CURRENT_STATE_ = State.DOOR_OPENED

    def send_door_request(self, data):
        door_req = self.LOCK_SERVICE_TYPE.Request()
        door_req.data = data
        door_fut = self.door_lock_client_.call_async(door_req)
        self.get_logger().info("Sending a request to the door lock service")
        return door_fut
      
    def verifyQR(self, msg):
        verifier = QRVerify()
        return verifier.verify(msg)
    
    def qr_msg_callback(self, msg):
        if self.CURRENT_STATE_ != State.WAITING_FOR_QR:
            return
        msg_content = msg.data
        if msg_content in self.submitted_qrs_:
            return
        self.submitted_qrs_.add(msg_content)
        self.get_logger().info("Received a QR message: {0}".format(msg_content))
        self.set_state_(State.DOOR_OPENED, qr_msg=msg_content)

    def publish_state(self):
        if self.state_pub_.get_subscription_count() > 0:
            self.get_logger().debug("Publishing current state.")
            msg = String()
            msg.data = str(self.CURRENT_STATE_)
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
