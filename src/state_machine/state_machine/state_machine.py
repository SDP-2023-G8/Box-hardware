# Python
from enum import Enum
import asyncio

# ROS
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool
from std_msgs.msg import String


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

        # Current state publisher
        self.state_pub_ = self.create_publisher(self.STATE_PUB_TYPE, self.STATE_PUB_TOPIC_NAME_, 10)
        self.get_logger().info("Current state publisher topic name: {0}".format(self.state_pub_.topic_name))
        self.pub_timer = self.create_timer(1. / self.state_pub_freq_.value, self.publish_state)

        # Locking / unlocking the door
        self.door_lock_client_ = self.create_client(self.LOCK_SERVICE_TYPE, self.LOCK_SERVICE_NAME)
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
        self.get_logger().info("State {0} requested.".format(str(self.CURRENT_STATE_)))

        if new_state == State.WAITING_FOR_QR:
            # Lock the door
            result = self.send_door_request(True)
            self.get_logger().debug("Door lock response: {0}.".format(result))
            if not result.success:
                self.get_logger().error("Failed to lock the door.")
            self.get_logger().info("Door locked.")

            # Start QR subscription
            self.qr_msg_subscription= self.create_subscription(
                self.QR_MSG_TYPE, 
                self.QR_TOPIC_NAME, 
                self.qr_msg_callback,
                10)
            self.get_logger().info("Created a new subscription to {0}".format(self.qr_msg_subscription.topic_name))
            self.CURRENT_STATE_ = State.WAITING_FOR_QR

        elif new_state == State.DOOR_OPENED:
            qr_msg = kwargs['qr_msg']
            self.get_logger().info("Veryfing QR: {0}".format(qr_msg))
            if self.verifyQR(qr_msg):
                self.get_logger().info("QR code authorized.")
                result = self.send_door_request(False)
                self.get_logger().debug("Door lock response: {0}.".format(result))
                if not result.success:
                    self.get_logger().error("Failed to unlock the door.")
                self.get_logger().info("Door unlocked.")
                # Stop listening for QR messages
                self.qr_msg_subscription.destroy()
                self.CURRENT_STATE_ = State.DOOR_OPENED
                self.lock_future = self.hold_door_open(5)
            else:
                self.get_logger().info("QR code not authorized.")

    async def hold_door_open(self, delay):
        self.get_logger().info("Door opened for {0}s.".format(delay))
        await asyncio.sleep(delay)
        self.get_logger().info("Locking the door...")

        # Lock the door
        result = self.send_door_request(True)
        self.get_logger().debug("Door lock response: {0}.".format(result))
        if not result.success:
            self.get_logger().error("Failed to unlock the door.")
            return False
        self.get_logger().info("Door locked.")

        # Change the state
        self.set_state_(State.WAITING_FOR_QR)
        return True

    def send_door_request(self, data):
        door_req = self.LOCK_SERVICE_TYPE.Request()
        door_req.data = data
        door_future = self.door_lock_client_.call_async(door_req)
        self.get_logger().info("Waiting for the door to be locked.")
        rclpy.spin_until_future_complete(self, door_future)
        return door_future.result()
      
    def verifyQR(self, msg):
        # TODO
        return True
    
    def qr_msg_callback(self, msg):
        msg_content = msg.data
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

    qr_node = StateMachine()

    rclpy.spin(qr_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    qr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
