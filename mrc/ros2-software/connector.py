# import eventlet first and monkeypatch.
import eventlet
eventlet.monkey_patch()
# import remaining libraries. We do this so the auto-formatter doesn't ruin
# the import order when it's in a block.

if True:

    import socket  # for UDP

    # Important APIs
    from rclpy.node import Node
    from flask import Flask, request, jsonify, make_response

    # Getting them working together
    from threading import Thread
    from flask_socketio import SocketIO

    # Message formats
    from dataclasses import dataclass
    from turtlesim.msg import Pose
    from geometry_msgs.msg import Twist

# Define a data structure that stores the robot information.


@dataclass
class RobotPose:
    x: float
    y: float
    theta: float  # in radians


class MRCNode(Node):
    ''' MRCNode is the ROS2 Node that establishes a flask connection,
    it will accept messages from UDP and publish on the network,
    similarly from ROS2 and publish via Flask.

    Flask is used for TCP and "slow, important" messages, such as pose info.
    UDP is best for fast and "fire many" messages, such as controller input.
    '''

    def __init__(self, *args, publish_to_topic, subscribe_to_topic):
        super(MyNode, self).__init__('my_node')
        self.log = self.get_logger() #shorthand
        self.log.info('initialising network connections')

        # Create UDP
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )
        self.sock.bind(("0.0.0.0", 1337))

        # ROS publisher and subscriber.
        self.publisher = self.create_publisher(Twist, publish_to_topic, 10)
        self.subscription = self.create_subscription(Pose, subscribe_to_topic self.subscriber_callback, 1)

        # Flask components:
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")

        # Ros components (seperate thread for the self.process func)
        self.ros_thread = Thread(target=self.process, daemon=True)
        self.ros_thread.start()

        # Initialise variables and log finished
        self.robot = RobotPose(0, 0, 0)
        self.log.info('done!')

    def subscriber_callback(self, msg):
        '''
        This is an *internal* update to the robot state.
        msg is of type Pose from the libraries above.
        We just keep track of self.robot so when the user requests it
        we can give the latest information.
        '''
        self.robot = RobotPose(msg.x, msg.y, msg.theta)

    def pose_status(self):
        '''
        This is the message format that is sent via TCP. Make this in an easy to
        read way so that the receiving end (ie: ESP8266 ) can understand it easily.

        string format is easy, as one can just "read until" '|', then read until ','
        '''
        return f'|{self.robot.x:.2f},{self.robot.y:.2f},{self.robot.theta:.2f}'

    def process(self):
        '''
        This is the process that will keep everything ticking together.
        It works in the background from another thread, an updates the ros,
        the UDP, and leaves flask to the main thread. Thanks to greenlet.

        Sometimes, things just work and it's not understood why, so be careful.
        '''
        self.log.info('Starting process loop')  # Works fine
        alive = True
        while alive:
            try:

                # Spin the ROS node once...
                rclpy.spin_once(self, timeout_sec=0.0)
                # ...check the udp...
                data, addr = self.sock.recvfrom(1024)
                self.publish_movement(data.decode())
                # ...and then momentarily sleep so the other process (flask via socket.io) can run.
                eventlet.greenthread.sleep()

            except Exception as e:
                self.log.info(f'Proc died: {e}')
                alive = False

    def publish_movement(self, incoming):
        '''
        This will decode the data from UDP ( if it can)
        and publish on the ROS2 network. handy for sending
        commands to the robot, and much more responsive than
        UDP.
        '''
        movement_speed = 2.0
        angular_speed = 2.0

        for data in incoming:
            msg = Twist()
            if data == 'u':
                msg.linear.x = movement_speed
            elif data == 'd':
                msg.linear.x = -movement_speed
            elif data == 'l':
                msg.angular.z = angular_speed
            elif data == 'r':
                msg.angular.z = -angular_speed
            else: #it's not a character that we know about.
                continue

            # Publish the Twist message on the network.
            self.publisher.publish(msg)
            self.log.info(f'UDP publish [{data}] = {msg}').

