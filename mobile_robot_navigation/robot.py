import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        
        self.ranges = None
        self.create_subscription(LaserScan, '/r2d2/laserscan', self.listener_callback_laser, qos_profile)

        self.pose = None
        self.create_subscription(Odometry, '/r2d2/odom', self.listener_callback_odom, qos_profile)

        self.pub_cmd_vel = self.create_publisher(Twist, '/r2d2/cmd_vel', 10)

    def wait(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            
            rclpy.spin_once(self)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
        # self.get_logger().info(str(self.ranges))
        
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
        # self.get_logger().info(str(self.pose))

    def navigation(self):
        try:
            rclpy.spin_once(self)
            self.navigation_start()
            while(rclpy.ok):
                rclpy.spin_once(self)
                self.navigation_update()
        except (KeyboardInterrupt):
            pass

    def navigation_start(self):
        raise NotImplementedError()

    def navigation_update(self):
        raise NotImplementedError()

