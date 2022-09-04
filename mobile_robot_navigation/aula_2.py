from turtle import distance
import rclpy
import time
import numpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

class NavigationControl(Node):

    def __init__(self):
        super().__init__('aula_2')
        
        self.ranges = None
        self.create_subscription(LaserScan, '/r2d2/laserscan', self.listener_callback_laser, 10)

        self.pose = None
        self.create_subscription(Odometry, '/r2d2/odom', self.listener_callback_odom, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/r2d2/cmd_vel', 10)

    def sleep(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            
            rclpy.spin_once(self)

    def listener_callback_laser(self, msg):
        self.ranges = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def start_control(self):
        self.get_logger().info('Inicializando o nó.')
        self.sleep(1)

        objetivo = 2
        p_gain = 0.5
        i_gain = 0.0
        d_gain = 0.0

        integral = 0
        old_error = 0
        
        while(rclpy.ok):
            rclpy.spin_once(self)

            distancia_frente = numpy.array(self.ranges[40:60]).mean()
            distancia_direita = numpy.array(self.ranges[0:10]).mean()

            if(distancia_frente < 2):

                cmd = Twist()
                cmd.angular.z = 0.5
                self.pub_cmd_vel.publish(cmd)

            else:
            
                error = objetivo - distancia_direita
                integral = integral + error;     
                dif_erro = error - old_error; 
                old_error = error
                power = p_gain*error + i_gain*integral + d_gain*dif_erro

                cmd = Twist()
                cmd.linear.x = 0.5
                cmd.angular.z = power
                self.pub_cmd_vel.publish(cmd)

                self.get_logger().info('Distância: {}. Power: {}.'.format(distancia_direita, power))

        self.get_logger().info('Finalizando o nó.')
        self.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    
    try:
        node.start_control()
    except (KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
