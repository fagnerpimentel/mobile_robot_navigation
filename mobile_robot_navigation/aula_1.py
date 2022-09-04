import rclpy
import time
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

class NavigationControl(Node):

    def __init__(self):
        super().__init__('aula_1')
        
        self.pose = None
        self.create_subscription(Odometry, '/r2d2/odom', self.listener_callback_odom, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, '/r2d2/cmd_vel', 10)

        self.ir_para_frente = Twist(linear=Vector3(x= 1.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.ir_para_tras =   Twist(linear=Vector3(x=-1.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.girar_direita =  Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=-0.5))
        self.girar_esquerda = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.5))
        self.parar =          Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

    def sleep(self, max_seconds):
        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            
            rclpy.spin_once(self)

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def start_control(self):
        self.get_logger().info('Inicializando o nó.')
        self.sleep(1)

        self.get_logger().info('Ir para frente por 5 segundos.')            
        self.pub_cmd_vel.publish(self.ir_para_frente)
        self.sleep(5)

        self.get_logger().info('Ir para tras por 5 segundos.')            
        self.pub_cmd_vel.publish(self.ir_para_tras)
        self.sleep(5)

        self.get_logger().info('Girar para direita por 5 segundos.')
        self.pub_cmd_vel.publish(self.girar_direita)
        self.sleep(5)

        self.get_logger().info('Girar para esquerda por 5 segundos.')
        self.pub_cmd_vel.publish(self.girar_esquerda)
        self.sleep(5)

        self.get_logger().info('Parar por 5 segundos.')
        self.pub_cmd_vel.publish(self.parar)
        self.get_logger().info(
            'Estou na posição (x:{:.2f}, y:{:.2f}), angulo ({:.2f}).'
            .format(self.pose.position.x, self.pose.position.y, 180/3.14*self.pose.orientation.z))
        self.sleep(5)

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
