# Aula 4: Mapeamento. 
# 
# Utilize os comandos a seguir para imprimir mensagens no sistema de log do ROS:
#     self.get_logger().debug ('Exemplo de mensagem de debug.')
#     self.get_logger().info  ('Exemplo de mensagem de informação.')
#     self.get_logger().warn  ('Exemplo de mensagem de aviso.')
#     self.get_logger().error ('Exemplo de mensagem de erro comum.')
#     self.get_logger().fatal ('Exemplo de mensagem de erro fatal.')
#
# Utilize o comando 'wait' para fazer o robô esperar para receber o próximo comando.
# s é uma variável numérica que representa a quantidade de seguntos que o robô deve esperar até receber o próximo comando.
#     self.wait(s) 
#
# Você pode definir comandos de velocidades para o seu robô utilizando a seguinte sintaxe:
# Aqui você utiliza o Objeto Twist que é composto por uma velocidade linear (x,y,z) e uma velocidade angular (x,y,z).
# Para criar um comando de velocidade que faz o robô ir para frente, por exemplo, a velocidade linear será (1,0,0) e a angular será (0,0,0).
#     self.ir_para_frente = Twist(linear=Vector3(x=1.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=0.0))
#
# Utilize a função 'self.pub_cmd_vel.publish(cmd)' para enviar o comando de velocidade para o seu robô. EX:
#     self.pub_cmd_vel.publish(self.ir_para_frente)
#
# Utilize a seguinte variável para pegar os valores do laser do seu robô.
# Essa variável é um vetor de [0:180] com os valores de cada faixa de laser do robô entre -90 e 90 graus em relação a frente do robô.
#     self.laser
#
# Utilize a seguinte variável para pegar a pose do seu robô: pose = posição + orientação.
# Essa variável é um objeto do tipo Pose conposto por posição (x,y,z) e orientação (x,y,z,w).
#     self.pose
#
# A posição é dada em quaternion. Para transformala em euler, utilize a seguinte função:
#     roll, pitch, yaw = tf_transformations.euler_from_quaternion(
#                 [self.pose.orientation.x,
#                 self.pose.orientation.y,
#                 self.pose.orientation.z,
#                 self.pose.orientation.w]) 


from .robot import R2D2

import rclpy
import numpy
import math

from geometry_msgs.msg import Twist, Vector3

import tf_transformations

class RobotControl(R2D2):

    # Essa função é chamada apenas uma vez quando o seu nó é executado.
    # Modifique essafunção para inicializar a variáveis que você vai precisar para controlar o seu robô. 
    def navigation_start(self):
        pass # <-- substitua esse comando pelo seu código

    # Essa função é chamada a cada passo de execução do seu nó.
    # Modifiqe essa função para executar a programação de controle do seu robô
    def navigation_update(self):
        pass # <-- substitua esse comando pelo seu código


# Não precisa modificar nada a partir dessa linha
def main(args=None):

    rclpy.init(args=args)

    r2d2_control = RobotControl()
    r2d2_control.navigation()
    r2d2_control.destroy_node()

    rclpy.try_shutdown()