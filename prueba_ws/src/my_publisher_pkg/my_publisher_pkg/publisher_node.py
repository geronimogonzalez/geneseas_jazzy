import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import signal
import sys
import time

class ThrusterPublisher(Node):
    def __init__(self):
        super().__init__('thruster_publisher')

        # ---------- Tópicos ----------
        self.left_topic = '/geneseas/motor_l'
        self.right_topic = '/geneseas/motor_r'
        self.central_topic = '/geneseas/motor_c'
        self.dir_topic = '/geneseas/motor_d'
        self.state_topic = '/wamv/prueba/estado'

        # ---------- Parámetros ----------
        self.declare_parameter('mode', 'derecho')
        self.declare_parameter('thrust', 100.0)
        self.declare_parameter('left_thrust', 100.0)
        self.declare_parameter('right_thrust', 100.0)
        self.declare_parameter('alpha', 1.0)  # ← Nuevo parámetro

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.thrust = self.get_parameter('thrust').get_parameter_value().double_value
        self.left_thrust = self.get_parameter('left_thrust').get_parameter_value().double_value
        self.right_thrust = self.get_parameter('right_thrust').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value  # ← Cargar alpha

        self.get_logger().info(f'Modo: {self.mode}')
        if self.mode == 'derecho':
            self.get_logger().info(f'Thrust único: {self.thrust}, alpha: {self.alpha}')
        else:
            self.get_logger().info(f'Thrust izquierdo: {self.left_thrust}, derecho: {self.right_thrust}, alpha: {self.alpha}')

        # ---------- Publicadores ----------
        self.left_pub = self.create_publisher(Float32, self.left_topic, 10)
        self.right_pub = self.create_publisher(Float32, self.right_topic, 10)
        self.central_pub = self.create_publisher(Float32, self.central_topic, 10)
        self.dir_pub = self.create_publisher(Float32, self.dir_topic, 10)
        self.state_pub = self.create_publisher(Int32, self.state_topic, 10)

        # Publicar estado inicial
        self.publish_motor_state(1)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.running = True

    def publish_motor_state(self, value):
        estado_msg = Int32()
        estado_msg.data = value
        self.state_pub.publish(estado_msg)
        self.get_logger().info(f'Estado del motor publicado: {value}')

    def timer_callback(self):
        msg_left = Float32()
        msg_right = Float32()
        msg_central = Float32()
        msg_dir = Float32()

        msg_central.data = 0.0
        msg_dir.data = 0.0

        if self.mode == 'derecho':
            msg_left.data = self.thrust * self.alpha  # ← Se aplica alpha
            msg_right.data = self.thrust
            self.get_logger().info(f'[DERECHO] L: {msg_left.data} (alpha={self.alpha}), R: {msg_right.data}')
        elif self.mode == 'curva':
            msg_left.data = self.left_thrust * self.alpha  # ← Se aplica alpha
            msg_right.data = self.right_thrust
            self.get_logger().info(f'[CURVA] L: {msg_left.data} (alpha={self.alpha}), R: {msg_right.data}')
        else:
            self.get_logger().warn(f'Modo desconocido: {self.mode}. Publicando 0')
            msg_left.data = 0.0
            msg_right.data = 0.0

        self.left_pub.publish(msg_left)
        self.right_pub.publish(msg_right)
        self.central_pub.publish(msg_central)
        self.dir_pub.publish(msg_dir)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterPublisher()

    def shutdown_handler(sig, frame):
        node.get_logger().info('Ctrl+C detectado. Enviando thrust 0 y apagando estado...')

        stop_msg = Float32()
        stop_msg.data = 0.0
        node.left_pub.publish(stop_msg)
        node.right_pub.publish(stop_msg)
        node.central_pub.publish(stop_msg)
        node.dir_pub.publish(stop_msg)

        estado_msg = Int32()
        estado_msg.data = 0
        node.state_pub.publish(estado_msg)

        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f'Excepción inesperada: {e}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

