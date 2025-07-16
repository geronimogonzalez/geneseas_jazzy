import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import time
import signal
import sys

class SquareWavePublisher(Node):
    def __init__(self):
        super().__init__('square_wave_publisher')

        # --- Parámetros configurables ---
        self.declare_parameter('valor_alto', 50.0)
        self.declare_parameter('valor_bajo', -50.0)
        self.declare_parameter('periodo', 2.0)
        self.declare_parameter('alpha', 1.0)  # ← NUEVO parámetro para escalar motor izquierdo

        self.valor_alto = self.get_parameter('valor_alto').value
        self.valor_bajo = self.get_parameter('valor_bajo').value
        self.periodo = self.get_parameter('periodo').value
        self.alpha = self.get_parameter('alpha').value  # ← se guarda como atributo

        self.get_logger().info(f"Onda cuadrada configurada: alto={self.valor_alto}, bajo={self.valor_bajo}, periodo={self.periodo}s, alpha={self.alpha}")

        # --- Publicadores ---
        self.left_pub = self.create_publisher(Float32, '/geneseas/motor_l', 10)
        self.right_pub = self.create_publisher(Float32, '/geneseas/motor_r', 10)
        self.central_pub = self.create_publisher(Float32, '/geneseas/motor_c', 10)
        self.dir_pub = self.create_publisher(Float32, '/geneseas/motor_d', 10)
        self.state_pub = self.create_publisher(Int32, '/wamv/prueba/estado', 10)

        # --- Estado inicial ---
        self.publish_state(1)

        self.current_value = self.valor_alto
        self.last_switch_time = time.time()

        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_wave)

    def publish_wave(self):
        now = time.time()
        if now - self.last_switch_time >= self.periodo:
            self.current_value = self.valor_bajo if self.current_value == self.valor_alto else self.valor_alto
            self.last_switch_time = now
            self.get_logger().info(f"Valor cambiado a: {self.current_value}")

        # Publicar en motores L y R
        msg_left = Float32()
        msg_left.data = self.current_value * self.alpha  # ← se usa self.alpha
        self.left_pub.publish(msg_left)

        msg_right = Float32()
        msg_right.data = self.current_value
        self.right_pub.publish(msg_right)

        # Publicar 0.0 en motores C y D
        zero_msg = Float32()
        zero_msg.data = 0.0
        self.central_pub.publish(zero_msg)
        self.dir_pub.publish(zero_msg)

    def publish_state(self, value):
        estado_msg = Int32()
        estado_msg.data = value
        self.state_pub.publish(estado_msg)
        self.get_logger().info(f"Estado publicado en /wamv/prueba/estado: {value}")

def main(args=None):
    rclpy.init(args=args)
    node = SquareWavePublisher()

    def shutdown_handler(sig, frame):
        node.get_logger().info("Apagando SquareWavePublisher: motores en 0")

        stop_msg = Float32()
        stop_msg.data = 0.0
        node.left_pub.publish(stop_msg)
        node.right_pub.publish(stop_msg)
        node.central_pub.publish(stop_msg)
        node.dir_pub.publish(stop_msg)

        node.publish_state(0)

        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error en ejecución: {e}")
        try:
            node.publish_state(0)
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()

