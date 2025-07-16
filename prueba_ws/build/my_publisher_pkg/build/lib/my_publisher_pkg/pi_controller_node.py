import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Imu
from simple_pid import PID
import signal
import sys
import time
import math
from tf_transformations import euler_from_quaternion


class IMUPIController(Node):
    def __init__(self):
        super().__init__('imu_pi_controller')

        # --- Parámetros ROS ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('right_thrust', 0.5)
        self.declare_parameter('output_limits', [-100.0, 100.0])
        self.declare_parameter('sample_time', 0.1)
        self.declare_parameter('realimentacion', 1.0)
        self.declare_parameter('offset_motor', 0.50)

        # --- Obtener parámetros ---
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.right_thrust = self.get_parameter('right_thrust').value
        output_limits = self.get_parameter('output_limits').value
        sample_time = self.get_parameter('sample_time').value
        self.realimentacion = self.get_parameter('realimentacion').value
        self.offset_motor = self.get_parameter('offset_motor').value

        # --- Configuración del controlador PID ---
        self.pid = PID(kp, ki, kd, sample_time=sample_time)
        self.pid.output_limits = tuple(output_limits)

        self.initialized = False
        self.initial_yaw = 0.0

        # --- Publicadores ---
        self.left_pub = self.create_publisher(Float32, '/geneseas/motor_l', 10)
        self.right_pub = self.create_publisher(Float32, '/geneseas/motor_r', 10)
        self.central_pub = self.create_publisher(Float32, '/geneseas/motor_c', 10)
        self.dir_pub = self.create_publisher(Float32, '/geneseas/motor_d', 10)
        self.state_pub = self.create_publisher(Int32, '/wamv/prueba/estado', 10)

        # --- Suscriptor al IMU ---
        self.imu_sub = self.create_subscription(Imu, '/bno055/bno055/imu', self.imu_callback, 10)

        # --- Timer para publicar thrust derecho constante ---
        self.timer = self.create_timer(sample_time, self.publish_motors)

        # --- Publicar estado activo ---
        self.publish_state(1)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # Primer lectura: usar como referencia
        if not self.initialized:
            self.initial_yaw = yaw
            self.pid.setpoint = 0.0
            self.get_logger().info(f"Setpoint inicial fijado en yaw: {yaw:.2f} rad")
            self.initialized = True
            return

        # Error relativo entre yaw actual e inicial
        error = self.relative_angle(self.realimentacion * (-yaw + self.initial_yaw))
        control = self.pid(error)

        # Publicar al motor izquierdo
        thrust_msg = Float32()
        thrust_msg.data = control + self.offset_motor
        self.left_pub.publish(thrust_msg)

        self.get_logger().info(f"Yaw: {yaw:.2f} | Error: {error:.2f} | Control: {control:.2f}")

    def relative_angle(self, angle):
        """Normaliza un ángulo a [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_motors(self):
        # Publicar thrust constante en motor derecho
        right_msg = Float32()
        right_msg.data = self.right_thrust
        self.right_pub.publish(right_msg)

        # Publicar 0.0 en motores central y dirección
        zero_msg = Float32()
        zero_msg.data = 0.0
        self.central_pub.publish(zero_msg)
        self.dir_pub.publish(zero_msg)

    def publish_state(self, value):
        msg = Int32()
        msg.data = value
        self.state_pub.publish(msg)
        self.get_logger().info(f"Estado publicado en /wamv/prueba/estado: {value}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUPIController()

    def shutdown_handler(sig, frame):
        node.get_logger().info("Apagando PI Controller: motores en 0")

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
