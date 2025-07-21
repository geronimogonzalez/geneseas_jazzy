import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import signal
import sys
import random

node = None

def publish_zeros():
    global node
    if node is not None:
        node.get_logger().info('Nodo detenido con Ctrl+C. Publicando ceros...')
        node.pub_u1.publish(Float32(data=0.0))
        node.pub_u2.publish(Float32(data=0.0))
        node.pub_u3.publish(Float32(data=0.0))
        node.pub_u4.publish(Float32(data=0.0))

def signal_handler(sig, frame):
    publish_zeros()
    rclpy.shutdown()
    sys.exit(0)

def generate_prbs(length, n_bits=8):
    max_len = 2**n_bits - 1
    if length > max_len:
        raise ValueError("length debe ser <= 2^n_bits - 1")

    state = np.ones(n_bits, dtype=int)
    prbs = []

    for _ in range(length):
        new_bit = state[-2] ^ state[-3] ^ state[-4] ^ state[-8]
        prbs.append(state[-1])
        state = np.roll(state, 1)
        state[0] = new_bit

    return 2 * np.array(prbs) - 1  # escala a [-1, 1]

class PRBSNode(Node):
    def __init__(self):
        super().__init__('prbs_publisher')

        self.declare_parameter('switch_interval', 4.0)
        self.declare_parameter('amplitudes', [1.0])

        self.switch_interval = self.get_parameter('switch_interval').value
        self.amplitudes = self.get_parameter('amplitudes').get_parameter_value().double_array_value
        self.amplitudes = list(self.amplitudes) if self.amplitudes else [1.0]

        self.get_logger().info(f'Amplitudes configuradas: {self.amplitudes}')

        total_length = 2**8 - 1
        prbs_total = generate_prbs(total_length)
        u1_seq = prbs_total[::2]
        u2_seq = prbs_total[1::2]
        min_len = min(len(u1_seq), len(u2_seq))
        self.u1_seq = u1_seq[:min_len]
        self.u2_seq = u2_seq[:min_len]

        self.index = 0
        self.steps_per_change = int(self.switch_interval * 10)
        self.step_counter = 0

        # Inicial amplitudes aleatorias por motor
        self.ampl_u1 = random.choice(self.amplitudes)
        self.ampl_u2 = random.choice(self.amplitudes)

        # Inicial valores
        self.value_u1 = float(self.u1_seq[0]) * self.ampl_u1
        self.value_u2 = float(self.u2_seq[0]) * self.ampl_u2

        # Publishers
        self.pub_u1 = self.create_publisher(Float32, '/geneseas/motor_l', 10)
        self.pub_u2 = self.create_publisher(Float32, '/geneseas/motor_r', 10)
        self.pub_u3 = self.create_publisher(Float32, '/geneseas/motor_c', 10)
        self.pub_u4 = self.create_publisher(Float32, '/geneseas/motor_d', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.step_counter == 0:
            # Cambiar índice PRBS
            self.value_u1 = float(self.u1_seq[self.index])
            self.value_u2 = float(self.u2_seq[self.index])

            # Elegir amplitudes aleatorias nuevas para cada motor que use PRBS
            self.ampl_u1 = random.choice(self.amplitudes)
            self.ampl_u2 = random.choice(self.amplitudes)

            # Multiplicar por amplitudes
            self.value_u1 *= self.ampl_u1
            self.value_u2 *= self.ampl_u2

            self.index = (self.index + 1) % len(self.u1_seq)

        self.step_counter = (self.step_counter + 1) % self.steps_per_change

        # Publicar
        self.pub_u1.publish(Float32(data=self.value_u1))
        self.pub_u2.publish(Float32(data=self.value_u2))
        self.pub_u3.publish(Float32(data=0.0))
        self.pub_u4.publish(Float32(data=0.0))

        self.get_logger().info(
            f'u1: {self.value_u1:.3f}, u2: {self.value_u2:.3f}, u3: 0.000, u4: 0.000'
        )


def signal_handler(sig, frame):
    publish_zeros()
    sys.exit(0)  # Solo salir, sin llamar a rclpy.shutdown()

def main(args=None):
    global node
    rclpy.init(args=args)
    node = PRBSNode()
    signal.signal(signal.SIGINT, signal_handler)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
