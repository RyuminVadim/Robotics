import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Объявляем и получаем параметр `turtlename`
        self.turtlename = self.declare_parameter(
            'turtlename', 'turtle').get_parameter_value().string_value

        # Инициализируем трансформер
        self.tf_broadcaster = TransformBroadcaster(self)

        # Подпишитесь на тему черепахи{1}{2}/pose и вызовите handle_turtle_pose
        # функция обратного вызова для каждого сообщения
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            1)
        self.subscription  # предотвращаем предупреждение о неиспользуемой переменной

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Прочитайте содержимое сообщения и назначьте его
        # соответствующие переменные tf
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Черепаха существует только в 2D, поэтому мы получаем перевод x и y
        # координаты из сообщения и установите координату z на 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # По той же причине черепаха может вращаться только вокруг одной оси
        # и именно поэтому мы устанавливаем вращение по x и y на 0 и получаем
        # поворот по оси Z из сообщения
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

	# Отправляем преобразование
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
