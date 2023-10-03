import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PublisherSubscriber(Node):
	def __init__(self):
		# Инициализация узла с именем 'text_to_cmd_vel'
		super().__init__('text_to_cmd_vel') 

		# Создание издателя (publisher) для сообщений типа 'Twist' в тему '/turtle1/cmd_vel' с буфером сообщений размером 10
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

		# Создание подписчика (subscription) для сообщений типа 'String' из темы 'cmd_text' с буфером сообщений размером 10
		self.subscription = self.create_subscription(String, 'cmd_text', self.callback, 10)

		# Инициализация объекта сообщения типа 'Twist'
		self.msg = Twist()

		# Предотвращение предупреждения о неиспользуемой переменной 'subscription'
		self.subscription 
	        
	def callback(self, msg):
		# Вывод информации о принятом сообщении
		self.get_logger().info('I heard: "%s"' % msg.data)

		match msg.data:
			case 'turn_right':
				self.msg.angular.z = -1.57  # Установка угловой скорости
				self.msg.linear.x = 0.0 # Установка линейной скорости
			case 'turn_left':
				self.msg.angular.z = 1.57
				self.msg.linear.x = 0.0
			case 'move_forward':
				self.msg.linear.x = 2.0
				self.msg.angular.z = 0.0
			case 'move_backward':
				self.msg.linear.x = -2.0
				self.msg.angular.z = 0.0
			case _:self.get_logger().info('Wrong command: choose <turn_left> or <turn_right> or <move_forward> or <move_backward>')

		# Отправка сообщения с управляющими данными в тему '/turtle1/cmd_vel'
		self.publisher_.publish(self.msg)

		# Вывод информации о том, что было опубликовано
		self.get_logger().info('Publishing: "%s"' % self.msg)

def main(args=None):
	# Инициализация ROS
    rclpy.init(args=args)

	# Создание экземпляра класса
    text_to_cmd_vel = PublisherSubscriber()

	# Запуск узла и ожидание событий (блокирующая операция)	
    rclpy.spin(text_to_cmd_vel) 

	# Уничтожение узла после завершения работы
    text_to_cmd_vel.destroy_node()

	# Завершение работы с ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()
