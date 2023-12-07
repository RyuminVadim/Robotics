import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/robot/scan', self.laser_callback, 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.is_obstacle = False

    def laser_callback(self, msg):
        ranges = msg.ranges
        index = len(ranges) // 2
        center_index1 = index
        center_index2 = index + 1
        center_index3 = index - 1
        center_distance1 = ranges[center_index1]
        center_distance2 = ranges[center_index2]
        center_distance3 = ranges[center_index3]

        self.get_logger().info(f"Center distance: {center_distance1}")
	    # Check if there is an obstacle within a certain distance
        dist = 3.0
        if ((center_distance1 < dist) and (center_distance2 < dist) and (center_distance3 < dist)):        
            self.is_obstacle = False
        else: self.is_obstacle =True

    def timer_callback(self):
        twist = Twist()

        if self.is_obstacle:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

