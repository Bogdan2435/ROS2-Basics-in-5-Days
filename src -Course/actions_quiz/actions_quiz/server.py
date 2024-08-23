import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from actions_quiz_msg.action import Distance
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
# from example_interfaces.msg import Float32
from actions_quiz_msg.msg import TotalDistance

import time
import math

from rclpy.executors import MultiThreadedExecutor

class ActionsQuizServer(Node):

    def __init__(self):
        super().__init__('actions_quiz_server')
        self._action_server = ActionServer(self, Distance, 'distance_as',self.execute_callback) 
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.subscriber_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher_ = self.create_publisher(TotalDistance, 'total_distance', 10)
        self.cmd_distance = TotalDistance()
        self.total_distance = 0

    def subscriber_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # self.get_logger().info(f'Received odometry -> X: {self.x}, Y: {self.y}')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.total_distance = 0

        feedback_msg = Distance.Feedback()

        for i in range(0, goal_handle.request.seconds):

            self.get_logger().info('actual X: "%s"' % str(self.x))
            # self.get_logger().info('MERGE')
            self.past_x = self.x
            self.past_y = self.y
            time.sleep(1)
            distance = math.sqrt((self.x - self.past_x) ** 2 + (self.y - self.past_y) ** 2)
            self.total_distance += distance
            feedback_msg.current_dist = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.cmd_distance.data = self.total_distance
            self.publisher_.publish(self.cmd_distance)
            self.get_logger().info('Total actual distance: "%s"' % str(self.total_distance))
            self.get_logger().info('Feedback: {0} '.format(feedback_msg.current_dist))
        
        goal_handle.succeed()
        result = Distance.Result()
        result.status = True
        result.total_dist = self.total_distance
        self.get_logger().info('Result: {0}'.format(result.status))
        return result

    # def execute_callback(self, goal_handle):
    #     self.get_logger().info('Executing goal...')
    #     self.total_distance = 0
    #     feedback_msg = Distance.Feedback()

    #     self.past_x = self.x
    #     self.past_y = self.y

    #     for _ in range(goal_handle.request.seconds):
    #         # Sleep using rclpy which allows callbacks to be processed
    #         rclpy.spin_once(self, timeout_sec=1.0)

    #         distance = math.sqrt((self.x - self.past_x) ** 2 + (self.y - self.past_y) ** 2)
    #         self.total_distance += distance

    #         feedback_msg.current_dist = self.total_distance
    #         self.get_logger().info(f'Total actual distance: "{self.total_distance}"')
    #         self.get_logger().info(f'Feedback: {feedback_msg.current_dist}')

    #         self.past_x = self.x
    #         self.past_y = self.y

    #     goal_handle.succeed()
    #     result = Distance.Result()
    #     result.status = True
    #     result.total_dist = self.total_distance
    #     self.get_logger().info(f'Result: {result.status}')
    #     return result

# def main(args=None):
#     rclpy.init(args=args)

#     my_action_server = ActionsQuizServer()

#     rclpy.spin(my_action_server)

def main(args=None):
    rclpy.init(args=args)

    my_action_server = ActionsQuizServer()

    # Use a multi-threaded executor to handle callbacks and action server concurrently
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(my_action_server, executor=executor)

    my_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()