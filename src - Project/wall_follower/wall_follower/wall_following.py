######//////////////////////////////??/////////////////#######################

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.srv import FindWall
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.action import ActionClient
from custom_interfaces.action import OdomRecord


class Wall_Follower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.group1 = ReentrantCallbackGroup()

        self.find_wall_client = self.create_client(FindWall, '/find_wall', callback_group=self.group1)

        while not self.find_wall_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.find_wall_req = FindWall.Request()
        self.find_wall_response = FindWall.Response()
        self.find_wall_started = False
        self.find_wall_completed = False
        self.wall_follower_can_start = False

        self._action_client = ActionClient(self, OdomRecord, 'record_odom', callback_group=self.group1)
        self.action_called = False
        self.first_tour_done = False

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group1)

        self.timer_period = 0.5
        self.laser_forward = 0
        self.laser_right = 0
        self.cmd = Twist()
        self.last_state = 'none'
        
        self.timer = self.create_timer(self.timer_period, self.motion)

    def send_request(self):
        self.find_wall_response = self.find_wall_client.call(self.find_wall_req)

    def laser_callback(self,msg):
        ###########################################
        # Simulation clockwise
        self.laser_right = msg.ranges[540] 
        self.laser_forward = msg.ranges[0]

        # #real robot
        # self.laser_right = msg.ranges[180] 
        # self.laser_forward = msg.ranges[360]

    def send_goal(self):
        goal_msg = OdomRecord.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.first_tour_done = True
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback, current distance : {0}'.format(feedback.current_total))

    def motion(self):

        if not self.find_wall_started:
            self.find_wall_started = True 
            self.get_logger().info('Find wall started')
            self.send_request()
        elif not self.find_wall_completed:
            if not self.find_wall_response.wallfound:
                time.sleep(self.timer_period)
            else:
                self.get_logger().info('Find wall finished')
                self.find_wall_completed = True
                self.wall_follower_can_start = True
        elif self.wall_follower_can_start:
            if not self.action_called:
                self.get_logger().info('Action odom started')
                self.action_called = True
                self.send_goal()

            # ------- Real Robot clockwise
            if not self.first_tour_done:
                if self.laser_forward > 0.5:
                    if self.laser_right > 0.3:
                        self.cmd.linear.x = 0.04
                        self.cmd.angular.z = -0.2
                    elif self.laser_right < 0.25:
                        self.cmd.angular.z = 0.3
                        self.cmd.linear.x = 0.04
                    else:
                        self.cmd.linear.x = 0.05
                        self.cmd.angular.z = 0.0
                else:
                    self.cmd.angular.z = 0.5
                    self.cmd.linear.x = 0.04

                self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    wall_follower_node = Wall_Follower()
    executor = MultiThreadedExecutor(num_threads=5)
    # Add the node to the executor
    executor.add_node(wall_follower_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        wall_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()