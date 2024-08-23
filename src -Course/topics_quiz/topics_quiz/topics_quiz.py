import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np


def euler_from_quaternion(quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

class Simple_Topics_Quiz(Node):

    def __init__(self):
        super().__init__('topics_quiz_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.subscriber_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.timer_period = 0.5
        self.move_to_tunnel = False
        self.turn_90 = False
        self.move_to_destination = False
        self.odometry = 0
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def subscriber_callback(self,msg):
        self.odometry = msg.pose.pose
        
    def motion(self):
        self.get_logger().info('I receive: "%s"' % str(self.odometry))

        if self.move_to_tunnel == False:
            if self.odometry.position.x > 1.0:
                self.cmd.linear.x = 0.0
                self.move_to_tunnel = True
                self.get_logger().info('I stop:')
            else:
                self.cmd.linear.x = 0.4
        else:
            if self.turn_90 == False:
                self.cmd.angular.z = 0.3
                orientation = self.odometry.orientation
                orient = [orientation.x, orientation.y, orientation.z, orientation.w]
                roll, pitch, yaw = euler_from_quaternion(orient)
                self.get_logger().info('Yaw "%s"' % str(yaw))
                if yaw > 1.52:
                    self.cmd.angular.z = 0.0
                    self.turn_90 = True
            else:
                if self.move_to_destination == False:
                    if self.odometry.position.y < 0.8:
                        self.cmd.linear.x = 0.4
                        self.get_logger().info('Odom_Y "%s"' % str(self.odometry.position.y))
                    else:
                        self.cmd.linear.x = 0.0
                        self.publisher_.publish(self.cmd)
                        self.move_to_destionation = True
                        self.destroy_node()

        self.publisher_.publish(self.cmd)

            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    topics_quiz = Simple_Topics_Quiz()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(topics_quiz)
    # Explicity destroy the node
    topics_quiz.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()