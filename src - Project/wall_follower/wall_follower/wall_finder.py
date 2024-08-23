# from geometry_msgs.msg import Twist
# from custom_interfaces.srv import FindWall
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import ReliabilityPolicy, QoSProfile
# from sensor_msgs.msg import LaserScan
# import time
# import math

# class FindWallServer(Node):

#     def __init__(self):
#         super().__init__('find_wall')
#         self.srv = self.create_service(FindWall, 'find_wall', self.custom_service_callback)
#         self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
#         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#         # self.timer_period = 0.5
#         self.laser_scans = []
#         self.laser_forward = 0
#         self.cmd = Twist()
#         # self.timer = self.create_timer(self.timer_period, self.motion)
    
#     def laser_callback(self, msg):
#         self.laser_scans = []
#         for i in range(0, 720):
#             self.laser_scans.append(msg.ranges[i])
#         self.laser_forward = msg.ranges[0]

#     def custom_service_callback(self, request, response):

#         self.get_logger().info('I receive FRONT: "%s"' % str(self.laser_forward))
#         msg = Twist()

#         self.get_logger().info('Total len: "%s"' % str(len(self.laser_scans)))

#         min_distsance = min(self.laser_scans)

#         min_distance_index = self.laser_scans.index(min(self.laser_scans))

#         self.get_logger().info('Minimum Distance Index is : "%s"' % str(min_distance_index))

#         self.get_logger().info('Minimum Distance is : "%s"' % str(min_distsance))

#         time.sleep(1)

#         # if angle is 0 i don't have to turn, if angle is 90 i have to turn to the right
#         # angle is 180 i have to turn to the back and if angle is 270 i have to turn to left

#         # Turn to face the wall

#         angle = min_distance_index / 2 
#         angle_radians = math.radians(angle)
#         angular_velocity = 0.5
#         duration = abs(angle_radians / angular_velocity)
#         msg.angular.z = angular_velocity
#         self.publisher_.publish(msg)
#         time.sleep(duration)
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)
        

#         # Move to the wall if it isn't close enough
#         # if self.laser_forward > 0.4:
#         if min_distsance > 0.3:
#             linear_speed = 0.05
#             distance_to_travel = min_distsance - 0.3
#             time_to_travel = distance_to_travel / linear_speed
#             msg.linear.x = linear_speed
#             self.publisher_.publish(msg)
#             time.sleep(time_to_travel)
#             msg.linear.x = 0.0
#             self.publisher_.publish(msg)
        
#         # Rotate to have the wall on the right
#         angle = 90
#         angle_radians = math.radians(angle)
#         angular_velocity = 0.5
#         duration = abs(angle_radians / angular_velocity)
#         msg.angular.z = angular_velocity
#         self.publisher_.publish(msg)
#         time.sleep(duration)
#         msg.angular.z = 0.0
#         self.publisher_.publish(msg)

#         response.wallfound = True
        
#         # if request.move == "Turn Right":
#         #     # define the linear x-axis velocity of /cmd_vel topic parameter to 0.1
#         #     msg.linear.x = 0.1
#         #     # define the angular z-axis velocity of /cmd_vel topic parameter to -0.5 to turn right
#         #     msg.angular.z = -0.5
#         #     # Publish the message to the topic
#         #     self.publisher_.publish(msg)
#         #     # print a pretty message
#         #     self.get_logger().info('Turning to right direction!!')
#         #     # response state
#         #     response.success = True
#         # elif request.move == "Turn Left":
#         #     # define the linear x-axis velocity of /cmd_vel topic parameter to 0.1
#         #     msg.linear.x = 0.1
#         #     # define the angular z-axis velocity of /cmd_vel topic parameter to 0.5 to turn left
#         #     msg.angular.z = 0.5
#         #     # Publish the message to the topic
#         #     self.publisher_.publish(msg)
#         #     # print a pretty message
#         #     self.get_logger().info('Turning to left direction!!')
#         #     # response state
#         #     response.success = True
#         # elif request.move == "Stop":
#         #     # define the linear x-axis velocity of /cmd_vel topic parameter to 0
#         #     msg.linear.x = 0.0
#         #     # define the angular z-axis velocity of /cmd_vel topic parameter to 0
#         #     msg.angular.z = 0.0
#         #     # Publish the message to the topic
#         #     self.publisher_.publish(msg)
#         #     # print a pretty message
#         #     self.get_logger().info('Stop there!!')
#         #     # response state
#         #     response.success = True
#         # else:
#         #     # response state
#         #     response.success = False
#         self.get_logger().info('Request ready to send')
#         return response


# def main(args=None):
#     rclpy.init(args=args)
#     service = FindWallServer()
#     rclpy.spin(service)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


from geometry_msgs.msg import Twist
from custom_interfaces.srv import FindWall
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
import time
import math
from rclpy.executors import MultiThreadedExecutor

class FindWallServer(Node):

    def __init__(self):
        super().__init__('find_wall')
        self.real_robot = False
        self.srv = self.create_service(FindWall, 'find_wall', self.custom_service_callback)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.timer_period = 0.5
        self.laser_scans = []
        self.laser_forward = 0
        self.cmd = Twist()
        
        # self.timer = self.create_timer(self.timer_period, self.motion)
    
    def laser_callback(self, msg):
        
        self.laser_scans = []
        for i in range(0, 720):
        #     if self.real_robot:
        #         int(self.laser_scans.append(msg.ranges[i])) + 360) % 720
        #     else:
            self.laser_scans.append(msg.ranges[i])
        if self.real_robot:
            self.laser_forward = msg.ranges[360]
        else:
            self.laser_forward = msg.ranges[0]

    def custom_service_callback(self, request, response):

        self.get_logger().info('I receive FRONT: "%s"' % str(self.laser_forward))
        msg = Twist()

        self.get_logger().info('Total len: "%s"' % str(len(self.laser_scans)))

        min_distsance = min(self.laser_scans)

        min_distance_index = self.laser_scans.index(min(self.laser_scans))

        if self.real_robot:
            min_distance_index = int(min_distance_index)
            min_distance_index = (min_distance_index + 360) % 720

        self.get_logger().info('Minimum Distance Index is : "%s"' % str(min_distance_index))

        self.get_logger().info('Minimum Distance is : "%s"' % str(min_distsance))

        # time.sleep(1)

        # if angle is 0 i don't have to turn, if angle is 90 i have to turn to the right
        # angle is 180 i have to turn to the back and if angle is 270 i have to turn to left

        # Turn to face the wall

        angle = min_distance_index / 2 
        angle_radians = math.radians(angle + ((angle*20)/100)) # add 20% more to rotate enough
        angular_velocity = 0.5
        duration = abs(angle_radians / angular_velocity)
        start_time = time.time()
        while time.time() - start_time <= duration:
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)
        # time.sleep(duration)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        # Move to the wall if it isn't close enough
        # if self.laser_forward > 0.4:
        if min_distsance > 0.3:
            linear_speed = 0.05
            distance_to_travel = min_distsance - 0.3
            time_to_travel = distance_to_travel / linear_speed
            start_time = time.time()
            while time.time() - start_time <= time_to_travel:
                msg.linear.x = linear_speed
                self.publisher_.publish(msg)
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
        
        # Rotate to have the wall on the right 
        angle = 110 # it should rotate only 90 but while testing, 90 isn't enough
        angle_radians = math.radians(angle)
        angular_velocity = 0.5
        duration = abs(angle_radians / angular_velocity)
        start_time = time.time()
        while time.time() - start_time <= duration:
            msg.angular.z = angular_velocity
            self.publisher_.publish(msg)
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        response.wallfound = True
    
        self.get_logger().info('Request ready to send')
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    wall_finder_node = FindWallServer()
    # Create a MultiThreadedExecutor with 3 threads
    executor = MultiThreadedExecutor(num_threads=3)
    # Add the node to the executor
    executor.add_node(wall_finder_node)
    try:
        # Spin the executor
        executor.spin()
    finally:
        # Shutdown the executor
        executor.shutdown()
        wall_finder_node.destroy_node()
    rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     service = FindWallServer()
#     rclpy.spin(service)
#     rclpy.shutdown()


if __name__ == '__main__':
    main()