#!/usr/bin/env python3
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
print("hi")
class ObstacleDetect(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('move_scan')
        
        # create the publisher object for ROBOT1
        self.publisher_1= self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        # create the subscriber object for ROBOT1
        self.subscriber_1 = self.create_subscription(LaserScan, 'scan_robot1', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # create the publisher object for ROBOT1
        self.publisher_2= self.create_publisher(Twist, 'robot1/cmd_vel', 10)
        # create the subscriber object for ROBOT1
        self.subscriber_2 = self.create_subscription(LaserScan, 'scan_robot2', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        
        
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
        # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359] 
        
        
    def motion(self):
        # print the data
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # Logic of move
        if self.laser_forward > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
        elif self.laser_forward < 5 and self.laser_forward >= 1:
            self.cmd.linear.x = 0.05
            self.cmd.angular.z = 0.0         
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5
            
        # Publishing the cmd_vel values to a Topic
        #self.publisher_1.publish(self.cmd)
        
        self.publisher_2.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    move_scan = ObstacleDetect()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(move_scan)
    # Explicity destroy the node
    move_scan.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()
