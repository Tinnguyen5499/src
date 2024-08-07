import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class CurrentStates(Node):

############ INITIALIZING ROS AND VARIABLES############################################################

    def __init__(self):
        super().__init__('current_states')

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.publish_states)
        self.get_logger().info('Publishing states from odometry')

        #### Initialize variable #####
        self.Odometry = Odometry()

########### MAIN FUNCTIONS ###############################################################

    #Updating Pose
    def update_pose(self,msg):
        self.Odometry = msg

    #Main function that control how the robot move to its goal
    def publish_states(self):

        # Initialize robot's positions
        robot_x = self.Odometry.pose.pose.position.x
        robot_y = self.Odometry.pose.pose.position.y
        robot_orientation = self.Odometry.pose.pose.orientation
        robot_yaw = self.get_yaw_from_quaternion(robot_orientation)

        # Logging States
        self.get_logger().info(f'x position:{robot_x:.2f}')
        self.get_logger().info(f'y position:{robot_y:.2f}')
        self.get_logger().info(f'yaw:{np.rad2deg(robot_yaw):.2f}')
        

######### AUXILARY FUNCTIONS #################################################################

    def stop_robot(self):
        move_command = Twist()
        move_command.linear.x = 0.0
        move_command.angular.z = 0.0
        self.vel_publisher.publish(move_command)

    def get_yaw_from_quaternion(self, orientation):
        q = orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, theta):
        while theta > np.pi:
            theta -= 2 *np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        return theta

    def convert_angle(self,angle):
        if angle <0:
            angle += 2 * np.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    node = CurrentStates()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()



