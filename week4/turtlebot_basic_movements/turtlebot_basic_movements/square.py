import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from std_srvs.srv import Empty
import time

class SquareMovement(Node):

    def __init__(self):
        super().__init__('square_movement')

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.robot_controller)
        self.get_logger().info('I am making a square')

        #### Initialize variable #####
        self.Odometry = Odometry()
        self.linear_velocity = 0.2
        self.angular_velocity = 0.2
        self.total_distance = 0
        self.total_angle = 0

        ### Robot State ######
        self.moving_straight = 1
        self.turning = 0

        ### Constant #######
        self.wanted_length = 1
        self.wanted_angle = np.pi/2

########### MAIN FUNCTIONS ###############################################################

        #Updating Pose
    def update_pose(self,msg):
        self.Odometry = msg

    #Main function that control how the robot move to its goal
    def robot_controller(self):
        move_command = Twist()

        # Initialize robot's positions
        robot_x = self.Odometry.pose.pose.position.x
        robot_y = self.Odometry.pose.pose.position.y
        robot_orientation = self.Odometry.pose.pose.orientation
        robot_yaw = self.get_yaw_from_quaternion(robot_orientation)

        if self.moving_straight == 1 and self.turning == 0:
            move_command.linear.x = self.linear_velocity
            move_command.angular.z = 0.0
            self.total_distance += self.linear_velocity * self.timer_period
            self.get_logger().info('Going straight')
            if self.total_distance >= self.wanted_length:
                    self.stop_robot()
                    self.moving_straight = 0
                    self.turning = 1
                    self.total_distance = 0
                    self.get_logger().info('Target distance reached. Turning now')

            
        if self.turning == 1 and self.moving_straight == 0:
            move_command.linear.x = 0.0
            move_command.angular.z = self.angular_velocity
            self.total_angle += self.angular_velocity * self.timer_period
            self.get_logger().info('Turning')
            if self.total_angle >= self.wanted_angle:
                    self.stop_robot()
                    self.moving_straight = 1
                    self.turning = 0
                    self.total_angle = 0
                    self.get_logger().info('Target angle reached. Going straight now')

        self.vel_publisher.publish(move_command)

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

    node = SquareMovement()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()





