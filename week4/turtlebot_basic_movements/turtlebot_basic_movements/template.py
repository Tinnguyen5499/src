import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class TurnLeft(Node):
############ INITIALIZING ROS AND VARIABLES############################################################

    def __init__(self):
        super().__init__('turn_left')

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.robot_controller)
        self.get_logger().info('Messages')

        #### Initialize variable #####
        self.Odometry = Odometry()
        self.linear_velocity = 0.2   ## burger's max v is 0.22m/s
        self.angular_velocity = 0.2  ## burger's max omega is 2.84 rad/s
        self.total_angle = 0         ## Initialize total angle to find relative angle travelled
        self.wanted_angle = pi/2     ## Wanted angle pi/2 = turing 90 degree angle

        self.initial_position = None
        self.initial_yaw = None
        

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

        # Zero-ing robot position so it will be relative to the original position of the robot

        if self.initial_position == None:                   
            self.initial_position = (robot_x, robot_y)

        if self.initial_yaw == None:                   
            self.initial_yaw = robot_yaw

        self.relative_angle = self.normalize_angle(robot_yaw - self.initial_yaw)

        current_position =(robot_x,robot_y)
        self.relative_distance = self.calculate_distance(self.initial_position,current_position)

        # Main control logic
        move_command.angular.z = self.angular_velocity  #### Possitive Omega turns left

        if self.relative_angle >= self.wanted_angle:
            self.stop_robot()
            self.destroy_node()

        
        ###Publishing message#########
        self.vel_publisher.publish(move_command)


######### AUXILARY FUNCTIONS #################################################################
    
    def calculate_distance(self, initial_position, current_position):
        x1, y1 = initial_position
        x2, y2 = current_position
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

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

    node = TurnLeft()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()



