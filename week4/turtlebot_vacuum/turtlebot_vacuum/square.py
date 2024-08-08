import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time

class Square(Node):
############ INITIALIZING ROS AND VARIABLES############################################################

    def __init__(self):
        super().__init__('square')

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.move_square)
        self.get_logger().info('Messages')

        #### Initialize variable #########
        self.Odometry = Odometry()
        self.linear_velocity = 0.2   ## burger's max v is 0.22m/s
        self.angular_velocity = 0.5  ## burger's max omega is 2.84 rad/s
        self.total_angle = 0         ## Initialize total angle to find relative angle travelled

        #### Set upInitial States ######
        self.initial_position = None
        self.initial_yaw = None
        self.total_theta_travelled= 0
        self.previous_relative_angle = None
        self.turn_direction = 1

        #### Initialize Robot Moving Mode ###########
        self.u_turning = 0
        self.moving_straight = 0
        self.aligning = 1

        self.wanted_distance = 2

        self.radius = 0.25

        self.number_of_turn = 0



########### MAIN FUNCTIONS ###############################################################

    #Updating Pose
    def update_pose(self,msg):
        self.Odometry = msg

    def move_square(self):
        move_command = Twist()

        # Initialize robot's positions
        self.robot_x = self.Odometry.pose.pose.position.x
        self.robot_y = self.Odometry.pose.pose.position.y
        self.robot_orientation = self.Odometry.pose.pose.orientation
        self.robot_yaw = self.get_yaw_from_quaternion(self.robot_orientation)

        # Zero-ing robot position so it will be relative to the original position of the robot

        if self.initial_position == None:                   
            self.initial_position = (self.robot_x, self.robot_y)
        if self.initial_yaw == None:                   
            self.initial_yaw = self.robot_yaw

        self.current_position =(self.robot_x,self.robot_y)
        self.relative_distance = self.calculate_distance(self.initial_position,self.current_position)
        self.initial_robot_x, self.initial_robot_y = self.initial_position


        self.relative_angle = np.pi/2 + self.normalize_angle(self.robot_yaw - self.initial_yaw)
        self.relative_robot_x = self.robot_x - self.initial_robot_x
        self.relative_robot_y = self.robot_y - self.initial_robot_y  



        # Main control logic
        
        move_command = Twist()

        if self.number_of_turn >= 10:
            move_command = self.stop_robot()
            self.u_turning = 0
            self.moving_straight = 0
            self.aligning = 0
            self.vel_publisher.publish(move_command)
            self.get_logger().info(f'Vacuum Completed')
            



        if self.aligning == 1:
            self.align()
            if abs(self.angle_diff) < 0.005 and self.u_turning ==0 and self.aligning == 1:
                self.aligning = 0
                self.initial_position = self.current_position
                self.moving_straight = 1
                self.number_of_turn += 1
                self.get_logger().info(f'number of turn = {self.number_of_turn}')
                
        if self.moving_straight == 1 and self.u_turning == 0:
            self.move_straight()
  
            if self.total_distance >= self.wanted_distance:
                self.total_distance = 0
                move_command= Twist()
                move_command.linear.x = 0.0
                move_command.linear.z = 0.0
                self.vel_publisher.publish(move_command)
                self.moving_straight = 0
                self.u_turning = 1
                self.turn_direction = self.turn_direction * -1 # switching direction


        if self.u_turning == 1 and self.moving_straight == 0:
            self.u_turn()

            if self.total_theta_travelled >= np.pi:
                self.total_theta_travelled = 0.0
                self.u_turning = 0
                self.moving_straight = 0
                move_command= Twist()
                move_command.linear.x = 0.0
                move_command.linear.z = 0.0
                self.vel_publisher.publish(move_command)
                self.aligning = 1



    ##### All necesary functions ###############################

    def u_turn(self):
        #r = V/omega
        omega = self.angular_velocity
        v = self.radius*omega
        move_command = Twist()
        move_command.linear.x = v 
        move_command.angular.z = omega * self.turn_direction

        if self.previous_relative_angle == None:
            self.previous_relative_angle=self.relative_angle

        angle_increment = self.relative_angle - self.previous_relative_angle
        if angle_increment > np.pi:
            angle_increment -= 2 * np.pi
        elif angle_increment < -np.pi:
            angle_increment += 2 * np.pi

        self.total_theta_travelled += abs(angle_increment)
        self.previous_relative_angle = self.relative_angle
        self.vel_publisher.publish(move_command)

        
    def move_straight(self):
        move_command = Twist()
        self.total_distance = self.calculate_distance(self.initial_position,self.current_position)
        if self.total_distance < self.wanted_distance:
            move_command.linear.x = self.linear_velocity 

        if self.total_distance >= self.wanted_distance:
            move_command = self.stop_robot()
        self.vel_publisher.publish(move_command)

    def align(self):
        move_command = Twist()
        self.wanted_orientation = np.pi/2 * self.turn_direction
        self.angle_diff = self.normalize_angle(self.wanted_orientation-self.relative_angle)

        if abs(self.angle_diff) > 0.005 and self.u_turning == 0:
            move_command.angular.z = self.angular_velocity*self.angle_diff
            self.get_logger().info(f'Aligning, current theta = {self.relative_angle}')
            self.vel_publisher.publish(move_command)

        elif abs(self.angle_diff) <= 0.005 and self.u_turning ==0:
            self.get_logger().info('Finish aligning')

    def calculate_distance(self, initial_position, current_position):
        x1, y1 = initial_position
        x2, y2 = current_position
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def stop_robot(self):
        move_command = Twist()
        move_command.linear.x = 0.0
        move_command.angular.z = 0.0
        return move_command

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

    node = Square()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()






    

                

