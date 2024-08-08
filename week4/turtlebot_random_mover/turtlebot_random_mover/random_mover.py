import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time

class RandomMover(Node):
############ INITIALIZING ROS AND VARIABLES############################################################

    def __init__(self):
        super().__init__('random_mover')

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.robot_controller)
        self.get_logger().info('Messages')

        #### Initialize variable #########
        self.Odometry = Odometry()
        self.linear_velocity = 0.2   ## burger's max v is 0.22m/s
        self.angular_velocity = 0.2  ## burger's max omega is 2.84 rad/s
        self.total_angle = 0         ## Initialize total angle to find relative angle travelled

        #### Set upInitial States ######
        self.initial_position = None
        self.initial_yaw = None

        ##### PID parameters ##############

        self.kp_linear = 0.5
        self.kp_angular = 0.5
        self.goal_tolerance = 0.1
        self.angle_tolerance = 0.1

        #### Node State ###################
        self.robot_state = 'generate_goal'

        #### Robot Moving Mode ###########
        self.turn_then_move = 0 # 1 to turn on mode where the robot turn first then move toward target
                                # 0 to turn on mode where the robut turn and move toward target at the same time



########### MAIN FUNCTIONS ###############################################################

    #Updating Pose
    def update_pose(self,msg):
        self.Odometry = msg

    #Main function that control how the robot move to its goal
    def robot_controller(self):
        self.get_logger().info('Control loop activated')
        if self.robot_state == 'generate_goal':
            self.generate_goal()
        elif self.robot_state == 'move_to_goal':
            self.move_to_goal()

    def generate_goal(self):
        self.stop_robot()
        self.random_goal_generate()
        #self.goal_visualize()
        self.robot_state = 'move_to_goal'

    def move_to_goal(self):
        move_command = Twist()
        self.get_logger().info('Moving to goal')

        ## Initialize goal's positions

        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y

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

        initial_robot_x, initial_robot_y = self.initial_position
        self.relative_robot_x = robot_x - initial_robot_x
        self.relative_robot_y = robot_y - initial_robot_y

        # Calculate necessary distance and angle different to control robot movement

        distance_to_goal = self.calculate_distance_to_goal(self.relative_robot_x, self.relative_robot_y, goal_x, goal_y)
        angle_to_goal = self.calculate_angle_to_goal(self.relative_robot_x, self.relative_robot_y, goal_x, goal_y)
        angle_diff= self.normalize_angle(angle_to_goal-self.relative_angle)

        # Main control logic
        move_commnad = Twist()

        # Check if the robot has reached goal. If yes end loop by changing robot__state
        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            #self.delete_goal_marker()
            self.get_logger().info('Goal Reached!')
            self.robot_state = 'generate_goal'
            self.get_logger().info(f'robot state: {self.robot_state}')

        ### Regular mode turn and move at the same time ######

        if self.turn_then_move == 0:

            move_command.linear.x =self.kp_linear * distance_to_goal
            if move_command.linear.x > 0.2:
                move_command.linear.x = 0.2
            move_command.angular.z = self.kp_angular * angle_diff
        
        ### Turn then move mode #####
        if self.turn_then_move == 1:

            if abs(angle_diff) > self.angle_tolerance:
                move_command.angular.z = self.kp_angular * angle_diff
                self.get_logger().info(f'turning toward goal - angle_diff = {angle_diff}')

            if distance_to_goal > self.goal_tolerance and abs(angle_diff) < self.angle_tolerance:
                move_command.linear.x = self.kp_linear * distance_to_goal
                if move_command.linear.x > 0.2:
                    move_command.linear.x = 0.2
                move_command.angular.z= 0.0
                self.get_logger().info(f'moving toward goal - distance = {distance_to_goal}')
        
        
        ###Publishing message#########
        self.vel_publisher.publish(move_command)


######### AUXILARY FUNCTIONS #################################################################
    
    def random_goal_generate(self):
        self.get_logger().info('generating random goal')
        self.goal = PoseStamped()
        self.goal.pose.position.x = float(np.random.randint(0,2))
        self.goal.pose.position.y = float(np.random.randint(0,2))
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation.w = 1.0
        return

    def calculate_distance_to_goal(self, pose_x, pose_y, goal_x, goal_y):
        return np.sqrt((goal_x-pose_x)**2+(goal_y-pose_y)**2)

    def calculate_angle_to_goal(self, pose_x, pose_y, goal_x, goal_y):
        return np.arctan2(goal_y-pose_y,goal_x-pose_x)   

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

    node = RandomMover()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()



