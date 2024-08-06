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

class RandomMover(Node):

    def __init__(self):
        super().__init__('random_mover')

    ######## Initializing Necessary Publishers, Subscribers, Client for the Node ###############
        # Initialize the mode of moving of the robot
        self.turn_then_move = 1 # 1 to turn on mode where the robot turn first then move toward target
                                # 0 to turn on mode where the robut turn and move toward target at the same time
    

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose,10)
        

        #Creating a Client that request from gazebo to create goal marker and another client to delete the goal marker
        self.goal_visualization_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.goal_visualization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.delete_visualization_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_visualization_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        #Create a Client that request from gazebo to reset the world everytime the world to avoid restarting gazebo everytime script is run for debugging
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        while not self.reset_world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset world not available,waiting ...')

        self.reset_world()


        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.robot_control_loop)
        self.get_logger().info('I am moving to goal')


        ### Initialize variables ########

        self.kp_linear = 0.5
        self.goal_tolerance = 0.1

        ### Define PID variables for turning/angular velocity ###:

        self.kp_angular = 0.1  # Proportional gain for angular control
        self.ki_angular = 0.00  # Integral gain for angular control
        self.kd_angular = 0.1 # Derivative gain for angular control

        self.previous_angle_error = 0.0  # Initialize previous angle error
        self.integral_angle_error = 0.0  # Initialize integral of angle error

        ### Initialize the robot's states #####
        self.robot_state = 'generate_goal'




    ######## Main Function #######################################################
    
    #Robot control loop to generate a goal then make robot move to the goal"
    def robot_control_loop(self):
        self.get_logger().info('Control loop activated')
        if self.robot_state == 'generate_goal':
            self.generate_goal()
        elif self.robot_state == 'move_to_goal':
            self.move_to_goal()


    #Function to generate and visualize goal in gazebo
    def generate_goal(self):
        #if goal_reached = False
        #    self.robot_state = 'move_to_goal'
        #if goal_reached = True
        self.stop_robot()
        self.random_goal_generate()
        self.goal_visualize()
        self.robot_state = 'move_to_goal'

    #Function that continously update pose of the robot
    def update_pose(self,msg):
        self.Odometry = msg
    
    #Main Function that control how the robot move to its goal
    def move_to_goal(self):
        self.get_logger().info('Moving to goal')

        # Initialize robot's and goal's positions

        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        
        robot_x = self.Odometry.pose.pose.position.x
        robot_y = self.Odometry.pose.pose.position.y
        robot_orientation = self.Odometry.pose.pose.orientation
        robot_yaw = self.get_yaw_from_quaternion(robot_orientation)

        # Calculate necessary distance and angle different to control robot movement

        distance_to_goal = self.calculate_distance_to_goal(robot_x, robot_y, goal_x, goal_y)
        angle_to_goal = self.calculate_angle_to_goal(robot_x, robot_y, goal_x, goal_y)
        angle_diff= self.normalize_angle(angle_to_goal-robot_yaw)

        # Calculating PID terms for the control of angular difference

        proportional = self.kp_angular * angle_diff
        self.integral_angle_error += angle_diff * self.timer_period # Accumulate integral error
        integral = self.ki_angular * self.integral_angle_error
        derivative = self.kd_angular * (angle_diff - self.previous_angle_error) / self.timer_period
        self.pervious_angle_error = angle_diff
        
        angular_velocity = proportional + integral + derivative
        

        # Check if the robot has reached goal. If yes end loop by changing robot__state
        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            self.delete_goal_marker()
            self.get_logger().info('Goal Reached!')
            self.robot_state = 'generate_goal'
            self.get_logger().info(f'robot state: {self.robot_state}')
            
        # Main control algorithm of robot

        move_command = Twist()

        if self.turn_then_move == 0:

            move_command.linear.x = self.kp_linear * distance_to_goal
            move_command.angular.z = self.kp_angular * angle_diff  # old algorithm only proportional
            #move_command.angular.z = angular_velocity # implemented PID control
        
        if self.turn_then_move == 1:

            if abs(angle_diff) > self.goal_tolerance:
                move_command.angular.z = self.kp_angular * angle_diff # old algorithm only proportional
                #move_command.angular.z = angular_velocity
                self.get_logger().info(f'turning toward goal - angle_diff = {angle_diff}')

            if distance_to_goal > self.goal_tolerance and abs(angle_diff) < self.goal_tolerance:
                move_command.linear.x = self.kp_linear * distance_to_goal
                move_command.angular.z= 0.0
                self.get_logger().info(f'moving toward goal - distance = {distance_to_goal}')
        
        self.vel_publisher.publish(move_command)

        # Debugging logs
        #self.get_logger().info(f'distance to goal {distance_to_goal}')
        #self.get_logger().info(f'angle to goal {angle_to_goal}')
        #self.get_logger().info(f'robot yaw {robot_yaw}')
        #self.get_logger().info(f'angle difference {angle_diff}')

        ##########



    ####### Necessary Functions for the main functions to work####e###########################################

    def reset_world(self):
        request=Empty.Request()
        future = self.reset_world_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            self.get_logger().info('The world is reset')
        else:
            self.get_logger().error('Failed to reset the world')
        self.stop_robot()
        self.delete_goal_marker()

    def random_goal_generate(self):
        self.get_logger().info('generating random goal')
        self.goal = PoseStamped()
        self.goal.pose.position.x = np.random.uniform(-5,5)
        self.goal.pose.position.y = np.random.uniform(-5,5)
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation.w = 1.0
        return

    def calculate_distance_to_goal(self, pose_x, pose_y, goal_x, goal_y):
        return np.sqrt((goal_x-pose_x)**2+(goal_y-pose_y)**2)

    def calculate_angle_to_goal(self, pose_x, pose_y, goal_x, goal_y):
        return np.arctan2(goal_y-pose_y,goal_x-pose_x)
    
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

    def goal_visualize(self):
        self.get_logger().info('goal visualizing')
        req = SpawnEntity.Request()
        req.name ='goal_marker'
        req.xml = """
        <sdf version='1.6'>
          <model name='goal_marker'>
            <static>true</static>
            <link name='link'>
              <visual name='visual'>
                <geometry>
                  <sphere>
                    <radius>0.1</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        req.robot_namespace = 'goal_marker'
        req.initial_pose = self.goal.pose
        future = self.goal_visualization_client.call_async(req)
        #rclpy.spin_until_future_complete(self, future)
        #if future.result() is not None:
        #    self.get_logger().info('Goal marker spawned successfully')
        #else:
        #    self.get_logger().error('Failed to spawn goal marker')

        #self.get_logger().info(f'Random goal spawn at x = {self.goal.pose.position.x}, y ={self.goal.pose.position.y}')

    def delete_goal_marker(self):
        request = DeleteEntity.Request()
        request.name = 'goal_marker'
        future = self.delete_visualization_client.call_async(request)
        #rclpy.spin_until_future_complete(self,future)
        #if future.result() is not None:
        #   self.get_logger().info('Goal marker deleted successfully')
        #else:
        #    self.get_logger().error('Gailed to delete goal marker')




def main(args=None):
    rclpy.init(args=args)

    node = RandomMover()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






