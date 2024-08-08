import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

####For visualization only #####
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
######for visualization only #####

class Spiral(Node):
############ INITIALIZING ROS AND VARIABLES############################################################

    def __init__(self):
        super().__init__('spiral')

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
        self.initial_position = None
        self.initial_yaw = None
        self.linear_velocity = 0.1   ## burger's max v is 0.22m/s
        self.angular_velocity = 0.2  ## burger's max omega is 2.84 rad/s
        self.total_angle = 0         ## Initialize total angle to find relative angle travelled

        #### Archimedes spiral parameters ####
        self.a = 0.03
        self.total_theta_travelled = 1.5
        self.previous_relative_angle = 0
        
        #### ADDED FOR TO VISUALIZE - DELETE FOR THE REAL ROBOT ##########################
        self.spawn_marker_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn entity service not available, waiting ...')

        self.marker_names = []
        self.loop_count = 0
        ##################################################################################       
        
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

        current_position =(robot_x,robot_y)

        self.relative_angle = self.normalize_angle(robot_yaw - self.initial_yaw)
        self.relative_distance = self.calculate_distance(self.initial_position,current_position)
        # Main control logic
        # r = V/omega [1] , r = a * theta [2]
        # -> omega = V / a * total_theta_travelled
   
        self.angular_velocity = self.linear_velocity/(self.a * self.total_theta_travelled)
        move_command.linear.x = self.linear_velocity
        move_command.angular.z = self.angular_velocity

        # Correctly accumulate total angle traveled
        angle_increment = self.relative_angle - self.previous_relative_angle
        if angle_increment > np.pi:
            angle_increment -= 2 * np.pi
        elif angle_increment < -np.pi:
            angle_increment += 2 * np.pi

        self.total_theta_travelled += angle_increment

        self.previous_relative_angle = self.relative_angle

        self.get_logger().info(f'linear = {move_command.linear.x}, angular = {move_command.angular.z}, total theta = {self.total_theta_travelled}, r ={move_command.linear.x/move_command.angular.z}')
        
        ###Publishing message#########
        self.vel_publisher.publish(move_command)

    
    ############VISUALIZATION ONLY - DELETE FOR ACTUAL ROBOT###############        
    # Visualize the path by spawning markers
        self.loop_count += 1
        if self.loop_count % 20 == 0:
            self.visualize_path(robot_x,robot_y)

    ############VISUALIZATION ONLY - DELETE FOR ACTUAL ROBOT###############
    

######### AUXILARY FUNCTIONS #################################################################
    
    ##### ADDED FOR VISUALIZATION - DELETE FOR REAL ROBOT ######################################################
    def visualize_path(self, x, y):
        marker_name = f'marker_{time.time()}'
        req = SpawnEntity.Request()
        req.name = marker_name
        req.xml = f"""
        <sdf version='1.6'>
          <model name='{marker_name}'>
            <static>true</static>
            <link name='link'>
              <visual name='visual'>
                <geometry>
                  <sphere>
                    <radius>0.05</radius>
                  </sphere>
                </geometry>
                <material>
                  <ambient>0 0 1 1</ambient>
                  <diffuse>0 0 1 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        req.robot_namespace = marker_name
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.0
        req.initial_pose.orientation.w = 1.0
        future = self.spawn_marker_client.call_async(req)

    ##############################################################################################
    
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

    node = Spiral()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()



