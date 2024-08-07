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

class SpiralPattern(Node):

    def __init__(self):
        super().__init__('spiral_pattern')

    ###### Initializing Necessary Publishers, Subscribers, Clients ##############################

        #Creating Subscriber to subscribe to Odometry topic to get robot's position
        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.update_pose,10)
        self.subscriptions

        #Creating Publisher that create random goal and publish command to move to random goal
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #Create a Client that request from gazebo to reset the world everytime the world to avoid restarting gazebo everytime script is run for debugging
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        while not self.reset_world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset world not available,waiting ...')

        #Publishing velocity message with timer_period increment
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period,self.move_spiral)
        self.get_logger().info('I am moving to goal')


        # Assuming the turtle doesn't have mass omega = V/r
        # r = V/omega [1]
        # Using Archimedes spiral formula
        # r = a * theta [2] where a is a constant that control the distance between loops
        # Combine two formula we can get a function of Omega where V stay constant
        # r = V/omega -> omega =V/r -> omega = V / a * total_theta_travelled

        ### Initialize variables ########
        self.Odometry = Odometry()
        #Robot Path Variables
        #testing
        self.a = 0.1
        self.linear_velocity = 0.4
        self.total_theta_travelled = 0.5
        self.omega = 0


        #### ADDED FOR TO VISUALIZE - DELETE FOR THE REAL ROBOT ##########################
        self.spawn_marker_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn entity service not available, waiting ...')

        self.delete_marker_client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete entity service not available, waiting ...')

        self.marker_names = []
        self.loop_count = 0
        ##################################################################################
        
        self.reset_world()

########### MAIN FUNCTIONS ###############################################################

    #Updating Pose
    def update_pose(self,msg):
        self.Odometry = msg

    #Main function that control how the robot move to its goal
    def move_spiral(self):
        move_command = Twist()

        # Initialize robot's positions
        robot_x = self.Odometry.pose.pose.position.x
        robot_y = self.Odometry.pose.pose.position.y
        robot_orientation = self.Odometry.pose.pose.orientation
        robot_yaw = self.get_yaw_from_quaternion(robot_orientation)

        # Calculate robot's omega as it changes as the total theta travelled increased

        self.omega = self.linear_velocity / (self.a * self.total_theta_travelled)

        move_command.linear.x = self.linear_velocity
        move_command.angular.z = self.omega

        self.total_theta_travelled += self.omega * self.timer_period

        self.get_logger().info(f'linear = {move_command.linear.x}, angular = {move_command.angular.z}, total theta = {self.total_theta_travelled}')

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

    def delete_all_markers(self):
        for marker_name in self.marker_names:
            request = DeleteEntity.Request()
            request.name = marker_name
            future = self.delete_marker_client.call_async(request)
        self.marker_names = []  
    ##############################################################################################
    
    
    def reset_world(self):
        request=Empty.Request()
        future = self.reset_world_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            self.get_logger().info('The world is reset')
        else:
            self.get_logger().error('Failed to reset the world')
        self.stop_robot()
        ### Added for visualizatoin ####
        self.delete_all_markers()

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

    def calculate_angle_from_center(self,pose_x,pose_y,center_x,center_y):
        return np.arctan2(pose_y-center_y,pose_x-center_x)

    def calculate_distance_form_center(self, pose_x, pose_y, center_x, center_y):
        return np.sqrt((pose_x-center_x)**2+(pose_y-center_y)**2)

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

    node = SpiralPattern()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


