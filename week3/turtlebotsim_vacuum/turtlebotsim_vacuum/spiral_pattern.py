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
    super().__init__('spiral_pattern')

###### Initializing Necessary Publishers, Subscribers, Clients ##############################

    #Creating Subscriber to subscribe to Odometry topic to get robot's position
     self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose,10)

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


    # Assuming the turtle doesn't have mass omega = V/r
    # r = V/omega [1]
    # Using Archimedes spiral formula
    # r = b * theta [2] where b is a constant that control the distance between loops
    # Combine two formula we can get a function of V where omega stay constant
    # and we get theta from our simulation
    # V/omega = b* theta
    # V = b * theta * omega

    ### Initialize variables ########
    
    #Robot control variables#
    self.kp_linear = 0.5
    self.goal_tolerance = 0.1
    self.kp_angular = 0.5

    #Robot Path Variables#
    self.a = 0.1
    self.omega = 1.0
    self.previous_theta = 0
    self.get_logger().info(f'previous theta = {self.previous_theta}')

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

    # The robot needs to follow Archemedes spiral r = a * theta
    # Needs to calculate the theta of the robot to know when robot has made a convolution of 360 degree
        current theta = self.convert_angle(self.calculate_angle_from_center)

    

    









######### AUXILARY FUNCTIONS #################################################################

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

    def calculate_angle_from_center(self,pose_x,pose_y):
        return np.arctan2(pose_y-5.5,pose_x-5.5)

    def calculate_distance_form_center(self, pose_x, pose_y):
        return np.sqrt((pose_x-5.5)**2+(pose_y-5.5)**2)

    
    
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
    


