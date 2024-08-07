import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from std_srvs.srv import Empty

class SpiralPattern(Node):

    def __init__(self):
        super().__init__('spiral_pattern')

        #Creating Subscriber to subscribe to the pose
        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose',self.update_pose,10)
        self.subscriptions

        #Create a client that call the reset service to reset the turtle's postion
        self.reset_client = self.create_client(Empty, '/reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting ...')

        #The actual request
        self.reset_turtle()

        #Creating publisher that publish the velocity to turtlesim
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10)


        # Assuming the turtle doesn't have mass omega = V/r
        # r = V/omega [1]
        # Using Archimedes spiral formula
        # r = a * theta [2] where b is a constant that control the distance between loops
        # Combine two formula we can get a function of V where omega stay constant
        # and we get theta from our simulation
        # V/omega = b* theta
        # V = a * theta * omega
        
        #initialize the constants
        self.a = 0.1
        self.v = 0.5
        self.total_theta_travelled = 0.5
        self.pose = Pose()

        #Timer for how fast the velocity is being published
        self.timer_period = 0.1 #second
    
        self.timer = self.create_timer(self.timer_period, self.move_spiral)
        self.get_logger().info('I am moving')
      

    def update_pose(self,msg):
        self.pose = Pose()
        self.pose = msg
        
############ Main Function ##############################################################################
    def move_spiral(self):
        # The robot needs to follow Archemedes spiral r = a * theta
        # r = radius, a = constant determine the distance between lines, theta = total theta travelled
        # r = V/omega -> omega =V/r -> omega = V / a * total_theta_travelled

        #Initial velocity

        move_command = Twist()

        self.omega = self.v / (self.a * self.total_theta_travelled)

        move_command.linear.x = self.v
        move_command.angular.z = self.omega
        
        self.total_theta_travelled += self.omega * self.timer_period

        self.get_logger().info(f'linear = {move_command.linear.x} - angular = {move_command.angular.z}')
        self.vel_publisher.publish(move_command)

####### Auxilary Functions ################################################################################
    def reset_turtle(self):
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            self.get_logger().info('Turtle position reset')
        else:
            self.get_logger().error('Failed to reset turtle position')

    def calculate_angle_from_center(self,pose_x,pose_y):
        return np.arctan2(pose_y-5.5,pose_x-5.5)

    def convert_angle(self, angle):
        if angle < 0:
            angle += 2 * np.pi
        return angle

    def calculate_distance_from_center(self, pose_x, pose_y):
        return np.sqrt((pose_x-5.5)**2+(pose_y-5.5)**2)



def main(args=None):
    rclpy.init(args=args)

    node = SpiralPattern()

    rclpy.spin(node)

    rode.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()





    
