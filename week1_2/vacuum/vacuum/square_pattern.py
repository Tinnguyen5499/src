import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from std_srvs.srv import Empty

class SquarePattern(Node):
    def __init__(self):
        super().__init__('square_pattern')

        #Creating Subscriber to subscribe to the pose
        self.pose = Pose()
        self.start = Pose()
        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose',self.update_pose,10)
        self.subscriptions

        #Create a client that call the reset service to reset the turtle's postion
        self.reset_client = self.create_client(Empty, '/reset')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting ...')

        #The actual request to reset position
        self.reset_turtle()
        
        #Initialize variables
            #Start point variables
        self.start.x = 0.5
        self.start.y = 10.5
        #Square movement variables
        self.linear_distance_travelled = 0
        self.angular_distance_travelled = 0
        self.line_length=9
        self.speed = 1.0
        self.angular_speed = 1.0
        self.aligning = 0
        self.turn_direction = 1
        self.radius = 0.3
        self.half_circle = np.pi * self.radius
        ######initializingstates of functions#####
        self.u_turning = 0
        self.moving_straight = 0
        self.aligning = 1


    #Creating publisher that publish the velocity to turtlesim
        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.timer_period =0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.state = 'move_to_start'

    def control_loop(self):
        if self.state == 'move_to_start':
            self.move_to_start()
        elif self.state == 'move_square':
            self.move_square()

    ####Main functions - the function that is being published to turtle######

    #Move turtle to the start point before doing the square
    def move_to_start(self):
        move_command = Twist()
        self.distance = self.calculate_distance_to_start(self.pose.x, self.pose.y, self.start.x, self.start.y)
        calculated_angle = self.calculate_angle_to_start(self.pose.x, self.pose.y, self.start.x, self.start.y)
        self.angle_diff= self.normalize_angle(calculated_angle-self.pose.theta)

        if abs(self.angle_diff) > 0.1:
            
            move_command.angular.z = 1 * self.angle_diff
            self.get_logger(). info(f'turning toward start: angle_diff = {self.angle_diff}')
            self.vel_publisher.publish(move_command)
            
        if self.distance > 0.1 and abs(self.angle_diff) < 0.1:
            move_command.linear.x = 0.5 * self.distance
            self.vel_publisher.publish(move_command)
            self.get_logger().info(f'moving toward start : ditance = {self.distance}')

        if (self.distance < 0.1 and abs(self.angle_diff) < 0.1):
            self.get_logger().info('Reached startpoint, move square starts')
            self.state = 'move_square'


    #After turtle get to start point, start doing the square sweep
    def move_square(self):

        if self.aligning == 1:
            self.align()
            if abs(self.angle_diff) < 0.005 and self.u_turning ==0 and self.aligning == 1:
                self.aligning = 0
                self.moving_straight = 1
        

        if self.moving_straight == 1 and self.u_turning == 0:
            self.move_straight()
            #self.align()
  
            if self.linear_distance_travelled >= self.line_length:
                self.linear_distance_travelled = 0
                self.get_logger().info(f'current direction {self.turn_direction}')
                self.get_logger().info(f'u-turning now')
                self.get_logger().info(f'reset distance traveled + change direction')
                move_command= Twist()
                move_command.linear.x = 0.0
                move_command.linear.z = 0.0
                self.vel_publisher.publish(move_command)
                self.moving_straight = 0
                self.u_turning = 1
                self.turn_direction = self.turn_direction * -1 # switching direction
                


        if self.u_turning == 1 and self.moving_straight == 0:
            self.u_turn()

            if self.angular_distance_travelled >= self.half_circle:
                self.angular_distance_travelled = 0.0
                self.u_turning = 0
                self.moving_straight = 0
                move_command= Twist()
                move_command.linear.x = 0.0
                move_command.linear.z = 0.0
                self.vel_publisher.publish(move_command)
                self.get_logger().info(f'moving straight now')
                self.get_logger().info(f'turning = {self.u_turning}')
                self.aligning = 1
                    



    ##### All necesary functions ###############################

    def align(self):
        move_command = Twist()
        wanted_orientation = -np.pi/2 * self.turn_direction
        self.angle_diff = self.normalize_angle(wanted_orientation-self.pose.theta)
        self.get_logger().info(f'wanted orientation = {wanted_orientation}')
        self.get_logger().info(f'theta = {self.pose.theta}')
        self.get_logger().info(f'angle diff = {self.angle_diff}')

        if abs(self.angle_diff) > 0.005 and self.u_turning == 0:
            move_command.angular.z = self.angular_speed*self.angle_diff
            self.get_logger().info(f'Aligning, current theta = {self.pose.theta}')
            self.vel_publisher.publish(move_command)

        elif abs(self.angle_diff) <= 0.005 and self.u_turning ==0:
            self.get_logger().info('Finish aligning')




    def move_straight(self):
        move_command = Twist()
        move_command.linear.x = self.speed
        self.linear_distance_travelled += self.speed * self.timer_period
        self.get_logger().info('I am moving straight')
        self.get_logger().info(f'distance travelled = {self.linear_distance_travelled}')
        self.vel_publisher.publish(move_command)

 
    def u_turn(self):
        #r = V/omega
        omega = self.speed
        v = self.radius*omega
        move_command = Twist()
        move_command.linear.x = v 
        move_command.angular.z = omega * self.turn_direction * -1
        self.angular_distance_travelled += v * self.timer_period
        self.get_logger().info('I AM DOING A U TURNNNNNNNNNNNN')
        self.get_logger().info(f'angular distance travelled = {self.angular_distance_travelled}')
        self.vel_publisher.publish(move_command)

    def update_pose(self,msg):
        self.pose = Pose()
        self.pose = msg

    def reset_turtle(self):
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            self.get_logger().info('Turtle position reset')
        else:
            self.get_logger().error('Failed to reset turtle position')

    def calculate_angle_to_start(self,pose_x,pose_y,start_x,start_y):
        return np.arctan2(start_y-pose_y,start_x-pose_x)

    def convert_angle(self, angle):
        if angle < 0:
            angle += 2 * np.pi
        return angle

    def calculate_distance_to_start(self, pose_x, pose_y,start_x, start_y):
        return np.sqrt((start_x-pose_x)**2+(start_y-pose_y)**2)

    def normalize_angle(self, theta):
        while theta > np.pi:
            theta -= 2 *np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        return theta

def main(args=None):
    rclpy.init(args=args)

    node = SquarePattern()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



    

