import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

import numpy as np

class TurtleRandomMover(Node):

    def __init__(self):
        super().__init__('turtle_random_mover')
        #Creating Subscriber
        self.pose = Pose()
        self.goal = Pose()
        move_command = Twist()
        self.pose_subscriber = self.create_subscription(Pose,'/turtle1/pose',self.update_pose,10)
        self.subscriptions
        #Creating a Client that request from the Spawn service
        self.goal_turtle_client=self.create_client(Spawn, '/spawn')
        while not self.goal_turtle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #Creating Publisher
        self.pose_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 5)
        timer_period = 0.5
        #Timer for creating random goal
        self.timer = self.create_timer(timer_period, self.move_to_goal)
        self.get_logger().info('I am moving')
        #moving toward random goal by publishing the goal
        
        #self.turn_toward_goal()
        #self.move_toward_goal()
        

    def update_pose(self,msg):
        self.pose = msg
        #self.get_logger().info(f'current location: x = {msg.x}, y = {msg.y}, theta = {msg.theta}')

    def move_to_goal(self):
        move_command = Twist()
        distance = self.calculate_distance(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        calculated_angle = self.calculate_angle(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        angle_diff= self.normalize_angle(calculated_angle-self.pose.theta)
        

        if (distance < 0.1 and abs(angle_diff) < 0.1) or (distance is None and angle_diff is None):
            self.random_goal_generate()
            self.get_logger().info(f'Goal generated at spawn at x = {self.goal.x}, y ={self.goal.y}, theta = {self.goal.theta}')
            self.spawn_goal_turtle(self.goal.x,self.goal.y,self.goal.theta)

        if abs(angle_diff) > 0.1:
            move_command.angular.z = 1*angle_diff
            self.get_logger().info(f'turning toward goal - angle_diff = {angle_diff}')
            self.pose_publisher.publish(move_command)

        if distance > 0.1 and abs(angle_diff) < 0.1:
            move_command.linear.x = 1 * distance
            self.pose_publisher.publish(move_command)
            self.get_logger().info(f'moving toward goal - distance = {distance}')

            #move_command.linear.x = 0.0
            self.pose_publisher.publish(move_command)
            self.get_logger().info('Reached goal!')






        #distance = self.calculate_distance(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        #calculated_angle = self.calculate_angle(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        #angle_diff= self.normalize_angle(calculated_angle-self.pose.theta)

        #if abs(angle_diff) > 0.1:
        #    move_command.angular.z = 1*angle_diff
        #    self.get_logger().info(f'angle_diff = {angle_diff}')

        #else:
        #    if distance > 0.1:
        #        move_command.linear.x = 1 * distance
        #    else:
        #        move_command.linear.x = 0.0
        #        move_command.angular.z = 0.0
        #        self.get_logger().info('Reached goal!')

        #self.pose_publisher.publish(move_command)

    def random_goal_generate(self):
        self.goal.x = np.random.uniform(0,11)
        self.goal.y = np.random.uniform(0,11)
        self.goal.theta= np.random.uniform(0,np.pi*2)
        return

    def calculate_distance(self, pose_x, pose_y, goal_x, goal_y):
        return np.sqrt((goal_x-pose_x)**2+(goal_y-pose_y)**2)

    def calculate_angle(self, pose_x, pose_y, goal_x, goal_y):
        return np.arctan2(goal_y-pose_y,goal_x-pose_x)

    def normalize_angle(self, theta):
        while theta > np.pi:
            theta -= 2 *np.pi
        while theta < -np.pi:
            theta += 2 * np.pi
        return theta


    def spawn_goal_turtle(self, x, y, theta):
        self.req = Spawn.Request()
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.goal_turtle_client.call_async(self.req)
        self.get_logger().info(f'Goal turtle spawn at x = {x}, y ={y}, theta = {theta}')
        return
    
    '''
    def turn_toward_goal(self):
        calculated_angle = self.calculate_angle(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        angle_diff= self.normalize_angle(calculated_angle-self.pose.theta)
        move_command = Twist()
        while abs(angle_diff) > 0.1:
            move_command.angular.z = 1*angle_diff
            self.get_logger().info(f'turning toward goal - angle_diff = {angle_diff}')
            self.pose_publisher.publish(move_command)
            
        move_command.angular.z = 0.0
        self.get_logger().info(f'turning complete')


        return

    def move_toward_goal(self):
        move_command = Twist()
        distance = self.calculate_distance(self.pose.x, self.pose.y, self.goal.x, self.goal.y)
        while distance > 0.1:
            move_command.linear.x = 0.5 * distance
            self.pose_publisher.publish(move_command)
            self.get_logger().info(f'moving toward goal - distance = {distance}')

        move_command.linear.x = 0.0
        self.pose_publisher.publish(move_command)
        self.get_logger().info('Reached goal!')
        return
    '''


def main(args=None):
    rclpy.init(args=args)

    node = TurtleRandomMover()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
