import rclpy
from rclpy.node import Node
from numpy import random
from seven_states.msg import States

class RandStatesPub(Node):

    def __init__(self):
        super().__init__('RandStatesPub')
        self.publisher_ = self.create_publisher(States, 'RandStates', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = States()
        msg.x = random.randint(255)
        msg.y = random.randint(255)
        msg.z = random.randint(255)
        msg.xdot = random.randint(255)
        msg.ydot = random.randint(255)
        msg.zdot = random.randint(255)
        msg.yaw = random.randint(255)
        self.publisher_.publish(msg)
        self.get_logger().info(f'x = {msg.x} y = {msg.y} z = {msg.z} xdot = {msg.xdot} ydot = {msg.ydot} zdot = {msg.zdot} yaw = {msg.yaw} ' )


def main(args=None):
    rclpy.init(args=args)

    random_states= RandStatesPub()

    rclpy.spin(random_states)

    random_states.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()

