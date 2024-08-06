import rclpy
from rclpy.node import Node
from numpy import random
from seven_states.msg import States

class RandStatesSub(Node):

    def __init__(self):
        super().__init__('RandStatesSub')
        self.subscription = self.create_subscription(
                States,
                'RandStates',
                self.listener_callback,
                10)
        self.subscription

    def listener_callback(self,msg):
        self.get_logger().info(f'x = {msg.x} y = {msg.y} z = {msg.z} xdot = {msg.xdot} ydot = {msg.ydot} zdot = {msg.zdot}')


def main(args=None):
    rclpy.init(args=args)

    states_sub = RandStatesSub()

    rclpy.spin(states_sub)

    states_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
