import rclpy
from rclpy.node import Node

from ez_cart_interfaces.msg import Intent, WheelVels

class IntentSubscriber(Node):

    def __init__(self):
        super().__init__('intent_subscriber')
        self.subscription = self.create_subscription(
            Intent,                                              # CHANGE
            'intent',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f"' % msg.wheel_vels.left) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    intent_subscriber = IntentSubscriber()

    rclpy.spin(intent_subscriber)

    intent_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
