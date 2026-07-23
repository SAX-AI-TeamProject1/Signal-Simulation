import rclpy
import numpy as np
from rclpy.node import Node
import std_msgs.msg

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('min_sub')

        #self.create_timer(timer_period_sec=0.5, callback= self.timer_callback)
                
        self.subscr = self.create_subscription(std_msgs.msg.String, 'pub_test', self.timer_callback, 10)

        self.sub_cnt = 0

        self.subscr # gc 안당하게
        pass

    def timer_callback(self, msg):
        self.get_logger().info(f"{self.sub_cnt}: {msg}")
        #self.subscr.
        self.sub_cnt += 1
        pass




def main(args = None):
    rclpy.init(args=args)

    sub_node = MinimalSubscriber()

    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.try_shutdown()
    


if __name__ == '__main__':
    main()
