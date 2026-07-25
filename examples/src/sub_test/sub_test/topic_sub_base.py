import rclpy
import numpy as np
from rclpy.node import Node
import std_msgs.msg
import custom_msg2.msg
# import custom_msg2.msg as my_msg 이런 방식으로 해도 좋을듯/

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('min_sub')

        #self.create_timer(timer_period_sec=0.5, callback= self.timer_callback)
        self.subscr = self.create_subscription(custom_msg2.msg.MyCustomMsg, 'pub_test', self.on_subscribe, 10)

        self.sub_cnt = 0

        self.subscr # gc 안당하게
        pass

    # 역직렬화 안 된 바이트
    # create_subscription(..., raw=True) 를 주면 콜백에 bytes 가 들어옵니다.
    # 그때는 rclpy.serialization.deserialize_message(data, MyCustomMsg) 로 직접 변환해야 합니다.
    '''
    다만 지금 목적(단순 구독·출력)에는 불필요합니다.
    '''

    #subscribe의 자료형을 create_subscr에 등록하면, 자동으로 역직렬화되어 인자로 들어온다.
    def on_subscribe(self, msg):
        self.get_logger().info(f"{self.sub_cnt}: {msg.x}, {msg.y}")
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
