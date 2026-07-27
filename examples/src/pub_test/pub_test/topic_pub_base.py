import rclpy
from rclpy.node import Node
import std_msgs.msg
import custom_msg2.msg
from custom_msg2.srv import SrvBigInt


class MinimalPublisher(Node): # NODE를 상속해야 ROS2 기능 함수들을 사용가능... 얘가 핵심 객체 클래스

    def __init__(self): # 자식도 자기만에 내부에서 쓸준비를 함 초기화 시켜줌
        super().__init__('minimal_publisher')
        #부모 Node 초기화 = 내부에서 쓸수있게 준비 과정 > Node 이름을 minimal_publisher로 변경
        # 퍼블리셔 생성: (메시지 타입, 토픽 이름, 큐 사이즈)

        #String Type의 메세지
        # 토픽의 이름은 'pub_test'로 설정
        # topic의 이름이 같은 노드끼리는 서로 통신이 가능하다. -> **subscriber와 publisher의 topic 이름이 같아야 통신 가능**
        self.publisher_ = self.create_publisher(msg_type=custom_msg2.msg.MyCustomMsg, topic='pub_test',qos_profile=10)
        # req = SrvBigInt.Request()
        # req.a = 10
        # req.b = 20
        #MyCustomMsg.
        # 이런거 외부에서 주입 받아서 하면 좋지 않을까? (예를 들어, 토픽 이름이나 메시지 타입을 외부에서 주입받는 방식)
        
        # 타이머 생성: (주기(sec), 콜백 함수)
        self.create_timer(0.5, self.timer_callback)
        self.pub_msg_cnt = 0

    def timer_callback(self):
        # msg = std_msgs.msg.String()
        msg = custom_msg2.msg.MyCustomMsg()
        msg.x = 10
        msg.y = 20
        # msg.data = f"Topic Test, cnt={self.pub_msg_cnt}"
        self.publisher_.publish(msg) #퍼블리셔에 메세지 보내기
        self.get_logger().info(f'Publishing: {msg.x}, {msg.y}') #c++ spdlog와 비슷한 기능, 로그를 출력하는 기능
        
        self.pub_msg_cnt += 1 # 메세지 발행 횟수 증가


def main(args=None):
    # rclpy 초기화, 인자는 명령행 인자를 그대로 전달
    '''
    rclpy.init() 내부
        >>> context 주입이 없다면, default 생성{global 락 잡고, global 객체를 Context() 객체로 초기화}
        >>> contxt.init() 호출하여, 외부에서 rclpy::init()에서 전달한 args와 같은 값들을 전달
        >>> 이 context 객체는 __context 객체를 가짐(c로 작성된 객체, rclpy.Context() ) {init하면서 __context는 객체를 가지게 됨, context::init()전까지는 None 객체}
        >>> 이 context 객체는 global refCnt를 가짐(shared_ptr같은 RAII 디자인)
        >>> 로깅 활성화, 이때 이 Context::init 함수를 호출하면서 최초로 refCnt = 1이 되는데, logging 초기화 수행
    '''
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher() #이거는 위에 객체
    # 객체를 만들어서 함수에 인자로 사용할수있음 self.에 결국 minimal_publisher 가 들어감!
    # Node를 상속받은 클래스(설계도)만 만드는 것으로는 실제 객체가 생성되지 않음
    # MinimalPublisher()로 객체를 만들어야 __init__()이 실행되고
    # self가 실제 생성된 객체를 가리키며 기능들이 작동함
    # 여기까지는 global_executor는 아직 존재 하지 않음.

    
    # rclpy.spin() 함수는 노드가 종료될 때까지 콜백 함수를 계속 호출하는 역할을 합니다.
    '''
    # rclpy/__init__.py - 96Line
    # The global spin functions need an **executor** to do the work
    # A persistent executor can re-run async tasks that yielded in rclpy.spin*().

    rclpy.spin(Node node, *, executor=None)
        >>> executor가 없다면, global_executor를 가져옴
        >>> global_executor는 rclpy::init()에서 생성됨 <<< AI를 믿으면 안되는;{박제}
        >>> global_executor는 rclpy::init()에서 생성되지 않음. global_executor는 인자로 executor 주입이 없을 때, rclpy::spin()에서 생성됨.
        >>> 이때 global_executor는 singleThreadedExecutor 객체로 생성됨. - rclpy/__init__.py - 105Line
        >>> 만들어진 excutor는 context::On_Shutdown()에 등록됨. 
            => context가 죽으면 executor도 같이 소멸해야하니
               하지만, 뭔가 수행해야해서 get_global_exec()를 호출하는 경우를 위해 현재 self.exec를 None으로 바꾸고 old_executor를 반환함. => rclpy/__init__.py - 112Line
               (이건 예상? 일단 수행은 해야하니까??)
        >>> rcpy::*shutdown() 쪽에서 context on_shutdown() 호출함.
        >>> executor::__init__()에서 global_context 등록함. (인자로 전달 안하는 케이스)

        >>> executor는 인자로 입력 받은 node를 자신에 등록 (이때 lock 잡음)



        >>> Executor <- SingleThreadedExecutor : 상속 관계
        >>> Executor::spin()은 상속 객체가 구현하지 않은면 셉남
        >>> 밑에 spin()의 Executor는 SingleThreadedExecutor << 얘의 spin_once()을 봐야 함
        spin 상태를 위한 동기화 lock -> 이거 bool 객체에 대한 mutex
        , 돌고 있느데, 또 돌려고 시도 한다? 셉 남
        >>> rclpy/executors.py/_spin_once_impl 까지 확인 됨.

    '''

    rclpy.spin(minimal_publisher)
    # 타이머 0.5초되면 도달(True) > executor 감지 > callback실행 > 터미널출력 > 다음 0.5초도달까지 기다림(False)

    '''
    rclpy/Timer.py - 65줄
    Timer의 실행 시간이 되면 callback은 ready 상태가 된다.
    Executor가 callback을 가져가 실행하기 전까지 ready=True이다.

    rclpy::spin() 에서 global executor를 가져옴(인자 전달 안하는 케이스)

    '''

    # 노드 종료 처리
    minimal_publisher.destroy_node() # > 내 노드 정리

    rclpy.utilities.try_shutdown() # 이걸로 해도 되지 않나? > ROS2 전체 종료


if __name__ == '__main__':
    main()