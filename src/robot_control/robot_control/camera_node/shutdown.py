"""
종료 판단 한 곳.

발행을 하는 모듈(image_publisher, command_publisher)과 조립자(node)가
모두 같은 판단 기준을 써야 해서 여기로 뺐다. 각자 rclpy.ok() 를 부르면
나중에 조건이 하나 늘 때 고쳐야 할 곳이 흩어진다.
"""

import rclpy


def is_shutting_down(stop_event):
    """
    종료가 시작됐는지 판단한다.

    Ctrl-C 는 rclpy 컨텍스트를 먼저 무효화한 뒤 spin() 을 빠져나온다.
    그래서 콜백/워커가 실행 중이면 이미 죽은 컨텍스트로 발행을 시도하게 되고
    RCLError("publisher's context is invalid") 가 난다. 발행 전에 이걸로 거른다.

    두 조건을 OR 로 보는 이유:
        stop_event  — 우리가 스스로 내린 종료 결정(destroy_node, 발행 실패)
        rclpy.ok()  — 바깥(시그널/launch)에서 내려온 종료

    인자:
        stop_event: threading.Event. 노드가 만들어 모든 부품에 나눠 준 종료 신호.

    반환:
        True 면 아무것도 발행하면 안 된다.
    """
    return stop_event.is_set() or not rclpy.ok()
