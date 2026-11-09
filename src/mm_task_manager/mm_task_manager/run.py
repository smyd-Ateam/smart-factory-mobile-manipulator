"""노드 실행 공통 함수.

rclpy 기본 SIGINT 처리기는 context를 먼저 닫아서 종료 처리(예: /cmd_vel 0 발행)를 할 수 없다.
SignalHandlerOptions.NO로 끄고 KeyboardInterrupt로 받은 뒤 on_exit를 실행한다.
"""

import signal

import rclpy
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor,
                             SingleThreadedExecutor)
from rclpy.signals import SignalHandlerOptions


def run(node_cls, args=None, multithreaded=False, on_exit=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_cls()
    executor = MultiThreadedExecutor() if multithreaded else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 정리 도중 두 번째 SIGINT가 오면 destroy_node가 끊기므로 무시
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if on_exit is not None:
            on_exit(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.try_shutdown()
