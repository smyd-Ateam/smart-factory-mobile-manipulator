"""시험용 lifecycle_manager is_active 서비스 2개.

active_after_sec(sim time) 뒤부터 success=true. -1이면 끝까지 false (S4).
"""

from rclpy.node import Node
from std_srvs.srv import Trigger

from mm_task_manager.run import run

SERVICES = ('/lifecycle_manager_localization/is_active',
            '/lifecycle_manager_navigation/is_active')


class MockLifecycle(Node):

    def __init__(self):
        super().__init__('mock_lifecycle')
        self.active_after = self.declare_parameter('active_after_sec', 0.0).value
        for name in SERVICES:
            self.create_service(Trigger, name, self._handle)

    def _handle(self, request, response):
        now = self.get_clock().now().nanoseconds * 1e-9
        response.success = self.active_after >= 0.0 and now >= self.active_after
        return response


def main(args=None):
    run(MockLifecycle, args)
