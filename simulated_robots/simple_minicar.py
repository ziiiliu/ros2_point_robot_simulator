import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from ctrl_msgs.msg import MinicarControl
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class Minicar(SimulatedRobotBase):
    def __init__(self, uuid, node):
        super().__init__(uuid, node)

        self.control = MinicarControl()
        self.control_subscription = self.node.create_subscription(
            MinicarControl, f"/{self.uuid}/cmd_vel", self.control_callback, 1
        )

    def control_callback(self, ctrl: MinicarControl):
        self.reset_watchdog()
        self.control = ctrl

    def stop(self):
        self.control = MinicarControl()

    def step(self, dt):
        v = np.clip(self.control.velocity, -0.2, 0.2)
        s = np.clip(self.control.steering, -1.0, 1.0) * 0.3
        self.position += self.orientation.apply(np.array([v, 0, 0]) * dt)
        L = 0.15
        self.orientation *= R.from_euler("z", np.tan(s) / L * v * dt)


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_minicar", r"/minicar_\d+/", Minicar)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
