import rclpy
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class Minicar(SimulatedRobotBase):
    def __init__(self, uuid, node):
        super().__init__(uuid, node)
        self.orientation_offset = R.from_euler("xyz", [0, 0, np.pi / 2])

    def step(self, dt):

        self.position += self.orientation.apply(
            np.array(
                [
                    np.clip(self.velocity.linear.x, -0.1, 0.1),
                    0,
                    0,
                ]
            )
            * dt
        )
        L = 0.15
        steering_angle = np.clip(self.velocity.angular.z, -0.3150, 0.3150)
        self.velocity.linear.x
        self.orientation *= R.from_euler(
            "z",
            np.tan(steering_angle) / L * self.velocity.angular.z * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_minicar", r"/minicar_\d+/", Minicar)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
