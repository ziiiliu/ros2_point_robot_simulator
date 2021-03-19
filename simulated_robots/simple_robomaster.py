import rclpy
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class RoboMaster(SimulatedRobotBase):
    def __init__(self, uuid, node):
        super().__init__(uuid, node)
        self.orientation_offset = R.from_euler("xyz", [0, 0, 0])

    def step(self, dt):
        self.position += self.orientation.apply(
            np.array(
                [
                    np.clip(self.velocity.linear.x, -2.5, 3.5),
                    -np.clip(self.velocity.linear.y, -2.8, 2.8),
                    0,
                ]
            )
            * dt
        )
        self.orientation *= R.from_euler(
            "xyz",
            np.array([0, 0, np.clip(-self.velocity.angular.z, -10.5, 10.5)]) * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_robomaster", r"/robomaster_\d+/", RoboMaster)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
