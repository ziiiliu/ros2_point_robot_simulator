import rclpy
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class TurtlebotROS(SimulatedRobotBase):
    def __init__(self, uuid, node):
        super().__init__(uuid, node)

    def step(self, dt):
        self.position += self.orientation.apply(
            np.array(
                [
                    np.clip(self.velocity.linear.x, -0.2, 0.2),
                    0,
                    0,
                ]
            )
            * dt
        )
        self.orientation *= R.from_euler(
            "xyz",
            np.array(
                [0, 0, np.clip(self.velocity.angular.z, -np.pi * 0.75, np.pi * 0.75)]
            )
            * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_turtlebot", r"/turtlebot_\d+/", TurtlebotROS)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
