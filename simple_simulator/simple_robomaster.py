import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator
from robomaster_msgs.msg import WheelSpeed

from rclpy.qos import qos_profile_sensor_data


class RoboMaster(SimulatedRobotBase):
    MAX_V_LINEAR_X_M_S = 3.5
    MAX_V_LINEAR_Y_M_S = 2.8
    MAX_V_ROT_Z_RAD_S = 10.5

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.velocity = Twist()
        self.velocity_subscription = self.node.create_subscription(
            Twist,
            f"/{self.uuid}/cmd_vel",
            self.velocity_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.wheelspeed_subscription = self.node.create_subscription(
            WheelSpeed,
            f"/{self.uuid}/cmd_wheels",
            self.wheelspeed_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def wheelspeed_callback(self, wheels):
        self.reset_watchdog()

        # https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
        ws_rpm = np.array([wheels.fl, wheels.fr, wheels.rl, wheels.rr])
        ws_rpm[np.abs(ws_rpm) < 13] = 0.0  # Robot RPM dead band
        ws = ws_rpm / 60 * 2 * np.pi
        lx = 0.15
        ly = 0.15
        r = 0.06
        t_plus = (
            r
            / 4
            * np.array(
                [
                    [1, 1, 1, 1],
                    [-1, 1, 1, -1],
                    [-1 / (lx + ly), 1 / (lx + ly), -1 / (lx + ly), 1 / (lx + ly)],
                ]
            )
        )
        v = t_plus @ ws

        self.velocity.linear.x = v[0]
        self.velocity.linear.y = v[1]
        self.velocity.angular.z = v[2]

    def velocity_callback(self, vel):
        self.reset_watchdog()
        self.velocity = vel

    def stop(self):
        self.velocity = Twist()

    def step(self, dt):
        if self.stopped:
            self.stop()
            return

        self.position += self.orientation.apply(
            np.array(
                [
                    np.clip(
                        self.velocity.linear.x,
                        -self.MAX_V_LINEAR_X_M_S,
                        self.MAX_V_LINEAR_X_M_S,
                    ),
                    -np.clip(
                        self.velocity.linear.y,
                        -self.MAX_V_LINEAR_Y_M_S,
                        self.MAX_V_LINEAR_Y_M_S,
                    ),
                    0,
                ]
            )
            * dt
        )
        self.orientation *= R.from_euler(
            "xyz",
            np.array(
                [
                    0,
                    0,
                    np.clip(
                        -self.velocity.angular.z,
                        -self.MAX_V_ROT_Z_RAD_S,
                        self.MAX_V_ROT_Z_RAD_S,
                    ),
                ]
            )
            * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_robomaster", r"/robomaster_\d+/", RoboMaster)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
