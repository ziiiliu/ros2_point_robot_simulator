import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node

from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator

from rclpy.qos import qos_profile_sensor_data


class FpvQuad(SimulatedRobotBase):

    MAX_V_LINEAR_M_S = 10
    MAX_A_LINEAR_M_S_S = 10

    MAX_V_ROT_Z_RAD_S = 1000.0

    # drift_vel = 0.5 * np.array([0.0, 0.0, -1.0])

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

        self.pwm_command = UInt16MultiArray()
        self.pwm_command.data = [1500, 1500, 1000, 1500]
        self.pwm_subscription = self.node.create_subscription(
            UInt16MultiArray,
            f"/{self.uuid}/pwm",
            self.pwm_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.previous_velocity = np.array([0.0, 0.0, 0.0])

    def velocity_callback(self, vel):
        self.reset_watchdog()
        self.velocity = vel

    def pwm_callback(self, pwm):
        self.reset_watchdog()
        self.pwm_command = pwm

    def stop(self):
        self.velocity = Twist()

    def step(self, dt):

        # acceleration constraint: ensure velocity does not differ too
        # much from previous velocity

        velocity_array = twist_to_v(self.velocity)
        previous_velocity_array = (
            self.previous_velocity
        )  # self.prev... NOT stored as twist

        # simulate drift if desired
        # velocity_array += self.drift_vel

        # acceleration constraint - needs to be absolute magnitude, not axis-wise
        acceleration = (velocity_array - previous_velocity_array) / dt
        acceleration_mag = np.linalg.norm(acceleration)
        if acceleration_mag > self.MAX_A_LINEAR_M_S_S:
            accel_reduction_factor = self.MAX_A_LINEAR_M_S_S / acceleration_mag
            acceleration *= accel_reduction_factor
            velocity_array = previous_velocity_array + acceleration * dt

        # absolute velocity constraint - same as above
        velocity_mag = np.linalg.norm(velocity_array)
        if velocity_mag > self.MAX_V_LINEAR_M_S:
            vel_reduction_factor = self.MAX_V_LINEAR_M_S / velocity_mag
            velocity_array *= vel_reduction_factor

        # update position
        self.position += velocity_array * dt

        # update previous velocity variable
        self.previous_velocity = velocity_array

        # get orientation from pwm command
        roll_rad = pwm_to_rad(self.pwm_command.data[0])
        pitch_rad = pwm_to_rad(self.pwm_command.data[1])
        yaw_rad = pwm_to_rad(self.pwm_command.data[3])
        self.orientation = R.from_euler(
            "xyz",
            np.array(
                [
                    - roll_rad,
                    - pitch_rad,
                    yaw_rad,
                ]
            ),
        )


def pwm_to_rad(pwm_rotation):
    return (pwm_rotation - 1500) * np.pi / 1500.


def twist_to_v(vel):
    return np.array([vel.linear.x, vel.linear.y, vel.linear.z])


def clip_array(array_1, array_min, array_max):
    new_array = np.array(
        [
            np.clip(array_1[0], array_min[0], array_max[0]),
            np.clip(array_1[1], array_min[1], array_max[1]),
            np.clip(array_1[2], array_min[2], array_max[2]),
        ]
    )
    return new_array


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_fpvquad", r"/fpvquad_\d+/", FpvQuad)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
