import rclpy
from geometry_msgs.msg import Twist
from freyja_msgs.msg import CtrlCommand
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node

from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator

from rclpy.qos import qos_profile_sensor_data


class FpvQuad(SimulatedRobotBase):
    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.control = CtrlCommand()
        self.v_ned = np.zeros(3)
        self.a_ned = np.zeros(3)
        self.control_subscription = self.node.create_subscription(
            CtrlCommand,
            f"/{self.uuid}/rpyt_command",
            self.control_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def control_callback(self, control: CtrlCommand):
        self.reset_watchdog()
        self.control = control

    def stop(self):
        self.control = CtrlCommand()

    def step(self, dt):
        self.orientation = R.from_euler(
            "xyz",
            np.array(
                [
                    self.control.roll,
                    self.control.pitch,
                    self.orientation.as_euler("xyz")[2] + self.control.yaw * dt,
                ]
            ),
        )
        #self.node.get_logger().info( f"Orientation: {self.orientation}" );
        roll = self.control.roll
        pitch = self.control.pitch
        _, _, yaw = self.orientation.as_euler("xyz")

        rot_force = np.array(
            [
                np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw),
                np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(yaw),
                np.cos(roll) * np.cos(pitch),
            ]
        )
        m = 0.85  # kg
        g = 9.81  # m/s^2
        f_ned = -rot_force * self.control.thrust + [0, 0, m * g]
        self.a_ned = f_ned / m
        self.v_ned += self.a_ned * dt
        self.position += self.v_ned * dt
        self.node.get_logger().info(f"Pos {self.position}")
        if self.position[2] > 0.0:
            self.v_ned = np.zeros(3)
            self.position[2] = 0.0

    def publish_tf(self):
        tf = TransformStamped()
        tf.header.frame_id = "map_ned"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.child_frame_id = self.rigid_body_label

        tf.transform.translation.x = self.position[0]
        tf.transform.translation.y = self.position[1]
        tf.transform.translation.z = self.position[2]

        orientation = self.orientation.as_quat()
        tf.transform.rotation.x = orientation[0]
        tf.transform.rotation.y = orientation[1]
        tf.transform.rotation.z = orientation[2]
        tf.transform.rotation.w = orientation[3]

        self.tf_publisher.sendTransform(tf)


def pwm_to_rad(pwm_rotation):
    return (pwm_rotation - 1500) * np.pi / 1500.0


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
