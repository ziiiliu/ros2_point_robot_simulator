import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist, TransformStamped
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator
from robomaster_msgs.msg import WheelSpeed
from std_msgs.msg import String
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from rclpy.qos import qos_profile_sensor_data

class PositionPredictor3D(nn.Module):
    
    def __init__(self, n_feature, n_hidden=64, n_output=1, n_layer=1):
        super(PositionPredictor3D, self).__init__()
        self.input = nn.Linear(n_feature, n_hidden)
        self.fcs = []
        for i in range(n_layer):
            self.fcs.append(nn.Linear(n_hidden, n_hidden))
        self.fcs = nn.ModuleList(self.fcs)
        self.predict = nn.Linear(n_hidden, n_output)
    
    def forward(self, X):
        res = F.relu(self.input(X))
        for fc in self.fcs:
            res = F.relu(fc(res))
        res = self.predict(res)
        return res


class NNRoboMaster(SimulatedRobotBase):
    MAX_V_LINEAR_X_M_S = 3.5
    MAX_V_LINEAR_Y_M_S = 2.8
    MAX_V_ROT_Z_RAD_S = 10.5

    ROBOT_WHEEL_RADIUS = 0.06
    ROBOT_HALF_BASE_LEN_X = 0.15
    ROBOT_HALF_BASE_LEN_Y = 0.15

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation,
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.lateral_velocity_world = np.zeros(3)
        self.angular_velocity_world = 0.0

        self.velocity_body = Twist()
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

        self.dynamics_model = PositionPredictor3D(n_feature=6, n_layer=5, n_output=3)
        self.dynamics_model.load_state_dict(torch.load('/home/ziii/cs/sim2real/ros2_dynamics_learner/models/nn_4layer_3D_dynamics.pt'))
        self.dynamics_model.eval()

        len_xy = self.ROBOT_HALF_BASE_LEN_X + self.ROBOT_HALF_BASE_LEN_Y
        self.dyn_inv = (
            np.linalg.pinv(
                [
                    [1, 1, len_xy],
                    [1, -1, -len_xy],
                    [1, 1, -len_xy],
                    [1, -1, len_xy],
                ]
            )
            * self.ROBOT_WHEEL_RADIUS
        )
        self.state_publisher_ = self.node.create_publisher(String, 'robomaster_1_state', 10)


    def wheelspeed_callback(self, wheels):
        self.reset_watchdog()

        ws_rpm = np.array([wheels.fl, wheels.fr, wheels.rr, wheels.rl])
        ws_rpm[np.abs(ws_rpm) < 13] = 0.0  # Robot RPM dead band
        ws = ws_rpm / 60 * 2 * np.pi

        rot_inv = self.orientation.as_matrix().transpose()
        v = rot_inv @ self.dyn_inv @ ws

        self.lateral_velocity_world[0] = v[0]
        self.lateral_velocity_world[1] = -v[1]
        self.angular_velocity_world = -v[2]

    def velocity_callback(self, vel):
        self.reset_watchdog()
        self.velocity_body = vel
        self.lateral_velocity_world = self.orientation.apply(
            np.array(
                [
                    np.clip(
                        self.velocity_body.linear.x,
                        -self.MAX_V_LINEAR_X_M_S,
                        self.MAX_V_LINEAR_X_M_S,
                    ),
                    -np.clip(
                        self.velocity_body.linear.y,
                        -self.MAX_V_LINEAR_Y_M_S,
                        self.MAX_V_LINEAR_Y_M_S,
                    ),
                    0,
                ]
            )
        )

        self.angular_velocity_world = -self.velocity_body.angular.z

    def stop(self):
        self.lateral_velocity_world = np.zeros(3)
        self.angular_velocity_world = 0.0

    def step(self, dt):
        if self.stopped:
            self.stop()
            return
        
        nn_input = torch.tensor([self.velocity_body.linear.x, self.velocity_body.linear.y, 
                                 self.velocity_body.angular.z, *self.position[:2], 
                                 self.orientation.as_euler('xyz')[-1]], dtype=torch.float32).unsqueeze(dim=0)

        results = self.dynamics_model(nn_input).squeeze().tolist()

        self.position = np.array([*results[:2],0])
        self.orientation = R.from_euler("xyz", np.array([0,0,np.clip(
                        results[-1],
                        -self.MAX_V_ROT_Z_RAD_S,
                        self.MAX_V_ROT_Z_RAD_S)]))

        self.publish_state()

    def publish_tf(self):
        tf = TransformStamped()
        tf.header.frame_id = "map"
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

    def publish_state(self):
        timer_period = 0.1
        msg = String()
        # msg.data = f"timestamp: {self.node.get_clock().now().nanoseconds}, position: {self.position}, velocity: {self.velocity_body}"
        msg.data = f"timestamp: {time.time()}, position: {self.position}, velocity: {self.velocity_body}"
        self.state_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("nn_robomaster", r"/robomaster_\d+/", NNRoboMaster)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
