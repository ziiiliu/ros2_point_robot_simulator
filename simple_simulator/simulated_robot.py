import numpy as np
import copy

from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, TransformStamped
from ctrl_msgs.srv import EmergencyStop
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data


class SimulatedRobotBase:
    WATCHDOG_FREQUENCY_HZ = 5

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__()
        assert len(initial_orientation) == 3
        assert len(initial_position) == 3

        self.orientation = R.from_euler("xyz", initial_orientation)
        self.prev_orientation = copy.deepcopy(self.orientation)
        self.position = np.array(initial_position)
        self.prev_position = self.position.copy()
        self.lin_velocity = np.zeros(3)
        self.ang_velocity = np.zeros(3)

        self.node = node
        self.uuid = uuid
        self.rigid_body_label = rigid_body_label

        self.pose_publisher = self.node.create_publisher(
            PoseStamped,
            f"/motive/{self.rigid_body_label}/pose",
            qos_profile=qos_profile_sensor_data,
        )
        self.vel_publisher = self.node.create_publisher(
            TwistStamped, f"/motive/{uuid}/vel", qos_profile=qos_profile_sensor_data
        )
        self.emergency_stop_srv = self.node.create_service(
            EmergencyStop, f"/{self.rigid_body_label}/emergency_stop", self.emergency_stop
        )

        self.tf_publisher = TransformBroadcaster(node=self.node)
        self.orientation_offset = R.from_euler("z", 0)

        self.watchdog = None
        self.stopped = False

    def emergency_stop(self, req, resp):
        self.node.get_logger().debug(f"Received emergency stop request {req}")
        self.stopped = req.stop
        resp.success = True
        return resp

    def reset_watchdog(self):
        # Reset the watchdog. The watchdog only starts running after it was reset once.
        if self.watchdog is None:
            self.watchdog = self.node.create_timer(
                1 / self.WATCHDOG_FREQUENCY_HZ, self.stop
            )
        self.watchdog.reset()

    def update_velocity(self, dt):
        self.lin_velocity = (self.position - self.prev_position) / dt
        self.ang_velocity = (self.orientation * self.prev_orientation.inv()).as_euler(
            "xyz"
        ) / dt
        self.prev_position = self.position.copy()
        self.prev_orientation = copy.deepcopy(self.orientation)

    def step(self, dt):
        # dt: time step
        raise NotImplementedError()

    def stop(self):
        # Called by watchdog if wasn't reset in time
        raise NotImplementedError()

    def get_ros_pose(self):
        msg = Pose()

        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]

        # This offset is inversing the mocap_offset in tb_control action_server
        orientation = (self.orientation * self.orientation_offset).as_quat()
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]

        return msg

    def get_ros_vel(self):
        vel_twist = Twist()
        vel_twist.linear.x = self.lin_velocity[0]
        vel_twist.linear.y = self.lin_velocity[1]
        vel_twist.linear.z = self.lin_velocity[2]
        vel_twist.angular.x = self.ang_velocity[0]
        vel_twist.angular.y = self.ang_velocity[1]
        vel_twist.angular.z = self.ang_velocity[2]
        return vel_twist

    def publish_tf(self, agent_pose=None):
        if agent_pose is None:
            agent_pose = self

        """Broadcast the goal as a tf for visualization"""
        tf = TransformStamped()
        tf.header.frame_id = "mocap"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.child_frame_id = self.rigid_body_label

        tf.transform.translation.x = float(agent_pose.position[0])
        tf.transform.translation.y = float(agent_pose.position[1])
        tf.transform.translation.z = float(agent_pose.position[2])
        # This offset is inversing the mocap_offset in tb_control action_server
        orientation = (agent_pose.orientation * self.orientation_offset).as_quat()

        tf.transform.rotation.x = orientation[0]
        tf.transform.rotation.y = orientation[1]
        tf.transform.rotation.z = orientation[2]
        tf.transform.rotation.w = orientation[3]

        self.tf_publisher.sendTransform(tf)
