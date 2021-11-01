import numpy as np

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
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
        self.position = np.array(initial_position)

        self.node = node
        self.uuid = uuid
        self.rigid_body_label = rigid_body_label

        self.pose_publisher = self.node.create_publisher(
            PoseStamped,
            f"/motive/{self.rigid_body_label}/pose",
            qos_profile=qos_profile_sensor_data,
        )
        self.tf_publisher = TransformBroadcaster(node=self.node)
        self.orientation_offset = R.from_euler("z", 0)

        self.watchdog = None

    def reset_watchdog(self):
        # Reset the watchdog. The watchdog only starts running after it was reset once.
        if self.watchdog is None:
            self.watchdog = self.node.create_timer(
                1 / self.WATCHDOG_FREQUENCY_HZ, self.stop
            )
        self.watchdog.reset()

    def step(self, dt):
        # dt: time step
        raise NotImplementedError()

    def stop(self):
        # Called by watchdog if wasn't reset in time
        raise NotImplementedError()

    def get_ros_pose(self, agent_pose=None):
        if agent_pose is None:
            agent_pose = self

        msg = Pose()

        msg.position.x = agent_pose.position[0]
        msg.position.y = agent_pose.position[1]
        msg.position.z = agent_pose.position[2]

        # This offset is inversing the mocap_offset in tb_control action_server
        orientation = (agent_pose.orientation * self.orientation_offset).as_quat()
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]

        return msg

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
