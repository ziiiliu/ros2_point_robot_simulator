import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R

class HolonomicRobot():
    def __init__(self):
        self.orientation = R.from_euler('z', 0, degrees=True)
        self.position = np.zeros(3) # x/y/z

    def step(self, dt, velocity):
        # dt: time step
        # velocity: dict of linear, angular of np arrays of shape (3,) xyz each

        self.position += self.orientation.apply(velocity['linear']*dt)
        self.orientation *= R.from_euler('xyz', velocity['angular']*dt)

class HolonomicRobotROS(HolonomicRobot):
    def __init__(self, uuid, node):
        super().__init__()

        self.node = node
        self.uuid = uuid
        self.orientation_offset = R.from_euler('z', -270, degrees=True)
        self.velocity = Twist()

        self.pose_publisher = self.node.create_publisher(PoseStamped, f'/motion_capture_server/rigid_bodies/{self.uuid}/pose', 1)

        self.velocity_subscription = self.node.create_subscription(
            Twist,
            f'/{self.uuid}/cmd_vel',
            self.velocity_callback,
            1)

    def velocity_callback(self, vel):
        self.velocity = vel

    def publish_pose(self):
        msg = PoseStamped()

        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        msg.pose.position.z = self.position[2]

        # This offset is inversing the mocap_offset in tb_control action_server
        orientation = (self.orientation * self.orientation_offset).as_quat()
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]

        msg.header.frame_id = "mocap"
        msg.header.stamp = self.node.get_clock().now().to_msg()

        self.pose_publisher.publish(msg)

