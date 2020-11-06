import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import re

class Turtlebot():
    def __init__(self):
        self.orientation = R.from_euler('z', [0], degrees=True).as_quat()[0]
        self.position = np.zeros(2) # x/y

    def step(self, dt, velocity):
        # dt: time step
        # velocity: dict of linear, angular of np arrays of shape (3,) xyz each

        own_r = R.from_quat(self.orientation)
        dp = own_r.apply(velocity['linear']*dt)
        self.position += dp[:2]

        r = velocity['angular']
        vel_r = R.from_euler('xyz', velocity['angular']*dt)
        self.orientation = (own_r*vel_r).as_quat()

class TurtlebotROS(Turtlebot):
    def __init__(self, uuid, node):
        Turtlebot.__init__(self)

        self.node = node
        self.uuid = uuid
        self.orientation_offset = R.from_euler('z', -270, degrees=True)
        self.ros_velocity = Twist()

        self.pose_publisher = self.node.create_publisher(PoseStamped, f'/motion_capture_server/rigid_bodies/{self.uuid}/pose', 1)

        self.velocity_subscription = self.node.create_subscription(
            Twist,
            f'/{self.uuid}/cmd_vel',
            self.velocity_callback,
            1)

    def velocity_callback(self, vel):
        self.ros_velocity = vel

    def step(self, dt):
        v = self.ros_velocity.linear
        v_lin = np.array([v.x, 0, 0])
        r = self.ros_velocity.angular
        v_ang = np.array([0, 0, r.z])
        super().step(dt, {'linear': v_lin, 'angular': v_ang})

        msg = PoseStamped()

        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]

        # This offset is inversing the mocap_offset in tb_control action_server
        orientation = (R.from_quat(self.orientation.copy())*self.orientation_offset).as_quat()
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]

        msg.header.frame_id = "mocap"

        self.pose_publisher.publish(msg)

class SimpleTurtlebot(Node):

    def __init__(self):
        super().__init__('simple_turtlebot')
        self.uuid_regex = re.compile('/turtlebot_\d+/')
        self.agents = {}
        self.timer_period = 1/120 # seconds
        self.pose_publisher_timer = self.create_timer(self.timer_period, self.step_all_agents) # 120 Hz
        self.agent_discover_timer = self.create_timer(1/4, self.discover_agents)

    def get_all_agents(self):
        '''Gets UUIDs of all online agents'''
        topics = self.get_topic_names_and_types()
        topics_ttypes = [(topic, ttype) for topic, ttype
            in topics if self.uuid_regex.match(topic)]

        if not any(topics_ttypes):
            return []

        agent_topics, agent_ttypes = zip(*topics_ttypes)
        return list(set([t.split('/')[1] for t in agent_topics]))

    def discover_agents(self):
        for agent_key in self.get_all_agents():
            if agent_key not in self.agents.keys():
                self.agents[agent_key] = TurtlebotROS(agent_key, self)
                print(f"discovered {agent_key}")

    def step_all_agents(self):
        for agent in self.agents.values():
            agent.step(self.timer_period)


def main(args=None):
    rclpy.init(args=args)

    publisher = SimpleTurtlebot()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

