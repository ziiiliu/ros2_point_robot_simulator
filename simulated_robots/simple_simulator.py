import rclpy
import re

import numpy as np

from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import time


class SimpleSimulator(Node):
    def __init__(self, node_name, uuid_regex, robot_class):
        super().__init__(node_name)

        self.uuid_regex = re.compile(uuid_regex)
        self.robot_class = robot_class
        self.agents = {}

        self.declare_parameter(
            # Scramble poses to simulate same rigid bodies of multiple agents
            "unique_rigid_bodies",
            value=True,
        )

        self.declare_parameter(
            "n_agents",
            value=-1,
        )
        n_agents = self.get_parameter(f"n_agents")._value
        while True:
            all_agents = self.get_all_agents()
            self.get_logger().info(
                f"Discovered {len(all_agents)} agents (expecting {n_agents})"
            )
            if n_agents < 0 or len(all_agents) == n_agents:
                break
            self.get_logger().info("Retrying...")
            time.sleep(1)

        for agent_key in all_agents:
            self.declare_parameter(
                f"{agent_key}_initial_position",
                value=[0.0, 0.0, 0.0],
            )
            self.declare_parameter(
                f"{agent_key}_initial_orientation",
                value=[0.0, 0.0, 0.0],
            )
            self.declare_parameter(
                f"{agent_key}_rigid_body_label",
                value=None,
            )

            initial_pos = self.get_parameter(f"{agent_key}_initial_position")._value
            initial_orientation = self.get_parameter(
                f"{agent_key}_initial_orientation"
            )._value

            rigid_body_label = self.get_parameter(
                f"{agent_key}_rigid_body_label"
            )._value
            if rigid_body_label is None:
                rigid_body_label = agent_key
            self.agents[agent_key] = self.robot_class(
                agent_key, rigid_body_label, self, initial_pos, initial_orientation
            )

        self.unique_rigid_bodies = self.get_parameter("unique_rigid_bodies")._value

        self.timer_period = 1 / 10  # seconds
        self.pose_publisher_timer = self.create_timer(
            self.timer_period, self.step_all_agents
        )  # 120 Hz

    def get_all_agents(self):
        """Gets UUIDs of all online agents"""
        topics = self.get_topic_names_and_types()
        topics_ttypes = [
            (topic, ttype) for topic, ttype in topics if self.uuid_regex.match(topic)
        ]

        if not any(topics_ttypes):
            return []

        agent_topics, agent_ttypes = zip(*topics_ttypes)
        return list(set([t.split("/")[1] for t in agent_topics]))

    def step_all_agents(self):
        agents_published = list(self.agents.values())
        if not self.unique_rigid_bodies:
            np.random.shuffle(agents_published)
        for agent, agent_publish in zip(self.agents.values(), agents_published):
            agent.step(self.timer_period)
            agent.publish_pose(agent_publish)
            agent.publish_tf(agent_publish)
