import rclpy
import re

import numpy as np

from rclpy.node import Node


class SimpleSimulator(Node):
    def __init__(self, node_name, uuid_regex, robot_class):
        super().__init__(node_name)
        self.uuid_regex = re.compile(uuid_regex)
        self.robot_class = robot_class
        self.agents = {}
        self.timer_period = 1 / 10  # seconds
        self.pose_publisher_timer = self.create_timer(
            self.timer_period, self.step_all_agents
        )  # 120 Hz
        self.agent_discover_timer = self.create_timer(1 / 4, self.discover_agents)

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

    def discover_agents(self):
        for agent_key in self.get_all_agents():
            if agent_key not in self.agents.keys():
                self.agents[agent_key] = self.robot_class(agent_key, self)

    def step_all_agents(self):
        for agent in self.agents.values():
            agent.step(self.timer_period)
            agent.publish_pose()
            agent.publish_tf()
