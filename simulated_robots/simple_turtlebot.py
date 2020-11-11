import rclpy
import re

import numpy as np

from rclpy.node import Node
from .holonomic_robot import HolonomicRobotROS

class TurtlebotROS(HolonomicRobotROS):
    def __init__(self, uuid, node):
        super().__init__(uuid, node)

    def step(self, dt):
        super().step(dt, {
            'linear': np.array([self.velocity.linear.x, 0, 0]),
            'angular': np.array([0, 0, self.velocity.angular.z])
        })

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

    def step_all_agents(self):
        for agent in self.agents.values():
            agent.step(self.timer_period)
            agent.publish_pose()
            agent.publish_tf()


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleTurtlebot()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

