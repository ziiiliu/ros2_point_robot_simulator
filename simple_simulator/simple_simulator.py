import re
import numpy as np
from rclpy.node import Node


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
            "uuids",
            value=[],
        )
        uuids = self.get_parameter(f"uuids").value
        assert len(uuids) > 0
        self.get_logger().info(f"Create simulation for UUIDs {uuids}")

        for agent_key in uuids:
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

            initial_pos = self.get_parameter(f"{agent_key}_initial_position").value
            initial_orientation = self.get_parameter(
                f"{agent_key}_initial_orientation"
            ).value

            rigid_body_label = self.get_parameter(f"{agent_key}_rigid_body_label").value
            if rigid_body_label is None:
                rigid_body_label = agent_key
            self.agents[agent_key] = self.robot_class(
                agent_key, rigid_body_label, self, initial_pos, initial_orientation
            )

        self.unique_rigid_bodies = self.get_parameter("unique_rigid_bodies").value

        self.timer_period = 1 / 120  # seconds
        self.pose_publisher_timer = self.create_timer(
            self.timer_period, self.step_all_agents
        )  # 120 Hz

    def step_all_agents(self):
        agents_published = list(self.agents.values())
        if not self.unique_rigid_bodies:
            np.random.shuffle(agents_published)
        for agent, agent_publish in zip(self.agents.values(), agents_published):
            agent.step(self.timer_period)
            agent.publish_pose(agent_publish)
            agent.publish_tf(agent_publish)
