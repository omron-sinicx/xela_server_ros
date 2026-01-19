#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XELA Friction Calculator Node

Subscribes to total_force topics and calculates friction coefficient (mu).
mu = |F_tangential| / |F_normal|
   = sqrt(F_x^2 + F_y^2) / |F_z|

Publishes:
  - xela_friction/sensor_<pos>/friction (std_msgs/Float32)
"""

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped


class XelaFrictionCalculator:
    def __init__(self):
        rospy.init_node("xela_friction_calculator", anonymous=False)

        # Parameters
        # sensor_ids: list of sensor IDs to process (e.g., [1, 2])
        # If passed as a string via roslaunch (e.g. "[1, 2]"), it might need parsing,
        # but rospy.get_param usually handles lists from yaml/rosparam correctly.
        # Default to [1, 2] if not specified.
        self.sensor_ids = rospy.get_param("~sensor_ids", [1, 2])

        # Ensure sensor_ids is a list (handle string input just in case)
        if isinstance(self.sensor_ids, str):
            try:
                # Remove brackets and split
                self.sensor_ids = [
                    int(x.strip())
                    for x in self.sensor_ids.strip("[]").split(",")
                    if x.strip()
                ]
            except ValueError:
                rospy.logwarn(
                    f"Failed to parse sensor_ids: {self.sensor_ids}. Using default [1, 2]"
                )
                self.sensor_ids = [1, 2]

        self.force_topic_prefix = rospy.get_param(
            "~force_topic_prefix", "/xela_force_converter/sensor_"
        )
        self.epsilon = rospy.get_param(
            "~epsilon", 0.01
        )  # Threshold for normal force to avoid div/0

        rospy.loginfo(
            f"XELA Friction Calculator initialized for sensors: {self.sensor_ids}"
        )
        rospy.loginfo(f"  Topic prefix: {self.force_topic_prefix}")
        rospy.loginfo(f"  Epsilon: {self.epsilon}")

        self.subs = []
        self.pubs = {}

        for sensor_id in self.sensor_ids:
            # Subscribe to total_force
            topic_name = f"{self.force_topic_prefix}{sensor_id}/total_force"
            self.subs.append(
                rospy.Subscriber(
                    topic_name, Vector3Stamped, self.create_callback(sensor_id)
                )
            )

            # Publisher for friction
            pub_topic = f"/xela_friction/sensor_{sensor_id}/friction"
            self.pubs[sensor_id] = rospy.Publisher(pub_topic, Float32, queue_size=10)

            rospy.loginfo(f"  Subscribed to: {topic_name}")
            rospy.loginfo(f"  Publishing to: {pub_topic}")

    def create_callback(self, sensor_id):
        """Create a closure for the callback to capture sensor_id"""

        def callback(msg):
            self.process_force(msg, sensor_id)

        return callback

    def process_force(self, msg, sensor_id):
        fx = msg.vector.x
        fy = msg.vector.y
        fz = msg.vector.z

        # Tangential force magnitude
        ft = np.sqrt(fx**2 + fy**2)

        # Normal force magnitude (absolute value)
        fn = abs(fz)

        # Calculate friction coefficient
        if fn < self.epsilon:
            # Normal force is too small, friction is undefined/zero
            mu = 0.0
        else:
            mu = ft / fn

        # Publish
        out_msg = Float32()
        out_msg.data = float(mu)
        self.pubs[sensor_id].publish(out_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = XelaFrictionCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass
