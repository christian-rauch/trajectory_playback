#!/usr/bin/env python
from __future__ import print_function
import rospy
import genpy
import rosbag

from control_msgs.msg import FollowJointTrajectoryActionGoal

import os


class TrajectoryExport:
    def __init__(self):
        rospy.init_node("trajectory_export", anonymous=True)

        try:
            self.export_path = os.path.expanduser(rospy.get_param("~export_path"))
        except KeyError as e:
            print(e.message + " is undefined")
            return

        if os.path.exists(self.export_path):
            raise UserWarning("path "+self.export_path+" already exists!")
        else:
            os.makedirs(self.export_path)

        topic = "/arm_controller/follow_joint_trajectory/goal"

        if rospy.has_param("~bag_path"):
            # read bag file
            bag = rosbag.Bag(os.path.expanduser(rospy.get_param("~bag_path")), mode='r', skip_index=True)

            for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
                self.store_traj(msg)
            print("done")
        else:
            # subscribe topic
            print("listen to",topic)
            rospy.Subscriber(topic, FollowJointTrajectoryActionGoal, self.store_traj)
            rospy.spin()

    def store_traj(self, msg):
        time = msg.header.stamp
        traj_goal = msg.goal # control_msgs/FollowJointTrajectoryGoal

        msg_str = genpy.message.strify_message(traj_goal)

        yaml_file = open(os.path.join(self.export_path,"traj_"+str(time)+".yaml"), "w")
        yaml_file.write(msg_str)
        yaml_file.close()


if __name__ == '__main__':
    TrajectoryExport()