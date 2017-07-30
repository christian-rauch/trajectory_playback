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

        topic_arm = "/arm_controller/follow_joint_trajectory/goal"
        topic_hand = "/gripper/sdh_controller/follow_joint_trajectory/goal"
        topics = [topic_arm, topic_hand]

        if rospy.has_param("~bag_path"):
            # read bag file
            bag = rosbag.Bag(os.path.expanduser(rospy.get_param("~bag_path")), mode='r', skip_index=True)

            for topic, msg, t in bag.read_messages(topics=topics):
                if topic == topic_arm:
                    self.store_traj(msg, "arm")
                elif topic == topic_hand:
                    self.store_traj(msg, "hand")
            print("done")
        else:
            # subscribe topics
            print("listen to",topics)
            rospy.Subscriber(topic_arm, FollowJointTrajectoryActionGoal, self.store_traj, callback_args="arm")
            rospy.Subscriber(topic_hand, FollowJointTrajectoryActionGoal, self.store_traj, callback_args="hand")
            rospy.spin()

    def store_traj(self, msg, robot):
        time = msg.header.stamp
        traj_goal = msg.goal # control_msgs/FollowJointTrajectoryGoal

        msg_str = genpy.message.strify_message(traj_goal)

        yaml_file = open(os.path.join(self.export_path,"traj_"+str(time)+"_"+robot+".yaml"), "w")
        yaml_file.write(msg_str)
        yaml_file.close()


if __name__ == '__main__':
    TrajectoryExport()