#!/usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import genpy
import yaml

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory

import os
import glob
from curtsies import Input


class TrajectoryPlayback:
    def __init__(self):
        rospy.init_node("trajectory_playback", anonymous=False)

        self.client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("waiting for trajectory controller...")
        if not self.client.wait_for_server():
            exit()

        try:
            self.export_path = os.path.expanduser(rospy.get_param("~export_path"))
        except KeyError as e:
            print(e.message + " is undefined")
            exit()

        self.traj_list = sorted(glob.glob(os.path.join(self.export_path, "traj_*.yaml")))

        self.pub_vis = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1)

        print("found",len(self.traj_list),"trajectories")

    def run(self):
        for traj_path in self.traj_list:
            print(os.path.split(traj_path)[1])
            # read yaml file
            yaml_file = open(traj_path, 'r')
            msg_str = yaml_file.read()
            yaml_file.close()
            yaml_args = yaml.load(msg_str)

            # format message
            traj_goal = FollowJointTrajectoryGoal()
            genpy.message.fill_message_args(traj_goal, yaml_args)

            print("visualise (v), execute (e), skip (s), exit(Ctrl+C):")

            execute = False
            skip = False
            visualise = False
            while not (execute or skip):
                with Input(keynames='curses', sigint_event=True) as input_generator:
                    for e in input_generator:
                        if repr(e) == '<SigInt Event>':
                            print("exit")
                            return
                        elif e == 'v':
                            visualise = True
                            break
                        elif e == 'e':
                            execute = True
                            skip = False
                            break
                        elif e == 's':
                            execute = False
                            skip = True
                            break

                if visualise:
                    print("visualise")
                    # format visualisation message
                    traj_vis = DisplayTrajectory()
                    traj_vis.model_id = "lwr"
                    traj_vis.trajectory_start.joint_state.name = traj_goal.trajectory.joint_names
                    traj_vis.trajectory_start.joint_state.position = traj_goal.trajectory.points[0].positions
                    traj_vis.trajectory_start.joint_state.velocity = traj_goal.trajectory.points[0].velocities
                    rtraj = RobotTrajectory()
                    rtraj.joint_trajectory = traj_goal.trajectory
                    traj_vis.trajectory.append(rtraj)

                    self.pub_vis.publish(traj_vis)

                    visualise = False

            if execute:
                print("execute trajectory")

                # send goal
                self.client.send_goal(traj_goal)
                print("state:", self.client.get_state())
                # wait for result
                if self.client.wait_for_result():
                    trajectory_result = self.client.get_result()
                    success = (trajectory_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL)
                    if success:
                        print("success")
                    else:
                        print("trajectory error")
                        return
                else:
                    self.client.cancel_goal()
                    print("interrupted")
            elif skip:
                print("skip trajectory")
            else:
                print("???")
        print("done")


if __name__ == '__main__':
    tp = TrajectoryPlayback()
    tp.run()