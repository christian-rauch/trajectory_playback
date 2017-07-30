#!/usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import genpy
import yaml

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from std_srvs.srv import Trigger

import os
import glob
from curtsies import Input


class TrajectoryPlayback:
    def __init__(self):
        rospy.init_node("trajectory_playback", anonymous=False)

        self.action_client_arm = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_hand = actionlib.SimpleActionClient('/gripper/sdh_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("waiting for trajectory controller...")
        # if not self.action_client_arm.wait_for_server():
        #     exit()

        try:
            self.export_path = os.path.expanduser(rospy.get_param("~export_path"))
        except KeyError as e:
            print(e.message + " is undefined")
            exit()

        self.pub_vis = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1)

        self.srv_sdh_init = rospy.ServiceProxy("/gripper/sdh_controller/init", Trigger)
        self.srv_sdh_disconnect = rospy.ServiceProxy("/gripper/sdh_controller/disconnect", Trigger)

        self.traj_list = sorted(glob.glob(os.path.join(self.export_path, "traj_*.yaml")))
        print("found",len(self.traj_list),"trajectories")

    def run(self):
        for traj_path in self.traj_list:
            filename = os.path.splitext(os.path.split(traj_path)[1])[0]
            _, time, part = filename.split("_")
            print(time,":",part)
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
                # chose action client based on robot part
                if part == "arm":
                    ac = self.action_client_arm
                elif part == "hand":
                    ac = self.action_client_hand
                else:
                    ac = None

                print("execute trajectory")

                if part == "hand":
                    # init driver
                    print("init sdh")
                    self.srv_sdh_init()


                # send goal
                ac.send_goal(traj_goal)
                # wait for result
                if ac.wait_for_result():
                    trajectory_result = ac.get_result()
                    success = (trajectory_result.error_code == FollowJointTrajectoryResult.SUCCESSFUL)
                    if success:
                        print("success")
                    else:
                        print("trajectory error")
                        return
                else:
                    ac.cancel_goal()
                    print("interrupted")

                if part == "hand":
                    # shutdown driver
                    print("disconnect from SDH")
                    self.srv_sdh_disconnect()

            elif skip:
                print("skip trajectory")
            else:
                print("???")
        print("done")


if __name__ == '__main__':
    tp = TrajectoryPlayback()
    tp.run()