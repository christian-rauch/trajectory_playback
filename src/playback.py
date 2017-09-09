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
        rospy.init_node("trajectory_playback", anonymous=True)

        self.action_client_arm = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_hand = actionlib.SimpleActionClient('/gripper/sdh_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("waiting for trajectory controller...")
        # if not self.action_client_arm.wait_for_server():
        #     exit()

        try:
            self.export_path = os.path.expanduser(rospy.get_param("~export_path"))
            self.full_power_cycle = rospy.get_param("~full_power_cycle", True)
            self.play_all = rospy.get_param("~play_all", False)
        except KeyError as e:
            print(e.message + " is undefined")
            exit()

        self.pub_vis = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=1)

        # define services that are called before and after sending hand joint values
        if self.full_power_cycle:
            self.srv_sdh_start = rospy.ServiceProxy("/gripper/sdh_controller/init", Trigger)
            self.srv_sdh_stop = rospy.ServiceProxy("/gripper/sdh_controller/shutdown", Trigger)
        else:
            self.srv_sdh_start = rospy.ServiceProxy("/gripper/sdh_controller/motor_on", Trigger)
            self.srv_sdh_stop = rospy.ServiceProxy("/gripper/sdh_controller/motor_off", Trigger)

        self.traj_list = sorted(glob.glob(os.path.join(self.export_path, "traj_*.yaml")))
        print("found",len(self.traj_list),"trajectories")
        
    def start(self):
        if not self.full_power_cycle:
            print("init SDH once")
            srv = rospy.ServiceProxy("/gripper/sdh_controller/init", Trigger)
            srv()
            
    def stop(self):
        if not self.full_power_cycle:
            print("shutdown SDH")
            srv = rospy.ServiceProxy("/gripper/sdh_controller/shutdown", Trigger)
            srv()

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

            if self.play_all:
                # automatically send each trajecotry
                execute = True
            else:
                # wait for user input
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
                    try:
                        res = self.srv_sdh_start()
                        if res.success==False:
                            print("res",res)
                    except rospy.service.ServiceException as e:
                        rospy.logerr("service init: %s",e.message)


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
                    try:
                        res = self.srv_sdh_stop()
                        if res.success==False:
                            print("res",res)
                    except rospy.service.ServiceException as e:
                        rospy.logerr("service shutdown: %s",e.message)

            elif skip:
                print("skip trajectory")
            else:
                print("???")
        print("done")


if __name__ == '__main__':
    tp = TrajectoryPlayback()
    tp.start()
    tp.run()
    tp.stop()
