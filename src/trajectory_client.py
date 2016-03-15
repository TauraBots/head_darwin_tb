#!/usr/bin/env python
#This is from dynimixel ROS tutorial
#http://wiki.ros.org/dynamixel_controllers/Tutorials/

import roslib
roslib.load_manifest('head_darwin_tb')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


#for the head joint
class Joint:
        def __init__(self, motor_name):
            self.name = motor_name
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')
            self.m_Pan_p_gain = 0.1
            self.m_Pan_d_gain = 0.22
            self.m_Tilt_p_gain = 0.1
            self.m_Tilt_d_gain = 0.22
            self.m_Pan_err = 0
            self.m_Pan_err_diff = 0
            self.m_Tilt_err = 0
            self.m_Tilt_err_diff = 0
            self.m_PanAngle = 0
            self.m_TiltAngle = 0

        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            char = self.name[0]
            goal.trajectory.joint_names = ['head_pan_joint',  'head_tilt_joint']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def move_to_xy(self, point2D):
            self.m_Pan_err_diff = point2D[0] - self.m_Pan_err
            self.m_Pan_err = point2D[0]
            self.m_Tilt_err_diff = point2D[1] - self.m_Tilt_err
            self.m_Tilt_err = point2D[1]

            pOffset = self.m_Pan_err * self.m_Pan_p_gain
            pOffset *= pOffset
            if(self.m_Pan_err < 0):
                  pOffset = -pOffset
            dOffset = self.m_Pan_err_diff * self.m_Pan_d_gain
            dOffset *= dOffset
            if(self.m_Pan_err_diff < 0):
                  dOffset = -dOffset
            self.m_PanAngle += (pOffset + dOffset)

            pOffset = self.m_Tilt_err * self.m_Tilt_p_gain
            pOffset *= pOffset
            if(self.m_Tilt_err < 0):
                  pOffset = -pOffset
            dOffset = self.m_Tilt_err_diff * self.m_Tilt_d_gain
            dOffset *= dOffset
            if(self.m_Tilt_err_diff < 0):
                  dOffset = -dOffset
            self.m_TiltAngle += (pOffset + dOffset)
            print "Tilt: %.2f Pan: %2f" % (self.m_TiltAngle, self.m_PanAngle)
             arm.move_joint([self.m_PanAngle,self.m_TiltAngle])


def main():
            arm = Joint('f_head')
            arm.move_to_xy([1, 1])
            arm.move_joint([0,0])
            arm.move_joint([0.9,0.6])
            arm.move_joint([0,0])


if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
