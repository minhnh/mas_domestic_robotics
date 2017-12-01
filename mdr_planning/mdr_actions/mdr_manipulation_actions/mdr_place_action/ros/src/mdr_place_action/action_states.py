#!/usr/bin/env python
import rospy
import actionlib
import simple_script_server
import geometry_msgs.msg
import tf
import math
import smach

from smach_ros import ActionServerWrapper, IntrospectionServer
from mdr_place_action.msg import PlaceAction, PlaceResult, PlaceFeedback

import mdr_lwr_kinematics.kinematics as kinematics
import mdr_lwr_kinematics.move_arm as move_arm


class SetupPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['place_goal'],
                             output_keys=['place_feedback', 'place_result'])

    def execute(self, userdata):
        feedback = PlaceFeedback()
        feedback.current_state = 'SETUP_PLACE'
        feedback.message = '[place] starting place skill'
        userdata.place_feedback = feedback
        return 'succeeded'


class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['place_goal'],
                             output_keys=['place_feedback', 'place_result'])

        self.kinematics = kinematics.Kinematics('arm')
        self.move_arm = move_arm.MoveArm('arm')

        # distance from object center to pre-grasp
        self.pre_grasp_distance = rospy.get_param('~pre_grasp_distance')
        # distance from object to grasp
        self.grasp_distance = rospy.get_param('~grasp_distance')
        # height of the post-grasp over the object center
        self.post_grasp_height = rospy.get_param('~post_grasp_height')
        # at which angle of approach to start searching for a solution
        self.orbit_start_angle = 30.0 * math.pi / 180.0

        rospy.loginfo('Pre-grasp distance: %f', self.pre_grasp_distance)
        rospy.loginfo('Grasp distance: %f', self.grasp_distance)
        rospy.loginfo('Post-grasp height: %f', self.post_grasp_height)

        self.sss = simple_script_server.simple_script_server()

    def execute(self, userdata):
        '''
        Perform the place task

        :param req: mdr_behavior_msgs.PlaceRequest
            req.position: geometry_msgs.msg.PointStamped
        '''
        rospy.logdebug('Start planning trajectory')

        userdata.place_goal.position.point.y += 0.05

        # pre-calculate the trajectory to the position
        traj = self.plan_trajectory(userdata.place_goal.position)

        if (not traj):
            rospy.loginfo('No trajectory found')
            return 'failed'

        rospy.logdebug('Found a trajectory')

        # move the arm along the calculated trajectory to the grasp
        # self.sss.move('arm', traj[0:len(traj)-2], blocking = True)
        self.move_arm.move(traj[0:len(traj) - 2])

        # open the hand
        sdh_handle = self.sss.move('gripper', 'cylopen')

        # move to post-grasp
        rospy.logdebug('Moving to post-grasp')
        self.move_arm.move(traj[len(traj) - 2:len(traj)])
        self.sss.move('gripper', 'cylclosed')

        rospy.loginfo('Grasped the object successfully')
        return 'succeeded'

    def plan_trajectory(self, position):
        '''
        Create a trajectory from the current position to the grasp position. An
        intermediate pre-grasp in front of the object is inserted between
        current and goal position.

        :param position: geometry_msgs.msg.PointStamped
        :return: None or ([Float[7], ..., Float[7]]
        '''

        # Prepregrasp pose
        prepregrasp = [-1.51, -1.65, -2.37, -1.62, 0.53, 1.96, -2.37]

        angle_counter = -0.2
        while (angle_counter < math.pi):
            angle_counter += 0.2

            # first we try to find a solution that is oriented with the hand's
            # pre-grasp pose later grasps try to approach opposite of that angle
            offset_angle_counter = angle_counter + self.orbit_start_angle
            if (offset_angle_counter >= math.pi / 2.0):
                angle = math.pi / 2.0 + self.orbit_start_angle - offset_angle_counter
            else:
                angle = offset_angle_counter

            rospy.loginfo('Planning grasp with approach angle: %f', angle)

            # determine the trajectory to the pre-grasp pose
            height = 0.1
            distance = self.pre_grasp_distance
            traj_pregrasp = self.calculate_trajectory_abs(position, distance, angle, height, prepregrasp)
            if (not traj_pregrasp):
                continue

            rospy.loginfo('Found pre-grasp')

            # determine the trajectory to the grasp pose
            height = 0.0
            distance = self.grasp_distance
            traj_grasp = self.calculate_trajectory_abs(position, distance, angle, height, traj_pregrasp)
            if (not traj_grasp):
                continue

            rospy.loginfo('Found grasp')

            # determine the trajectory to the post-grasp pose
            height = self.post_grasp_height
            distance = self.grasp_distance
            traj_post = self.calculate_trajectory_abs(position, distance, angle, height, traj_grasp)
            if (not traj_post):
                continue

            rospy.loginfo('Found post-grasp')

            # if all trajectories have been determined we return the result
            return [list(prepregrasp), list(traj_post), list(traj_grasp), list(traj_pregrasp), list(prepregrasp)]

        return None

    def calculate_trajectory_abs(self, position, distance, angle, height, configuration):
        '''
        Based on the goal position, the distance and angle from the goal
        position and a given configuration determine the joint angles to reach
        the distance in front of the provided position.

        :param position: geometry_msgs.msg.PointStamped
        :param distance: Float
        :param angle: Float
        :param height: Float
        :param configuration: Float[7]
        :return: None or Float[7]
        '''
        pose = geometry_msgs.msg.PoseStamped()
        pose = self.calculate_pose_abs(position, distance, angle, height)

        # determine the inverse kinematics solution
        return self.kinematics.inverse_kinematics(pose, configuration)

    def calculate_pose_abs(self, position, distance, angle, height):
        '''
        Determine a position that is the given distance in front of the provided
        position. The angle is given in the object frame and describes the angle
        of approach to the provided position.

        :param position: geometry_msgs.msg.PointStamped
        :param distance: Float
        :param angle: Float
        :param height: Float
        :return: geometry_msgs.msg.PoseStamped
        '''

        pose = geometry_msgs.msg.PoseStamped()
        pose.header = position.header

        deltaX = distance * math.cos(angle)
        deltaY = distance * math.sin(angle)
        roll = 0.0
        pitch = -math.pi / 2.0
        yaw = angle

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        pose.pose.position.x = position.point.x + deltaX
        pose.pose.position.y = position.point.y + deltaY
        pose.pose.position.z = position.point.z + height
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['place_goal'],
                             output_keys=['place_feedback', 'place_result'])
        self.result = result

    def execute(self, userdata):
        result = PlaceResult()
        result.success = self.result
        userdata.place_result = result
        return 'succeeded'
