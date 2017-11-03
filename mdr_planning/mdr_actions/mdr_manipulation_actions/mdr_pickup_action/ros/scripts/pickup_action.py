#!/usr/bin/env python
import rospy
import smach

from smach_ros import ActionServerWrapper, IntrospectionServer
from mdr_pickup_action.msg import PickUpAction, PickUpResult
from mdr_pickup_action.action_states import SetupPickUp, PickUp, SetActionLibResult

class PickUpSkill(smach.StateMachine):
    def __init__(self, timeout=10):
        smach.StateMachine.__init__(self,
                outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILED', 'PREEMPTED'],
                input_keys=['pickup_goal'],
                output_keys=['pickup_feedback', 'pickup_result'])

        with self:
            smach.StateMachine.add('SETUP_PICKUP', SetupPickUp(),
                                   transitions = {'succeeded': 'PICKUP',
                                                'failed': 'SETUP_PICKUP'})

            smach.StateMachine.add('PICKUP', PickUp(),
                                   transitions = {'succeeded': 'SET_ACTION_LIB_SUCCESS',
                                                'failed': 'SET_ACTION_LIB_FAILED'})

            smach.StateMachine.add('SET_ACTION_LIB_FAILED', SetActionLibResult(False),
                                   transitions = {'succeeded': 'OVERALL_FAILED'})

            smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                                   transitions = {'succeeded': 'OVERALL_SUCCESS'})


if __name__ == "__main__":
    rospy.init_node('pickup_server')

    # construct state machine
    sm = PickUpSkill()

    # smach viewer
    sis = IntrospectionServer('pickup_smach_viewer', sm, '/PICKUP_SMACH_VIEWER')
    sis.start()

    asw = ActionServerWrapper(
        server_name = 'pickup_server',
        action_spec = PickUpAction,
        wrapped_container = sm,
        succeeded_outcomes = ['OVERALL_SUCCESS'],
        aborted_outcomes = ['OVERALL_FAILED'],
        preempted_outcomes = ['PREEMPTED'],
        goal_key = 'pickup_goal',
        feedback_key = 'pickup_feedback',
        result_key = 'pickup_result')

    # Run the server in a background thread
    asw.run_server()
    rospy.spin()