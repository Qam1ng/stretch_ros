#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import argparse as ap
import math

import hello_helpers.hello_misc as hm

class GetVoiceCommands:

    def __init__(self, mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on):
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on

        self.step_size = 'medium'
        self.rad_per_deg = math.pi / 180.0
        self.small_deg = 3.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.005  # 0.02
        self.medium_deg = 6.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.04
        self.big_deg = 12.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.06
        self.mode = 'position'

    def get_deltas(self):
        if self.step_size == 'small':
            deltas = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            deltas = {'rad': self.medium_rad, 'translate': self.medium_translate}
        if self.step_size == 'big':
            deltas = {'rad': self.big_rad, 'translate': self.big_translate}
        return deltas

    def process_speech_command(self, speech_text, node):
        command = None
        c = speech_text.lower().strip()
        # Now we can map speech commands to actions, similar to the keyboard commands.

        ####################################################
        ## MAPPING RELATED CAPABILITIES
        ####################################################
        if ("start mapping" in c) and self.mapping_on:
            number_iterations = 4
            for n in range(number_iterations):
                trigger_request = TriggerRequest()
                trigger_result = node.trigger_head_scan_service(trigger_request)
                rospy.loginfo('trigger_result = {0}'.format(trigger_result))

                trigger_request = TriggerRequest()
                trigger_result = node.trigger_drive_to_scan_service(trigger_request)
                rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("global localization" in c) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_global_localization_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("local localization" in c) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_local_localization_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("drive to scan" in c) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_drive_to_scan_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("head scan" in c) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_head_scan_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("align with cliff" in c) and self.mapping_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_align_with_nearest_cliff_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        ####################################################
        ## OTHER CAPABILITIES
        ####################################################
        if ("write hello" in c) and self.hello_world_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_write_hello_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("open drawer down" in c) and self.open_drawer_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_open_drawer_down_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("open drawer up" in c) and self.open_drawer_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_open_drawer_up_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("clean surface" in c) and self.clean_surface_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_clean_surface_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("grasp object" in c) and self.grasp_object_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_grasp_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if ("deliver object" in c) and self.deliver_object_on:
            trigger_request = TriggerRequest()
            trigger_result = node.trigger_deliver_object_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        ####################################################
        ## BASIC VOICE COMMANDS
        ####################################################
        if ("lift up" in c):
            command = {'joint': 'joint_lift', 'delta': self.get_deltas()['translate']}
        if ("lift down" in c):
            command = {'joint': 'joint_lift', 'delta': -self.get_deltas()['translate']}

        if self.mode == 'position':
            if ("move forward" in c):
                command = {'joint': 'translate_mobile_base', 'inc': self.get_deltas()['translate']}
            if ("move backward" in c):
                command = {'joint': 'translate_mobile_base', 'inc': -self.get_deltas()['translate']}
            if ("turn left" in c):
                command = {'joint': 'rotate_mobile_base', 'inc': self.get_deltas()['rad']}
            if ("turn right" in c):
                command = {'joint': 'rotate_mobile_base', 'inc': -self.get_deltas()['rad']}

        if ("arm out" in c):
            command = {'joint': 'wrist_extension', 'delta': self.get_deltas()['translate']}
        if ("arm in" in c):
            command = {'joint': 'wrist_extension', 'delta': -self.get_deltas()['translate']}
        if ("wrist forward" in c):
            command = {'joint': 'joint_wrist_yaw', 'delta': -self.get_deltas()['rad']}
        if ("wrist back" in c):
            command = {'joint': 'joint_wrist_yaw', 'delta': self.get_deltas()['rad']}
        if ("pitch forward" in c):
            command = {'joint': 'joint_wrist_pitch', 'delta': -self.get_deltas()['rad']}
        if ("pitch back" in c):
            command = {'joint': 'joint_wrist_pitch', 'delta': self.get_deltas()['rad']}
        if ("roll forward" in c):
            command = {'joint': 'joint_wrist_roll', 'delta': -self.get_deltas()['rad']}
        if ("roll back" in c):
            command = {'joint': 'joint_wrist_roll', 'delta': self.get_deltas()['rad']}
        if ("grip" in c):
            command = {'joint': 'joint_gripper_finger_left', 'delta': -self.get_deltas()['rad']}
        if ("release" in c):
            command = {'joint': 'joint_gripper_finger_left', 'delta': self.get_deltas()['rad']}
        if ("head up" in c):
            command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.get_deltas()['rad'])}
        if ("head down" in c):
            command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if ("head left" in c):
            command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.get_deltas()['rad'])}
        if ("head right" in c):
            command = {'joint': 'joint_head_pan', 'delta': -(2.0 * self.get_deltas()['rad'])}
        if ("big step" in c):
            rospy.loginfo('Changing to BIG step size')
            self.step_size = 'big'
        if ("medium step" in c):
            rospy.loginfo('Changing to MEDIUM step size')
            self.step_size = 'medium'
        if ("small step" in c):
            rospy.loginfo('Changing to SMALL step size')
            self.step_size = 'small'
        if ("quit" in c):
            rospy.loginfo('Voice command control exiting...')
            rospy.signal_shutdown('Received quit command, exiting.')

        return command


class VoiceCommandControlNode(hm.HelloNode):

    def __init__(self, mapping_on=False, hello_world_on=False, open_drawer_on=False, clean_surface_on=False, grasp_object_on=False, deliver_object_on=False):
        hm.HelloNode.__init__(self)
        self.voice_commands = GetVoiceCommands(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        self.rate = 10.0
        self.joint_state = None
        self.mapping_on = mapping_on
        self.hello_world_on = hello_world_on
        self.open_drawer_on = open_drawer_on
        self.clean_surface_on = clean_surface_on
        self.grasp_object_on = grasp_object_on
        self.deliver_object_on = deliver_object_on

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def speech_callback(self, msg):
        speech_text = msg.data
        rospy.loginfo(f"Received speech command: {speech_text}")
        command = self.voice_commands.process_speech_command(speech_text, self)
        self.send_command(command)

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            elif 'delta' in command:
                if joint_name in joint_state.name:
                    joint_index = joint_state.name.index(joint_name)
                    joint_value = joint_state.position[joint_index]
                else:
                    joint_value = 0.0  # Default if joint not found
                delta = command['delta']
                new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)

    def main(self):
        hm.HelloNode.main(self, 'voice_command_control', 'voice_command_control', wait_for_first_pointcloud=False)

        if self.mapping_on:
            rospy.loginfo('Node ' + self.node_name + ' waiting to connect to /funmap/trigger_head_scan.')

            rospy.wait_for_service('/funmap/trigger_head_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_head_scan.')
            self.trigger_head_scan_service = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_drive_to_scan.')
            self.trigger_drive_to_scan_service = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', Trigger)

            rospy.wait_for_service('/funmap/trigger_global_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_global_localization.')
            self.trigger_global_localization_service = rospy.ServiceProxy('/funmap/trigger_global_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_local_localization')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_local_localization.')
            self.trigger_local_localization_service = rospy.ServiceProxy('/funmap/trigger_local_localization', Trigger)

            rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')
            self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy('/funmap/trigger_align_with_nearest_cliff', Trigger)

            rospy.wait_for_service('/funmap/trigger_reach_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
            self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger)

            rospy.wait_for_service('/funmap/trigger_lower_until_contact')
            rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
            self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        if self.hello_world_on:
            rospy.wait_for_service('/hello_world/trigger_write_hello')
            rospy.loginfo('Node ' + self.node_name + ' connected to /hello_world/trigger_write_hello.')
            self.trigger_write_hello_service = rospy.ServiceProxy('/hello_world/trigger_write_hello', Trigger)

        if self.open_drawer_on:
            rospy.wait_for_service('/open_drawer/trigger_open_drawer_down')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_down.')
            self.trigger_open_drawer_down_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_down', Trigger)

            rospy.wait_for_service('/open_drawer/trigger_open_drawer_up')
            rospy.loginfo('Node ' + self.node_name + ' connected to /open_drawer/trigger_open_drawer_up.')
            self.trigger_open_drawer_up_service = rospy.ServiceProxy('/open_drawer/trigger_open_drawer_up', Trigger)

        if self.clean_surface_on:
            rospy.wait_for_service('/clean_surface/trigger_clean_surface')
            rospy.loginfo('Node ' + self.node_name + ' connected to /clean_surface/trigger_clean_surface.')
            self.trigger_clean_surface_service = rospy.ServiceProxy('/clean_surface/trigger_clean_surface', Trigger)

        if self.grasp_object_on:
            rospy.wait_for_service('/grasp_object/trigger_grasp_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /grasp_object/trigger_grasp_object.')
            self.trigger_grasp_object_service = rospy.ServiceProxy('/grasp_object/trigger_grasp_object', Trigger)

        if self.deliver_object_on:
            rospy.wait_for_service('/deliver_object/trigger_deliver_object')
            rospy.loginfo('Node ' + self.node_name + ' connected to /deliver_object/trigger_deliver_object.')
            self.trigger_deliver_object_service = rospy.ServiceProxy('/deliver_object/trigger_deliver_object', Trigger)

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/speech_text', String, self.speech_callback)

        rospy.spin()

if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Voice command control for Stretch.')
        parser.add_argument('--mapping_on', action='store_true', help='Enable mapping control.')
        parser.add_argument('--hello_world_on', action='store_true', help='Enable Hello World trigger.')
        parser.add_argument('--open_drawer_on', action='store_true', help='Enable Open Drawer trigger.')
        parser.add_argument('--clean_surface_on', action='store_true', help='Enable Clean Surface trigger.')
        parser.add_argument('--grasp_object_on', action='store_true', help='Enable Grasp Object trigger.')
        parser.add_argument('--deliver_object_on', action='store_true', help='Enable Deliver Object trigger.')

        args, unknown = parser.parse_known_args()
        mapping_on = args.mapping_on
        hello_world_on = args.hello_world_on
        open_drawer_on = args.open_drawer_on
        clean_surface_on = args.clean_surface_on
        grasp_object_on = args.grasp_object_on
        deliver_object_on = args.deliver_object_on

        node = VoiceCommandControlNode(mapping_on, hello_world_on, open_drawer_on, clean_surface_on, grasp_object_on, deliver_object_on)
        node.main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo('Interrupt received, so shutting down')
