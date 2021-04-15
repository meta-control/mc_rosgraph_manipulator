#! /usr/bin/env python

import rospy
import sys
import actionlib

from metacontrol_msgs.msg import MvpReconfigurationResult
from metacontrol_msgs.msg import MvpReconfigurationAction
import subprocess
import os
import dynamic_reconfigure.client
from yaml import load
import rospkg
import rosparam
import roslib
import rosnode
import rostopic
from rospy.msg import AnyMsg
from roslib.message import get_message_class
from pyparsing import ParseResults
from importlib import import_module

roslib.load_manifest('rosparam')
rospack = rospkg.RosPack()


def kill_node(node_name, ns=''):
    command = "rosnode kill {0}".format(node_name)
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = ns
    p = subprocess.Popen(command, shell=True, env=my_env)
    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


def restart_command(arg, ns=''):
    rospy.loginfo("launching new configuration...")
    my_env = os.environ.copy()
    my_env["ROS_NAMESPACE"] = ns
    p = subprocess.Popen(arg, shell=True, env=my_env)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


class RosgraphManipulatorActionServer (object):

    _result = MvpReconfigurationResult()

    def __init__(self, name, configFilePath):
        params = rosparam.load_file(configFilePath)
        for param, ns in params:
            try:
                rosparam.upload_params(ns, param)
            except ROSParamException, e:
                print >> sys.stderr, "ERROR: "+str(e)
                pass  # ignore empty params
        self.reconfiguration_action_name = rospy.get_param(
            'reconfiguration_action_name')
        self.configurations = rospy.get_param('configurations')
        self.kill_nodes = rospy.get_param('kill_nodes', [])
        self.save_action = rospy.get_param('save_action', None)
        self.goal_msg_type = rospy.get_param('goal_msg_type', None)
        self.last_saved_goal = None

        action_msg_type = self.goal_msg_type.split('.msg.')

        ros_pkg = action_msg_type[0] + '.msg'
        action_type = action_msg_type[1]
        action_goal_type = action_msg_type[1] + 'Goal'

        self.action_class = getattr(import_module(ros_pkg), action_type)
        goal_msg_class = getattr(import_module(ros_pkg), action_goal_type)

        rospy.init_node('rosgraph_manipulator_action_server_node')

        rospy.loginfo('action_msg_type {}'.format(self.goal_msg_type))

        rospy.loginfo(
            'action_msg_type {0} - {1}'.format(action_msg_type[0],
                                               action_msg_type[1]))

        self._as = actionlib.SimpleActionServer(
            self.reconfiguration_action_name, MvpReconfigurationAction,
            execute_cb=self.reconfiguration_action_cb, auto_start=False)
        self._as.start()
        rospy.loginfo('RosgraphManipulator Action Server started.')
        if self.save_action is not None:
            rospy.loginfo('Subscribe to the %s/goal' % str(self.save_action))
            self.msg_type = rostopic.get_topic_type(self.save_action+'/goal')
            rospy.Subscriber(self.save_action+'/goal',
                             goal_msg_class, self.callback)

    def callback(self, goal_msg):
        self.last_saved_goal = goal_msg.goal

    def reconfiguration_action_cb(self, goal):
        rospy.loginfo(
            'Rosgraph Manipulator Action Server received goal %s' % str(goal))
        self._result.result = 1
        desired_configuration = goal.desired_configuration_name

        if (desired_configuration in self.configurations):
            for node in self.kill_nodes:
                kill_node(node)
                rospy.loginfo("Stopping node %s" % str(node))
                rospy.sleep(2)
            rospy.loginfo('Launching new configuration')
            self.command = self.configurations.get(desired_configuration)
            rospy.loginfo('Launching new configuration, command %s' %
                          str(self.command.get('command')))
            restart_command(self.command.get('command'))
            if (self.save_action is not None
               and self.last_saved_goal is not None):

                self._action_client = actionlib.SimpleActionClient(
                    self.save_action, self.action_class)
                wait = self._action_client.wait_for_server(rospy.Duration(6.0))
                if not wait:
                    rospy.logerr("%s action server not available" %
                                 str(self.save_action))
                    return
                rospy.loginfo(
                    "Connected to move_base server and sending Nav Goal")
                print(self.last_saved_goal)
                self._action_client.send_goal(self.last_saved_goal)
        else:
            self._result.result = -1
            self._as.set_aborted(self._result)
            rospy.logerr('Unknown configuration request %s' %
                         desired_configuration)
            return
        self._as.set_succeeded(self._result)
        return


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: rosrun mc_rosgraph_manipulator"
              + "rosgraph_manipulator.py /path/to/config/file")
    else:
        try:
            RosgraphManipulatorActionServer(
                'rosgraph_manipulator_action_server', sys.argv[1])
        except rospy.ROSInterruptException:
            rospy.signal_shutdown('shutdown')
            pass

    rospy.spin()
    pass
