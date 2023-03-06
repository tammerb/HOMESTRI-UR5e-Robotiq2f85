#! /usr/bin/env python3

import rospy
import actionlib
from move_groups import Arm, Gripper
from threading import Thread
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, Point
from homestri_msgs.msg import ManipulationAction, ManipulationResult
    
class ThreadWithReturnValue(Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs={}, Verbose=None):
        Thread.__init__(self, group, target, name, args, kwargs)
        self._return = None

    def run(self):
        if self._target is not None:
            self._return = self._target(*self._args,
                                                **self._kwargs)
    def join(self, *args):
        Thread.join(self, *args)
        return self._return

class ManipulationActionServer():
    def __init__(self):
        super().__init__()

        self.arm = Arm('arm')
        self.gripper = Gripper('gripper')

        self.actserv = actionlib.SimpleActionServer(
            'manip_as', 
            ManipulationAction, 
            execute_cb=self.action_cb, 
            auto_start=False
        )
        self.actserv.start()

    def action_cb(self, goal):
        print("GOAL:")
        print(goal)

        id = goal.id
        target = goal.target
        position = goal.position
        offset = goal.offset
        pose = goal.pose

        if id == 'arm_pose':
            self.start_thread_and_wait_for_result(self.arm.move_to_pose, (pose, offset, ))
        elif id == 'arm_approach':
            self.start_thread_and_wait_for_result(self.arm.move_to_offset, (offset, 0, 0, ))
        elif id == 'arm_retreat':
            self.start_thread_and_wait_for_result(self.arm.move_to_offset, (-offset, 0, 0, ))
        elif id == 'arm_home':
            self.start_thread_and_wait_for_result(self.arm.move_to_target, ('home', ))
        elif id == 'arm_target':
            self.start_thread_and_wait_for_result(self.arm.move_to_target, (target, ))
        elif id == 'gripper_open':
            self.start_thread_and_wait_for_result(self.gripper.move_to_target, ('open', ))
        elif id == 'gripper_close':
            self.start_thread_and_wait_for_result(self.gripper.move_to_target, ('close', ))
        elif id == 'gripper_target':
            self.start_thread_and_wait_for_result(self.gripper.move_to_target, (target, ))
        elif id == 'gripper_position':
            self.start_thread_and_wait_for_result(self.gripper.move_to_position, (position, ))
        else:
            rospy.logerr("INVALID ID")
            self.actserv.set_aborted()

    def start_thread_and_wait_for_result(self, function, args):
        thread = ThreadWithReturnValue(target=function, args=args)
        thread.start()

        while thread.is_alive():
            if self.actserv.is_preempt_requested():
                self.arm.stop()
                self.gripper.stop()
                thread.join()
                self.actserv.set_preempted()
                rospy.loginfo("PREEMPTED")
                return
            rospy.sleep(0.02)
            
        success = thread.join()
        if not success:
            self.actserv.set_aborted()
            return

        self.actserv.set_succeeded()

if __name__ == "__main__":
    rospy.init_node('manipulation_action_server')
    manipulation_action_server = ManipulationActionServer()
    rospy.spin()