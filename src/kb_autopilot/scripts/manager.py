#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from kb_autopilot.msg import State
from kb_autopilot.msg import Manager_State
from kb_utils.msg import Encoder
from kb_autopilot.msg import Controller_Commands
from kb_utils.msg import Command
from kb_autopilot.msg import Waypoint

#TODO Import waypoints from somewhere



class Manager:
    BACK_UP_DIST = 4.5
    HOME_N = -385.5
    HOME_E = 65.3

    def __init__(self):
        self.state_sub = rospy.Subscriber('state', State, self.state_callback, queue_size = 1)
        self.cmds_sub = rospy.Subscriber('command_in', Command, self.cmds_callback, queue_size = 1)
        self.enc_sub = rospy.Subscriber('encoder', Encoder, self.encoder_callback, queue_size = 1)
        self.controller_cmd_sub = rospy.Subscriber('controller_commands_in', Controller_Commands, self.controller_cmd_callback, queue_size = 1)

        self.manager_pub = rospy.Publisher('manager_state', Manager_State, queue_size = 1)
        self.controller_cmds_pub = rospy.Publisher('controller_commands_out', Controller_Commands, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('waypoint', Waypoint, queue_size = 150)

        self.states = enum(START=1, KISS=2, REVERSE=3, HOME=4, FINISHED=5)
        self.state = self.states.START

        self.range_n = 10
        self.range_e = 3

        self.v_enc = 0.0
        self.dist = 0.0
        self.v_cmd = 0.0

        self.psi = 0.0

        self.start_t = 0.0


    def enum(**enums):
        return type('Enum', (), enums)

    def publish_state(self):
        msg = Manager_State()
        msg.state = self.state
        self.manager_pub.publish(msg)

    def publish_waypoints(self, points):
        #Do something
        msg = Waypoint()
        for i in range(len(points)):
            if i == 0:
                msg.set_current = True
                msg.clear_wp_list = True
            else:
                msg.set_current = False
                msg.clear_wp_list = False
            msg.w = point
            self.waypoint_pub.publish(msg)

    def state_callback(self, msg):
        self.psi = msg.psi
        p_n = msg.p_north
        p_e = msg.p_east

        if self.state == self.states.START:
            if abs(p_n) < self.range_n and abs(p_e) < self.range_e:
                self.state = self.state.KISS
                self.publish_state()

        if self.state == self.state.HOME:
            if abs(p_n - HOME_N) < 1 and abs(p_e - HOME_E):
                self.state = self.states.FINISHED
                self.publish_state()


    def cmds_callback(self, msg):
        self.v_cmd = msg.throttle


    def encoder_callback(self, msg):
        self.v_enc = msg.vel

        if self.state == self.state.REVERSE:
            self.dist = msg.dist
            if abs(self.dist) < BACK_UP_DIST:
                vel = Controller_Commands()
                vel.psi_c = 0.0
                vel.u_c = -1.0
                vel.aux_valid = False
                self.controller_cmds_pub.publish(vel)
            elif abs(self.dist) >= BACK_UP_DIST and abs(self.psi + np.pi/2.0) > (10.0 * np.pi / 180.0):
                vel = Controller_Commands()
                vel.psi_c = -np.pi/2.0
                vel.u_c = 1.0
                vel.aux_valid = False
                self.controller_cmds_pub.publish(vel)
            else:
                self.state = self.states.HOME
                self.publish_state()
                #Publish waypoints

        if self.state == self.states.KISS:
            if self.v_enc < .01 and self.v_cmd != 0.0:
                if self.start_t == 0.0:
                    self.start_t =rospy.Time.now()
                t =  rospy.Time.now()
                if((t-self.start_t).to_sec() > 1.0):
                    self.state = self.states.REVERSE
                    self.publish_state()
            else:
                self.start_t = 0.0

        #Publish our state

    def controller_cmd_callback(self, msg): #this is our reference input to the controller
        if self.state != self.states.REVERSE:
            self.controller_cmds_pub.publish(msg)

    def run(self):
        self.publish_state()
        #publish Waypoints
        while not rospy.is_shutdown():
            rospy.spin()

def main():
    print("Initializing node... ")
    rospy.init_node("manager", log_level=rospy.DEBUG)

    manager = Manager()
    manager.run()

    print("Done.")


if __name__ == '__main__':
    main()
