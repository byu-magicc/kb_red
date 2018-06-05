#!/usr/bin/env python

import rospy
from kb_utils.msg import Command
from nav_msgs.msg import Odometry
from kb_autopilot.msg import Controller_Commands
from kb_autopilot.msg import State
from kb_utils.msg import Encoder
import numpy as np
import math
import time

class Controller:

    def __init__(self):
        self.path_sub = rospy.Subscriber('controller_cmd', Controller_Commands, self.reference_callback, queue_size = 1)
        # self.est_sub = rospy.Subscriber('odom', self.odometry_callback, queue_size = 1)
        self.est_sub = rospy.Subscriber('state', State, self.state_callback, queue_size = 1)   #will we be using the state message or odom from estimator?
        self.command_pub = rospy.Publisher('command', Command, queue_size = 1)
	self.enc_sub = rospy.Subscriber('encoder', Encoder,self.encoder_callback, queue_size = 1)

        self.thresh = 0.5 #antiwind up threshold

        #Variables for the linear velocity
        self.Kp_v = 0.5
        self.Kd_v = 0.03
        self.Ki_v =0.0# 0.1
        self.Km_v = 1.0 #Scale factor this probably needs to change
        self.prev_error_v = 0.0
        self.integrator_v = 0.0
        self.sigma = 2.5
        self.prev_v = 0.0
	self.e_sat_v = 0.3
	self.u_sat_v = 0.3

        self.prev_time = rospy.Time.now()
        self.v_dot = 0.0
        self.v_command = 0.0
        self.v_sat = 1.0

        #Variables for the angular velocity
        self.Kp_psi = 0.7
        self.Kd_psi = 0.3
        self.Ki_psi = 0.03
        self.Km_psi = .758 #Scale factor
        self.prev_error_psi = 0.0
        self.integrator_psi = 0.0
        self.prev_psi = 0.0
        self.psi_dot = 0.0
        self.psi_command = 0.0
        self.psi_sat = 1.0

        #Variables for storing values from messages
        self.v_ref = 0.0
        self.psi_ref = 0.0
        self.v = 0.0
        self.psi = 0.0
        self.command_v = 0.0
        self.command_psi = 0.0

    def reference_callback(self, msg):
        #will store the reference values
	print 'Message received'
        self.v_ref = msg.u_c
        self.psi_ref = msg.psi_c #heading angle

    def run(self):
	while not rospy.is_shutdown():
	    rospy.spin()

    def state_callback(self, msg):

        psi = msg.data.psi #Heading angle
        v = msg.data.u   #Body velocity
        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        self.prev_time = now

        #Angle controller
        error = self.psi_ref - heading

        if self.psi_dot < self.thresh: #Anti wind up. Integrator will only be used if the derivative is small
            self.integrator_psi = self.integrator_psi + dt / 2.0 * (error - self.prev_error_psi)
        self.psi_dot = (2 * self.sigma - dt)/(2 * self.sigma + dt) * self.psi_dot + 2.0 / (2 * self.sigma + dt) * (psi- self.prev_psi)
        self.prev_psi = w

        u_unsat = self.Kp_psi * error - self.Kd_psi * self.psi_dot + self.Ki_psi * self.integrator_psi

        self.psi_command = u_unsat / self.Km_psi

        if self.psi_command > 1.0 or self.psi_command < -1.0:
            self.psi_command = 1.0 * np.sign(self.psi_command)

        #Throttle Controller
        error = self.v_ref - v

        if self.v_dot < self.thresh: #Anti wind up. Integrator will only be used if the derivative is small
            self.integrator_v = self.integrator_v + dt / 2.0 * (error - self.prev_error_v)  #add wind up
        self.v_dot = (2 * self.sigma - dt)/(2 * self.sigma + dt) * self.v_dot + 2.0 / (2 * self.sigma + dt) * (v - self.prev_v)
        self.prev_v = v

        u_unsat = self.Kp_v * error - self.Kd_v * self.v_dot + self.Ki_v * self.integrator_v

        self.v_command = u_unsat / self.Km_v

        if self.v_command > 1.0 or self.v_command < -1.0:
            self.v_command = 1.0 * np.sign(self.v_command)

        vel = Command()
        vel.steer = self.psi_command
        vel.throttle = self.v_command

        self.command_pub.publish(vel)

    def encoder_callback(self, msg):
    	v = msg.vel
       # print 'Velocity', v
    	now = rospy.Time.now()
    	dt = (now - self.prev_time).to_sec()
    	self.prev_time = now

    	error = self.v_ref - v
	if error > self.e_sat_v:
	    error = self.e_sat_v
	elif error < - self.e_sat_v:
	    error = - self.e_sat_v

	if v<0.1 and v>-.1:
	    self.integrator_v = 0
        self.integrator_v = self.integrator_v + dt / 2.0 * (error - self.prev_error_v)
	#print self.integrator_v
    	self.v_dot = (2 * self.sigma - dt)/(2 * self.sigma + dt) * self.v_dot + 2.0 / (2 * self.sigma + dt) * (v - self.prev_v)
    	self.prev_v = v

    	u_unsat = self.Kp_v * error - self.Kd_v * self.v_dot + self.Ki_v * self.integrator_v

    	u = u_unsat / self.Km_v

    	if u > self.u_sat_v or u < -self.u_sat_v:
    	    self.v_command = self.u_sat_v * np.sign(u)
	else:
	    self.v_command = u

	#Anti wind up. Apply else where also
	if self.Ki_v != 0.0:
	    self.integrator_v = self.integrator_v + dt/self.Ki_v * (self.v_command - u)

    	vel = Command()
    	vel.steer = 0.0
    	vel.throttle = self.v_command
    	self.command_pub.publish(vel)
def main():
    """driver to interface to the Teensy on the KB_Car

    Command Line Arguments
    port -- the serial port to talk to the Teensy
    """
   # arg_fmt = argparse.RawDescriptionHelpFormatter
   # parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
   # parser.add_argument('port', type=str, default='')
   # args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("v_controller", log_level=rospy.DEBUG)

    controller = Controller()
    controller.run()

    print("Done.")

if __name__ == '__main__':
    main()

