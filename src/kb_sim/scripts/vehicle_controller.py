#!/usr/bin/env python

import rospy
import argparse
import random
import numpy as np
from math import cos, sin, atan2, sqrt
# from Roadmap import Roadmap
from datetime import datetime
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class VehicleController(object):

    class Roadmap:
        """A class to represent a road network"""

        def __init__(self, nodes, edges, bidirectional=True):
            """
            nodes: list of tuples (x, y). Defines the cartesian location of each intersection.
            edges: list of tuples (start, end). Defines the roads between intersections. Each edge is
                unidirectional.
            """
            self.graph = {(node[1],node[0]) : {} for node in nodes}
            for edge in edges:
                a = (nodes[edge[0]][1],nodes[edge[0]][0])
                b = (nodes[edge[1]][1],nodes[edge[1]][0])
                slope = [b[0] - a[0], b[1] - a[1]]
                # c = (a[0] + slope[0] * .1, a[1] + slope[1] * .1)
                dist = np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
                self.graph[a][b] = dist
                # self.graph[a][c] = dist
                # self.graph[c] = {b: dist}

                if bidirectional:
                    # d = (b[0] - slope[0] * .1, b[1] - slope[1] * .1)
                    # self.graph[b][d] = dist
                    # self.graph[d] = {a: dist}
                    self.graph[b][a] = dist

        def get_loc(self, state):
            """Returns the (x,y) location of a position along an edge
            edge -- tuple containing the (x,y) coordnates of
                    the beginning and ending nodes of the edge
            pos  -- value between 0 and 1 indicating the distance along the edge
            """
            edge = state[0]
            pos = state[1]

            try:
                self.graph[edge[0]][edge[1]]
            except KeyError:
                raise ValueError("Invalid roadmap edge.")

            loc = (pos*edge[1][0] + (1-pos)*edge[0][0],
                   pos*edge[1][1] + (1-pos)*edge[0][1])
            return loc

        def getNearestWaypoint(self, x, y):
            waypoint = None
            min_dist = 999999999
            for node in self.graph:
                dist = (x - node[0])**2 + (y - node[1])**2
                if dist < min_dist:
                    min_dist = dist
                    waypoint = node
            return waypoint

        def getNextWaypoint(self, waypoint, psi):
            options = self.graph[waypoint].keys()
            next = random.choice(options)
            next_psi = atan2(next[1] - waypoint[1], next[0] - waypoint[0])
            diff_angle = abs(((next_psi - psi) + np.pi) % (2*np.pi) - np.pi)
            # -(waypoint, next, psi, next_psi, diff_angle)
            while diff_angle > 2.2:
                next = random.choice(options)
                next_psi = atan2(next[1] - waypoint[1], next[0] - waypoint[0])
                diff_angle = abs(((next_psi - psi) + np.pi) % (2*np.pi) - np.pi)
                options.remove(next)
                # print(waypoint, next, psi, next_psi, diff_angle)
            return next

    def __init__(self, waypoints):
        self.threshold = 20
        self.roadmap = self.Roadmap(waypoints['nodes'], waypoints['edges'])
        rospy.Subscriber("gas_pedal/state", Float64, self.gasPedalCallback)
        rospy.Subscriber("hand_brake/state", Float64, self.brakeCallback)
        rospy.Subscriber("hand_wheel/state", Float64, self.steeringCallback)
        rospy.Subscriber("odometry", Odometry, self.odometryCallback)
        self.prev_waypoint = None
        self.waypoint = None

        self.gas_pub = rospy.Publisher("gas_pedal/cmd", Float64, queue_size=1)
        self.brake_pub = rospy.Publisher("hand_brake/cmd", Float64, queue_size=1)
        self.wheel_pub = rospy.Publisher("hand_wheel/cmd", Float64, queue_size=1)
        self.path_pub = rospy.Publisher("visualization/path", Marker, queue_size=1)

        self.maxSpeed = 4.5
        self.maxSteer = 0.6458
        self.nominal_speed = 1
        # self.omega = 2*np.pi*freq

        self.pid = {
            'x': {
                'tprev': datetime.now(),
                'i_err': 0,
                'err_prev': 0,
                'kp': 1,
                'kd': .5,
                'ki': .1,
                'sat': 1
            },
            'psi': {
                'tprev': datetime.now(),
                'i_err': 0,
                'err_prev': 0,
                'kp': 1,
                'kd': .5,
                'ki': .1,
                'sat': self.maxSteer
            }
        }

    def computePID(self, x, dx, xd, kp, kd, ki,
            sat, i_error, err_prev, t_prev):
        t_now = datetime.now()
        dt = t_now - t_prev
        t_prev = t_now

        error = xd - x
        prop = kp * error
        diff = -kd * dx
        i_error += ki * .5 * (error + err_prev) * dt.total_seconds()
        u_unsat = prop + diff + i_error

        if u_unsat > sat:
            u = sat
        elif u_unsat < -sat:
            u = -sat
        else:
            u = u_unsat

        if ki > 0:
            i_error += dt.total_seconds() / ki * (u - u_unsat)

        err_prev = error

        return u

    def odometryCallback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        dx = msg.twist.twist.linear.x
        dy = msg.twist.twist.linear.y
        self.dpsi = msg.twist.twist.angular.z
        orientation = msg.pose.pose.orientation
        self.phi, self.theta, self.psi = euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w))

        self.u = dx * cos(self.psi) + dy * sin(self.psi)
        v = dx * -sin(self.psi) + dy * cos(self.psi)
        if self.waypoint is None:
            self.waypoint = self.roadmap.getNearestWaypoint(self.x,self.y)
        else:
            dist = sqrt((self.x - self.waypoint[0])**2 + (self.y - self.waypoint[1])**2)
            if dist < self.threshold:
                self.waypoint = self.roadmap.getNextWaypoint(self.waypoint, self.psi)

        self.dpx = self.nominal_speed + self.waypoint[0]/10
        self.dpy = self.nominal_speed + self.waypoint[1]/10

        path = Marker()
        path.type = Marker.POINTS
        path.header.stamp = msg.header.stamp
        path.header.frame_id = 'world'
        path.scale.x = 1
        path.scale.y = 1
        path.color.r = 0.0
        path.color.g = 1.0
        path.color.b = 1.0
        path.color.a = 1.0
        p = Point()
        p.x = self.waypoint[0]
        p.y = self.waypoint[1]
        p.z = 0
        path.points.append(p)
        self.path_pub.publish(path)

    def steeringCallback(self, msg):
        if self.waypoint is not None:
            px_rel = (self.waypoint[1] - self.y) * sin(self.psi) + (self.waypoint[0] - self.x) * cos(self.psi)
            py_rel = (self.waypoint[1] - self.y) * cos(self.psi) + (self.waypoint[0] - self.x) * -sin(self.psi)
            psi_rel = atan2(py_rel, px_rel)
            psi_d = self.psi + psi_rel
            steering = self.computePID(
                self.psi, self.dpsi, psi_d, self.pid['psi']['kp'],
                self.pid['psi']['kd'], self.pid['psi']['ki'], self.pid['psi']['sat'],
                self.pid['psi']['i_err'], self.pid['psi']['err_prev'],
                self.pid['psi']['tprev'])

            self.wheel_pub.publish(steering)

    def brakeCallback(self, msg):
        desired_state = 0
        hand_brake_state = msg.data

        if abs(hand_brake_state - desired_state) > .01:
            self.brake_pub.publish(desired_state)

    def gasPedalCallback(self, msg):
        if self.waypoint is not None:
            px_rel = (self.waypoint[1] - self.y) * sin(self.psi) + (self.waypoint[0] - self.x) * cos(self.psi)
            py_rel = (self.waypoint[1] - self.y) * cos(self.psi) + (self.waypoint[0] - self.x) * -sin(self.psi)


            u_r = self.dpx * cos(self.psi) + self.dpy * sin(self.psi)
            u_rel = u_r - self.u

            gas = self.computePID(0, self.u, px_rel, self.pid['x']['kp'], self.pid['x']['kd'],
                self.pid['x']['ki'], self.pid['x']['sat'], self.pid['x']['i_err'],
                self.pid['x']['err_prev'], self.pid['x']['tprev'])

            if px_rel < -.1:
                gas = .1

            self.gas_pub.publish(gas)

    def clean_shutdown(self):
        print("\nExiting pointer...")
        return True

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    """
    Commands vehicle controls
    """

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    # parser.add_argument('waypoint_file', type=str, default="test")
    parser.add_argument('vehicle', type=str, default='atv0')
    args = parser.parse_args(rospy.myargv()[1:])
    # print("waypoint file: ", args.waypoint_file)

    nodes = rospy.get_param("/%s/%s_controller/waypoints/nodes" % (args.vehicle, args.vehicle))
    edges = rospy.get_param("/%s/%s_controller/waypoints/edges" % (args.vehicle, args.vehicle))

    print("Initializing Vehicle Controller node... ")
    rospy.init_node("vehicle_controller", log_level=rospy.DEBUG)

    waypoints = {
        "nodes": nodes[1:],
        "edges": edges[1:]
        # "nodes": [
        #     (-305, -152), (-305, 152), (0, -152),
        #     (0, 152), (305, -152), (305, 152)],
        # "edges":[
        #     (0,1), (0,2), (2,3), (2,4), (3, 1), (3,5), (4, 5)
        #     # ((-305, -155),(-285, 155)),
        #     # ((-305, -155),(0, -155)),
        #     # ((0,-155), (0,155)),
        #     # ((0, -155), (305, -155)),
        #     # ((0, 155), (305, 155)),
        #     # ((0, 155), (-305, 155)),
        #     # ((305, -155), (305, 155))
        # ]
    }

    vehicle_controller = VehicleController(waypoints)
    rospy.on_shutdown(vehicle_controller.clean_shutdown)
    vehicle_controller.run()

    print("Done.")

if __name__ == '__main__':
    main()
