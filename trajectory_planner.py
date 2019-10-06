import numpy as np 
import time
from trajectoryGen import cubicInterpolation
import copy

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self):
        # sets the next initial_wp
        self.initial_wp = copy.deepcopy(self.final_wp)

    def set_final_wp(self, waypoint):
        # sets the next final_wp
        self.final_wp = waypoint

    def go(self, max_speed = 2.5):
        pass

    def stop(self):
        pass

    def calc_time_from_waypoints(self, max_speed = 0.5):
        # Returns min time given initial wp, final wp and max speed
        return max(np.absolute(np.array(self.final_wp) - np.array(self.initial_wp)))/max_speed

    def generate_cubic_spline(self, T, initial_wp, final_wp):
        numberOfInterpolations = int(T/self.dt - 1)
        return cubicInterpolation(initial_wp, final_wp, 0.0, 0.0, T, numberOfInterpolations)

    def generate_plan(self, waypoint, max_speed):
        self.set_initial_wp()
        self.set_final_wp(waypoint)
        total_time = self.calc_time_from_waypoints(max_speed)
        plan_speeds = []
        plan_angles = []

        for i in range(self.num_joints):
            speeds, angles = self.generate_cubic_spline(total_time, self.initial_wp[i], self.final_wp[i])
            
            # add section between last interpolation point and final way point
            angles.append(self.final_wp[i])

            plan_speeds.append(speeds)
            plan_angles.append(angles)

        plan_speeds = (np.array(plan_speeds)).T
        edge_speeds = np.array([0.0]*self.num_joints)
        plan_speeds = (np.vstack((edge_speeds,plan_speeds))+np.vstack((plan_speeds,edge_speeds)))/2.0

        plan_angles = (np.array(plan_angles)).T

        return plan_speeds, plan_angles
        
    def execute_plan(self, plan_speeds, plan_angles, look_ahead=12):
        for i in range(np.shape(plan_speeds)[0]):
            self.rexarm.set_speeds_normalized(plan_speeds[i,:])
            self.rexarm.set_positions(plan_angles[i,:])
            # PLAN_ANGLE = plan_angles[min(i+look_ahead,np.shape(plan_angles)[0]-1),:]
            # self.rexarm.set_positions(PLAN_ANGLE)
            # self.rexarm.pause(self.dt)
