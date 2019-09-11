import time
import numpy as np
import copy

"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "execute"):
                self.execute()
            if (self.next_state == "record_next"):
                self.start_record()
            if (self.next_state == "replay"):
                self.replay()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "execute"):
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()

        if (self.current_state == "recording"):
            if (self.next_state == "finish"):
                self.finish_record()
            if (self.next_state == "record_next"):
                self.record_next()
            if (self.next_state == "estop"):
                self.estop()
        
        if self.current_state == "replay":
            if (self.next_state == "estop"):
                self.estop()
            

    """Functions run for each state"""
    def execute(self):
        self.status_message = "State: Execute - task 1.2"
        self.current_state = "execute"
        poses = [[ 0.0, 0.0, 0.0, 0.0, 0.0],
        [ 1.0, 0.8, 1.0, 0.5, 1.0],
        [-1.0,-0.8,-1.0,-0.5, -1.0],
        [-1.0, 0.8, 1.0, 0.5, 1.0],
        [1.0, -0.8,-1.0,-0.5, -1.0],
        [ 0.0, 0.0, 0.0, 0.0, 0.0]]
        for pose in poses:
            self.rexarm.set_positions(pose)
            self.rexarm.pause(1)
        self.next_state = "idle"

    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
    
    def start_record(self):
        self.status_message = "Start recording"
        self.current_state = "recording"
        self.next_state = "recording"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        self.history = []
        self.history.append([ 0.0, 0.0, 0.0, 0.0, 0.0])

    def record_next(self):
        '''
        record current position, change state to recording
        '''
        self.history.append(copy.deepcopy(self.rexarm.get_positions()))
        self.next_state = "recording"
        print(self.history)

    def finish_record(self):
        '''
        resume torque, change state to idle
        '''
        self.history.append([ 0.0, 0.0, 0.0, 0.0, 0.0])
        self.rexarm.enable_torque()
        self.current_state = "idle"
        self.next_state = "idle"
        np.savetxt("history.csv", np.array(self.history), delimiter=",")

    def replay(self):
        '''
        replay history if it exists, change state to idle
        '''
        print("Warining: replaying")
        if self.history is None:
            print('Error: Please start recording first')
            self.next_state = "idle"
            return
        self.current_state = "replay"
        for pose in self.history:
            print(pose)
            self.rexarm.set_positions(pose)
            self.rexarm.pause(1)
        print("finish replaying")
        self.next_state = "idle"
        self.idle()
        


    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board"]
        i = 0
        for j in range(4):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(4):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""
        world_points = [[0, 0, 1], [0, 85.9, 1], [85.9, 85.9, 1], [85.9, 0, 1]]
        A = []
        for world, cam in zip(world_points, self.kinect.rgb_click_points):
            A.append([0,0,0,cam[0],cam[1],cam[2],1,-world[1]*cam[0],-world[1]*cam[1],-world[1]*cam[2]])
            A.append([cam[0],cam[1],cam[2],0,0,0,1,-world[0]*cam[0],-world[0]*cam[1],-world[0]*cam[2]])
        A = np.array(A)
        # A.T@A


        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)