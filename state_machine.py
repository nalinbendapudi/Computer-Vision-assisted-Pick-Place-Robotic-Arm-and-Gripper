import time
import numpy as np
import copy
from kinematics import *

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
        self.intrinsic = np.eye(3)
        self.cam2world = np.array([[1, 0, 0],[0, 1, 0]])
        self.z_reference = self.kinect.z_reference

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously icoordinatesn a thread"""

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
            
    def pixel2world(self, x, y):
        d = self.kinect.currentDepthFrame[y][x]
        z = 0.1236 * np.tan(d/2842.5 + 1.1863)*1000
        p_c = np.matmul(np.linalg.inv(self.intrinsic), z*np.array([[x],[y],[1]])).reshape(-1)
        p_w = np.matmul(self.cam2world,  np.array([[p_c[0]], [p_c[1]], [1]]))
        return p_w, z

    """Functions run for each state"""
    def execute(self):
        # np.save('rgb.npy', self.kinect.currentVideoFrame)
        # np.save('depth.npy', self.kinect.currentDepthFrame)
        self.status_message = "State: Execute - task 1.2"
        self.current_state = "execute"

        pick_put_3D = []
        location_strings = ["Pick Pose",
                    "Put Pose"]
        i = 0
        for j in range(2):
            self.status_message = location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    x, y = self.kinect.last_click
                    p_w, z = self.pixel2world(x, y)
                    pick_put_3D.append([p_w[0], p_w[1], self.z_reference-z])
                    i = i + 1
                    self.kinect.new_click = False 
        pick_put_3D = np.array(pick_put_3D)
        print(pick_put_3D)
        cfgs = []

        # pick motions

        pick_end_effector_pose = np.eye(4)
        pick_end_effector_pose[:3,:3] = np.array([[1., 0., 0.],[0., -1., 0.],[0., 0., -1.]])
        pick_end_effector_pose[0][3] = pick_put_3D[0][0]
        pick_end_effector_pose[1][3] = pick_put_3D[0][1]
        pick_end_effector_pose[2][3] = pick_put_3D[0][2]

        pick_prepare_pose = copy.deepcopy(pick_end_effector_pose)
        pick_prepare_pose[2][3] = 90
        
        pick_target_cfg = IK(pick_end_effector_pose)
        pick_target_cfg.append(0.0)
        pick_prepare_target_cfg = IK(pick_prepare_pose)
        pick_prepare_target_cfg.append(0.0)

        enable = 1

        # for i in range(len(pick_target_cfg)):
        #     if pick_target_cfg[i] < self.rexarm.angle_limits[i,0] or pick_target_cfg[i] > self.rexarm.angle_limits[i,1]:
        #         print 'configuration exceeds joint limits\n'
        #         enable = 0
        #         break

        pick_intermediate_cfg_go = copy.deepcopy(pick_prepare_target_cfg)
        pick_intermediate_cfg_go[1] = pick_intermediate_cfg_go[2] = pick_intermediate_cfg_go[4] = 0

        gripper_close = copy.deepcopy(pick_target_cfg)
        gripper_close[6] = 2.65

        pick_prepare_target_cfg_back = copy.deepcopy(pick_prepare_target_cfg)
        pick_prepare_target_cfg_back[6] = 2.65

        pick_intermediate_cfg_back = copy.deepcopy(pick_intermediate_cfg_go)
        pick_intermediate_cfg_back[6] = 2.65

        pick_move_back = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.65]

        # put motions
        put_end_effector_pose = np.eye(4)
        put_end_effector_pose[:3,:3] = np.array([[1., 0., 0.],[0., -1., 0.],[0., 0., -1.]])
        put_end_effector_pose[0][3] = pick_put_3D[1][0]
        put_end_effector_pose[1][3] = pick_put_3D[1][1]
        put_end_effector_pose[2][3] = pick_put_3D[1][2]

        put_prepare_pose = copy.deepcopy(put_end_effector_pose)
        put_prepare_pose[2][3] = 90
        
        put_target_cfg = IK(put_end_effector_pose)
        put_target_cfg.append(2.65)
        put_prepare_target_cfg = IK(put_prepare_pose)
        put_prepare_target_cfg.append(2.65)

        put_intermediate_cfg_go = copy.deepcopy(put_prepare_target_cfg)
        put_intermediate_cfg_go[1] = put_intermediate_cfg_go[2] = put_intermediate_cfg_go[4] = 0

        gripper_open = copy.deepcopy(put_target_cfg)
        gripper_open[6] = 0.0

        put_prepare_target_cfg_back = copy.deepcopy(put_prepare_target_cfg)
        put_prepare_target_cfg_back[6] = 0.0

        put_intermediate_cfg_back = copy.deepcopy(put_intermediate_cfg_go)
        put_intermediate_cfg_back[6] = 0.0

        put_move_back = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


        # poses = [[ 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.0],
        # [-0.8,-0.8,-0.8,-0.8, -0.8, 0.0, 2],
        # [-0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.0],
        # [0.8, -0.8,-0.8,-0.8, -0.8, 0.0, 2],
        # [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
        if enable:
            cfgs.append(pick_intermediate_cfg_go)
            cfgs.append(pick_prepare_target_cfg)
            cfgs.append(pick_target_cfg)
            cfgs.append(gripper_close)
            cfgs.append(pick_prepare_target_cfg_back)
            cfgs.append(pick_intermediate_cfg_back)
            cfgs.append(pick_move_back)
            cfgs.append(put_intermediate_cfg_go)
            cfgs.append(put_prepare_target_cfg)
            cfgs.append(put_target_cfg)
            cfgs.append(gripper_open)
            cfgs.append(put_prepare_target_cfg_back)
            cfgs.append(put_intermediate_cfg_back)
            cfgs.append(put_move_back)
            for cfg in cfgs:
                # print 'FK:\n',get_transformation(pose)
                plan_speeds, plan_angles = self.tp.generate_plan(cfg)
                self.tp.execute_plan(plan_speeds, plan_angles)
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
        self.intrinsic = self.kinect.loadCameraCalibration()
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

        """TODO Perform camera calibration here"""
        self.kinect.depth2rgb_affine = self.kinect.getAffineTransform(self.kinect.depth_click_points, self.kinect.rgb_click_points)
        print('depth2rgb_affine\n',self.kinect.depth2rgb_affine)
        world_points = np.array([[-307, -307, 1], [-307, 307, 1], [307, 307, 1], [307, -307, 1]])
        p_cam = []
        for pt in self.kinect.rgb_click_points:
            p_cam.append(self.z_reference*np.matmul(np.linalg.inv(self.intrinsic), np.array([[pt[0]],[pt[1]], [1]])).reshape(-1))
        p_cam = np.array(p_cam)
        print(p_cam)
        print(np.linalg.norm(p_cam[0]-p_cam[1]))
        print(np.linalg.norm(p_cam[1]-p_cam[2]))
        print(np.linalg.norm(p_cam[2]-p_cam[3]))
        print(np.linalg.norm(p_cam[0]-p_cam[3]))
        self.cam2world = self.kinect.getAffineTransform(p_cam, world_points)
        print('cam2world\n',self.cam2world)
        for ptc, ptw in zip(p_cam[:,:2], world_points[:,:2]):
            print(ptc, ptw)
            print('transformed',np.matmul(self.cam2world, np.array([[ptc[0]], [ptc[1]], [1]])))
            print('residence',np.linalg.norm(np.matmul(self.cam2world, np.array([[ptc[0]], [ptc[1]], [1]])).reshape(-1)- ptw))
        self.kinect.kinectCalibrated = True


        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)