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
            if (self.next_state == "task1"):
                self.task1()
                
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

        if self.current_state == "task1":
            if (self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()
        
        if self.current_state == "task2":
            if (self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()

        if self.current_state == "task3":
            if (self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()

        if self.current_state == "task4":
            if (self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()

        if self.current_state == "task5":
            if (self.next_state == "estop"):
                self.estop()
            if(self.next_state == "idle"):
                self.idle()
            
    def pixel2world(self, x, y):
        d = self.kinect.currentDepthFrame[y][x]
        z = 0.1236 * np.tan(d/2842.5 + 1.1863)*1000
        p_c = np.matmul(np.linalg.inv(self.intrinsic), z*np.array([[x],[y],[1]])).reshape(-1)
        p_w = np.matmul(self.cam2world,  np.array([[p_c[0]], [p_c[1]], [1]]))
        return p_w, z

    def end_effector_orientation(self, x, y, grip_angle):
        if x > 0.0 and x >= np.absolute(y):
            rotation_angle = np.pi/2
            # R_z = get_T_from_angle(2, np.pi/2)
            # R_x = get_T_from_angle(0, grip_angle)
            # orientation = np.matmul(R_z, R_x)
        elif y > 0.0 and y >= np.absolute(x):
            rotation_angle = np.pi
            # R_z = get_T_from_angle(2, np.pi)
            # R_x = get_T_from_angle(0, grip_angle)
            # orientation = np.matmul(R_z, R_x)
        elif x < 0.0 and -x >= np.absolute(y):
            rotation_angle = -np.pi/2
            # R_z = get_T_from_angle(2, -np.pi/2)
            # R_x = get_T_from_angle(0, grip_angle)
            # orientation = np.matmul(R_z, R_x)
        else:
            rotation_angle = 0.0
        
        R_z = get_T_from_angle(2, rotation_angle)
        R_x = get_T_from_angle(0, grip_angle)
        orientation = np.matmul(R_z, R_x)

        return orientation

    """Functions run for each state"""
    def execute(self, pick6d = np.array([200,0,50,0,0,0]), put6d=np.array([200,0,50,0,0,0]), directCall = True):

        # np.save('rgb.npy', self.kinect.currentVideoFrame)
        # np.save('depth.npy', self.kinect.currentDepthFrame)
        self.status_message = "State: Execute - task 1.2"
        self.current_state = "execute"
        print("In Execute")
        '''
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
        pick_put_3D = np.array([[226.72200641,   2.38742778,  35.2483757 ],[-226.72200641,   2.38742778,  35.2483757 ]])
        '''
        pick_put_3D = np.array([[pick6d[0],  pick6d[1],  pick6d[2] ],[put6d[0],  put6d[1],  put6d[2]]])
        print "assigned pick position:\n", pick_put_3D[0]
        print "assigned put position:\n", pick_put_3D[1]


        # try some end effector orientation in sequence, execute if a viable trajectory is found
        angles = [np.pi, 11*np.pi/12, 5*np.pi/6, 3*np.pi/4, 2*np.pi/3, 7*np.pi/12, np.pi/2]
        success = 0
        cfgs = []
        for angle in angles:

            if success == 1:
                print "find viable solution\n"
                break
            
            print "current testing angle: ", angle 

            ###################################  pick motion  ###################################
            
            # pick orientation
            pick_orientation = self.end_effector_orientation(pick_put_3D[0][0], pick_put_3D[0][1], angle)

            # pick pose  
            pick_end_effector_pose = np.eye(4)
            pick_end_effector_pose[:3,:3] = copy.deepcopy(pick_orientation)
            pick_end_effector_pose[:3,3:4] = np.array([[pick_put_3D[0][0]],[pick_put_3D[0][1]],[pick_put_3D[0][2]]])

            if pick_end_effector_pose[2,3] < 35.0:
                pick_end_effector_pose[2,3] = 40.0
            else:
                pick_end_effector_pose[2,3] = pick_end_effector_pose[2,3] + 20.0

            # pick preparation pose
            pick_prepare_pose = copy.deepcopy(pick_end_effector_pose)
            pick_prepare_pose[2][3] = pick_end_effector_pose[2,3] + 50.0
            
            # compute configuration for pick pose
            pick_target_cfg = IK(pick_end_effector_pose, self.rexarm.angle_limits)

            if pick_target_cfg == None:
                continue

            pick_target_cfg.append(0.0)

            # compute configuration for pick preparation pose
            pick_prepare_target_cfg = IK(pick_prepare_pose, self.rexarm.angle_limits)

            if pick_prepare_target_cfg == None:
                continue
            
            # make sure the configuration difference between pick pose and pick preparation pose is not large
            difference = np.absolute(pick_prepare_target_cfg[0] - pick_target_cfg[0])

            if difference > np.pi/2:
                continue

            pick_prepare_target_cfg.append(0.0)

            # close gripper 
            gripper_close = copy.deepcopy(pick_target_cfg)
            gripper_close[6] = 2.65

            # preparation pose before moving back
            pick_prepare_target_cfg_back = copy.deepcopy(pick_prepare_target_cfg)
            pick_prepare_target_cfg_back[6] = 2.65

            # move back to origin
            pick_move_back = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.65]


            ###################################  put motion  ###################################

            # put orientation
            put_orientation = self.end_effector_orientation(pick_put_3D[1][0], pick_put_3D[1][1], angle)

            # put pose  
            put_end_effector_pose = np.eye(4)
            put_end_effector_pose[:3,:3] = copy.deepcopy(put_orientation)
            put_end_effector_pose[:3,3:4] = np.array([[pick_put_3D[1][0]],[pick_put_3D[1][1]],[pick_put_3D[1][2]]])

            if put_end_effector_pose[2,3] < 40.0:
                put_end_effector_pose[2,3] = 45.0
            else:
                put_end_effector_pose[2,3] = put_end_effector_pose[2,3] + 50.0

            print "put height: ", put_end_effector_pose[2,3], "\n"
            
            # put preparation pose
            put_prepare_pose = copy.deepcopy(put_end_effector_pose)
            put_prepare_pose[2][3] = put_end_effector_pose[2,3] + 50.0
            
            # compute configuration for put pose
            put_target_cfg = IK(put_end_effector_pose, self.rexarm.angle_limits)

            if put_target_cfg == None:
                continue

            put_target_cfg.append(2.65)

            # compute configuration for put preparation pose
            put_prepare_target_cfg = IK(put_prepare_pose, self.rexarm.angle_limits)

            if put_prepare_target_cfg == None:
                continue
            
            # make sure the configuration difference between put pose and put preparation pose is not large
            difference = np.absolute(put_prepare_target_cfg[0] - put_target_cfg[0])

            if difference > np.pi/2:
                continue

            put_prepare_target_cfg.append(2.65)

            # open gripper 
            gripper_open = copy.deepcopy(put_target_cfg)
            gripper_open[6] = 0.0

            # preparation pose before moving back
            put_prepare_target_cfg_back = copy.deepcopy(put_prepare_target_cfg)
            put_prepare_target_cfg_back[6] = 0.0

            # move back to origin
            put_move_back = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            success = 1
        
        if success:
            cfgs.append(pick_prepare_target_cfg)
            cfgs.append(pick_target_cfg)
            cfgs.append(gripper_close)
            cfgs.append(pick_prepare_target_cfg_back)
            cfgs.append(pick_move_back)
            cfgs.append(put_prepare_target_cfg)
            cfgs.append(put_target_cfg)
            cfgs.append(gripper_open)
            cfgs.append(put_prepare_target_cfg_back)
            cfgs.append(put_move_back)
            for i in range(len(cfgs)):
                plan_speeds, plan_angles = self.tp.generate_plan(cfgs[i])
                self.tp.execute_plan(plan_speeds, plan_angles)
                self.rexarm.pause(1)
        else:
            print "can not execute\n"
            raise NotImplementedError
        if directCall:  
            self.next_state = "idle"

    def pick_put(self, pick6d, put6d):
        self.execute(pick6d=pick6d, put6d=put6d, directCall=False)
        

    def inPickLocation(self,block):
        # TODO: check whether in the right part of the board
        return block[0] >= 0

    def task1(self):
        self.current_state = "task1"
        print("Executing task1")
        block_poses = copy.deepcopy(self.kinect.get_block_poses())
        print(block_poses)
        right_flag = True
        
        put_location = np.array([-200,100,0,0,0,0])
        while right_flag:
            right_flag = False
            for block in block_poses:
                if not self.inPickLocation(block):
                    put_location = block
                else: 
                    right_flag = True

            for block in block_poses:
                if self.inPickLocation(block):
                    try:
                        self.pick_put(block.copy(), put_location.copy())
                        block_poses = copy.deepcopy(self.kinect.get_block_poses())
                        break
                    except:
                        self.next_state = "idle"
                        return
    
        self.next_state = "idle"
        pass

    def task2(self):
        self.current_state = "task2"
        print("Executing task1")
        self.next_state = "idle"
        pass

    def task3(self):
        self.current_state = "task3"
        print("Executing task1")
        self.next_state = "idle"
        pass

    def task4(self):
        self.current_state = "task4"
        print("Executing task1")
        self.next_state = "idle"
        pass

    def task5(self):
        self.current_state = "task5"
        print("Executing task1")
        self.next_state = "idle"
        pass

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