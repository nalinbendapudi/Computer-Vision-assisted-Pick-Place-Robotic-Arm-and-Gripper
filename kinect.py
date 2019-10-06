import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import os
import blob_detector as bd
import copy
class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True
        
        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((4,2),int)
        self.depth_click_points = np.zeros((4,2),int)
        self.sm = None

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])
        self.z_reference = 940
        self.block_poses = []

    def get_sm(self,sm):
        self.sm = sm

    def captureVideoFrame(self):
        """                      
        Capture frame from Kinect, format is 24bit RGB    
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()
        

    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


    def captureDepthFrame(self):
        """                      
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
            if self.kinectCalibrated:
                self.detectBlocksInDepthImage()
        else:
            self.loadDepthFrame()

    
    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt  
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """ 
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates, 
        find the affine matrix transform between them.

        W = M*C
        coord1 in camera frame, coord2 in world frame
        return affine transformation from camera to world frame

        TODO: Rewrite this function to take in an arbitrary number of coordinates and 
        find the transform without using cv2 functions
        """
        C = []
        W = []
        for pt1, pt2 in zip(coord1, coord2):
            C.append([pt1[0],pt1[1], 1, 0, 0,0])
            C.append([0,0,0,pt1[0],pt1[1],1])
            W.append([pt2[0]])
            W.append([pt2[1]])
        C = np.array(C)
        W = np.array(W)
        M = np.matmul(np.matmul(np.linalg.inv(np.matmul(C.T,C)),C.T),W)
        M = M.reshape((2,3))
        return M


    def registerDepthFrame(self, frame):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        transformed = cv2.warpAffine(frame, M = self.depth2rgb_affine, dsize = (frame.shape[1],frame.shape[0]))
        return transformed

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
        if not os.path.exists('util/calibration.csv'):
            print('No calibration file found')
            raise NotImplementedError
        return np.loadtxt('util/calibration.csv', delimiter=',')
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        pass

    def check_valid_block(self, p_w, z):
        if p_w[0] < -307 or p_w[0] > 307 or p_w[1] < -307 or p_w[1] > 307:
            return False
        return True

    def detectBlocksInDepthImage(self, use_opencv = False):
        # False: blob detection
        # True: block detection
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        depth = cv2.GaussianBlur(self.currentDepthFrame, (5, 5), 0)
        height = self.z_reference - (0.1236 * np.tan(depth/2842.5 + 1.1863)*1000)
        # Filter out outliers
        height[height < 10] = 0
        height[height > 200] = 0
        # make blocks equal height
        # height[height > 20] = 40
        # Erosion
        kernel = np.ones((5,5),np.uint8)
        height = cv2.erode(height,kernel,iterations = 1)
        # self.currentDepthFrame = height
        '''
        OpenCV contour methods
        '''
        valid_cnts = []
        block_poses = []
        if use_opencv:
            _, contours, _ = cv2.findContours(height.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            font = cv2.FONT_HERSHEY_COMPLEX
            valid_cnts = []
            for cnt in contours:
                area = cv2.contourArea(cnt) 
                if area < 30:
                    continue
                approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt, True), True) 
                valid_cnts.append(approx)
        else:
            _, coordinates, nsig, names = bd.detectBlob(self.currentVideoFrame, height)
            for c, name in zip(coordinates, names):
                try:
                    p_w, z = self.sm.pixel2world(c[1], c[0])
                    if not self.check_valid_block(p_w, z):
                        continue
                except:
                    continue
                cv2.circle(self.currentVideoFrame, tuple(reversed(c)), int(nsig*np.sqrt(2)), color=bd.colors_rgb[name], thickness = 5)
                block_poses.append(np.array([p_w[0][0], p_w[1][0], self.sm.z_reference - z, 0, 0, 0]).reshape(-1))
        block = {}
        block['poses'] = block_poses
        block['colors'] = names
        self.blocks = block.copy()
        # self.block_contours = valid_cnts

    def get_block_poses(self):
        return copy.deepcopy(self.blocks['poses'])

    def get_block_pose_color(self):
        return copy.deepcopy(self.blocks)
