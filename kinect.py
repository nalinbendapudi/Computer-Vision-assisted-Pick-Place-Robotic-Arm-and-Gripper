import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect

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

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])

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
        pass

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
        pass
    
    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.  
        You will need to locate
        blocks in 3D space
        """
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """
        pass