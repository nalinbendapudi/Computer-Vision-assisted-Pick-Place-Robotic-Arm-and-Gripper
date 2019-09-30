# Author Linyi Jin
# Blob detector.
import cv2
import numpy as np
import matplotlib
# matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from scipy import ndimage, misc
import scipy.ndimage.filters as fi
from skimage.feature.peak import peak_local_max

def gkern2(kernlen=21, nsig=3):
    """Returns a 2D Gaussian kernel array."""
    # create nxn zeros
    inp = np.zeros((kernlen, kernlen))
    # set element at the middle to one, a dirac delta
    inp[kernlen//2, kernlen//2] = 1
    # gaussian-smooth the dirac, resulting in a gaussian filter mask
    return fi.gaussian_filter(inp, nsig)
    

def LoGkern2(kernlen=6*42+1, nsig=42):
    G = gkern2(kernlen, nsig)
    return cv2.Laplacian(G, ddepth=cv2.CV_64F, scale=nsig*nsig)



def detect(rgb, depth):
    nsig = 8
    kernlen = 6*nsig+1
    LoG = LoGkern2(kernlen, nsig)
    dest = cv2.filter2D(depth, ddepth=cv2.CV_64F,kernel = LoG)
    coordinates = peak_local_max(-dest, min_distance=10,threshold_abs=10, num_peaks=10)
    for c in coordinates:
        cv2.circle(rgb, tuple(reversed(c)), int(nsig*np.sqrt(2)), color=(0,255,0), thickness = 10)
    plt.subplot(221),plt.imshow(depth),plt.title('Input')
    plt.xticks([]), plt.yticks([])
    plt.subplot(222),plt.imshow(LoG),plt.title('LoG filter')
    plt.xticks([]), plt.yticks([])
    plt.subplot(223),plt.imshow(dest),plt.title('Response')
    plt.xticks([]), plt.yticks([])
    plt.subplot(224),plt.imshow(rgb),plt.title('Detection')
    plt.xticks([]), plt.yticks([])
    plt.show()
    exit()
    
def plot_result(rgb, depth):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,20))
    ax1.imshow(rgb)
    ax1.set_title('RGB')
    im2 = ax2.imshow(depth)
    f.colorbar(im2)
    ax2.set_title('depth')
    # plt.colorbar(plt2,cax=ax2)
    plt.show()

if __name__=="__main__":
    rgb = np.load('rgb.npy')
    depth = np.load('depth.npy')
    cv2.imwrite('rgb.png', rgb)
    cv2.imwrite('depth.png', depth)
    depth = cv2.GaussianBlur(depth, (5, 5), 0)
    base = 940
    height = base - (0.1236 * np.tan(depth/2842.5 + 1.1863)*1000)

    height[height < 0] = 0
    height[height > 200] = 0

    opencv = False

    '''
    OpenCV contour methods
    '''
    if opencv:
        _, contours, _ = cv2.findContours(height.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        font = cv2.FONT_HERSHEY_COMPLEX
        for cnt in contours:
            area = cv2.contourArea(cnt) 
            if area < 30:
                continue
            approx = cv2.approxPolyDP(cnt, 0.1*cv2.arcLength(cnt, True), True) 
            cv2.drawContours(rgb, [approx], 0, (0), 5)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            cv2.putText(rgb, "area_{}".format(area), (x, y), font, 1, (0))
        plot_result(rgb, height)

    else:
        detect(rgb, height)
