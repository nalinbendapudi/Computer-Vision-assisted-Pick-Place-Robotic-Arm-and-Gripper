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
from copy import deepcopy

# color_keys = ['Black', 'Red', 'Orange', 'Yellow', 'Green', 'Blue',  'Violet', 'Pink']
# gt_color = [(0,0,0), (255,0,0), (255,69,0), (255,255,0), (0,255,0), (0,0,255), (148,0,211), (255,192,203)]
colors_rgb = {'red': (255,0,0),
          'green': (0,255,0),
          'blue': (0,0,255),
          'yellow': (255,255,0),
          'orange': (255,127,0),
          'black': (0,0,0),
          'pink': (255,127,127),
          'purple': (127,0,255),
          'white': (255,255,255),}
colors_hsv = {'red': (123,230,173),
          'green': (57,58,195),
          'blue': (9,168,161),
          'yellow': (91,195,254),
          'orange': (111,212,248),
          'black': (163,33,107),
          'pink': (131,136,254),
          'purple': (160,94,176),
          'white': (135,2,252),}

colors_hsv = {'red':[[110, 146,118], [134, 210, 200]],
              'green':[[ 33,  37, 100],[ 66,  105, 218]],
              'blue':[[ -5, 111, 130], [ 14, 158, 240]],
              'yellow': [[-1, -1, 214], [100, 200, 294]],
              'orange': [[108, 87, 206], [122, 201, 294]],
              'black': [[129,  0,  41], [160,  61, 166]],
              'pink': [[122, 79, 202], [142, 188, 294]],
              'purple': [[146,  70,  93], [170, 123, 228]]}


def hsv2name(c):
    h, s, v = c
    for k, ran in colors_hsv.items():
        if h > ran[0][0] and h < ran[1][0]  and s > ran[0][1] and s < ran[1][1] and v > ran[0][2] and v < ran[1][2]:
            return k
    return 'white'

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


def get_box(centers, len):
    rtn = []
    adds = [[len/2, len/2], [len/2, -len/2], [-len/2, -len/2], [-len/2, len/2]]
    for center in centers:
        tmp = []
        for add in adds:
            tmp.append(center + add)
        rtn.append(np.array(tmp))
    return np.array(rtn)


def detectBlob(rgb, depth):
    nsig = 8
    kernlen = 6*nsig+1
    LoG = LoGkern2(kernlen, nsig)
    dest = cv2.filter2D(depth, ddepth=cv2.CV_64F,kernel = LoG)
    coordinates = peak_local_max(-dest, min_distance=10,threshold_abs=0.5, num_peaks=50)
    cnts = get_box(coordinates, 30)
    names = []
    for c in coordinates:
        name, _ = get_color(cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV), c)
        names.append(name)
    return cnts, coordinates, nsig, names


def detect(rgb, depth):
    nsig = 8
    kernlen = 6*nsig+1
    LoG = LoGkern2(kernlen, nsig)
    dest = cv2.filter2D(depth, ddepth=cv2.CV_64F,kernel = LoG)
    coordinates = peak_local_max(-dest, min_distance=10,threshold_abs=0.5, num_peaks=50)
    cnts = get_box(coordinates, 30)
    blobs = deepcopy(rgb)
    for c in coordinates:
        # cv2.circle(blobs, tuple(reversed(c)), int(nsig*np.sqrt(2)), color=(153,110,175), thickness = 10)
        name, _ = get_color(cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV), c)
        print(name, _)
        cv2.circle(blobs, tuple(reversed(c)), int(nsig*np.sqrt(2)), color=colors_rgb[name], thickness = 4)

    plt.subplot(221),plt.imshow(depth),plt.title('Input')
    plt.xticks([]), plt.yticks([])
    plt.subplot(222),plt.imshow(cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)),plt.title('LoG filter')
    plt.xticks([]), plt.yticks([])
    plt.subplot(223),plt.imshow(dest),plt.title('Response')
    plt.xticks([]), plt.yticks([])
    plt.subplot(224),plt.imshow(blobs),plt.title('Detection')
    plt.xticks([]), plt.yticks([])
    plt.show()

    
def plot_result(rgb, depth):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20,20))
    ax1.imshow(rgb)
    ax1.set_title('RGB')
    im2 = ax2.imshow(depth)
    f.colorbar(im2)
    ax2.set_title('depth')
    plt.show()

def get_color(hsv, coordinates):
    # import pdb;pdb.set_trace()
    color = hsv[coordinates[0]][coordinates[1]]
    # color = np.array(
    #     (np.asscalar(np.int16(color[0])), np.asscalar(np.int16(color[1])), np.asscalar(np.int16(color[2]))))  # HERE
    color = tuple([int(x) for x in color])
    name = hsv2name(color)
    return name, color

if __name__=="__main__":
    rgb = np.load('rgb.npy')
    depth = np.load('depth.npy')
    cv2.imwrite('rgb.png', rgb)
    cv2.imwrite('depth.png', depth)
    depth = cv2.GaussianBlur(depth, (5, 5), 0)
    base = 940
    height = base - (0.1236 * np.tan(depth/2842.5 + 1.1863)*1000)
    # Filter out outliers
    height[height < 10] = 0
    height[height > 200] = 0
    # make blocks equal height
    # height[height > 20] = 40
    # Erosion
    kernel = np.ones((5,5),np.uint8)
    height = cv2.erode(height,kernel,iterations = 1)

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
            cv2.drawContours(rgb, [approx], 0, (0, 255, 0), 2) 
            cv2.drawContours(height, [approx], 0, (100), 2)
            # x = approx.ravel()[0]
            # y = approx.ravel()[1]
            # cv2.putText(rgb, "area_{}".format(area), (x, y), font, 1, (0))
        plot_result(rgb, height)

    else:
        detect(rgb, height)
