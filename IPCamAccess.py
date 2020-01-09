import urllib
import urllib.request
import cv2
import numpy as np
from scipy import ndimage
from matplotlib import pyplot as plt


def IpCamState():
    url = 'http://192.168.2.8:8080/photo.jpg'
    imgResp = urllib.request.urlopen(url)
    imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
    imageGBR = cv2.imdecode(imgNp, -1)
    imageRGB = cv2.cvtColor(imageGBR, cv2.COLOR_BGR2RGB)
    image = ndimage.rotate(imageRGB, 270)
    # plt.imshow(image, interpolation='nearest')
    plt.show()
    return image



