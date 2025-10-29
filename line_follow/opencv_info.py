import cv2
print(cv2.getBuildInformation())
import sys
print(cv2.__file__)
print(cv2.__version__)
# import os
# os.environ["OPENNI2_DRIVERS_PATH"] = "/home/dell/OpenNI2/Bin/x64-Release/OpenNI2/Drivers"
# # Si besoin (rare) pour trouver des .so à côté de NiViewer:
# os.environ["LD_LIBRARY_PATH"] = "/home/dell/OpenNI2/Bin/x64-Release:" + os.environ.get("LD_LIBRARY_PATH","")

import cv2
cap = cv2.VideoCapture(cv2.CAP_OPENNI2)
print("cap.isOpened() =", cap.isOpened())
