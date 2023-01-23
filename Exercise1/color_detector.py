#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep
import os


def gst_pipeline_string():
    # Parameters from the camera_node
    # Refer here : https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/camera_driver/config/jetson_nano_camera_node/duckiebot.yaml
    res_w, res_h, fps = 640, 480, 30
    fov = 'full'
    # find best mode
    camera_mode = 3  # 
    # compile gst pipeline
    gst_pipeline = """ \
            nvarguscamerasrc \
            sensor-mode= exposuretimerange="100000 80000000" ! \
            video/x-raw(memory:NVMM), width=, height=, format=NV12, 
                framerate=/1 ! \
            nvjpegenc ! \
            appsink \
        """.format(
        camera_mode,
        res_w,
        res_h,
        fps
    )

    # ---
    print("Using GST pipeline: ``".format(gst_pipeline))
    return gst_pipeline


cap = cv2.VideoCapture()
cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Put here your code!
    # You can now treat output as a normal numpy array
    # Do your magic here
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    n_splits = os.environ['N_SPLITS']
    f_shape = frame.shape
    f_rows = f_shape[0]
    f_cols = f_shape[1]
    
    rows = f_rows // n_splits
    last_row = f_rows % n_splits
    
    for i in range(n_splits):
    	r = 0
    	g = 0
    	b = 0
    	row_num = rows
    	if i == n_splits - 1 and last_row > 0:
    	    row_num = last_row
    	for j in range(row_num):
    	    for k in range(f_cols):
    	        r_val = frame[j + (i*rows)][k][0]
    	        g_val = frame[j + (i*rows)][k][1]
    	        b_val = frame[j + (i*rows)][k][2]
    	        r += r_val
    	        g += g_val
    	        b += b_val
    	    
    	
    	if r >= g and r >= b:
    	    print("R", end = " ")
    	elif g >= r and g >= b:
    	    print("G", end = " ")
    	elif b >= r and b >= g:
    	    print("B", end = " ")
    	

    sleep(1)

cap.release()
cv2.destroyAllWindows()
