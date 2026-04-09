import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

PATH = './screen.png'

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

for i in range(10):
    frames = pipeline.wait_for_frames()
    color_frame =frames.get_color_frame()


frames = pipeline.wait_for_frames()
color_frame =frames.get_color_frame()

img = np.asanyarray(color_frame.get_data())
cv2.imwrite(PATH, img)