import sys
import pyzed.sl as sl
import numpy as np
import cv2

import os
import glob


SVO_PATH = '12345.svo2'
OUTPUT_DIR = 'action_12345'

MAIN_DIR = '.'

def convert(svo_input_path = SVO_PATH, output_dir = OUTPUT_DIR ):
    # Specify SVO path parameter
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_input_path)
    init_params.svo_real_time_mode = False  # Don't convert in realtime
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)

    # Create ZED objects
    zed = sl.Camera()

    # Open the SVO file specified as a parameter
    err = zed.open(init_params)
    if err > sl.ERROR_CODE.SUCCESS:
        sys.stdout.write(repr(err))
        zed.close()
        exit()

    # Prepare single image containers
    left_image = sl.Mat()
    depth_image = sl.Mat()
    
    rt_param = sl.RuntimeParameters()

    # Start SVO conversion to AVI/SEQUENCE
    out_rgb = os.path.join(output_dir ,"/rgb/")
    if not os.path.exists(out_rgb):
        os.makedirs(out_rgb)

    out_depth = os.path.join(output_dir ,"/depth/")
    if not os.path.exists(out_depth):
        os.makedirs(out_depth)

    while True:
        err = zed.grab(rt_param)
        if err <= sl.ERROR_CODE.SUCCESS:
            svo_time = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)

            # Retrieve SVO images
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

            
            # Generate file names
            filename1 = out_rgb + ("%s.png" % str(svo_time))
            filename2 = out_depth + ("depth%s.png" % str(svo_time))
            # Save Left images
            cv2.imwrite(str(filename1), left_image.get_data())
            # Save depth images (convert to uint16)
            cv2.imwrite(str(filename2), depth_image.get_data().astype(np.uint16))

        if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
            break
    zed.close()
    return 0

all_file = glob.glob(OUTPUT_DIR + "/*.svo2")

for file in all_file:
    output_dir = MAIN_DIR +'/' + file.split('/')[-1].split('.')[0]
    convert(svo_input_path=file, output_dir=output_dir)