#####################################################
##               Read bag from file                ##
#####################################################


# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path

# Create object for parsing command-line options
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
# Add argument which takes path to a bag file as an input
parser.add_argument("-i", "--input", type=str, help="path to bag file")
# Parse the command line arguments to an object
args = parser.parse_args()
# Safety if no parameter have been given
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
# Check if the given file have bag extension
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    # Create pipeline
    pipeline = rs.pipeline()
    
    print("All good")
    # Create a config object
    config = rs.config()

    # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
    rs.config.enable_device_from_file(config, args.input)

    # Configure the pipeline to stream the depth stream
    # Change this parameters according to the recorded bag file resolution
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

    # Start streaming from file
    pipeline.start(config)

    # Create opencv window to render image in
    cv2.namedWindow("Stream", cv2.WINDOW_AUTOSIZE)
    
    # Create colorizer object
    colorizer = rs.colorizer()

    # Streaming loop
    while True:
        # Get frameset of depth
        frames = pipeline.wait_for_frames()

        # Get depthall 3 frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        # Colorize depth frame to jet colormap
        depth_color_frame = colorizer.colorize(depth_frame)

        # Convert depth_frame to numpy array to render image in opencv
        depth_color_image = np.asanyarray(depth_color_frame.get_data())
        
        # Convert color_frame to numpy array to render image in opencv
        color_image = np.asanyarray(color_frame.get_data())
        
        # extract gyro data
        gyro_values = gyro_frame.as_motion_frame().get_motion_data()
        
        # write gyro values on the color img
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(color_image, 'Gyro: {:.2f},{:.2f},{:.2f}'.format(gyro_values.x, gyro_values.y, gyro_values.z), (10, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        # print('Gyro: {:.2f},{:.2f},{:.2f}'.format(gyro_values.x, gyro_values.y, gyro_values.z))	
        
        # Render image in opencv window
        cv2.imshow("Depth Stream", depth_color_image)
        # cv2.imshow("Color Stream", color_image)
        key = cv2.waitKey(1)
        # if pressed escape exit program
        if key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass
