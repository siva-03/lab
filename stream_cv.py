import pyrealsense2 as rs
import numpy as np
import cv2

# configure relasense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200, 0)

# start realsense pipeline
pipeline.start(config)

#Define video writer to save data
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (640, 480))

try:
    while True:
        # wait for frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if not color_frame or not depth_frame or not gyro_frame:
            continue

        # extranct gyro data
        gyro_values = gyro_frame.as_motion_frame().get_motion_data()

        # convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # write gyro values on the color img
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(color_image, 'Gyro: {:.2f},{:.2f},{:.2f}'.format(gyro_values.x, gyro_values.y, gyro_values.z), (10, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # WRITE COLOR IMG TO VIDEO
        # out.write(color_image)

        # display color img
        cv2.imshow('RGB-D with Gyro', color_image)

        # break the loop when q is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # release resources
    pipeline.stop()
    out.release()
    cv2.destroyAllWindows()
