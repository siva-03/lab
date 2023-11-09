import pyrealsense2 as rs
import rospy
import rosbag
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import numpy as np

def main():
    # Initialize realsense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs_stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200, 0)

    # start the pipeline
    pipeline.start(config)

    #Initialize ROS
    rospy.init_node('realsense_capture')
    image_pub = rospy.publisher('/camera/color/image_raw', Image, queue_size = 10)
    depth_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size = 10)
    gyro_pub = rospy.publisher('/camrea/gyro/data', Imu, queue_size = 10)
    bag = rosbag.Bag('realsense_data.bag', 'w')

    bridge = CvBridge()

    try:
        while not rospy.is_shutdown():
            # wait for frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            gyro_frame = frames.first_or_default(rs.stream.gyro)

            if color_frame and depth_frame and gyro_frame:
                
                # Convert realsense data to ros msgs
                color_image = bridge.cv2_to_imgmsg(np.array(color_frame.get_data()), 'bgr8')
                depth_image = bridge.cv2_to_imgmsg(np.array(depth_frame.get_data()), 'passthrough')
                gyro_msg = Imu()
                gyro_msg.header.stamp = rospy.Time.now()
                gyro_msg.angular_velocity.x = gyro_frame.as_motion_frame().get_motion_data().x
                gyro_msg.angular_velocity.y = gyro_frame.as_motion_frame().get_motion_data().y
                gyro_msg.angular_velocity.z = gyro_frame.as_motion_frame().get_motion_data().z

                # publish and save data to the bag file
                image_pub.publish(color_image)
                depth_pub.publish(depth_image)
                gyro_pub.publish(gyro_msg)
                bag.write('/camera/color/image_raw', color_image, rospy.Time.now())
                bag.write('/camera/depth/image_raw', depth_image, rospy.Time.now())
                bag.write('/camera/gyro/data', gyro_msg, rospy.Time.now())

    finally:
        pipeline.stop()
        bag.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
