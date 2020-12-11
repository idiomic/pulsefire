#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
import std_msgs.msg
import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2


# def write_ply(fn, verts):
#     verts = verts.reshape(-1, 3)
#     verts = np.hstack([verts])
#     with open(fn, 'wb') as f:
#         np.savetxt(f, verts, fmt='%f %f %f')


def image_callback(image_l, image_r):
    cv_image1 = bridge.imgmsg_to_cv2(image_l, desired_encoding='rgb8')
    cv_image2 = bridge.imgmsg_to_cv2(image_r, desired_encoding='rgb8')

    imgL = cv.pyrDown(cv_image1)
    imgR = cv.pyrDown(cv_image2)

    window_size = 3
    min_disp = 16
    num_disp = 112 - min_disp
    stereo = cv.StereoSGBM_create(minDisparity=min_disp,
                                  numDisparities=num_disp,
                                  blockSize=16,
                                  P1=8 * 3 * window_size ** 2,
                                  P2=32 * 3 * window_size ** 2,
                                  disp12MaxDiff=1,
                                  uniquenessRatio=10,
                                  speckleWindowSize=100,
                                  speckleRange=32
                                  )

    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    h, w = imgL.shape[:2]
    f = 0.8 * w
    Q = np.float32([[1, 0, 0, -0.5 * w],
                    [0, -1, 0, 0.5 * h],
                    [0, 0, 0, -f],
                    [0, 0, 1, 0]])
    points = cv.reprojectImageTo3D(disp, Q)

    mask = disp > disp.min()
    out_points = points[mask]
    # out_fn = 'out.ply'
    # write_ply(out_fn, out_points)
    #
    # cv.waitKey()

    header = std_msgs.msg.Header()
    header.stamp = rospy.get_rostime()
    header.frame_id = 'map'
    msg.header = header
    
    #msg.points= Point32(out_points.tolist())
    for i in range(len(out_points)):
        x=out_points[i][0]
        y=out_points[i][1]
        z=out_points[i][2]
        msg.points.append(Point32(x,y,z))
        #msg.points.append({'x':out_points[i][0], 'y':out_points[i][1], 'z':out_points[i][2]})
    #msg.points = out_point
    
    pub.publish(msg)


def main():
    rospy.init_node('pc2', anonymous=True)

    global bridge, msg, pub
    bridge = CvBridge()
    msg = PointCloud()

    pub = rospy.Publisher('/points_in', PointCloud, queue_size=10)
    rate = rospy.Rate(10)

    left = message_filters.Subscriber('/camera/left/rgb/image_raw', Image)
    right = message_filters.Subscriber('/camera/right/rgb/image_raw', Image)
    ts = message_filters.TimeSynchronizer([left, right], 10)
    ts.registerCallback(image_callback)

    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
