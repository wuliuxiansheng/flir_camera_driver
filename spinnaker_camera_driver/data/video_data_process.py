#!/usr/bin/env python

import os
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


def main():
    pass


if __name__ == '__main__':

    # fetch images from bag file
    bagfile = "./kitchen-PG3_2022-02-22-16-26-08.bag"
    bag = rosbag.Bag(bagfile)
    bridge = CvBridge()
    camera_name = bagfile[bagfile.index('P'):bagfile.index('_')]
    output_dir = bagfile[bagfile.index('P'):-4]
    os.mkdir(output_dir)
    stamp_file = open(output_dir+'.txt', 'w')

    frame_size = (1600, 1200)
    video_name = output_dir + '.avi'
    video = cv2.VideoWriter(filename=video_name, apiPreference=0,
                            fourcc=cv2.VideoWriter_fourcc(*'XVID'), fps=22,
                            frameSize=frame_size)
    count = 0
    for topic, msg, t in bag.read_messages(topics="/kitchen/"+camera_name+"/image_raw"):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(os.path.join(output_dir, "frame%i.png" % count), cv_img)
        video.write(np.asarray(cv_img))
        stamp_file.write(str(msg.header.stamp.secs) +
                         " " + str(msg.header.stamp.nsecs) + "\n")
        count += 1
    bag.close()
    cv2.destroyAllWindows()
    video.release()
    stamp_file.close()

    # generate video
    # video_name = output_dir + '.avi'
    # images = [img for img in os.listdir(output_dir)]
    # frame = cv2.imread(os.path.join(output_dir, images[0]))
    # height, width, layers = frame.shape
    # video = cv2.VideoWriter(filename=video_name, apiPreference=0,
    #                         fourcc=cv2.VideoWriter_fourcc(*'XVID'), fps=22,
    #                         frameSize=(width, height))
    # for image in images:
    #     video.write(cv2.imread(os.path.join(output_dir, image)))
    # cv2.destroyAllWindows()
    # video.release()
