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
    bagfile = "./kitchen-PG2_2022-05-20-18-49-27.bag"
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
        cv2.imwrite(os.path.join(output_dir, "frame%06i.png" % count), cv_img)
        video.write(np.asarray(cv_img))
        time_stamp_sec = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        stamp_file.write(str(msg.header.stamp.secs) +
                         " " + str(msg.header.stamp.nsecs) + "\n")
        # stamp_file.write(str(time_stamp_sec) + "\n")
        count += 1
    bag.close()
    cv2.destroyAllWindows()
    video.release()
    stamp_file.close()

    # generate video
    video_name = output_dir + '.avi'
    filenames = os.listdir(output_dir)
    sorted_filenames = sorted(filenames)
    # This sorting is used when no zeros in file names
    # sorted_filenames = sorted(
    #     filenames, key=lambda x: int(x.split('e')[1].split('.')[0]))
    images = [img for img in sorted_filenames]
    frame = cv2.imread(os.path.join(output_dir, images[0]))
    height, width, layers = frame.shape
    video = cv2.VideoWriter(filename=video_name, apiPreference=0,
                            fourcc=cv2.VideoWriter_fourcc(*'XVID'), fps=22,
                            frameSize=(width, height))
    for image in images:
        video.write(cv2.imread(os.path.join(output_dir, image)))
    cv2.destroyAllWindows()
    video.release()
