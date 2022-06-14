#!/usr/bin/env python

import os
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


import time
from datetime import datetime
from dateutil import tz


def get_time_s_from_local_str(time_local_str, input_time_format='%H:%M:%S.%f',
                              date_local_str=None, input_date_format='%Y-%m-%d'):
    # Get the current local date if no date was provided.
    if date_local_str is None:
        now_local_datetime = datetime.now()
        date_local_str = now_local_datetime.strftime(input_date_format)

    # Combine the date and time.
    local_str = '%s %s' % (date_local_str, time_local_str)
    local_datetime = datetime.strptime(
        local_str, input_date_format + ' ' + input_time_format)

    return local_datetime.timestamp()


def data_generation(data_dir, filename):
    bagfile = data_dir + filename
    bag = rosbag.Bag(bagfile)
    bridge = CvBridge()
    camera_name = bagfile[bagfile.index('P'):bagfile.index('_')]
    output_dir = data_dir + bagfile[bagfile.index('P'):-4]
    os.mkdir(output_dir)
    stamp_file = open(output_dir+'.txt', 'w')

    frame_size = (1600, 1200)
    video_name = output_dir + '.avi'
    video = cv2.VideoWriter(filename=video_name, apiPreference=0,
                            fourcc=cv2.VideoWriter_fourcc(*'XVID'), fps=22,
                            frameSize=frame_size)
    count = 0
    for topic, msg, t in bag.read_messages(topics="/kitchen/"+camera_name+"/image_raw"):
        time_stamp_sec = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        stamp_file.write(str(msg.header.stamp.secs) +
                         " " + str(msg.header.stamp.nsecs) + "\n")
        # stamp_file.write(str(time_stamp_sec) + "\n")
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(os.path.join(
            output_dir, "frame%06i.png" % count), cv_img)
        video.write(np.asarray(cv_img))
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


def search_image(data_dir, filename, time_stamp):
    bagfile = data_dir + filename
    bag = rosbag.Bag(bagfile)
    bridge = CvBridge()
    camera_name = bagfile[bagfile.index('P'):bagfile.index('_')]
    output_dir = data_dir + \
        bagfile[bagfile.index('P'):-4] + '_' + str(time_stamp)
    os.mkdir(output_dir)
    stamp_file = open(output_dir+'.txt', 'w')

    count = 0
    for topic, msg, t in bag.read_messages(topics="/kitchen/"+camera_name+"/image_raw"):
        time_stamp_sec = msg.header.stamp.secs + msg.header.stamp.nsecs/1e9
        if abs(time_stamp_sec - time_stamp_search) < 1:
            stamp_file.write(str(msg.header.stamp.secs) +
                             " " + str(msg.header.stamp.nsecs) + "\n")
            # stamp_file.write(str(time_stamp_sec) + "\n")
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(os.path.join(
                output_dir, "frame%06i.jpg" % count), cv_img)
        if (time_stamp_sec - time_stamp_search) > 1:
            break
        count += 1
    bag.close()
    cv2.destroyAllWindows()
    stamp_file.close()


if __name__ == '__main__':

    # time_stamp_set = [get_time_s_from_local_str(time_local_str='18:57:21.440474',
    #                                             date_local_str='2022-06-07',
    #                                             input_time_format='%H:%M:%S.%f',
    #                                             input_date_format='%Y-%m-%d'),
    #                   get_time_s_from_local_str(time_local_str='18:57:44.140452',
    #                                             date_local_str='2022-06-07',
    #                                             input_time_format='%H:%M:%S.%f',
    #                                             input_date_format='%Y-%m-%d'),
    #                   get_time_s_from_local_str(time_local_str='18:18:48.542680',
    #                                             date_local_str='2022-06-07',
    #                                             input_time_format='%H:%M:%S.%f',
    #                                             input_date_format='%Y-%m-%d')]

    # # fetch images from bag file
    # data_dir = "/media/storage/"
    # filenames = ["kitchen-PG1_2022-06-07-18-10-57.bag", "kitchen-PG2_2022-06-07-18-10-57.bag",
    #              "kitchen-PG3_2022-06-07-18-10-57.bag", "kitchen-PG4_2022-06-07-18-10-57.bag",
    #              "kitchen-PG5_2022-06-07-18-10-57.bag"]
    # for time_stamp_search in time_stamp_set:
    #     for filename in filenames:
    #         search_image(data_dir, filename, time_stamp_search)

    data_dir = "/run/user/1000/gvfs/afp-volume:host=ActionNet-Data.local,user=Chao,volume=data/experiments/06-07-2022/camera/"
    filenames = ["kitchen-PG1_2022-06-07-17-13-26.bag", "kitchen-PG1_2022-06-07-18-10-57.bag",
                 "kitchen-PG2_2022-06-07-17-13-26.bag", "kitchen-PG2_2022-06-07-18-10-57.bag",
                 "kitchen-PG3_2022-06-07-17-13-26.bag", "kitchen-PG3_2022-06-07-18-10-57.bag",
                 "kitchen-PG4_2022-06-07-17-13-26.bag", "kitchen-PG4_2022-06-07-18-10-57.bag",
                 "kitchen-PG5_2022-06-07-17-13-26.bag", "kitchen-PG5_2022-06-07-18-10-57.bag"]
    for filename in filenames:
        data_generation(data_dir, filename)
