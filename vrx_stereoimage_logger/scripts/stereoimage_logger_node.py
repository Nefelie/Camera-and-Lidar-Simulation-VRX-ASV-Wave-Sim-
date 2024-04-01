#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import os
import threading

left_image_data = None
right_image_data = None
left_timestamp = None
right_timestamp = None
lock = threading.Lock()

save_directory = os.path.expanduser("~/vrx_asv_wave_sim/camera_logs")

def left_image_callback(msg):
    global left_image_data, left_timestamp
    with lock:
        left_image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        left_timestamp = msg.header.stamp.sec

def right_image_callback(msg):
    global right_image_data, right_timestamp
    with lock:
        right_image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        right_timestamp = msg.header.stamp.sec

def save_images():
    global left_image_data, right_image_data, left_timestamp, right_timestamp
    with lock:
        if left_image_data is not None and right_image_data is not None and left_timestamp == right_timestamp:
            os.makedirs(save_directory, exist_ok=True)

            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(left_timestamp))

            left_filename = os.path.join(save_directory, "L_{}.jpeg".format(timestamp_str))
            right_filename = os.path.join(save_directory, "R_{}.jpeg".format(timestamp_str))

            cv2.imwrite(left_filename, cv2.cvtColor(left_image_data, cv2.COLOR_RGB2BGR))
            cv2.imwrite(right_filename, cv2.cvtColor(right_image_data, cv2.COLOR_RGB2BGR))

            print("Left image saved as {}".format(left_filename))
            print("Right image saved as {}".format(right_filename))


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('stereo_image_reader_node')
    left_subscription = node.create_subscription(
        Image,
        '/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw',
        left_image_callback,
        2
    )
    right_subscription = node.create_subscription(
        Image,
        '/wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw',
        right_image_callback,
        2
    )

    timer_period = 2.0  # seconds
    timer = node.create_timer(timer_period, save_images)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("Running stereo LOGGER file")
    main()
