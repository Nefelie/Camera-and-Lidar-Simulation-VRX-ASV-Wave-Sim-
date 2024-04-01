#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import os

image_data = None

# Specify the directory to save images
save_directory = os.path.expanduser("~/vrx_asv_wave_sim/camera_logs")

def image_callback(msg):
    global image_data
    image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

def save_image():
    global image_data
    if image_data is not None:
        # Create the save directory if it doesn't exist
        os.makedirs(save_directory, exist_ok=True)

        filename = os.path.join(save_directory, "camera_{}.jpeg".format(int(time.time())))
        cv2.imwrite(filename, cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR))
        print("Image saved as {}".format(filename))

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('image_reader_node')
    subscription = node.create_subscription(
        Image,
        '/wamv/sensors/cameras/front_left_camera_sensor/image_raw',
        image_callback,
        2
    )

    timer_period = 2.0  # seconds
    timer = node.create_timer(timer_period, save_image)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("Running LOGGER file")
    main()
