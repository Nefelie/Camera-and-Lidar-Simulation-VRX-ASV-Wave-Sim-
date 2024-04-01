#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2
import numpy as np
import time
import os
import csv

point_cloud_data = None

# Specify the directory to save point cloud data
save_directory = os.path.expanduser("~/vrx_asv_wave_sim/lidar_logs")

def point_cloud_callback(msg):
    global point_cloud_data
    point_cloud_data = msg.data
    # print("Received point cloud data: ", point_cloud_data)

def save_point_cloud():
    global point_cloud_data
    if point_cloud_data is not None:
        # Create the save directory if it doesn't exist
        os.makedirs(save_directory, exist_ok=True)

        # Generate a unique filename
        filename = os.path.join(save_directory, "lidar_points_{}.csv".format(int(time.time())))
        
        # Reshape the point cloud data into a 2D array
        points = np.frombuffer(point_cloud_data, dtype=np.float32).reshape(-1, 4)

        # Get the current timestamp
        timestamp = np.array([[time.time()] * len(points)])

        # Exclude intensity from the points array
        points_without_intensity = points[:, :3]

        # Combine timestamp with points
        points_with_timestamp = np.hstack((timestamp.T, points_without_intensity))

        # Save the point cloud data as a CSV file
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'X', 'Y', 'Z'])
            writer.writerows(points_with_timestamp)
        
        print("Point cloud saved as {}".format(filename))



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('lidar_reader_node')
    subscription = node.create_subscription(
        PointCloud2,
        '/wamv/sensors/lidars/lidar_wamv_sensor/points',
        point_cloud_callback,
        2
    )

    timer_period = 2.0  # seconds
    timer = node.create_timer(timer_period, save_point_cloud)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    print("Running lidar LOGGER file")
    main()
