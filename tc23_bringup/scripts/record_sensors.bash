#!/bin/bash
ros2 bag record \
    --storage mcap \
    --max-bag-size 2000000000 \
    /d455/imu \
    /d455/aligned_depth_to_color/image_raw \
    /d455/aligned_depth_to_color/camera_info \
    /d455/color/camera_info \
    /d455/color/image_raw \
    /d455/depth/depth/camera_info \
    /d455/depth/depth/image_raw \
    /velodyne_packets \
    /diagnostics
