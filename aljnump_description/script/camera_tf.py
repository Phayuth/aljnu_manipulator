import numpy as np
from pytransform3d.transformations import transform_from_pq, pq_from_transform

"""
Problem : Determine H camera_link to tool0 from Hand-eye calibration.

Solution : H camera_link to tool0 =
H camera_color_optical_frame to tool0 @ H camera_link to camera_color_optical_frame

Note : If we have H child to parent, use <ros2 run tf2_ros tf2_echo parent child>
       ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_link
"""

# xyz qwqxqyqz
HccoTtool0_pq = [
    -0.03192296,
    -0.09189364,
    0.02288404,
    0.99972914,
    0.01883875,
    0.00332055,
    0.01325577,
]
HccoTtool0 = transform_from_pq(HccoTtool0_pq)

HclTcco_pq = [
    0.015,
    0.000,
    0.000,
    0.503,
    0.500,
    -0.498,
    0.500,
]
HclTcco = transform_from_pq(HclTcco_pq)

HclTtool0 = HccoTtool0 @ HclTcco
print(f"> HclTtool0 (homogeneous matrix):\n{HclTtool0}")
HclTtool0_pq = pq_from_transform(HclTtool0)
print(f"> HclTtool0 (translation + quaternion): {HclTtool0_pq}")
