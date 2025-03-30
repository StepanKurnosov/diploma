"""
helper functions to deal with a quaternion math
"""

def quaternion_multiply( quat_a, quat_b ):

    quat_result = [
        quat_a[0] * quat_b[0] - quat_a[1] * quat_b[1] - quat_a[2] * quat_b[2] - quat_a[3] * quat_b[3],
        quat_a[0] * quat_b[1] + quat_a[1] * quat_b[0] + quat_a[2] * quat_b[3] - quat_a[3] * quat_b[2],
        quat_a[0] * quat_b[2] - quat_a[1] * quat_b[3] + quat_a[2] * quat_b[0] + quat_a[3] * quat_b[1],
        quat_a[0] * quat_b[3] + quat_a[1] * quat_b[2] - quat_a[2] * quat_b[1] + quat_a[3] * quat_b[0]
    ]
    
    return quat_result