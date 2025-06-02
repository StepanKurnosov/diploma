from random import gauss
import numpy as np
import math

def quaternion_multiply( quat_a, quat_b ):
    quat_result = [
        quat_a[0] * quat_b[0] - quat_a[1] * quat_b[1] - quat_a[2] * quat_b[2] - quat_a[3] * quat_b[3],
        quat_a[0] * quat_b[1] + quat_a[1] * quat_b[0] + quat_a[2] * quat_b[3] - quat_a[3] * quat_b[2],
        quat_a[0] * quat_b[2] - quat_a[1] * quat_b[3] + quat_a[2] * quat_b[0] + quat_a[3] * quat_b[1],
        quat_a[0] * quat_b[3] + quat_a[1] * quat_b[2] - quat_a[2] * quat_b[1] + quat_a[3] * quat_b[0]
    ]
    
    return quat_result

def make_rand_vector(dims): # создание рандомного единичного вектора
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]

def vec_to_quat(vec):
    result = np.insert(vec, 0, 0)
    return result

def conjugate_quat(quat):
    result = np.array([quat[0], -quat[1], -quat[2], -quat[3]])
    return result


def loc2global(init_vector, turn_q):
    reverse_q = conjugate_quat(turn_q) # сопряженный кватернион
    init_quat = vec_to_quat(init_vector) # представляем вектор через кватернион  
    final_quat = quaternion_multiply(quaternion_multiply(turn_q, init_quat), reverse_q) 
    final_vector = np.array([final_quat[1], final_quat[2], final_quat[3]])
    return final_vector

def global2loc(init_vector, turn_q):
    reverse_q = conjugate_quat(turn_q) # сопряженный кватернион
    init_quat = vec_to_quat(init_vector) # представляем вектор через кватернион 
    final_quat = quaternion_multiply(quaternion_multiply(reverse_q, init_quat), turn_q) 
    final_vector = np.array([final_quat[1], final_quat[2], final_quat[3]])
    return final_vector

def angle_between_vectors(vec1, vec2):
    return np.degrees(math.acos(np.dot(vec1, vec2)/ (np.linalg.norm(vec1)* np.linalg.norm(vec2)))) 

# vec1 = np.array([1, 0, 0])
# vec2 = np.array([0.7071, 0, 0.7071, 0])
# print(loc2global(vec1, vec2))
# print(global2loc(loc2global(vec1, vec2), vec2))

