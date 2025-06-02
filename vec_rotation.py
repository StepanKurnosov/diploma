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

def angle_between_vectors(vec1, vec2):
    return np.degrees(math.acos(np.dot(vec1, vec2)/ (np.linalg.norm(vec1)* np.linalg.norm(vec2)))) 


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

# vec_y = np.array([0, 1, 0])
# vec_y = np.array([0, 1, 0])
# # проверка функций вращения
# vec_x = np.array([1, 0, 0])
# vec_y = np.array([0, 1, 0])
# vec_z = np.array([0, 0, 1])

# q_rotation_around_x = np.array([0.7071, 0.7071, 0, 0])
# q_rotation_around_y = np.array([0.7071, 0, 0.7071, 0])
# q_rotation_around_z = np.array([0.7071, 0, 0, 0.7071])
# # 1) Случий № 1: проверка loc2global - перевод из локальной СК в общую
# # есть кватернион ориентации тела q, есть вектор в локальной системе аппарата, требуется найти координаты в общей системе координат
# # допустим возьмем ось Ох аппарата, в ЛСК имеет координаты (1, 0, 0), аппарата развернут от изначального положения на 90 градусов вокруг оси Оу
# # тогда в общей СК кооридинаты оси аппарата будут рассчитываться через loc2global и будет равны (0, 0, -1)
# print('Случий № 1.1: ', loc2global(vec_x, q_rotation_around_y)) # ожидание (0, 0, -1)
# print('Случий № 1.2: ', loc2global(vec_y, q_rotation_around_z)) # ожидание (-1, 0, 0)
# print('Случий № 1.3: ', loc2global(vec_z, q_rotation_around_x)) # ожидание (0, -1, 0)
# print('\n')

# # 2) Случий № 2: проверка global2loc - перевод из общей в локальную СК
# # есть кватернион ориентации тела q, есть вектор в общей СК, требуется найти координаты в локальной системе координат
# # допустим возьмем ось Оy общей СК, в ОСК имеет координаты (0, 1, 0), аппарата развернут от изначального положения на 90 градусов вокруг оси Оx
# # тогда в локальной СК кооридинаты оси Oy будут рассчитываться через global2loc и будет равны (0, 0, -1)
# print('Случий № 2.1: ', global2loc(vec_y, q_rotation_around_x)) # ожидание (0, 0, -1)
# print('Случий № 2.2: ', global2loc(vec_z, q_rotation_around_y)) # ожидание (-1, 0, 0)
# print('Случий № 2.3: ', global2loc(vec_x, q_rotation_around_z)) # ожидание (0, -1, 0)
