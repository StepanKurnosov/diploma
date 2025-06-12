from rigidbody import RigidBody
from runge_kutta import RungeKutta4Integrator
import scipy.linalg
from scipy.spatial.transform import Rotation
import numpy as np
import math
from random import gauss
import csv
import vec_rotation
import graphs
import matplotlib
import magnit_field


<<<<<<< HEAD
lam1 = 0.01
k = 0.05  # коэффициент слайдинга (усилие затухания)

=======

>>>>>>> abedd481a64923962dd71ae7c43423b4b36ad8a4

sight_axis = np.array([1, 0, 0]) # ось спутника, которую хотим направить
q_target = np.array([1, 0, 0, 0]) # эксперимент с возвращением спутника в "начальное положение" 
#q_target = vec_rotation.make_rand_vector(4) # эксперимент со случайной ориентпцией
target_vector = vec_rotation.loc2global(sight_axis, q_target)




def calculate_control_moment(w, B, orien_quat): # рассчет управляющего момента

    # перевод магнитного поля  и целевого вектор в систему аппарата
    B_loc = vec_rotation.global2loc(B, orien_quat)
    w_glob = vec_rotation.loc2global(w, orien_quat)
    print("Цель: ", target_vector)
    print("Реальность: ", vec_rotation.loc2global(sight_axis, orien_quat))


    #Вариант № 1: нелогичный, но рабочий в достаточно общем, но единичном случае
    # ВСЕГДА ориентируеют объект в изначальное положение (1, 0, 0, 0)
    # q = orien_quat[0] * np.array([orien_quat[1], orien_quat[2], orien_quat[3]])
    # s = w_glob + lam1 * q

    #Вариант № 2: логичный, но нерабочий
    # ВСЕГДА стабилизирует, останавливает, но не направляет в нужную сторону
    q_current_rev = vec_rotation.conjugate_quat(orien_quat)
    q_target_rev = vec_rotation.conjugate_quat(q_target)
    q_err = vec_rotation.quaternion_multiply(q_target, q_current_rev)
    if q_err[0] < 0:  # Коррекция на кратчайший путь
        q_err[0] = -q_err[0]
        q_err[1] = -q_err[1]
        q_err[2] = -q_err[2]
        q_err[3] = -q_err[3]

    q = np.array([q_err[1], q_err[2], q_err[3]])
    

    print("Ошибка угла: ", q_err[0])
    s = w - lam1 * q

    # расчет скользящей поверхности
   

    s_norm = np.linalg.norm(s)
    if s_norm < 1e-3:  # Порог для "нахождения на поверхности"
        print("На скользящей поверхности: s ≈ 0", s_norm)
    else:
        print("Мы не на скользязей", s_norm)
    # расчет магнитного и механического моментов 
    magnit_moment = - k * np.cross(B_loc, s)
    control_moment = np.cross(magnit_moment, B_loc)
    return control_moment


# создать объект и интегратор
body_model = RigidBody()
integrator = RungeKutta4Integrator()


# ========================================================================================
# setup the body
# ========================================================================================

# setup initial body state
body_model.angular_velocity = 10*np.random.random(3) # угловая скорость тела
# добавил случайное положение тела
body_model.orientation      = Rotation.from_quat( vec_rotation.make_rand_vector(4), scalar_first = True ) # ориентация тела полученная из кватерниона

# setup body parameters
body_model.Inertia          = np.array( [   [1, 0, 0],
                                            [0, 1, 0],
                                            [0, 0, 1] ] )
body_model.InertiaInverted  = scipy.linalg.inv( body_model.Inertia )

# init the integrator
integrator.state_derivative_funtion = body_model.get_derivative_function()
integrator.state                    = np.array([   *body_model.angular_velocity, 
                                                   *body_model.orientation.as_quat( scalar_first= True)])


# ========================================================================================
# simulation loop
# ========================================================================================
log_frames = {
    "times":    [],
    "data":     [] 
    }

# setup simulation parameters
<<<<<<< HEAD
max_time = 2400 # взял условоно один шаг - 1 минута
=======
max_time = 2000 # взял условоно один шаг - 1 минута
>>>>>>> abedd481a64923962dd71ae7c43423b4b36ad8a4
time_step = 1

# simulate from 0 to a max time
max_step = round( max_time / time_step )
for step_count in range(0, max_step ):
    alpha = step_count * 2 * math.pi / 120
<<<<<<< HEAD
    #B = magnit_field.calculete_B_in_decard(r, alpha)
    B = np.array([math.sin(alpha), 0, math.cos(alpha)]) # с учетом того, что 1 шаг - 1 минута, один оборот КА сделает за 120 шагов
=======
    B = magnit_field.calculete_B_in_decard(r, alpha)
    #B = np.array([math.sin(alpha), 0, math.cos(alpha)]) # с учетом того, что 1 шаг - 1 минута, один оборот КА сделает за 120 шагов
>>>>>>> abedd481a64923962dd71ae7c43423b4b36ad8a4
    B_loc = vec_rotation.global2loc(B, body_model.orientation.as_quat( scalar_first= True))
    # set a torque
    # torque is calculated with the goal 
    # to achieve 1 degree/second angular rate at the end of the simulation

    current_vector = vec_rotation.loc2global(sight_axis, body_model.orientation.as_quat( scalar_first= True))
    body_model.torque = calculate_control_moment(body_model.angular_velocity, B, body_model.orientation.as_quat( scalar_first= True))
    q_target_rev = vec_rotation.conjugate_quat(q_target)
    q_err = vec_rotation.quaternion_multiply(q_target_rev, body_model.orientation.as_quat( scalar_first= True))
    q = np.array([q_err[1], q_err[2], q_err[3]])
    current_dir = vec_rotation.loc2global(sight_axis, body_model.orientation.as_quat( scalar_first= True))
    target_dir = vec_rotation.loc2global(sight_axis, q_target)
    angle_error = np.arccos(np.dot(current_dir, target_dir)) * 180 / np.pi


    desired_torque = - body_model.angular_velocity + np.cross(current_vector, vec_rotation.global2loc(target_vector, body_model.orientation.as_quat( scalar_first= True)))

    # calculate a new state
    integrator.update( time_step )
    
    # update body state variables from the integrator's result
    body_model.orientation      = Rotation.from_quat( integrator.state[3:], scalar_first = True )
    body_model.angular_velocity = integrator.state[0:3]

    # log time
    log_frames["times"].append( integrator.time )
    # log data
    log_frames["data"].append(
        {
            "time":                 integrator.time,
            "w x":                  body_model.angular_velocity[0] * 180 / math.pi,
            "w y":                  body_model.angular_velocity[1] * 180 / math.pi, 
            "w z":                  body_model.angular_velocity[2] * 180 / math.pi,
            "w":                    scipy.linalg.norm( body_model.angular_velocity * 180 / math.pi ),
            "проекция w на B":      np.dot( body_model.angular_velocity, B_loc ) / ( scipy.linalg.norm(body_model.angular_velocity) * scipy.linalg.norm( B ) ),
            "угол между current и target":               angle_error,
            "угол между M и B":     vec_rotation.angle_between_vectors(body_model.torque, B_loc),
            "угол между жел. M и B":vec_rotation.angle_between_vectors(desired_torque, B_loc),
            "ошибка ориентации":    q_err,
            "текущая ориентация":   body_model.orientation.as_quat( scalar_first= True)
            
        }
    )



    # print(  "step:",    step_count, 
    #         "time:",    integrator.time, 
    #         "w:",       body_model.angular_velocity * 180 / math.pi, 
    #         "w_norm",   scipy.linalg.norm( body_model.angular_velocity * 180 / math.pi ),
    #         "w_on_B_projection", np.dot( body_model.angular_velocity, B ) / ( scipy.linalg.norm( body_model.angular_velocity) * scipy.linalg.norm( B ) ),
    #         "B: ", B)
 
    
graphs.display_results( log_frames, [
    # figures
    [
        # subplots
        {
            "subplot_title":"Угловая скорость",
            "y_label": "Градусы °/ с",
            "lines":
                [
                    # lines
                    "w x",
                    "w y",
                    "w z",
                    "w"
                ]
        },
    ],
    [
        # subplots
        {
            "subplot_title":"Угол отклонения от целевого вектора",
            "y_label": "Градусы °",
            "lines":
                [
                    # lines
                    "угол между current и target"
                ]
        }
    ],
    [
        # subplots
        {
            "subplot_title":"Угол расхождения управляющего момента и B",
            "y_label": "Градусы",
            "lines":
                [
                    # lines
                    "угол между M и B"
                ]
        },
        {
            "subplot_title":   "Угол расхождения желаемого момента и B",
            "y_label": "Градусы",
            "lines":
                [
                    # lines
                    "угол между жел. M и B"
                ]
        }
    ],
    [
        # subplots
        {
            "subplot_title":"целевая орентация",
            "y_label": "Градусы",
            "lines":
                [
                    # lines
                    "ошибка ориентации"
                ]
        },
        {
            "subplot_title":   "текущая ориентация",
            "y_label": "Градусы",
            "lines":
                [
                    # lines
                    "текущая ориентация"
                ]
        }
    ]
] 
)