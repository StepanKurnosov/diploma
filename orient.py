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

def make_rand_vector(dims): # создание рандомного единичного вектора
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]


k_w = 0.1 # коэффициент усиления для w
k_q = 0.005 # коэффициент усиления для q

target_vector = make_rand_vector(3)
current_vector = np.array([1, 0, 0])

def calculate_control_moment(w, B, target_vector, orien_quat): # рассчет управляющего момента

    # перевод магнитного поля  и целевого вектор в систему аппарата
    B_loc = vec_rotation.global2loc(B, orien_quat) 
    target_vector_loc = vec_rotation.global2loc(target_vector, orien_quat)

    # расчет кватерниона поворота 
    dot = np.dot(current_vector, target_vector_loc) # изменил порядок векторов, так вроде правильнее, если мы хотим найти кватернион поворота
    cross = np.cross(current_vector, target_vector_loc) # текущего вектора к целевому
    norm_cross = np.linalg.norm(cross)
    if norm_cross < 1e-6:
        cross_normalized = np.array([0, 0, 0])  # или единичная ось
    else:
        cross_normalized = cross / norm_cross

    half_angle = math.acos(dot)/2
    q = np.array([math.cos(half_angle), cross_normalized[0] * math.sin(half_angle), cross_normalized[1]*math.sin(half_angle), cross_normalized[2]*math.sin(half_angle)])
    q_v = q[1:4].copy()


    # расчет магнитного и механического моментов 
    magnit_moment = -k_w*(np.cross(B_loc, w)) - k_q*(np.cross(B_loc, q_v))
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
body_model.orientation      = Rotation.from_quat( make_rand_vector(4), scalar_first = True ) # ориентация тела полученная из кватерниона

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

target_vector = make_rand_vector(3)
sight_axis = make_rand_vector(3)
# setup simulation parameters
max_time = 2400 # взял условоно один шаг - 1 минута
time_step = 1

# simulate from 0 to a max time
max_step = round( max_time / time_step )
for step_count in range(0, max_step ):
    alpha = step_count * 2 * math.pi / 120
    B = np.array([math.sin(alpha), 0, math.cos(alpha)]) # с учетом того, что 1 шаг - 1 минута, один оборот КА сделает за 120 шагов
    B_loc = vec_rotation.global2loc(B, body_model.orientation.as_quat( scalar_first= True))
    # set a torque
    # torque is calculated with the goal 
    # to achieve 1 degree/second angular rate at the end of the simulation
    body_model.torque = calculate_control_moment(body_model.angular_velocity, B, target_vector, body_model.orientation.as_quat( scalar_first= True))
    orient_vector = vec_rotation.loc2global(current_vector, body_model.orientation.as_quat( scalar_first= True))
    desired_torque = - body_model.angular_velocity + np.cross(orient_vector, target_vector)

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
            "угол между current и target":                vec_rotation.angle_between_vectors(target_vector, orient_vector),
            "угол между M и B":     vec_rotation.angle_between_vectors(body_model.torque, B_loc),
            "угол между жел. M и B":vec_rotation.angle_between_vectors(desired_torque, B_loc)
        }
    )

 
    
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
        }
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
            "y_label": "Градусы ",
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
    ]
] 
)