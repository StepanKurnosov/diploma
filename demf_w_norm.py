"""
Example:
Simulating a rigid body rotation
"""

from rigidbody import RigidBody
from runge_kutta import RungeKutta4Integrator
import scipy.linalg
from scipy.spatial.transform import Rotation
import numpy as np
import math
from random import gauss

def make_rand_vector(dims): # создание рандомного единичного вектора
    vec = [gauss(0, 1) for i in range(dims)]
    mag = sum(x**2 for x in vec) ** .5
    return [x/mag for x in vec]

def calculate_control_moment(w, B): # рассчет управляющего момента
    magnit_moment = k*(np.cross(w, B))
    control_moment = np.cross(magnit_moment, B)
    return control_moment


B = make_rand_vector(3)
print("Вектор магнитного поля: ", B)
k = 0.1 # коэффициент усиления



# create an object and an integrator
body_model = RigidBody()
integrator = RungeKutta4Integrator()


# ========================================================================================
# setup the body
# ========================================================================================

# setup initial body state
body_model.angular_velocity = 10*np.random.random(3) # угловая скорость тела
print("Угловая скорость: ", body_model.angular_velocity * 180 / math.pi)
body_model.orientation      = Rotation.from_quat( [ 1, 0, 0, 0 ], scalar_first = True ) # ориентация тела полученная из кватерниона
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

# setup simulation parameters
max_time = 100
time_step = 1

# simulate from 0 to a max time
max_step = round( max_time / time_step )
for step_count in range(0, max_step ):
    
    # set a torque
    # torque is calculated with the goal 
    # to achieve 1 degree/second angular rate at the end of the simulation
    body_model.torque = calculate_control_moment(body_model.angular_velocity, B)


    # calculate a new state
    integrator.update( time_step )
    
    # update body state variables from the integrator's result
    body_model.orientation      = Rotation.from_quat( integrator.state[3:], scalar_first = True )
    body_model.angular_velocity = integrator.state[0:3]


    print(  "step:",    step_count, 
            "time:",    integrator.time, 
            "w:",       body_model.angular_velocity * 180 / math.pi, 
            "body_model.torque:",       body_model.torque,
            "w_norm",   scipy.linalg.norm( body_model.angular_velocity * 180 / math.pi ),
            "w_on_B_projection", np.dot( body_model.angular_velocity, B ) / ( scipy.linalg.norm( body_model.angular_velocity) * scipy.linalg.norm( B ) ) )

# at the end of the simulation
# the body should be rotated by 50 degrees around y axis
# and should have 1 degree/second angular velocity