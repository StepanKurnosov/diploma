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

sight_axis = [1, 0, 0]
target_vector = np.array([1, 2, 3])
current_vector = np.array([4, 5, 6])

def calculate_control_moment(target_vector, current_vector):
    axis_of_rotation = np.cross(current_vector, target_vector)
    control_moment = - (np.cross(axis_of_rotation, target_vector)) / np.linalg.norm(target_vector)**2

    return control_moment

    


# create an object and an integrator
body_model = RigidBody()
integrator = RungeKutta4Integrator()


# ========================================================================================
# setup the body
# ========================================================================================

# setup initial body state
body_model.angular_velocity = np.array( [0, 0, 0] ) # угловая скорость тела
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
    body_model.torque = np.array( [ 0, 
                                    1 / max_time / 180 * math.pi,
                                    0] )

    # calculate a new state
    integrator.update( time_step )
    
    # update body state variables from the integrator's result
    body_model.orientation      = Rotation.from_quat( integrator.state[3:], scalar_first = True )
    body_model.angular_velocity = integrator.state[0:3]

    print(  "step:",    step_count, 
            "time:",    integrator.time, 
            "w:",       body_model.angular_velocity * 180 / math.pi, 
            "q:",       body_model.orientation.as_quat( scalar_first= True) )

# at the end of the simulation
# the body should be rotated by 50 degrees around y axis
# and should have 1 degree/second angular velocity