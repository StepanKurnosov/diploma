
"""
Rigid body rotation model
"""

from scipy.spatial.transform import Rotation
import numpy as np
from quat_lib import quaternion_multiply


class RigidBody:

    def __init__(self):
        
        self.orientation = Rotation.from_quat([1,0,0,0], scalar_first = True)

        self.angular_velocity = np.array([0,0,0])

        self.torque           = np.array([0,0,0])

        self.Inertia          = np.array([[1, 0, 0],
                                          [0, 1, 0], 
                                          [0, 0, 1]])

        self.InertiaInverted  = np.array([[1, 0, 0],
                                          [0, 1, 0], 
                                          [0, 0, 1]])

    def get_derivative_function(self):

        """
        Creates a function, that calculates the state derivative.
        The resulting function satisfies interface requirements for the Runge-Kutta method,
        while also being able to access class instance's values.
        This way changing class values will change derivative.
        """

        # Euler equation:
        # dL/dt + w x L = M
        # I * dw/dt + w x L = M

        # from the Euler equation dw/dt
        # dw/dt = I.inv() * ( M - w x L )

        # quaternion derivative
        # dq/dt = 1 / 2 * w * q

        def derivative( time, state ):
            w = np.array( state[0:3] )
            L = self.Inertia @ w
            dw_dt = self.InertiaInverted @ ( self.torque - np.cross( w, L ) )
            
            dq_dt = 1 / 2 * np.array( quaternion_multiply( [0, *w], state[3:] ) )
            state_derivative = np.array([
                *dw_dt,
                *dq_dt]
            )
            return state_derivative

        return derivative
