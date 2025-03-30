"""
Runge-Kutta integration method of 4th order from
the book
========================================= 
Numerical Recipes: 
The Art of Scientific Computing (3rd ed.)
=========================================
"Section 17.1 Runge-Kutta Method"
-----------------------------------------
Press, William H.; Teukolsky, Saul A.; Vetterling, William T.; Flannery, Brian P. (2007),
Cambridge University Press, ISBN 978-0-521-88068-8.
"""
import numpy as np
import scipy.linalg

def runge_kutta_step( state, current_time, delta_time, state_derivative_function ):
    
    # k_1 = f( tn, yn )
    k_1 = state_derivative_function( current_time + delta_time, state )
    
    # k_2 = f( tn + h/2, yn + h * k_1/ 2 )
    k_2 = state_derivative_function( current_time + delta_time / 2, state + delta_time * k_1 / 2)
    
    # same as before, but this time use k_2
    # k_3 = f( tn + h/2, yn + h * k_2/ 2 )
    k_3 = state_derivative_function( current_time + delta_time / 2, state + delta_time * k_2 / 2 )
    
    # k_4 = f( tn + h, yn + h * k_3 )
    k_4 = state_derivative_function( current_time + delta_time, state + delta_time * k_3 )

    # yn+1 = y_n + h / 6 * ( k_1 + 2*k_2 + 2*k_3 + k_4 )
    # tn+1 = tn + h
    new_state = state +  delta_time / 6 * ( k_1 + 2*k_2 + 2*k_3 + k_4 )

    return new_state


class RungeKutta4Integrator:

    def __init__(self):

        self.state = None
        self.time = 0
        self.state_derivative_funtion = None

    def update( self, delta_time ):

        self.state = runge_kutta_step( self.state, self.time, delta_time, self.state_derivative_funtion )

        # normalizing a quaternion
        self.state[3:] = self.state[3:] / scipy.linalg.norm( self.state[3:] )


        self.time = self.time + delta_time