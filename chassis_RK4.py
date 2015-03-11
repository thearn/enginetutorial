"""
    chassis.py - Chassis component for the vehicle example problem.
"""

# This openMDAO component determines the vehicle acceleration based on the
# power output of the engine, modified by the transmission torque ratio.

from math import pi

# pylint: disable-msg=E0611,F0401
from openmdao.main.api import Component
from openmdao.main.datatypes.api import Float, Array

import numpy as np

import rk4

class Chassis(rk4.RK4):



    Cf = Float(0.035, iotype='in',
                    desc='Friction Coefficient (multiplies W)')
    Cd = Float(0.3, iotype='in',
               desc='Drag Coefficient (multiplies V**2)')
    area = Float(2.164, iotype='in', units='m**2',
                      desc='Frontal area')
    mass_engine = Float(200.0, iotype='in', units='kg',
                             desc='Engine weight estimation')

    tire_circ = Float(1.905, iotype='in', units='m',
                           desc='Circumference of tire')

    #state variables
    state = Array([], iotype="out")
    #initial mass of vehicle (kg) and velocity (m/s)

    state_init = Array([1400, 0], iotype="in")

    #external variables
    t = Array([], iotype="in")
    torque_ratio = Array([], iotype='in',
                         desc='Ratio of output torque to engine torque')
    engine_torque = Array([], iotype='in', units='N*m',
                            desc='Torque at wheels')

    def __init__(self, h = 4, end_time=764):

        super(Chassis, self).__init__()
        self.h=h

        self.t = np.arange(0, end_time+h, h)
        self.torque_ratio = np.ones(len(self.t))
        self.engine_torque = 200* np.ones(len(self.t))
        #self.engine_torque =  200*np.sin(np.linspace(0, 2*np.pi, len(self.t)))
        self.state=np.zeros((2,len(self.t)))


        self.state_var = 'state'
        self.init_state_var = 'state_init'
        self.external_vars=['t', 'torque_ratio', 'engine_torque']

    def f_dot(self, external, state):

        mass = state[0]
        V = state[1]
        t = external[0]
        torque_ratio = external[1]
        engine_torque = external[2]

        sign_V = np.sign(V)

        torque = engine_torque*torque_ratio
        tire_radius = self.tire_circ/(2.0*pi)
        friction = self.Cf*mass*9.8
        drag = .5*(1.225)*self.Cd*self.area*V*V

        #print sign_V*(friction +drag), V, sign_V


        #acceleration = (torque/tire_radius - sign_V*(friction +drag))/mass
        acceleration = (torque/tire_radius)/mass
        m_dot = -0.001*t  #fuel burn rate (kg/s)

        f_dot = np.array([m_dot, acceleration])


        return f_dot

    def list_deriv_vars(self):
        return ("t", "torque_ratio", "engine_torque", "state_init",),("state",)

    def df_dy(self, external, state):
        J = np.zeros((2,2))

        m, v = state
        t, tr, et = external

        J[0,0] = 0
        J[0,1] = 0
        J[1,0] = -(et*tr)/ (self.tire_circ/(2.0*pi))/(m**2)
        J[1,1] = 0

        return J

    def df_dx(self,external,state):
        J = np.zeros((2,3))

        m, v = state
        t, tr, et = external

        J[0,0] = -.001
        J[0,1] = 0
        J[0,2] = 0

        J[1,0] = 0
        J[1,1] = (et)/ (self.tire_circ/(2.0*pi))/(m)
        J[1,2] = (tr)/ (self.tire_circ/(2.0*pi))/(m)

        return J






if __name__ == '__main__':

    trial = Chassis()

    trial.run()

    #trial.check_gradient()


    import matplotlib.pyplot as plt
    plt.plot(trial.t, trial.state[0])
    plt.title('mass vs time')
    plt.xlabel('time (s)')
    plt.ylabel('mass (kg)')
    plt.figure()
    plt.plot(trial.t, trial.state[1])
    plt.title('speed vs time')
    plt.xlabel('time (s)')
    plt.ylabel('speed (m/s)')

    plt.figure()
    plt.title('torque vs time')
    plt.plot(trial.t, trial.engine_torque)
    plt.show()
