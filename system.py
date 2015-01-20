from math import pi

# pylint: disable-msg=E0611,F0401
from openmdao.main.api import Assembly
from openmdao.main.api import Component
from openmdao.main.datatypes.api import Float, Array
from openmdao.main.api import set_as_top
from scipy.interpolate import interp1d
import numpy as np
import rk4
from openmdao.lib.drivers.api import SLSQPdriver, NewtonSolver

from chassis_RK4 import Chassis
from error import Error 
import time

class System(Assembly):

     def configure(self):
        self.add('chassis', Chassis())
        self.add('error', Error())
        
        self.connect('chassis.state[1]', 'error.current_speed')
        self.add('driver', SLSQPdriver())
        self.driver.add_objective('sum(chassis.engine_torque)')
        self.driver.add_parameter('chassis.engine_torque', -500, 500)
        self.driver.add_parameter('chassis.torque_ratio', 0.1, 3)

        self.driver.add_constraint('error.norm=0')
        self.driver.accuracy = 1.0e-16
        self.driver.maxiter = 100
        
        self.driver.gradient_options.force_fd = False
        self.driver.workflow.add(['chassis', 'error'])
         

if __name__ == '__main__':


    """
    min: error + sum(torque)
    101.135969 seconds process time
    error: 245.592006097
    obj:

    """

    trial = set_as_top(System()) 
    t0 = time.clock()

    # from openmdao.util.dotgraph import plot_system_tree
    # trial._setup()
    # plot_system_tree(trial._system)
    # quit()

    trial.run()
    print time.clock() - t0, "seconds process time"
    print "error:", trial.error.norm
    print "tire circ:", trial.chassis.tire_circ
    print "objective:", sum(trial.chassis.engine_torque)
    import matplotlib.pyplot as plt
    plt.figure()
    plt.subplot(311)
    plt.title("actual vs. target velocity")
    plt.plot(trial.chassis.t, trial.error.target_speed,c='b')
    plt.plot(trial.chassis.t, trial.error.current_speed,c='r')
    plt.subplot(312)
    plt.title("engine torque")
    plt.plot(trial.chassis.t, trial.chassis.engine_torque,c='b')
    plt.subplot(313)
    plt.title("torque ratio")
    plt.plot(trial.chassis.t, trial.chassis.torque_ratio,c='b')
    plt.show()



