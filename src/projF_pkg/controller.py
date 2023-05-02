import numpy as np
import casadi as ca
from lyapunov import BarrierControl, LyapunovControl



'''
Authors: Julia Pecego, Daniel Bostwick, Thena Guttieri, Shreya Mardia 
'''

class Controller:

    def __init__(self, drone):
        self.drone = drone
    
    def eval_input(self):
        self._u = self.drone.get_vel()
        return self._u

    def get_input(self):
        return self._u

class CLFController:

    def __init__(self, drone):
        self.drone = drone
        self.lyapunov = LyapunovControl(self.drone)
    



class CBFController:

    def __init__(self, drone):

        self.drone = drone
        self.barrier = BarrierControl(drone)
        
        self.nominalController = CLFController(drone)


    def eval_input(self):

        nom_in = self.nominalController.eval_input()

        opti = ca.Opti()
        q = opti.variable(3,1)
        
        x, y, z = q
        gamma = 1

        h, hdot = self.barrier(q)

        opti.subject_to(hdot + (gamma*h) >= 0)
        opti.subject_to(y == np.sqrt(100*x**2 + 100*z**2))
        opti.subject_to(y < 0.85)
        opti.subject_to(y > 1.15)


        cost = (q-nom_in).T @ (q-nom_in) 
        opti.minimize(cost)
        option = {'verbose': False, 'ipopt.print_level': 0, 'print_time': 0}
        opti.solver('ipopt', option)
        sol = opti.solve()
        u = sol.value(q)
        self._u = u

        return self._u