from djitellopy import Tello
import numpy as np

'''
Authors: Julia Pecego, Daniel Bostwick, Thena Guttieri, Shreya Mardia 
'''

class Drone:

    def __init__(self):

        self.tello = Tello()

    def get_state(self):
        '''
        Returns:
            dict with all fields 
        '''
        return self.tello.get_current_state()

    def get_vel(self):
        '''
        Returns:
            3x1 numpy array of drone directional velocity (int) 
        '''
        x_vel = self.tello.get_speed_x()
        y_vel = self.tello.get_speed_y()
        z_vel = self.tello.get_speed_z
        return np.array([x_vel, y_vel, z_vel])
    
    def get_accel(self):
        '''
        Returns:  
            3x1 numpy array of drone acceleration (float)
        '''
        x_acc = self.tello.get_acceleration_x()
        y_acc = self.tello.get_acceleration_y()
        z_acc = self.tello.get_acceleration_z()
        return np.array([x_acc, y_acc, z_acc])
    
    def get_height(self):
        '''
        Returns:  
            (int) -> height in cm
        '''
        return self.tello.get_height()
    

