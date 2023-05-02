from djitellopy import Tello
import numpy as np
from controller import CLFController, CBFController
from lyapunov import LyapunovControl, BarrierControl
from state_control import Drone

'''
Authors: Julia Pecego, Daniel Bostwick, Thena Guttieri, Shreya Mardia 
'''


tello = Tello()

max_height = 2

def fly():

    tello.connect()
    tello.takeoff()

    tello.get_frame_read()
    #Do some stuff here to get video capture and read AR tag

    ar_tag_in_sight = True

    if ar_tag_in_sight and tello.get_height() < max_height:
        try:
            CLFController(tello) 
        except: 
            CBFController(tello)
    else:
        tello.land() 