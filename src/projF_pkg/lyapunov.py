import numpy as np

'''
Authors: Julia Pecego, Daniel Bostwick, Thena Guttieri, Shreya Mardia 
'''


class LyapunovControl:

    def ___init__(self, drone):

        self.self = None
        self.drone = drone



class BarrierControl:

    def __init__(self, drone):

        self.drone = drone
        self._vals = None

    def point_cloud(self, s):
        '''
        Creates a pointcloud of our conic region in the Face Frame.
        --------------------------------
        Input:
            s (int): sample size that point cloud fits into
        Return:
            self.cloud (ndarray): numpy array of points in the shape of a cone 
        '''
        # s = 500
        self.cloud = np.ones((3, 2*s**2))

        x_r = np.linspace(-0.1, 0.1, s*2)
        z_r = np.linspace(-0.1, 0.1, s*2)

        i = 0 
        for x in x_r:
            for z in z_r:
                # print(x)
                y = np.sqrt(130*x**2 + 130*z**2)
                if y < 0.85 or y > 1.15:
                    continue
                else:
                    self.cloud[:, i] = [x, y ,z]
                # cloud[:, i+1] = [x, -y, z]
                # print(cloud[:,i])
                i += 1

        return self.cloud

    #closest point
    def barrier(self, u):
        '''
        Evaluates a barrier that is from the Face Frame in the Drone's Frame
        --------------------------------
        Input:
            u (int): inputs to the system
        Return:
            self._vals (list): list containing h, and h_Dot
        '''
        rotation = np.array([[-1, 0, 0],
                            [0, -1, 0],
                            [0, 0 ,1]])

        translation = np.array([0, 1, 0])

        dist = 10000
        point_cloud = self.point_cloud(500)

        #Need to change z value for sure, Not going to be 1.7 everytime
        # qD = np.array([0, 0, 1.7])

        #Might have to change how to index into get_state()
        qD = self.drone.get_state()

        for i in range(0, len(point_cloud)):
            transformed_cloud = rotation @ point_cloud[:,i] + translation
            if (qD - transformed_cloud).T @ (qD - transformed_cloud) < dist:
                closest_point = transformed_cloud
        h = (qD - closest_point).T @ (qD - closest_point)
        h_Ddot = 2 * (qD - closest_point).T @ rotation @ u

        self._vals = [h, h_Ddot]
        return self._vals 

