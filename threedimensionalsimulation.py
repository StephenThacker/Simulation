#Expands the simulation to occur in 3 dimensions.
import numpy as np
import math


#Rotates an object in 3 dimensions.
def threedimrotationaloperator_z_axis(theta,vector):
    A_1 = np.cos(theta)
    A_2 = -1*np.sin(theta)
    B_1 = np.sin(theta)
    B_2 = np.cos(theta)
    operat = np.array([[A_1,A_2,0],[B_1,B_2,0], [0,0,1]])
    return np.matmul(operat,vector)

def threedimrotationaloperator_x_axis(theta,vector):
    B_2 = np.cos(theta)
    B_3 = -1*np.sin(theta)
    C_2 = np.sin(theta)
    C_3 = np.cos(theta)
    operat = np.array([[1, 0, 0],[0, B_2, B_3],[0, C_2, C_3]])
    return np.matmul(operat, vector)

def threedimrotationaloperator_y_axis(theta,vector):
    A_1 = np.cos(theta)
    A_3 = np.sin(theta)
    C_1 = -1*np.sin(theta)
    C_3 = np.cos(theta)
    operat = np.array([[A_1, 0,A_3],[0,1,0],[C_1,0,C_3]])
    return np.matmul(operat,vector)


def rotate_angle_three_dim(theta_x, theta_y, theta_z, vector):
    return threedimrotationaloperator_z_axis(theta_z, threedimrotationaloperator_y_axis(theta_y, threedimrotationaloperator_x_axis(theta_x,vector)))


