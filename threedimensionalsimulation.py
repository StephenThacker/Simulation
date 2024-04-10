#Expands the simulation to occur in 3 dimensions.
from twodimensionalsimulation import *
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
    return threedimrotationaloperator_z_axis(theta_z, threedimrotationaloperator_y_axis(theta_y,threedimrotationaloperator_x_axis(theta_x, vector)))

class three_d_projectile():
    def __init__(self, time_to_release1):
        self.tang_force_x = 3
        self.tang_force_y = -1
        self.tang_force_z = -1
        self.time_to_release = time_to_release1
        self.time_increments = 2
        self.steps = int(self.time_to_release/self.time_increments)
        self.ang_vel_x = 0
        self.ang_vel_y = 0
        self.ang_vel_z = 0
        self.radius = 15
        self.mass = 20
        self.ground_level = -20
        self.coordinates = np.transpose(np.array([0,0, self.radius]))
        self.tangential_velocity_x = 0
        self.tangential_velocity_y = 0
        self.tangential_velocity_z = 0
        self.theta_displaced_x = 0
        self.theta_displaced_y = 0
        self.theta_displaced_z = 0
        self.overall_velocity_vector_x = 0
        self.overall_velocity_vector_y = 0
        self.overall_velocity_vector_z = 0
        self.x_position_post_release = 0
        self.y_position_post_release = 0
        self.z_position_post_release = 0
        self.gravitational_constant_accel = -6.67430
        self.current_angular_acceleration_x = 0
        self.current_angular_acceleration_y = 0
        self.current_angular_acceleration_z = 0
        self.current_angular_velocity_x = 0
        self.current_angular_velocity_y = 0
        self.current_angular_velocity_z = 0
        self.overall_tang_velocity_vector = 0
        self.post_release_position_x = 0
        self.post_release_position_y = 0
        self.post_release_position_z = 0
        self.current_post_release_tang_vel_x = 0
        self.current_post_release_tang_vel_y = 0
        self. current_post_release_tang_vel_z = 0
    def projectile_pre_release_routine(self):
        self.current_angular_acceleration_x = calculate_angular_acceleration(self.tang_force_x,self.mass, self.radius)
        self.current_angular_acceleration_y = calculate_angular_acceleration(self.tang_force_y,self.mass, self.radius)
        self.current_angular_acceleration_z = calculate_angular_acceleration(self.tang_force_z,self.mass, self.radius)
        theta_x = -1*calculate_angle_of_rotation_on_sphere_by_ang_velocity(self.time_increments, self.current_angular_velocity_x, self.current_angular_acceleration_x)
        theta_y = -1*calculate_angle_of_rotation_on_sphere_by_ang_velocity(self.time_increments, self.current_angular_velocity_y, self.current_angular_acceleration_y)
        theta_z = -1*calculate_angle_of_rotation_on_sphere_by_ang_velocity(self.time_increments, self.current_angular_velocity_z, self.current_angular_acceleration_z)
        self.theta_displaced_x += theta_x
        self.theta_displaced_y += theta_y
        self.theta_displaced_z += theta_z
        self.coordinates = rotate_angle_three_dim(theta_x,theta_y,theta_z,self.coordinates)
        self.current_angular_velocity_x = calculate_angular_velocity2(self.current_angular_velocity_x, self.current_angular_acceleration_x, self.time_increments)
        self.current_angular_velocity_y = calculate_angular_velocity2(self.current_angular_velocity_y, self.current_angular_acceleration_y, self.time_increments)
        self.current_angular_velocity_z = calculate_angular_velocity2(self.current_angular_velocity_z, self.current_angular_acceleration_z, self.time_increments)
        #Need to adjust the
        self.tangential_velocity_x = self.calculate_tangential_velocity_at_sphere_point_rollx()
        self.tangential_velocity_y = self.calculate_tangential_velocity_at_sphere_point_pitchy()
        self.tangential_velocity_z =self.calculate_tangential_velocity_at_sphere_point_yawz()
        self.overall_tang_velocity_vector = np.add(self.tangential_velocity_x,self.tangential_velocity_y)
        self.overall_tang_velocity_vector = np.add(self.overall_tang_velocity_vector, self.tangential_velocity_z)
        return [self.coordinates, self.overall_tang_velocity_vector]

    def calculate_tangential_velocity_at_sphere_point_rollx(self):
        angular_vel_vector = np.array([self.current_angular_velocity_x, 0, 0])
        three_dimcoordin_radius = np.array([0,self.coordinates[1],self.coordinates[2]])
        tangential_velocity = np.cross(three_dimcoordin_radius,angular_vel_vector)
        return tangential_velocity

    def calculate_tangential_velocity_at_sphere_point_pitchy(self):
        angular_vel_vector = np.array([0, self.current_angular_velocity_y, 0])
        three_dimcoordin_radius = np.array([self.coordinates[0],0, self.coordinates[2]])
        tangential_velocity = np.cross(three_dimcoordin_radius, angular_vel_vector)
        return tangential_velocity

    def calculate_tangential_velocity_at_sphere_point_yawz(self):
        angular_vel_vector = np.array([0,0, self.current_angular_velocity_z])
        three_dimcoordin_radius = np.array([self.coordinates[0], self.coordinates[1],0])
        tangential_velocity = np.cross(three_dimcoordin_radius, angular_vel_vector)
        return tangential_velocity

    def projectile_after_release(self):
        return [self.projectile_x_axis_position(),self.projectile_y_axis_position(),self.projectile_z_axis_position()]

    def projectile_x_axis_position(self):
        self.x_position_post_release = position_in_1_dimension(self.x_position_post_release,self.current_post_release_tang_vel_x, self.time_increments,0)
        return self.x_position_post_release

    def projectile_y_axis_position(self):
        self.y_position_post_release = position_in_1_dimension(self.y_position_post_release, self.current_post_release_tang_vel_y,self.time_increments, 0)
        return self.y_position_post_release

    def projectile_z_axis_position(self):
        self.z_position_post_release = position_in_1_dimension(self.z_position_post_release, self.overall_tang_velocity_vector[2],self.time_increments,self.gravitational_constant_accel)
        self.overall_tang_velocity_vector[2] = final_velocity(self.overall_tang_velocity_vector[2], self.gravitational_constant_accel, self.time_increments)
        return self.z_position_post_release


    def simulation(self):

        Array_positionx = []
        Array_positiony = []
        Array_positionz = []
        Array_overall_tan_velx = []
        Array_overall_tan_vely = []
        Array_overall_tan_velz = []

        for i in range(0, self.steps):
            object = self.projectile_pre_release_routine()
            Array_positionx += [object[0][0]]
            Array_positiony += [object[0][1]]
            Array_positionz += [object[0][2]]
            Array_overall_tan_velx += [object[1][0]]
            Array_overall_tan_vely += [object[1][1]]
            Array_overall_tan_velz += [object[1][2]]
            self.x_position_post_release = object[0][0]
            self.y_position_post_release = object[0][1]
            self.z_position_post_release = object[0][2]
            self.current_post_release_tang_vel_x = object[1][0]
            self.current_post_release_tang_vel_y = object[1][1]
            self.current_post_release_tang_vel_z = object[1][2]

        X = Array_positionx
        Y = Array_positiony
        Z = Array_positionz
        u = Array_overall_tan_velx
        v = Array_overall_tan_vely
        w = Array_overall_tan_velz



        Array1_positionx = []
        Array1_positiony = []
        Array1_positionz = []
        Array1_overall_tan_velx = []
        Array1_overall_tan_vely = []
        Array1_overall_tan_velz = []

        while self.z_position_post_release >= self.ground_level:
            object = self.projectile_after_release()
            Array1_positionx += [object[0]]
            Array1_positiony += [object[1]]
            Array1_positionz += [object[2]]
            Array1_overall_tan_velx += [self.current_post_release_tang_vel_x]
            Array1_overall_tan_vely += [self.current_post_release_tang_vel_y]
            Array1_overall_tan_velz += [self.current_post_release_tang_vel_z]

        X1 = Array1_positionx
        Y1 = Array1_positiony
        Z1 = Array1_positionz
        U1 = Array1_overall_tan_velx
        V1 = Array1_overall_tan_vely
        W1 = Array1_overall_tan_velz


        X2 = X + X1
        Y2 = Y + Y1
        Z2 = Z+Z1
        U2 = u + U1
        V2 = v + V1
        W2 = w + W1


        fig = plt.figure()
        ax1 = fig.add_subplot(projection='3d')
        ax1.quiver(X2,Y2,Z2,U2,V2,W2)
        plt.show()
