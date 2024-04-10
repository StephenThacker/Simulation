from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from matplotlib import pyplot as plt
import math
import numpy as np

#functions and equations for uniform circular motion

def rotation_operator(theta,vector):
    A_1 = np.cos(theta)
    A_2 = -1*np.sin(theta)
    B_1 = np.sin(theta)
    B_2 = np.cos(theta)
    operat = np.array([[A_1, A_2],[B_1,B_2]])
    return np.matmul(operat,vector)

def gravitational_force(mass_object_1, mass_object_2,distance):
    grav_force = 6.67430*mass_object_2*mass_object_1
    grav_force = grav_force/(distance**2)
    return grav_force

def calculate_angular_acceleration(tangential_force, mass, radius):
    return tangential_force/(mass*radius)

def calculate_angular_velocity(initial_ang_vel, ang_accel, theta):
    temp = initial_ang_vel**2 + 2*ang_accel*theta
    return math.sqrt(temp)

def calculate_angular_velocity2(initial_velocity, angular_accel,time):
    return initial_velocity + angular_accel*time

def calculate_tangential_vel_magnitude(angular_velocity,radius):
    return angular_velocity*radius

def calculate_angle_against_horizontal_plane(x_coord,y_coord):
    hyp = math.sqrt(x_coord**2 + y_coord**2)
    div = y_coord/hyp
    return math.degrees(math.asin(div))

def calculate_angle_of_rotation_on_sphere_by_ang_velocity(time, initial_angular_vel, angular_acceleration ):
    theta = time*initial_angular_vel + (0.5)*(angular_acceleration)*(time**2)
    return theta

#kinematic equations for 2-dimensional motion

def final_velocity(initial_velocity, acceleration,time):
    return initial_velocity + acceleration*time

def position_in_1_dimension(initial_position, initial_vel, time, acceleration):
    return initial_position + initial_vel*time + 0.5*acceleration*time**2

def normalize_vector(vector):
    return vector/np.linalg.norm

class projectile():
    def __init__(self, time_to_release):
        self.tang_force = 0.01
        self.time_to_release = time_to_release
        self.time_increments = 0.25
        self.steps = int(self.time_to_release/self.time_increments)
        self.ang_vel = 0
        self.radius = 15
        self.mass = 20
        self.coordinates = np.transpose(np.atleast_2d(np.array([0, self.radius])))
        self.tangential_velocity = 0
        self.current_angular_acceleration = 0
        self.current_angular_velocity = 0
        self.theta_displaced = 0
        #y-coordinate corresponding to the ground.
        self.ground_level = -30
        #post release variables
        self.overall_velocity_vector_x = 0
        self.overall_velocity_vector_y = 0
        self.x_position_post_release = 0
        self.y_position_post_release = 0
        self.gravitational_constant_accel = -6.67430
        self.range_target_forgiveness = 3
        self.post_release_results_from_experiment = 0


    #instead of rewriting the entire thing, I can just vectorize this angular momentum in 3-dimensions.
    def projectile_pre_release_routine(self):
        #calculate current angular acceleration (i.e. the user changes the angular acceleration)
        self.current_angular_acceleration = calculate_angular_acceleration(self.tang_force, self.mass, self.radius)
        #calculates current theta based off of ang acceleration, velocity and updates coordinates
        theta = -1*calculate_angle_of_rotation_on_sphere_by_ang_velocity(self.time_increments, self.current_angular_velocity, self.current_angular_acceleration)
        self.theta_displaced += theta
        self.coordinates = rotation_operator(theta, self.coordinates)
        #updates velocity, calculates tangential velocity and the coordinates of the tangential velocity vector
        self.current_angular_velocity = calculate_angular_velocity2(self.current_angular_velocity, self.current_angular_acceleration, self.time_increments)
        self.tangential_velocity = self.calculate_tangential_velocity_at_sphere_point()
        return [self.coordinates, self.tangential_velocity]



    def projectile_after_release(self):

        return [self.projectile_x_axis_position(), self.projectile_y_axis_position()]

    def projectile_x_axis_position(self):
        self.x_position_post_release = position_in_1_dimension(self.x_position_post_release,self.overall_velocity_vector_x,self.time_increments,0)

        return self.x_position_post_release

    def projectile_y_axis_position(self):
        self.y_position_post_release = position_in_1_dimension(self.y_position_post_release,self.overall_velocity_vector_y,self.time_increments, self.gravitational_constant_accel)
        self.overall_velocity_vector_y = final_velocity(self.overall_velocity_vector_y, self.gravitational_constant_accel,self.time_increments)

        return self.y_position_post_release

    #Returns the vector that is the tangential velocity of an object moving in uniform circular motion
    #Takes the vector that corresponds to position on the sphere, rotates it 90 degrees and normalizes it to have the magnitude of the tangential velocity.
    #Need to go ahead and implement this in 3 dimensions, to make the math make sense.
    #positive angular velocity indicates counterclockwise motion
    def calculate_tangential_velocity_at_sphere_point(self):
        angular_vel_vector = np.array([0,0,self.current_angular_velocity])
        three_dimcoordin_radius = np.array([self.coordinates[0][0], self.coordinates[1][0], 0])
        tangential_velocity = np.cross(three_dimcoordin_radius, angular_vel_vector)
        return np.delete(tangential_velocity,2)

    def target_simulator(self,range):
        Array = []
        while len(Array) < 2:
            x = np.random.randint(low=-40, high=range)
            if x < (-1*self.radius) or x > (self.radius):
                Array += [x]
        return Array


    #Need to adjust the simulation to give the correct output, or write another method to do the job.
    def simulation(self):
        Array_positionx = []
        Array_positiony = []
        Array_overall_tan_velx = []
        Array_overall_tan_vely = []
        for i in range(0, self.steps):
            temporary = self.projectile_pre_release_routine()
            Array_positionx += [temporary[0][0]]
            Array_positiony += [temporary[0][1]]
            Array_overall_tan_velx += [temporary[1][0]]
            Array_overall_tan_vely += [temporary[1][1]]
            self.x_position_post_release = temporary[0][0]
            self.y_position_post_release = temporary[0][1]

        fig, ax1 = plt.subplots(1,1,figsize=(10,10))
        X = Array_positionx
        Y = Array_positiony
        u = Array_overall_tan_velx
        v = Array_overall_tan_vely
        ax1.quiver(X,Y,u,v)
        plt.show()



        self.overall_velocity_vector_x = self.tangential_velocity[0]
        self.overall_velocity_vector_y = self.tangential_velocity[1]

        self.current_angular_velocity = 0

        Array_positionx = [self.coordinates[0]]
        Array_positiony = [self.coordinates[1]]
        Array_overall_tan_velx = [self.overall_velocity_vector_x]
        Array_overall_tan_vely = [self.overall_velocity_vector_y]
        #continue simulation until the object hits the ground.
        while self.y_position_post_release >= self.ground_level:
            temporary = self.projectile_after_release()
            Array_positionx += [temporary[0]]
            Array_positiony += [temporary[1]]
            Array_overall_tan_velx += [self.overall_velocity_vector_x]
            Array_overall_tan_vely += [self.overall_velocity_vector_y]

        fig, ax1 = plt.subplots(1,1)
        X = Array_positionx
        Y = Array_positiony
        u = Array_overall_tan_velx
        v = Array_overall_tan_vely
        ax1.quiver(X,Y,u,v)
        plt.show()
        Array_positionx1 = Array_positionx + [[300]]
        Array_positiony1 = Array_positiony + [[400]]
        plt.scatter(Array_positionx1,Array_positiony1)
        plt.show()

        return [X,Y]

    def reinforcment_simulation(self):

        Array_positionx = []
        Array_positiony = []
        Array_overall_tan_velx = []
        Array_overall_tan_vely = []
        for i in range(0, self.steps):
            temporary = self.projectile_pre_release_routine()
            Array_positionx += [temporary[0][0]]
            Array_positiony += [temporary[0][1]]
            Array_overall_tan_velx += [temporary[1][0]]
            Array_overall_tan_vely += [temporary[1][1]]
            self.x_position_post_release = temporary[0][0]
            self.y_position_post_release = temporary[0][1]

        X = Array_positionx
        Y = Array_positiony
        u = Array_overall_tan_velx
        v = Array_overall_tan_vely

        self.overall_velocity_vector_x = self.tangential_velocity[0]
        self.overall_velocity_vector_y = self.tangential_velocity[1]

        self.current_angular_velocity = 0

        Array_positionx = [self.coordinates[0]]
        Array_positiony = [self.coordinates[1]]
        Array_overall_tan_velx = [self.overall_velocity_vector_x]
        Array_overall_tan_vely = [self.overall_velocity_vector_y]
            # continue simulation until the object hits the ground.
        while self.y_position_post_release >= self.ground_level:
            temporary = self.projectile_after_release()
            Array_positionx += [temporary[0]]
            Array_positiony += [temporary[1]]
            Array_overall_tan_velx += [self.overall_velocity_vector_x]
            Array_overall_tan_vely += [self.overall_velocity_vector_y]

        X = Array_positionx
        Y = Array_positiony

        self.post_release_results_from_experiment = [X,Y]

        return [X,Y]

    def calculates_closest_distance(self, array_x, array_y, coordinates_of_target):
        min = 0
        for i in range(0, len(array_x)):
            for j in range(0, len(array_y)):
                if min <= math.sqrt((array_x[i]-coordinates_of_target[0])**2 + (array_y[j]-coordinates_of_target[1])**2):
                    min = math.sqrt((array_x[i]-coordinates_of_target[0])**2 + (array_y[j]-coordinates_of_target[1])**2)
        return min

    def is_a_hit(self, min, forgiveness):
        if min <= forgiveness:
            return 1
        else:
            return 0

    def loss_function(self,hit, min_distance, max_distance):
        if hit == 1:
            return 10
        else:
            return 10*-1*math.log(min_distance/max_distance,math.e)

# Press the green button in the gutter to run the script.
