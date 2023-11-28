import math
import cmath
from matplotlib import pyplot as plt
import numpy as np


def gravitational_force(mass_object_1, mass_object_2,distance):
    grav_force = 6.67430*mass_object_2*mass_object_1
    grav_force = grav_force/(distance**2)
    return grav_force

def calculate_angle_against_horizontal_plane(x_coord,y_coord):
    hyp = math.sqrt(x_coord**2 + y_coord**2)
    div = y_coord/hyp
    return math.degrees(math.asin(div))

def rotate_complex_point(angle, complex_number):
    return cmath.exp(complex(0,1)*angle)*complex_number

def compute_centripetal_acceleration(tangential_velocity, radius, mass):
    return ((tangential_velocity**2)/radius)*mass

def compute_normal_force_of_object(force_gravity,centripetal_force):
    return force_gravity-centripetal_force

def does_object_leave_surface_of_sphere(magnitude_force_gravity,normal_force):
    print("normal force")
    print(normal_force)
    print("gravitational force")
    print(magnitude_force_gravity)
    if abs(normal_force) > abs(magnitude_force_gravity):
        return True
    else:
        return False

def acceleration_in_one_dimension(force, mass):
    return force/mass

def calculate_angular_acceleration(tangential_force, mass, radius):
    return tangential_force/(mass*radius)

def calculate_change_in_position_in_one_dimension(initial_velocity, time, acceleration):
    return initial_velocity*time + 0.5*(acceleration)*(time**2)

def calculate_angle_of_rotation_on_sphere_by_tangential_velocity(time, velocity,angular_acceleration, radius ):
    theta = (velocity*time)/radius + (0.5)*(angular_acceleration)*(time**2)
    return theta

def calculate_angular_velocity(initial_ang_vel, ang_accel, theta):
    temp = initial_ang_vel**2 + 2*ang_accel*theta
    return math.sqrt(temp)

#coordinates of sphere are a complex number
class sphere():
    def __init__(self, radius, mass, coordinates_of_sphere):
        self.is_car_on_this_sphere = False
        self.radius = radius
        self.mass = mass
        self.coordinates = coordinates_of_sphere

class car():
    def __init__(self, mass_car, starting_sphere, time_increments, *args):
        #If car not on a sphere, current sphere has a value of false
        self.current_sphere = starting_sphere
        self.mass = mass_car
        #represents horizontal force to be applied by the user
        self.user_applied_horizontal_force = .5
        self.current_coordinates = complex(0, self.current_sphere.radius)
        self.overall_force_on_car = complex(0, 0)
        self.current_angular_acceleration = calculate_angular_acceleration(self.user_applied_horizontal_force,self.mass,self.current_sphere.radius)
        self.current_angular_velocity = 0
        self.time_increment = time_increments
        self.car_current_Theta = (math.pi/2)
        self.grav_magnitude_car = gravitational_force(self.current_sphere.mass,self.mass, self.current_sphere.radius)
        self.tangential_velocity = 0
        self.normal_mag = 0
        self.coordinates_tang_velocity = 0

    def plot_overall_car_movement(self):
        return

    #We assume the car is on the sphere and a constant horizontal force. This subroutine calculates what the position of the car and the angular velocity of the car are
    #after the time in one time increment has ended. Checks to see if the car leaves the sphere.
    def car_on_sphere_routine(self):
        theta = calculate_angle_of_rotation_on_sphere_by_tangential_velocity(self.time_increment,self.current_angular_velocity, self.current_angular_acceleration, self.current_sphere.radius)
        self.current_angular_velocity = calculate_angular_velocity(self.current_angular_velocity, self.current_angular_acceleration, theta)
        #positive theta corresponds to counteclockwise motion. This is the opposite of positive and negative velocity in the x direction.
        #need to adjust the formula to account for this automatically, later.
        self.current_coordinates = rotate_complex_point((-1*theta), self.current_coordinates)
        self.car_current_Theta = (math.pi/2)-theta
        self.tangential_velocity = self.current_angular_velocity*self.current_sphere.radius
        self.coordinates_tang_velocity = self.coordinates_tangential_velocity()
        print(self.coordinates_tang_velocity)
        return self.current_coordinates

    #Computes the x,y components of the force of gravity acting on the sphere
    def compute_coordinates_gravity_current_sphere(self):
        x_coord = self.grav_magnitude_car*math.cos(self.car_current_Theta)
        y_coord = self.grav_magnitude_car*math.sin(self.car_current_Theta)
        #multiply by negative 1, since grav points towards inner of sphere.
        return complex(-1*x_coord, -1*y_coord)

    #Rotates force vector around the sphere, translates to origin (based on current coordinates) to get the components of the applied force as it rotates the sphere.
    def compute_coordinates_horizontal_applied_force(self):
        starting_horizontal_force = complex(self.user_applied_horizontal_force,self.current_sphere.radius)
        complex_num = rotate_complex_point((-1*(self.car_current_Theta + (math.pi/2))), starting_horizontal_force)
        complex_num = complex_num - self.current_coordinates
        return complex_num

    def coordinates_normal_force(self):
        radius =  math.sqrt(self.current_coordinates.real**2 + self.current_coordinates.imag**2)
        mag_normal = self.grav_magnitude_car - (self.mass*(self.tangential_velocity**2))/radius
        x_coord = mag_normal*math.cos(self.car_current_Theta)
        y_coord = mag_normal*math.sin(self.car_current_Theta)
        self.normal_mag = mag_normal
        return complex(x_coord, y_coord)

    def coordinates_tangential_velocity(self):
        radius = math.sqrt(self.current_coordinates.real**2 + self.current_coordinates.imag**2)
        dummy_velocity = complex(self.tangential_velocity, radius)
        dummy_velocity = rotate_complex_point(-1*(self.car_current_Theta - (math.pi/2)), dummy_velocity)
        print("start")
        print(dummy_velocity)
        print("stop")
        dummy_velocity = dummy_velocity - self.current_coordinates

        return dummy_velocity

    def sum_of_forces_acting_on_car(self,*args):
        #generalize this to include other spheres, later, i.e., take in 10 spheres
        complex = 0
        for elem in args:
            complex += elem
        #complex += self.compute_coordinates_horizontal_applied_force()
        #complex += self.compute_coordinates_gravity_current_sphere()
        #complex += self.coordinates_normal_force()
        return complex

    def does_car_leave_sphere(self):
        return does_object_leave_surface_of_sphere(self.grav_magnitude_car,self.normal_mag)


    def calculate_car_pos_in_space(self):
        self.overall_force_on_car = self.sum_of_forces_acting_on_car(self.compute_coordinates_gravity_current_sphere(), self.coordinates_normal_force())
        accel_x = self.overall_force_on_car.real/self.mass
        accel_y = self.overall_force_on_car.imag/self.mass
        initial_velocity = self.coordinates_tangential_velocity()
        initial_velocity_x = initial_velocity.real
        initial_velocity_y = initial_velocity.imag
        change_pos_x = calculate_change_in_position_in_one_dimension(initial_velocity_x, self.time_increment,accel_x)
        change_pos_y = calculate_change_in_position_in_one_dimension(initial_velocity_y, self.time_increment,accel_y)
        self.user_applied_horizontal_force = 0
        self.current_coordinates = self.current_coordinates + complex(change_pos_x, change_pos_y)
        return self.current_coordinates

    def calculate_car_position_II(self):
        self.user_applied_horizontal_force = 0
        change_pos_x = calculate_change_in_position_in_one_dimension(self.coordinates_tang_velocity.real, self.time_increment, 0)
        print("change_pos_x")
        print(change_pos_x)
        change_pos_y = calculate_change_in_position_in_one_dimension(self.coordinates_tang_velocity.imag, self.time_increment, 0)
        print("change_pos_y")
        print(change_pos_y)
        print("tangential velocity")
        print(self.coordinates_tang_velocity)
        print("previous coordinates")
        print(self.current_coordinates)
        self.current_coordinates = self.current_coordinates + complex(change_pos_x, change_pos_y)
        print("current coordinates")
        print(self.current_coordinates)
        return self.current_coordinates

    #Generalize this later for multiple spheres.
    def check_if_car_on_sphere(self, sphere):
        if math.sqrt(self.current_coordinates.real**2 + self.current_coordinates.imag**2) <= sphere.radius:
            self.current_sphere = sphere
            #adjust car position , to be on sphere.

def test_angular_acceleration_position():

    sphere1 =sphere(20,30,complex(0,0))
    car1 = car(2,sphere1,0.3)
    #Simulate car motion
    Array_x = []
    Array_y = []
    Array_x += [car1.current_coordinates.real]
    Array_y += [car1.current_coordinates.imag]

    Array_x_2 = []
    Array_y_2 = []
    for i in range(0,41):
        complex_num = car1.car_on_sphere_routine()
        Array_x += [complex_num.real]
        Array_y += [complex_num.imag]

    for i in range(0,32):
        complex_num = car1.car_on_sphere_routine()
        Array_x_2 += [complex_num.real]
        Array_y_2 += [complex_num.imag]

    plt.scatter(Array_x,Array_y)
    plt.xlabel("x_coordinate")
    plt.ylabel("y_coordinate")
    plt.xlim([-30,30])
    plt.ylim([-30,30])
    plt.show()
    plt.close()

    plt.scatter(Array_x_2,Array_y_2)
    plt.xlabel("x_coordinate")
    plt.ylabel("y_coordinate")
    plt.xlim([-30,30])
    plt.ylim([-30,30])
    plt.show()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    #Eventually need to generalize this code bit to be object oriented(when have multiple spheres)
    #test_angular_acceleration_position()
    sphere2 =sphere(20, 30, complex(0, 0))
    car2 = car(2, sphere2, 1)
    Array_x = []
    Array_y = []
    Array_x += [car2.current_coordinates.real]
    Array_y += [car2.current_coordinates.imag]

    #Run test simulation
    for i in range(0,1000):
        if car2.current_sphere != False:
            complex_num = car2.car_on_sphere_routine()
            Array_x += [complex_num.real]
            Array_y += [complex_num.imag]
            car2.overall_force_on_car = car2.sum_of_forces_acting_on_car(car2.compute_coordinates_horizontal_applied_force(),car2.compute_coordinates_gravity_current_sphere(),car2.coordinates_normal_force())
            if car2.does_car_leave_sphere() == True:
                car2.current_sphere = False
                print("here")
        else:
            complex_num = car2.calculate_car_position_II()
            Array_x += [complex_num.real]
            Array_y += [complex_num.imag]




    plt.scatter(Array_x,Array_y)
    plt.xlabel("x_coordinate")
    plt.ylabel("y_coordinate")
    plt.xlim([-100,100])
    plt.ylim([-100,100])
    plt.show()
    plt.close()
