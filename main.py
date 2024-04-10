from threedimensionalsimulation import *
import numpy as np

def PID_derivative(K_1, step_size, error):
    if len(error)<2:
        return 0
    else:
        return K_1*(error[1]-error[0])/step_size

def PID_proporitional(K_2, error):
    return K_2*error[0]


def PID_integration(K_3, step_size, error):
    sum = 0
    for i in range(0,len(error)-1):
        sum += error[i]*step_size

    return sum*K_3


def stopping_criteria(error):
    sum = 0
    for i in range(0,len(error)):
        sum += error[i]
    return sum



if __name__ == '__main__':

    projectile1 = three_d_projectile(80)
    print(rotate_angle_three_dim(0, np.pi, np.pi, [0,1,0]))
    print(projectile1.coordinates)
    print(rotate_angle_three_dim(0, np.pi, np.pi,projectile1.coordinates))
    print("steve")

    projectile1.simulation()
    #Code for PID control attempt
    '''
    target_coordinates = [50,50]
    seed1 = np.random.default_rng(231)
    seed = seed1.random()*30
    ball = projectile(20)
    error = []
    flag = True
    ref = math.sqrt(target_coordinates[0]**2 + target_coordinates[1]**2)
    while flag == True:
        results = ball.reinforcment_simulation()
        #limits the memory of the PID controller to the most recent 20 events.
        current_min = ball.calculates_closest_distance(results[0], results[1], target_coordinates)
        if len(error) <= 20:
            error = [0.5*ref - 0.5*current_min +0.05*current_min] + error
            ref = current_min
            print(current_min)
        else:
            error = [0.5*ref - 0.5*current_min +0.05*current_min] + error
            ref = current_min
            error.pop()
            print(ref)
        seed += PID_derivative(0, ball.time_increments,error) + PID_proporitional(0,error) + PID_integration(4, ball.time_increments, error)
        seed = seed/10
        ball.time_to_release = seed
        #stopping criteria
        if stopping_criteria(error) <= 10 and len(error) == 20:
            flag = False
        '''
