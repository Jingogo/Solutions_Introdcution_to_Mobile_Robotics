from read_data import read_world, read_sensor_data
from misc_tools import *
import numpy as np
import math
import copy
import matplotlib.pyplot as plt 

#plot preferences, interactive plotting mode
plt.axis([-1, 12, 0, 10])
plt.ion()
plt.show()

def initialize_particles(num_particles, num_landmarks):
    #initialize particle at pose [0,0,0] with an empty map

    particles = []

    for i in range(num_particles):
        particle = dict()

        #initialize pose: at the beginning, robot is certain it is at [0,0,0]
        particle['x'] = 0
        particle['y'] = 0
        particle['theta'] = 0

        #initial weight
        particle['weight'] = 1.0 / num_particles
        
        #particle history aka all visited poses
        particle['history'] = []

        #initialize landmarks of the particle
        landmarks = dict()

        for i in range(num_landmarks):
            landmark = dict()

            #initialize the landmark mean and covariance 
            landmark['mu'] = [0,0]
            landmark['sigma'] = np.zeros([2,2])
            landmark['observed'] = False

            landmarks[i+1] = landmark

        #add landmarks to particle
        particle['landmarks'] = landmarks

        #add particle to set
        particles.append(particle)

    return particles

def sample_motion_model(odometry, particles):
    # Updates the particle positions, based on old positions, the odometry
    # measurements and the motion noise 

    # delta_rot1 = odometry['r1']
    # delta_trans = odometry['t']
    # delta_rot2 = odometry['r2']

    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
    noise = [0.1, 0.1, 0.05, 0.05]

    '''your code here'''
    '''***        ***'''
    for particle in particles:
        pose = {}
        pose['x'] = particle['x']
        pose['y'] = particle['y']
        pose['theta'] =  particle['theta']

        new_pose = sample_odometry_motion(pose, odometry, noise)
        particle['x'] = new_pose['x']
        particle['y'] = new_pose['y']
        particle['theta'] = new_pose['theta']
        particle['history'].append([pose['x'], pose['y']])
    
    return 

def sample_odometry_motion(particle, odometry, alpha):
    x = particle['x']
    y = particle['y']
    theta = particle['theta']
    rot1 = odometry['r1'] 
    rot2 = odometry['r2']
    trans = odometry['t']

    alpha1, alpha2, alpha3, alpha4 = alpha

    std_dev_rot1 = alpha1*abs(rot1)+alpha2*trans
    std_dev_rot2 = alpha1*abs(rot2)+alpha2*trans
    std_dev_trans = alpha3*trans + alpha4*abs(rot1) + alpha4*abs(rot2)

    rot1_sample = rot1 - sample_gaussian_muller(0, std_dev_rot1)
    rot2_sample = rot2 - sample_gaussian_muller(0, std_dev_rot2)
    trans_sample = trans - sample_gaussian_muller(0, std_dev_trans)

    x_sample = x + trans_sample*math.cos(theta+rot1_sample)
    y_sample = y + trans_sample*math.sin(theta+rot1_sample)
    theta_sample = theta + rot1_sample + rot2_sample
    
    new_pose = {}
    new_pose['x'] = x_sample
    new_pose['y'] = y_sample
    new_pose['theta'] = theta_sample

    return new_pose

def measurement_model(particle, landmark):
    #Compute the expected measurement for a landmark
    #and the Jacobian with respect to the landmark.

    px = particle['x']
    py = particle['y']
    ptheta = particle['theta']

    lx = landmark['mu'][0]
    ly = landmark['mu'][1]

    #calculate expected range measurement
    meas_range_exp = np.sqrt( (lx - px)**2 + (ly - py)**2 )
    meas_bearing_exp = math.atan2(ly - py, lx - px) - ptheta

    h = np.array([meas_range_exp, meas_bearing_exp])

    # Compute the Jacobian H of the measurement function h 
    #wrt the landmark location
    
    H = np.zeros((2,2))
    H[0,0] = (lx - px) / h[0]
    H[0,1] = (ly - py) / h[0]
    H[1,0] = (py - ly) / (h[0]**2)
    H[1,1] = (lx - px) / (h[0]**2)

    return h, H

def eval_sensor_model(sensor_data, particles):
    #Correct landmark poses with a measurement and
    #calculate particle weight

    #sensor noise
    Q_t = np.array([[0.1, 0],\
                    [0, 0.1]])

    #measured landmark ids and ranges
    ids = sensor_data['id']
    ranges = sensor_data['range']
    bearings = sensor_data['bearing']

    #update landmarks and calculate weight for each particle
    for particle in particles:

        landmarks = particle['landmarks']

        px = particle['x']
        py = particle['y']
        ptheta = particle['theta'] 

        #loop over observed landmarks 
        for i in range(len(ids)):

            #current landmark
            lm_id = ids[i]
            landmark = landmarks[lm_id]
            
            #measured range and bearing to current landmark
            meas_range = ranges[i]
            meas_bearing = bearings[i]

            if not landmark['observed']:
                # landmark is observed for the first time
                
                # initialize landmark mean and covariance. You can use the
                # provided function 'measurement_model' above
                '''your code here'''
                '''***        ***'''
                lx = px + meas_range * np.cos(ptheta + meas_bearing)
                ly = py + meas_range * np.sin(ptheta + meas_bearing)
                landmark['mu'] = [lx, ly]
                #get expected measurement and Jacobian wrt. landmark position
                h, H = measurement_model(particle, landmark)
                #initialize covariance for this landmark
                H_inv = np.linalg.inv(H)
                landmark['sigma'] = H_inv.dot(Q_t).dot(H_inv.T)
                landmark['observed'] = True

            else:
                # landmark was observed before

                # update landmark mean and covariance. You can use the
                # provided function 'measurement_model' above. 
                # calculate particle weight: particle['weight'] = ...
                '''your code here'''
                '''***        ***'''
                h, H = measurement_model(particle, landmark)
                #Calculate measurement covariance and Kalman gain
                S = landmark['sigma']
                Q = H.dot(S).dot(H.T) + Q_t
                K = S.dot(H.T).dot(np.linalg.inv(Q))
                #Compute the difference between the observed and the expected measurement
                delta = np.array([meas_range - h[0], angle_diff(meas_bearing,h[1])])
                #update estimated landmark position and covariance
                landmark['mu'] = landmark['mu'] + K.dot(delta)
                landmark['sigma'] = (np.identity(2) - K.dot(H)).dot(S)
                # compute the likelihood of this observation
                fact = 1 / np.sqrt(math.pow(2*math.pi,2) * np.linalg.det(Q))
                expo = -0.5 * np.dot(delta.T, np.linalg.inv(Q)).dot(delta)
                weight = fact * np.exp(expo)
                particle['weight'] = particle['weight'] * weight
    #normalize weights
    normalizer = sum([p['weight'] for p in particles])
    
    for particle in particles:
        particle['weight'] = particle['weight'] / normalizer
        
    return 

def resample_particles(particles):
    # Returns a new set of particles obtained by performing
    # stochastic universal sampling, according to the particle 
    # weights.

    # distance between pointers
    step = 1.0/len(particles)
    # random start of first pointer
    u = np.random.uniform(0,step)
    # where we are along the weights
    c = particles[0]['weight']
    # index of weight container and corresponding particle
    i = 0
    new_particles = []
    #loop over all particle weights
    for particle in particles:
    #go through the weights until you find the particle
    #to which the pointer points
        while u > c:
            i = i + 1
            c = c + particles[i]['weight']
        #add that particle
        new_particle = copy.deepcopy(particles[i])
        new_particle['weight'] = 1.0/len(particles)
        new_particles.append(new_particle)
        u = u + step

    return new_particles

def sample_gaussian_muller(mean, std_dev):
    u1 = np.random.uniform(0,1)
    u2 = np.random.uniform(0,1)
    sample = math.cos(2*np.pi*u1)*math.sqrt(-2*math.log(u2))     
    return sample * std_dev + mean

def main():

    print("Reading landmark positions")
    landmarks = read_world("../data/world.dat")

    print("Reading sensor data")
    sensor_readings = read_sensor_data("../data/sensor_data.dat")

    num_particles = 100
    num_landmarks = len(landmarks)

    #create particle set
    particles = initialize_particles(num_particles, num_landmarks)

    #run FastSLAM
    for timestep in range(int(len(sensor_readings)/2)):

        #predict particles by sampling from motion model with odometry info
        sample_motion_model(sensor_readings[timestep,'odometry'], particles)

        #evaluate sensor model to update landmarks and calculate particle weights
        eval_sensor_model(sensor_readings[timestep, 'sensor'], particles)

        #plot filter state
        plot_state(particles, landmarks)

        #calculate new set of equally weighted particles
        particles = resample_particles(particles)

    plt.show('hold')

if __name__ == "__main__":
    main()