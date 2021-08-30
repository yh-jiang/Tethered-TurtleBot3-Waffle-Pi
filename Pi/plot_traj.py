#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def Plot():
    f = open('real_traj.txt', 'r')
    real_traj = f.read()
    real_traj = real_traj.split('\n')
    real_traj_data = np.zeros([len(real_traj)-1,2])
    for i in range(len(real_traj)-1):
        real_traj_data[i][0] = float(real_traj[i].split(',')[0])
        real_traj_data[i][1] = float(real_traj[i].split(',')[1])

    f2 = open('traj_opt.txt', 'r')
    reference_traj = f2.read()
    reference_traj = reference_traj.split('\n')
    reference_traj_data = np.zeros([len(reference_traj)-1,2])
    for i in range(len(reference_traj)-1):
        reference_traj_data[i][0] = float(reference_traj[i].split(',')[0])
        reference_traj_data[i][1] = float(reference_traj[i].split(',')[1])

    plt.plot(real_traj_data[:,0],real_traj_data[:,1], label = 'Real Traj')
    plt.plot(reference_traj_data[:,0], reference_traj_data[:,1], label = 'Reference Traj')
    plt.legend(loc='best')
    plt.show()
    f.close()
    f2.close()

if __name__ == '__main__':
    Plot()