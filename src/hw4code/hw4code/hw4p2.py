'''hw4p2.py

   This is skeleton code for HW4 Problem 2.  PLEASE EDIT (SEARCH FOR FIXME)!

   Implement the Newton-Raphson for seven target points.

'''

import numpy as np
import matplotlib.pyplot as plt

# Grab the fkin and Jac from P1.
from hw4code.hw4p1 import fkin, Jac


#
#  Utilities - Use for columns (d) and (e).
#
# Determine the number of 360 deg wrappings:
def wraps(q):
    return np.round(q / (2*np.pi))

# 3 DOF Multiplicities - return True of False!
def elbow_up(q):
    return np.sin(q[2]) < 0.0

def front_side(q):
    return l1 * np.cos(q[1]) + l2 * np.cos(q[1] + q[2]) > 0.0



#
#  Newton Raphson
#
def newton_raphson(xgoal):
    # Collect the distance to goal and change in q every step!
    xdistance = []
    qstepsize = []

    # Set the initial joint value guess.
    q = np.array([0.0, np.pi/2, -np.pi/2])

    # Number of steps to try.
    N = 100

    # IMPLEMENT THE NEWTON-RAPHSON ALGORITHM!

    FIXME - The NR algorithm should fit here....

    # Create a plot of x distances to goal and q step sizes, for N steps.
    N = 20
    xdistance = xdistance[:N+1]
    qstepsize = qstepsize[:N+1]

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    ax1.plot(range(len(xdistance)), xdistance)
    ax2.plot(range(len(qstepsize)), qstepsize)

    ax1.set_title(f'Convergence Data for {xgoal.T}')
    ax2.set_xlabel('Iteration')

    ax1.set_ylabel('Task Distance to Goal')
    ax1.set_ylim([0, max(xdistance)])
    ax1.set_xlim([0, N])
    ax1.set_xticks(range(N+1))
    ax1.grid()

    ax2.set_ylabel('Joint Step Size')
    ax2.set_ylim([0, max(qstepsize)])
    ax2.set_xlim([0, N])
    ax2.set_xticks(range(N+1))
    ax2.grid()

    plt.show()


#
#  Main Code
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # Prcess each target (goal position).
    for xgoal in [np.array([0.5,  1.0, 0.5]), 
                  np.array([1.0,  0.5, 0.5]),
                  np.array([2.0,  0.5, 0.5]),
                  np.array([0.0, -1.0, 0.5]),
                  np.array([0.0, -0.6, 0.5]),
                  np.array([0.5, -1.0, 0.5]),
                  np.array([-1.0, 0.0, 0.5])]:
        newton_raphson(xgoal)

if __name__ == "__main__":
    main()
