'''hw4p1.py

   This is skeleton code for HW4 Problem 1.  PLEASE EDIT (SEARCH FOR FIXME)!

   This should simply use NumPy to implement the known forward
   kinematics and Jacobian functions for the 3 DOF robot.

'''

import numpy as np


#
#  Forward Kinematics
#
def fkin(q):
    # Compute the tip position.

    theta_pan = q[0]
    theta_1 = q[1]
    theta_2 = q[2]

    x = np.array([-np.sin(theta_pan) * (np.cos(theta_1) + np.cos(theta_1+theta_2)),
                  np.cos(theta_pan) * (np.cos(theta_1) + np.cos(theta_1+theta_2)),
                  np.sin(theta_1) + np.sin(theta_1+theta_2)])
    #x = np.round(x, decimals=4)

    # Return the tip position as a NumPy 3-element vector.
    return x


#
#  Jacobian
#
def Jac(q):
    # Compute the Jacobian for the tip position.

    theta_pan = q[0]
    theta_1 = q[1]
    theta_2 = q[2]
    theta_12 = theta_1 + theta_2

    J = np.array([[-np.cos(theta_pan) * (np.cos(theta_1) + np.cos(theta_12)), np.sin(theta_pan) * (np.sin(theta_1) + np.sin(theta_12)), np.sin(theta_pan) * np.sin(theta_12)],
                  [-np.sin(theta_pan) * (np.cos(theta_1) + np.cos(theta_12)), -np.cos(theta_pan) * (np.sin(theta_1) + np.sin(theta_12)), -np.cos(theta_pan) * np.sin(theta_12)], 
                  [0, np.cos(theta_1) + np.cos(theta_12), np.cos(theta_12)]])
    J = np.round(J, decimals=4)

    # Return the Jacobian as a NumPy 3x3 matrix.
    return J


#
#  Main Code
#
#  This simply tests the above functions.
#
def main():
    # Run the test case.  Suppress infinitesimal numbers.
    np.set_printoptions(suppress=True)

    # First (given) test case with following joint coordinates.
    print("TEST CASE #1:")
    q = np.radians(np.array([20, 40, -30]))
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

    # Second test case with following joint coordinates.
    print("TEST CASE #2")
    q = np.radians(np.array([30, 20, 50]))
    print('q:\n',       q)
    print('fkin(q):\n', fkin(q))
    print('Jac(q):\n',  Jac(q))

if __name__ == "__main__":
    main()
