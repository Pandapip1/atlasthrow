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

    FIXME - Put the forward kinematics here.  Below just illustrates NumPy.

    x = np.array([1,
                  2,
                  3])

    # Return the tip position as a NumPy 3-element vector.
    return x


#
#  Jacobian
#
def Jac(q):
    # Compute the Jacobian for the tip position.

    FIXME - Put the Jacobian here.  Below again illustrates NumPy.
    
    J = np.eye(3)
    J = np.random.rand(3,3)
    J = np.array([[1, 2, 3],
                  [4, 5, 6],
                  [7, 8, 9]])

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
