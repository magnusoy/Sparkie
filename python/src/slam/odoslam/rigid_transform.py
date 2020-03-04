
## This is originally from:   http://nghiaho.com/?page_id=671

import numpy as np


# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0] # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points 
    #AA = A - np.tile(centroid_A, (N, 1))
    #BB = B - np.tile(centroid_B, (N, 1))
    # The following should be identical:
    AA = A - centroid_A
    BB = B - centroid_B
    # ...ie, there is no translation between AA and BB, only rotation

    # dot is matrix multiplication for array
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    inv_H = np.dot(BB.T, AA)
    invU, invS, invVt = np.linalg.svd(inv_H)
    inv_R = np.dot(invVt.T, invU.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2,:] *= -1
        R = np.dot(Vt.T, U.T)

    if np.linalg.det(inv_R) < 0:
        print("Reflection detected")
        invVt[2,:] *= -1
        inv_R = np.dot(invVt.T, invU.T)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    #print t

    return R, t, inv_R
