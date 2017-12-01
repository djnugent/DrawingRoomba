import numpy as np
import math
import time


# Source: http://nghiaho.com/?page_id=671
# This should be replaced with a ransac implementation like OpenCV. Also scale would be super helpful

## Finds the homographic transformation between 2 sets of N points with M dimensionality
class HomographySolver():

    def __init__(self,dim=2):
        self.m = dim

        # R = mxm rotation matrix
        # t = mx1 column vector
        self.R = np.mat(np.random.rand(self.m,self.m))
        self.t = np.mat(np.random.rand(self.m,1))

        # make R a proper rotation matrix, force orthonormal
        U, S, Vt = np.linalg.svd(self.R)
        self.R = U*Vt
        print(Vt.shape)

        # remove reflection
        if np.linalg.det(self.R) < 0:
           Vt[self.m-1,:] *= -1
           self.R = U*Vt


    # Input: expects NxM matrix of points
    # Returns R,t
    def solve_transformation(self,A,B):
        assert(len(A) == len(B))

        A = np.asmatrix(A)
        B = np.asmatrix(B)

        N = A.shape[0] # total points

        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)

        # centre the points
        AA = A - np.tile(centroid_A, (N, 1))
        BB = B - np.tile(centroid_B, (N, 1))

        # dot is matrix multiplication for array
        H = np.transpose(AA) * BB
        U, S, Vt = np.linalg.svd(H)

        self.R = Vt.T * U.T

        # special reflection case
        if np.linalg.det(self.R) < 0:
           print("Reflection detected")
           Vt[self.m-1,:] *= -1
           self.R = Vt.T * U.T

        self.t = -self.R*centroid_A.T + centroid_B.T

        # Find the error
        # Apply transformation
        A2 = (self.R*A.T) + np.tile(self.t, (1, N))
        A2 = A2.T

        err = A2 - B
        err = np.multiply(err, err)
        err = np.sum(err)
        rmse = math.sqrt(err/N);

        return self.R, self.t, rmse


if __name__ == "__main__":
    dim = 2
    hs = HomographySolver(dim = dim)

    # number of points
    n = 15

    # Create 2 sets of points
    A = np.mat(np.random.rand(n,dim))
    B = hs.R*A.T + np.tile(hs.t, (1, n))
    B = B.T

    print(A.shape,B.shape)

    # recover the transformation
    start = time.time()
    ret_R, ret_t, rmse = hs.solve_transformation(A,B)
    stop = time.time()


    print("Points A")
    print(A)
    print("")

    print("Points B")
    print(B)
    print("")

    print("Rotation")
    print(ret_R)
    print("")

    print("Translation")
    print(ret_t)
    print("")

    print("RMSE:", rmse)
    print("If RMSE is near zero, the function is correct!")
    print("Time:", stop-start)
