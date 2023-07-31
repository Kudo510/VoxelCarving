import numpy as np

if __name__ == "__main__":
    T = [[0.8863868489181252, 0.30847590220565513, -0.34519700436620776, 61.74752731678918,
           -0.46266121319591724, 0.5641481362161089, -0.683872416615828, 81.87875911009887,
             -0.01621591406854911, 0.7658847812577932, 0.6427733239398037, -66.8046794582247,
               0.0, 0.0, 0.0, 1.0]]
    K = [920.88464355, 0, 624.062744140625, 
        0, 924.34155273, 374.5955547,
        0, 0, 1]
    # Convert the list into a NumPy array
    T = np.array(T)
    # Convert the list into a NumPy array
    K = np.array(K)

    # Reshape the array into a 3x3 matrix

    # Reshape the array into a 4x4 matrix
    T = T.reshape((4, 4))
    T = np.linalg.inv(T)
    T=T[0:3,:]
    K = K.reshape((3, 3))
    P=K@T
    point3D=np.array([0,0,0,1])
    point2D=P@point3D
    point2D[0] /= point2D[2]
    point2D[1] /= point2D[2]
    point2D[2] /= point2D[2]
    print(point2D)