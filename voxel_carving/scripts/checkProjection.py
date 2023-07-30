import numpy as np

if __name__ == "__main__":
    T = [[0.8870166814211639, 0.30820633300323774, -0.3438171944177354, 62.04740163790905,
       -0.4614178674416257, 0.5639714040274654, -0.684857508570149, 81.70023126083908, 
       -0.017174355481584802, 0.7661234311362543, 0.6424639521232343, -66.74656060819173,
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
    point3D=np.array([80,0,0,1])
    point2D=P@point3D
    point2D[0] /= point2D[2]
    point2D[1] /= point2D[2]
    point2D[2] /= point2D[2]
    print(point2D)