"""Small rotation conversions for scene and pickup metadata."""
import numpy as np


def zyz(matrix):
    beta=np.arccos(np.clip(matrix[2,2],-1,1))
    if abs(np.sin(beta))>1e-7:
        return np.rad2deg([np.arctan2(matrix[1,2],matrix[0,2]),beta,
                          np.arctan2(matrix[2,1],-matrix[2,0])]).tolist()
    return np.rad2deg([np.arctan2(-matrix[1,0],-matrix[0,0]) if matrix[2,2]<0
                       else np.arctan2(matrix[1,0],matrix[0,0]),beta,0]).tolist()


def quaternion(matrix):
    # Eigenvector form remains stable near a 180-degree home orientation.
    m=matrix
    k=np.array([[m[0,0]-m[1,1]-m[2,2],m[1,0]+m[0,1],m[2,0]+m[0,2],m[2,1]-m[1,2]],
                [m[1,0]+m[0,1],m[1,1]-m[0,0]-m[2,2],m[2,1]+m[1,2],m[0,2]-m[2,0]],
                [m[2,0]+m[0,2],m[2,1]+m[1,2],m[2,2]-m[0,0]-m[1,1],m[1,0]-m[0,1]],
                [m[2,1]-m[1,2],m[0,2]-m[2,0],m[1,0]-m[0,1],m.trace()]])/3
    values,vectors=np.linalg.eigh(k)
    q=vectors[:,np.argmax(values)]
    return (q if q[3]>=0 else -q).tolist()
