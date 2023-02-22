import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion, PoseStamped, Point

###############################################################################

def get_quaternion(vec2, vec1=[1, 0, 0]):
    """get rotation matrix between two vectors using scipy"""
    vec1 = np.reshape(vec1, (1, -1))
    vec2 = np.reshape(vec2, (1, -1))
    r = R.align_vectors(vec2, vec1)
    return r[0].as_quat()

class DistanceMap:
    def __init__(self,map):
        self.w = map.width
        self.h = map.height
        self.map = np.reshape(map.data, (self.w,self.h))
        self.task = np.where(np.reshape(map.task, (self.w,self.h)))
        self.origin = [map.origin.x, map.origin.y, map.origin.z]
        self.resolution = map.resolution

    def get_position_at_(self,y,z):
        p = Point()
        p.z = self.origin[2]- z * self.resolution
        p.y = self.origin[1]- y * self.resolution
        p.x = self.map[z,y].item()
        return p

    def get_orientation_at_(self,y,z,n=3,verbose=True):
        temp_zs = np.arange(max(z-n+2,0),min(z+n-1,self.h))
        temp_ys = np.arange(max(y-n+2,0),min(y+n-1,self.w))
        grid_ys, grid_zs = np.meshgrid(temp_ys,temp_zs)
        list_ys = grid_ys.flatten()
        list_zs = grid_zs.flatten()

        # do fit
        ys = self.origin[2] - list_ys * self.resolution
        zs = self.origin[1] - list_zs * self.resolution
        xs = [self.map[j,i] for i,j in zip(list_ys,list_zs)]
        A = np.array([[i,j,k] for i,j,k in zip(xs,ys,zs)])
        b = np.ones((len(xs),1))
        fit = np.linalg.lstsq(A,b,rcond=None)[0]
        errors = b - np.dot(A,fit)
        
        n = [-i[0] if fit[0][0]<0 else i[0] for i in fit]
        quat = get_quaternion(n)
        q = Quaternion()
        q.x = quat[0]
        q.y = quat[1]
        q.z = quat[2]
        q.w = quat[3]

        if verbose:
            print("===========================")
            print("A in 'A * X = 1':")
            print(A)
            print("solution:")
            print("%f x + %f y + %f z = 1" % (fit[0], fit[1], fit[2]))
            print("errors:")
            print(errors)
            print("Normal vector:")
            print(n)
            print("Quaternion:")
            print(quat)
            print("===========================")

        return q

    def get_path(self):
        poses = []
        for i,(y,z) in enumerate(zip(*self.task)):
            print("Processing Point %d in Path" % (i+1))
            pose = PoseStamped()
            pose.header.frame_id='map'
            pose.pose.position = self.get_position_at_(y,z)
            pose.pose.orientation = self.get_orientation_at_(y,z)
            poses.append(pose)
        return poses
