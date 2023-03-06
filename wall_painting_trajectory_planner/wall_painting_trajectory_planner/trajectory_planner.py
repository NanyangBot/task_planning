import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, Pose, Point

###############################################################################

def get_quaternion(vec2, vec1=[1, 0, 0]):
    """get rotation matrix between two vectors using scipy"""
    vec1 = np.reshape(vec1, (1, -1))
    vec2 = np.reshape(vec2, (1, -1))
    r = R.align_vectors(vec2, vec1)
    return r[0].as_quat()

class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.frame = None
        self.origin = None
        self.resolution = None
        self.cw = None
        self.ch = None
        self.cleft = None
        self.cupper = None
        self.task = None

    def set_planning_scene(self, input_task_image, distance_map):
        self.map = np.reshape(distance_map.data,(distance_map.width,distance_map.height))
        self.frame = distance_map.header.frame_id
        self.origin = [distance_map.origin.x, distance_map.origin.y, distance_map.origin.z]
        self.resolution = distance_map.resolution
        self.cw = distance_map.canvas_width
        self.ch = distance_map.canvas_height
        self.cleft = distance_map.canvas_leftmost_pixel
        self.cupper = distance_map.canvas_uppermost_pixel
        self.task = self.get_task_from_image(input_task_image)

    def get_task_from_image(self, input_task_image):
        input_task_image_reshaped = np.reshape(input_task_image, (self.cw, self.ch))
        input_task_image_reshaped = cv2.threshold(input_task_image_reshaped, 175, 255, cv2.THRESH_BINARY)[1]
        task_map = np.ones(self.map.shape,int)*255
        task_map[cleft:cleft+cw,cupper:cupper-ch] = input_task_image_reshaped
        task = np.where(task_map<255)
        return task

    def get_position_at_(self,y,z):
        p = Point()
        p.z = self.origin[2]- z * self.resolution
        p.y = self.origin[1]- y * self.resolution
        p.x = self.map[z,y].item()
        return p

    def get_orientation_at_(self,y,z,n=3,verbose=False):
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

    def get_canvas(self):
        canvas_msg = MarkerArray()
        for i in range(self.cleft,self.cleft+self.cw):
            for j in range(self.cupper,self.cupper-self.ch):
                m = Marker()
                m.id = int(str(i)+str(j))
                m.pose.position = self.get_position_at_(i,j)
                m.header.frame_id = self.frame
                m.scale.x = 0.2
                m.scale.y = 0.2
                m.scale.z = 0.2
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0
                m.type = 2
                canvas_msg.markers.append(m)
        return canvas_msg

    def get_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.frame
        for i,(y,z) in enumerate(zip(*self.task)):
            print("Processing Point %d in Path" % (i+1))
            p = PoseStamped()
            p.pose.position = self.get_position_at_(y,z)
            p.pose.orientation = self.get_orientation_at_(y,z)
            p.header.frame_id = self.frame
            path_msg.poses.append(p)
        return path_msg

