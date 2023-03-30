import cv2
import numpy as np
from skimage.transform import resize, pyramid_reduce
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from wall_painting_trajectory_planner.tsp_solver import TSPsolver
#from wall_painting_trajectory_planner.dijkstra import PathSolver
from wall_painting_trajectory_planner.astar import PathSolver

###############################################################################

def get_square(image, square_size):

    height, width = image.shape
    if(height > width):
        differ = height
    else:
        differ = width
    differ += 4

    # square filler
    mask = np.zeros((differ, differ), dtype = "uint8")

    x_pos = int((differ - width) / 2)
    y_pos = int((differ - height) / 2)

    # center image inside the square
    mask[y_pos: y_pos + height, x_pos: x_pos + width] = image[0: height, 0: width]

    # downscale if needed
    if differ / square_size > 1:
        mask = pyramid_reduce(mask, differ / square_size)
    else:
        mask = resize(mask, (square_size, square_size))
    return mask

def get_quaternion(vec2, vec1=[1, 0, 0]):
    """get rotation matrix between two vectors using scipy"""
    vec1 = np.reshape(vec1, (1, -1))
    vec2 = np.reshape(vec2, (1, -1))
    r = R.align_vectors(vec2, vec1)
    return r[0].as_quat()

class TrajectoryPlanner:
    def __init__(self, logger):
        self.map = None
        self.frame = None
        self.origin = None
        self.resolution = None
        self.w = None
        self.h = None
        self.cw = None
        self.ch = None
        self.cleft = None
        self.ctop = None
        self.task = None
        self.logger = logger

    def set_planning_scene(self, distance_map):
        self.map = np.reshape(distance_map.data,(distance_map.height, distance_map.width))
        self.frame = distance_map.header.frame_id
        self.origin = [distance_map.origin.x, distance_map.origin.y, distance_map.origin.z]
        self.resolution = distance_map.resolution
        self.w = distance_map.width
        self.h = distance_map.height
        self.cw = distance_map.canvas_width
        self.ch = distance_map.canvas_height
        self.cleft = distance_map.canvas_origin.x
        self.ctop = distance_map.canvas_origin.y
        self.unknown_value = distance_map.unknown_value

    def plan_task(self, input_image):
        self.task = self.get_task_from_image(input_image)

    def get_path_from_image(self, image):
        self.logger.info('processing image')

        square_size = self.cw if self.cw > self.ch else self.ch
        mask = get_square(image, square_size)

        norm = cv2.normalize(mask, None, alpha=255, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        thresh2 = cv2.threshold(norm, 75, 255, cv2.THRESH_BINARY)[1]

        rot = cv2.rotate(thresh2, cv2.ROTATE_90_COUNTERCLOCKWISE)
        flip = cv2.flip(rot, 0)
        crop = np.copy(flip)

        if self.cw > self.ch:
            delta = square_size - self.ch
            crop = np.take(flip,np.array(range(delta//2,flip.shape[0]-delta//2)),axis=1)

        if self.cw < self.ch:
            delta = square_size - self.cw
            crop = np.take(flip,np.array(range(delta//2,flip.shape[1]-delta//2)),axis=0)

        self.logger.info("------------>  here'{},{},{},{}".format(self.cleft, self.ctop, self.cw, self.ch))
        dmap = np.zeros(self.map.T.shape,int)
        self.logger.info('image: {}, - dmap: {}'.format(crop.shape, dmap.shape))
        dmap[int(self.cleft):int(self.cleft+self.cw),int(self.ctop):int(self.ctop+self.ch)] = np.copy(crop)
        # dmap[0:int(self.cw),0:int(self.ch)] = np.copy(crop)
        pts = np.where(dmap>0)
        print(pts)

        #s = 3
        s = 5
        count = []
        for idx,(px,py) in enumerate(zip(*pts)):
            assert dmap[px,py] == 255
            temp_px = np.arange(max(px-s//2,0),min(px+s//2+1,dmap.shape[1])) # width
            temp_py = np.arange(max(py-s//2,0),min(py+s//2+1,dmap.shape[0])) # height
            grid_px, grid_py = np.meshgrid(temp_px, temp_py)
            list_px = grid_px.flatten()
            list_py = grid_py.flatten()
            vals = [dmap[i,j] for i,j in zip(list_px,list_py)]
            grid = np.count_nonzero(vals)
            count.append(grid)
        assert len(count) == len(pts[0])
        sort = np.argsort(count)
        start = sort[0]
        pts = np.array(pts).T

        # TSP solver (only knows starting point)
        #adj = [np.linalg.norm(pts-p,axis=1).tolist() for p in pts]
        #solver = TSPsolver()
        #path = solver.solve(adj,start,True)
        #new_pts = pts[path].T

        # Greedy search (only knows starting point)
        #adj = [np.linalg.norm(pts-p,axis=1).tolist() for p in pts]
        #solver = PathSolver()
        #path = solver.solve(adj,start,True)
        #new_pts = pts[path].T

        # A* search (knows starting point and goal point)
        end = sort[1]
        ctn = 0
        while (abs(start-end) == 1):
            end = sort[ctn+2]
            ctn += 1
        data = np.argwhere(dmap == 0)

        cols = dmap.shape[0]
        rows = dmap.shape[1]

        start_x, start_y = pts[start]
        end_x, end_y = pts[end]

        solver = PathSolver(cols, rows, [start_x, start_y], [end_x, end_y], data)
        path = solver.solve()
        new_pts = np.array([[p.x, p.y] for p in path[::-1]])
        new_pts = np.append(new_pts,[pts[end]],axis=0).T

        #self.logger.info(new_pts)
        self.logger.info("------------>  herewwwwwwwwwwwww")
        return new_pts

    def get_task_from_image(self, input_image):
        self.logger.info('looking at image 1')
        blur = cv2.blur(input_image, (35, 35))
        thresh = cv2.threshold(blur, 250, 255, 0)[1]
        bit = cv2.bitwise_not(thresh)
        output = cv2.connectedComponentsWithStatsWithAlgorithm(bit,8,cv2.CV_16U,-1)
        (numLabels, labels, _, _) = output
        image_group = np.zeros((numLabels-1,)+input_image.shape, dtype=np.uint8)
        for l in range(1,numLabels):
            image_group[l-1][np.where(labels == l)] = 255

        print(image_group.shape)
        pts = []
        for im in image_group:
            img_in = cv2.ximgproc.thinning(im, cv2.ximgproc.THINNING_ZHANGSUEN)
            pts.extend(self.get_path_from_image(img_in).tolist())
        pts = np.array(pts)
        #print(pts.shape)
        return pts

    def get_position_at_(self,y,z):
        p = Point()
        p.z = self.origin[2]- z * self.resolution
        # p.y = self.origin[1]- y * self.resolution
        p.y = self.origin[1]+ y * self.resolution
        p.x = -self.map[z,y].item()
        return p

    def get_orientation_at_(self,y,z,n=3,verbose=False):
        temp_zs = np.arange(max(z-n//2,0),min(z+n//2+1,self.h))
        temp_ys = np.arange(max(y-n//2,0),min(y+n//2+1,self.w))
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
        # quat = get_quaternion(n,[-1, 0, 0])
        quat = get_quaternion([-1, 0, 0], [-1, 0, 0])
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

    def get_wall(self):
        wall_msg = MarkerArray()
        for i in range(int(self.w)):
            for j in range(int(self.h)):
                if self.map[j,i].item() == self.unknown_value:
                    continue
                m = Marker()
                m.id = i*self.h+j
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
                wall_msg.markers.append(m)
        return wall_msg

    def get_path(self):
        path_msg = Path()
        path_msg.header.frame_id = self.frame

        print('- Plane Estimation -')
        print('Processing Point : ', end=' ')
        for i,(y,z) in enumerate(zip(*self.task)):
            if self.map[z,y].item() == self.unknown_value:
                print('-', end=' ')
                continue
            p = PoseStamped()
            p.pose.position = self.get_position_at_(y,z)
            p.pose.orientation = self.get_orientation_at_(y,z)
            p.header.frame_id = self.frame
            path_msg.poses.append(p)
            print(i, end=' ')
        print('\n---')

        return path_msg

