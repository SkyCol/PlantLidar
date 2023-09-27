import open3d as o3d
import numpy as np

def PointInTriangle(pt, v1, v2, v3):
    # judge whether or not pt is in the triangle v1v2v3
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
    def PointInAABB(pt, c1, c2):
        return c2[0] <= pt[0] <= c1[0] and \
        c2[1] <= pt[1] <= c1[1]
    b1 = sign(pt, v1, v2) <= 0
    b2 = sign(pt, v2, v3) <= 0
    b3 = sign(pt, v3, v1) <= 0
    return ((b1 == b2) and (b2 == b3)) and \
        PointInAABB(pt, list(map(max, v1, v2, v3)), list(map(min, v1, v2, v3)))

def compute_plane(pointa,pointb,pointc):
    # compute plane equation given three points
    x1,x2,x3 = pointa[0],pointb[0],pointc[0]
    y1,y2,y3 = pointa[1],pointb[1],pointc[1]
    z1,z2,z3 = pointa[2],pointb[2],pointc[2]
    A = (y2 - y1)*(z3 - z1) - (z2 -z1)*(y3 - y1)
    B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1)
    C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)
    D = -(A * x1 + B * y1 + C * z1)
    return A,B,C,D

def paint_color_by_height(pcd):
    xyz = np.asarray(pcd.points)
    colors = np.zeros([xyz.shape[0], 3])
    z_max = np.max(xyz[:, 2])
    z_min = np.min(xyz[:, 2])
    delta_c = abs(z_max - z_min) / (255 * 2)
    for j in range(xyz.shape[0]):
        color_n = (xyz[j, 2] - z_min) / delta_c
        if color_n <= 255:
            colors[j, :] = [0, 1 - color_n / 255, 1]
        else:
            colors[j, :] = [(color_n - 255) / 255, 0, 1]
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def get_angle_vector(a, b, degrees=False):
    """"
    calculate the included angle between vectors
    Parameters:
        a,b -- two vectors
        degrees：if True, return as angle system. if False, return as radian system
    """
    cos_a = np.inner(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    if cos_a < -1:
        cos_a = np.array([-1])
    if cos_a > 1:
        cos_a = np.array([1])
    rad = np.arccos(cos_a)  
    deg = np.rad2deg(rad)  
    angle = deg if degrees else rad
    return angle

def get_rotationMatrix_from_vectors(u, v):
    """
    create a rotation matrix from a 3D vector 'u' to a 3D vector 'v'
    Parameters:
        u -- orign vector as np.array(1,3)
        v -- destiny vector as np.array(1,3)
    Returns:
        R -- a rotation matrix as np.array(3,3)
    """
    w = np.cross(u, v)

    c = np.dot(u, v)
    s = np.linalg.norm(w)

    ssm = np.asarray([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    R = np.eye(3) + ssm + ssm.dot(ssm) * ((1 - c) / (s ** 2))
    return R

def euclidean_distance_matrix(coords):
    # compute distance matrix of each area in the coords array
    n = len(coords)
    dist_matrix = np.sqrt(np.sum((coords[:,np.newaxis,:] - coords[np.newaxis,:,:])**2, axis=-1))
    return dist_matrix

def compute_distance(pointa,pointb):
    # compute distance between two points
    return np.sqrt(np.sum(np.square(pointa-pointb)))

def compute_distance_array(array1, array2):
    # compute a distance matrix between two numpy array
    array1,array2 = np.array(array1),np.array(array2)
    m, n = len(array1), len(array2)
    array1 = np.repeat(array1, n, axis=0)
    array1 = np.reshape(array1, (m, n, -1))
    distance_array = np.sqrt(np.sum((array1 - array2)**2, axis=2))
    return distance_array

def bfs_lmf(start, matrix,res,radius):
    """
    local maximum filter on chm
    use bfs
    """ 
    m = matrix.shape[0]
    n = matrix.shape[1]
    queue = []
    queue.append(start)
    vis = np.zeros((m+1,n+1))
    vis[start[0]][start[1]] = 1
    dires = [[0, 1], [0, -1], [1, 0], [-1, 0],[-1,1],[1,1],[1,-1],[-1,-1]]
    while queue:
        x, y = queue.pop(0)
        for dx, dy in dires:
            nx, ny = x + dx, y + dy
            if 0 <= nx < m and 0 <= ny < n and not vis[nx][ny]:
                queue.append([nx, ny])
                vis[nx][ny] = 1
                dis = compute_distance([nx,ny],start)
                if dis*res>radius:
                    break
                if matrix[nx][ny]>=matrix[start[0]][start[1]]:
                    return 0
    return 1

# import OpenGL.GL as gl
# def visualize_point_cloud_with_GPS(pcd):
#     points = np.asarray(pcd.points)
#     # 定义回调函数
#     def draw_point_cloud(vis):
#         # 获取窗口和视口大小
#         width, height = 640,480
#         viewport = np.zeros(4, dtype=np.int32)
#         gl.glGetIntegerv(gl.GL_VIEWPORT, viewport)

#         # 设置点云坐标数据
#         gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
#         gl.glVertexPointer(3, gl.GL_DOUBLE, 0, points)

#         # 绘制点云
#         gl.glPointSize(1.0)
#         gl.glDrawArrays(gl.GL_POINTS, 0, len(points))

#         # 关闭点坐标数据
#         gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

#     # 创建 Open3D 窗口
#     vis = o3d.visualization.Visualizer()
#     vis.create_window(width=640, height=480)

#     # 设置回调函数并运行窗口
#     vis.register_animation_callback(draw_point_cloud)
#     vis.run()
#     vis.poll_events()

#     # 关闭窗口
#     vis.destroy_window()

class raster():
    def __init__(self,rast,res,extent):
        self.rast = rast
        self.res = res
        self.xmin = extent[0]
        self.xmax = extent[1]
        self.ymin = extent[2]
        self.ymax = extent[3]
    def print(self):
        print('--')
        print('a raster object')
        print('res:'+str(self.res))
        print('xmin:'+str(self.xmin),'xmax:'+str(self.xmax),'ymin:'+str(self.ymin),'ymax:'+str(self.ymax))
        print('--')

class Point():
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
    