"""LaserScan wire validation and explicit ROS/DROW frame conversion."""
import math
import numpy as np


def quaternion_rotation(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-9:
        raise ValueError('Invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def decode_scan(header,payload):
    if header.get('schema')!='laser_scan/v1':
        raise ValueError('Expected laser_scan/v1; camera packets are not accepted')
    n=header.get('count')
    if type(n) is not int or not 32<=n<=4096 or len(payload)!=4*n:
        raise ValueError('Invalid scan length')
    if not all(math.isfinite(float(header[key])) for key in ['stamp','odom_stamp']):
        raise ValueError('Non-finite timestamp')
    if abs(float(header['stamp'])-float(header['odom_stamp']))>.035001:
        raise ValueError('Scan and odometry timestamps exceed 35ms tolerance')
    angle_min=float(header['angle_min']);increment=float(header['angle_increment'])
    rmin,rmax=float(header['range_min']),float(header['range_max'])
    if not all(math.isfinite(v) for v in [angle_min,increment,rmin,rmax]):
        raise ValueError('Non-finite scan geometry')
    span=abs(increment)*(n-1)
    if increment==0 or not .1<=span<=2*math.pi+.01 or not 0<=rmin<rmax<=100:
        raise ValueError('Invalid angular or range bounds')
    for key,size in [('odom_position',3),('odom_quaternion',4),('lidar_position_base',3),('lidar_quaternion_base',4)]:
        value=np.asarray(header[key],dtype=float)
        if value.shape!=(size,) or not np.isfinite(value).all():raise ValueError('Invalid '+key)
    quaternion_rotation(header['odom_quaternion']);quaternion_rotation(header['lidar_quaternion_base'])
    return np.frombuffer(payload,dtype='<f4').copy()


def prepare_drow_scan(ranges,angle_min,increment,range_min,range_max):
    """DROW reads clockwise angles; ROS LaserScan usually runs counter-clockwise."""
    scan=np.asarray(ranges,dtype=np.float32).copy()
    invalid=(~np.isfinite(scan))|(scan<=range_min)|(scan>range_max)
    scan[invalid]=29.99
    if increment>0:scan=scan[::-1].copy()
    span=abs(increment)*(len(scan)-1)
    center=angle_min+increment*(len(scan)-1)/2
    return scan,math.degrees(span),center


def drow_xy_to_ros(points,center_angle):
    p=np.asarray(points,dtype=float).reshape(-1,2)
    c,s=math.cos(center_angle),math.sin(center_angle)
    # DROW y-right -> ROS y-left, then rotate the scan midpoint to its true bearing.
    return p@np.array([[c,s],[s,-c]])


def lidar_points_to_odom(points,header):
    p=np.asarray(points,dtype=float).reshape(-1,2)
    xyz=np.column_stack((p,np.zeros(len(p))))
    mount=quaternion_rotation(header['lidar_quaternion_base'])
    robot=quaternion_rotation(header['odom_quaternion'])
    return (xyz@mount.T+np.asarray(header['lidar_position_base']))@robot.T+np.asarray(header['odom_position'])
