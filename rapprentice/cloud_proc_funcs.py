import cloudprocpy
from rapprentice import berkeley_pr2, clouds
import cv2, numpy as np
import rapprentice

DEBUG_PLOTS = False

def extract_red(rgb, depth, T_w_k):
    """
    extract red points and downsample
    """
        
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]
    
    red_mask = ((h<10) | (h>150)) & (s > 100) & (v > 100)
    
    valid_mask = depth > 0
    
    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]
    
    z = xyz_w[:,:,2]   
    z0 = xyz_k[:,:,2]
    if DEBUG_PLOTS:
        cv2.imshow("z0",z0/z0.max())
        cv2.imshow("z",z/z.max())
        cv2.imshow("rgb", rgb)
        cv2.waitKey()
    
    height_mask = xyz_w[:,:,2] > .7 # XXXX pass in parameter
    
    
    good_mask = red_mask & height_mask & valid_mask
    if DEBUG_PLOTS:
        cv2.imshow("red",red_mask.astype('uint8')*255)
        cv2.imshow("above_table", height_mask.astype('uint8')*255)
        cv2.imshow("red and above table", good_mask.astype('uint8')*255)
        print "press enter to continue"
        cv2.waitKey()
        
        
    

    good_xyz = xyz_w[good_mask]
    
    cloud = cloudprocpy.CloudXYZ()    
    good_xyz1 = np.zeros((len(good_xyz), 4), 'float32')
    good_xyz1[:,:3] = good_xyz
    cloud.from2dArray(good_xyz1)
    
    cloud_ds = cloudprocpy.downsampleCloud(cloud, .01)
    
    return cloud_ds.to2dArray()[:,:3]
    
    



def extract_red_alphashape(cloud, robot):
    """
    extract red, get alpha shape, downsample
    """
    raise NotImplementedError
    
    # downsample cloud
    cloud_ds = cloudprocpy.downsampleCloud(cloud, .01)
    
    # transform into body frame
    xyz1_kinect = cloud_ds.to2dArray()
    xyz1_kinect[:,3] = 1
    T_w_k = get_kinect_transform(robot)
    xyz1_robot = xyz1_kinect.dot(T_w_k.T)
    
    # compute 2D alpha shape
    xyz1_robot_flat = xyz1_robot.copy()
    xyz1_robot_flat[:,2] = 0 # set z coordinates to zero
    xyz1_robot_flatalphashape = cloudprocpy.computeAlphaShape(xyz1_robot_flat)
    
    # unfortunately pcl alpha shape func throws out the indices, so we have to use nearest neighbor search
    cloud_robot_flatalphashape = cloudprocpy.CloudXYZ()
    cloud_robot_flatalphashape.from2dArray(xyz1_robot_flatalphashape)
    cloud_robot_flat = cloudprocpy.CloudXYZ()
    cloud_robot_flat.from2dArray(xyz1_robot_flat)
    alpha_inds = cloudprocpy.getNearestNeighborIndices(xyz1_robot_flatalphashape, xyz1_robot_flat)

    xyz_robot_alphashape = xyz1_robot_flatalphashape[:,:3]
    
    # put back z coordinate
    xyz_robot_alphashape[:,2] = xyz1_robot[alpha_inds,2] 

    return xyz_robot_alphashape[:,:3]
