import numpy as np

cx = 320-.5;
cy = 240-.5;



def depth_to_xyz(depth,f=535.):
    x,y = np.meshgrid(np.arange(640), np.arange(480))
    assert depth.shape == (480, 640)
    XYZ = np.empty((480,640,3))
    Z = XYZ[:,:,2] = depth / 1000. # convert mm -> meters
    XYZ[:,:,0] = (x - cx)*(Z/f)
    XYZ[:,:,1] = (y - cy)*(Z/f)

    return XYZ
    
    