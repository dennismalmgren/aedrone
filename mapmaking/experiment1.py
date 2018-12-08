import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
#import setup_path 
import airsim
import pprint

class SurfelMap:
    def __init__(self):
        surfels = []

class Surfel:
    def __init__(self, pos, normal, radius, timestamp):
        self.pos = pos
        self.normal = normal
        self.radius = radius
        self.creation_timestamp = timestamp
        self.update_timestamp = timestamp
        self.stability = 1

    def update(self, pos, normal, radius, timestamp):
        self.pos = pos
        self.normal = normal
        self.radius = radius
        self.update_timestamp = timestamp


class LidarDrone:
    def __init__(self):
        self.client = ''

    def parse_lidarData(self, data):
        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
        return points

    def read_lidarData(self):
        lidarData = self.client.getLidarData();
        if (len(lidarData.point_cloud) < 3):
            print("\tNo points received from Lidar data")
            points = np.array([])
        else:
            points = self.parse_lidarData(lidarData)
            print("\tReading: time_stamp: %d number_of_points: %d" % (lidarData.time_stamp, len(points)))
            print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
            print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
        return points

    def prepare_and_read_lidarData(self):
    
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
    
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()
        print("state: %s" % pprint.pformat(state))

        points = self.read_lidarData()
        return points

def save_lidarData(points):
    if points.size != 0:
        np.save('testLidar.npy', points)    # .npy extension is added if not given

def load_lidarData():
    points = np.load('testLidar.npy')    # .npy extension is added if not given
    return points

def dist_fun(point):
    return np.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)

def create_uv_map(w, h, fovup, fov, points):
    uvmap = np.zeros([w, h, 4])
    for xyz in enumerate(points):
        pt = xyz[1]
        xval = pt[0]
        yval = pt[1]
        zval = pt[2]
        dist = dist_fun(pt)
        ucoord = 0.5*(1-np.arctan2(yval, xval)/np.pi) * w
        u = int(round(ucoord))
        vcoord = (1-(np.arcsin(zval/dist) + fovup) / fov) * h
        v = int(round(vcoord))
        uvmap[u, v, 0:3] = pt
        uvmap[u, v, 3] = 1
    return uvmap

def create_image(h, w, image_map):
    #map to 0-255
    rmax = np.max(image_map[:, :, 0])
    rmin = np.min(image_map[:, :, 0])
    gmax = np.max(image_map[:, :, 1])
    gmin = np.min(image_map[:, :, 1])
    bmax = np.max(image_map[:, :, 2])
    bmin = np.max(image_map[:, :, 2])

    epsilon = 1e-6
    rspan = rmax - rmin + epsilon
    gspan = gmax - gmin + epsilon
    bspan = bmax - bmin + epsilon

    rgb_array = np.zeros([h, w, 3], np.uint8)

    for x in range(w):
        for y in range(h):
            if (image_map[x, y, 3] == 1):
                rgb_array[y, x, 0] = np.clip((image_map[x, y, 0] - rmin) / rspan * 215 + 20, 0, 254)
                rgb_array[y, x, 1] = np.clip((image_map[x, y, 1] - gmin) / gspan * 215 + 20, 0, 254)
                rgb_array[y, x, 2] = np.clip((image_map[x, y, 2] - bmin) / bspan * 215 + 20, 0, 254)

    img = Image.fromarray(rgb_array)
    return img

def create_n_map(w, h, uvmap):
    #todo: wrap image coordinates in the x-direction.
    nmap = np.zeros([w, h, 4])
    for x in range(w-1):
        for y in range(h-1):
            if (uvmap[x, y, 3] == 1 and uvmap[x + 1, y, 3] == 1 and uvmap[x, y + 1, 3] == 1):
                nmap[x, y, 0:3] = np.cross(uvmap[x+1, y, 0:3] - uvmap[x, y, 0:3]
                        , uvmap[x, y+1, 0:3] - uvmap[x, y,0:3])
                nmap[x, y, 3] = 1
    return nmap

def create_empty_map(w, h):
    emap = np.zeros([w, h, 4])
    return emap

def main():
    #drone = LidarDrone()

    #points = drone.prepare_and_read_lidarData()
   # for i in range(4):
   #     read_lidarData()

    #save_lidarData(points)
    #if points.size == 0:
    #    print('no points')
    #    return

    points = load_lidarData()

    print('Preprocessing')
    fovup = 30
    fovdown = 0
    fov = fovup - fovdown
    #Size of maps
    w = 50
    h = 5

    #Create uvmap and normal map, these are for the frame
    uvmap = create_uv_map(w, h, fovup, fov, points)
    nmap = create_n_map(w, h, uvmap)

    #now create uvmap and normal map for the model
    uvmap_m = create_empty_map(w, h)
    nmap_m = create_empty_map(w, h)

    #Create image
    #lets do greyscale
    T_W_C0 = np.eye(4)
    #Now we need TC0_C1

    img_n = create_image(h, w, nmap)
    img_uv = create_image(h, w, uvmap)

  #  img = create_image(h, w, uvmap)
    plt.imshow(img_n)
    plt.figure()
    plt.imshow(img_uv)
    plt.show()

if __name__ == "__main__":
    main()
