import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
#import setup_path 
import airsim
import pprint
<<<<<<< HEAD
from airsim.types import *
import time
import math
=======
import time
>>>>>>> 5f17f2333632b3dd1bbe1df9890ec608dda0c6e3

class SurfelMap:
    def __init__(self):
        self.surfels = []
    
    def addSurfel(self, surfel):
        self.surfels.append(surfel)


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
            #print("\tReading: time_stamp: %d number_of_points: %d" % (lidarData.time_stamp, len(points)))
            #print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
            #print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
        return points

    def prepare(self):
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
    
        state = self.client.getMultirotorState()
        s = pprint.pformat(state)
        print("state: %s" % s)

        airsim.wait_key('Press any key to takeoff')

    def convertRoute(self, starting, route):
        newRoute = [self.shiftz([x1 - x2 for (x1, x2) in zip(p, starting)]) for p in route]
        return newRoute

    def shiftz(self, vec):
        vec[2] = -vec[2]
        return airsim.Vector3r(vec[0], vec[1], vec[2])

    #Starting position is map specific. To get a relative path we measure coordinates in the editor
    #map overview and set as waypoints. Then we subtract the starting point and invert the relative z coordinate
    #to obtain parameters to send to move async.
    #this is the starting position, in airsim coordinates, in the Blocks environment

    def starting(self):
        return [56.90, -1, 2.02]

    def toEuler(self, quat):
        #Equations from wikipedia https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        #Roll
        sinr_cosp = 2 * (quat.w_val * quat.x_val + quat.y_val * quat.z_val)
        cosr_cosp = 1 - 2 * (quat.x_val * quat.x_val + quat.y_val * quat.y_val)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        #Pitch
        sinp = 2 * (quat.w_val * quat.y_val - quat.z_val * quat.x_val)
        if (math.fabs(sinp) >= 1):
            pitch = math.copysign(np.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        #Yaw
        siny_cosp = 2 * (quat.w_val * quat.z_val + quat.x_val * quat.y_val)
        cosy_cosp = 1 - 2 * (quat.y_val * quat.y_val + quat.z_val * quat.z_val)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]
    
    def set_strafe_travel_route(self, route, speed):
        return self.client.moveOnPathAsync(route, speed)
    
    def set_travel_route(self, route, speed):
        return self.client.moveOnPathAsync(route, speed, drivetrain = DrivetrainType.ForwardOnly, yaw_mode = YawMode(False))

    def travel_core(self, routeFun, speed, timeout, travel_route_fun):
        #Coordinates are in NED system North East Down (X Y Z).
        #Starting position is always (0 0 0) while Unreal 
        #coordinates are based on the environment and Z is upwards.
        # so here we define our route based on inspection in the unreal editor
        # and then translate it to unreal system. it will be translated back 
        # during the api calls.
        #1000 per block!
        starting = self.starting()
        route = routeFun()
        route = self.convertRoute(starting, route)
        self.pathCompleted = False
        self.client.takeoffAsync().join()
        currsecs = time.time()
        pathmovement = travel_route_fun(route, speed)
        
        newsecs = time.time()
        scancount = 0
        scanarray = []
        posearray = []
        orientationarray = []
        while newsecs - currsecs < timeout:
            scan = self.read_lidarData()
            state = self.client.getMultirotorState()
            position = state.kinematics_estimated.position
            orientation = state.kinematics_estimated.orientation
            rpy = self.toEuler(orientation)
            scanarray.append(scan)
            posearray.append(position)
            orientationarray.append(rpy)

            scancount = scancount + 1
            time.sleep(0)
            newsecs = time.time()
        print(scancount)

        pathmovement.join()
        self.client.hoverAsync().join() # And done.
        return scanarray, posearray, orientationarray

    #Travel along the route, with independent nose direction. Causes less yawing to get there.
    def travel_strafe_route(self, routeFun, speed, timeout):
        return self.travel_core(routeFun, speed, timeout, self.set_strafe_travel_route)

    #Travel along the route, with the front of the drone in the travel direction. Causes more yawing to get there.
    def travel_route(self, routeFun, speed, timeout):
        return self.travel_core(routeFun, speed, timeout, self.set_travel_route)

    #Analyzed route, circles around some pillars.
    def getFirstRoute(self):
        route = [[75, 27.5, 7], [105, 27.5, 7], [105, 82.5, 7], [55, 82.5, 7], [57, -1, 7]]
        return route

    #Test route, moves back and forth, to verify function accuracy without
    #having to go through a long route. Interrupting the unreal editor in the middle of a long route causes a hang.
    def getShortRoute(self):
        starting = self.starting()
        route = [[75, 27.5, 7], starting]
        return route


def save_lidarData(points, index=0):
    if points.size != 0:
        np.save('testLidar' + str(index) + '.npy', points)    # .npy extension is added if not given

<<<<<<< HEAD
def load_lidarData(filename='testLidar.npy'):
    points = np.load(filename)    # .npy extension is added if not given
=======
def load_lidarData(index=0):
    points = np.load('testLidar' + str(index) + '.npy')    # .npy extension is added if not given
>>>>>>> 5f17f2333632b3dd1bbe1df9890ec608dda0c6e3
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
        u = int(ucoord)
        vcoord = (1-(np.arcsin(zval/dist) + fovup) / fov) * h
        v = int(vcoord)
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

<<<<<<< HEAD
def main():
#Convert poses
    poses = np.load('scan_round_3_pose.npy')    
    newposes = [[p.x_val, p.y_val, p.z_val] for p in poses]
    np.save('scan_round_3_poses.npy', newposes)

    #Convert points:
    #counts = np.load('scan_round_3_counts.npy')
    #    counter = 0
   # for pose in poses:
     #   filespecifier = '{0:05d}'.format(counter)
     #   np.save('scan/scan_round_3_' + filespecifier, scan)
     #   counter = counter + 1


def fn_2():
    drone = LidarDrone()
    #lets just ignore the fact that it's bouncing

    drone.prepare()
    scanarray, posearray, orientationarray = drone.travel_strafe_route(drone.getFirstRoute, 3, 120)
    np.save('scan_round_strafe_3.npy', np.asarray(scanarray))
    np.save('scan_round_strafe_3.npy1', np.asarray(posearray))
    np.save('scan_round_strafe_3.npy2', np.asarray(orientationarray))

    return
   # for i in range(4):
   #     read_lidarData()

    #save_lidarData(points)
    #if points.size == 0:
    #    print('no points')
    #    return
    #This is our point cloud, currently T=1
    P = load_lidarData()
    #Aim is to estimate pose T_W_CT
    print('Preprocessing')

    fovup = 30
    fovdown = 0
=======
def gatherLidarData():
    drone = LidarDrone()
    points = drone.prepare_and_read_lidarData()
    save_lidarData(points)
    points = drone.read_lidarData()
    save_lidarData(points, 1)

def clamp(x, u, l):
    return np.nanmin([u, np.nanmax([x, l])])

def main():
   # gatherLidarData()
    surfelMap = SurfelMap()

    fovup = 15
    fovdown = -15
>>>>>>> 5f17f2333632b3dd1bbe1df9890ec608dda0c6e3
    fov = fovup - fovdown
    fovhor = 360

    #Size of maps
    w = 50
    h = 5
    p = np.max([w / fovhor, h / fov]) # pixel size
    points1 = load_lidarData()
    #So lets imagine points1 are for t=0

<<<<<<< HEAD
    #Create uvmap and normal map, these are for the frame
    V_D = create_uv_map(w, h, fovup, fov, P)
    N_D = create_n_map(w, h, V_D)

    #now create uvmap and normal map for the model
    V_M = create_empty_map(w, h)
    N_M = create_empty_map(w, h)
=======
    points2 = load_lidarData(1)
    #and points2 are for t=1

    #that gives us:
>>>>>>> 5f17f2333632b3dd1bbe1df9890ec608dda0c6e3

    print('1. Preprocessing')
    #Create uvmap and normal map, these are for the frame
    V_D = create_uv_map(w, h, fovup, fov, points1)
    N_D = create_n_map(w, h, V_D)

    print('2. Map Representation')
    #There is none, obviously. so it's empty.
    V_M = create_empty_map(w, h)
    N_M = create_empty_map(w, h)
    #At the last pose estimate TW_C0
    print('3. Odometry Estimation')
    #easy
    T_W_C0 = np.eye(4)
<<<<<<< HEAD
    #Now we need to estimate T_C0_C1 (?)
    #Which is used to update T_W_C1
    # T_W_C1 = T_W_C0 * T_C0_C1
    #Use ICP to align V_D with V_M
    #Should yield T_C0_C1
    M = SurfelMap()

    img_n = create_image(h, w, N_D)
    img_uv = create_image(h, w, V_D)
=======
    print('4. Map Update')
    #Initialize surfels for previously unseen areas
    #Given the current pose T_W_C0
    #integrate the points inside V_D into the surfel map
    for x in range(w):
        for y in range(h):
            vs = V_D[x, y, :]
            ns = N_D[x, y, :]
            rs = np.sqrt(2) * np.linalg.norm(vs) * p / clamp(-np.sum(vs * ns) / np.linalg.norm(vs), 0.5, 1.0)
            #ignore crossovers with existing surfels.
            surfel = Surfel(vs, ns, rs, time.time())
            surfelMap.addSurfel(surfel)

            #Project vs to u, v: should actually be x, y
            #xval = vs[0]
            #yval = vs[1]
            #zval = vs[2]
            #dist = dist_fun(pt)
            #ucoord = 0.5*(1-np.arctan2(yval, xval)/np.pi) * w
            #u = int(ucoord)
            #vcoord = (1-(np.arcsin(zval/dist) + fovup) / fov) * h
            #v = int(vcoord)

            #V_D = np.zeros([w, h, 4])

    print('5. Loop Closure Detection')
    #None in phase 1
    print('6. Loop Closure Verification')
    #None in phase 1
    print('7. Pose graph optimization')
    #None in phase 1

    #Now we enter time step 2, t = 1
    #snap we need time stamps from the actual thing...
    print('1. Preprocessing')
    V_D = create_uv_map(w, h, fovup, fov, points2)
    N_D = create_n_map(w, h, V_D)
    print('2. Map Representation')

    print('3. Odometry Estimation')
    print('4. Map Update')
    print('5. Loop Closure Detection')
    print('6. Loop Closure Verification')
    print('7. Pose graph optimization')


    #img_n = create_image(h, w, N_D)
    #img_uv = create_image(h, w, V_D)
>>>>>>> 5f17f2333632b3dd1bbe1df9890ec608dda0c6e3

  #  img = create_image(h, w, uvmap)
    plt.imshow(img_n)
    plt.figure()
    plt.imshow(img_uv)
    plt.show()

if __name__ == "__main__":
    main()
