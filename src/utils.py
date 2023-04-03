import math

def transformCoordinateOdomToMap(x, y, map): 
    j = y / map.resolution - map.originy / map.resolution
    i = x / map.resolution - map.originx / map.resolution
    return (i, j);  
  
def transformCoordinateMapToOdom(x, y, map):
    # print(map.resolution, map.originx, map.originy)
    i = (x + map.originx / map.resolution) * map.resolution
    j = (y + map.originy / map.resolution) * map.resolution
    return (i, j)

def map_callback(data, map):
    # print(data.info.width, data.info.height, data.info.resolution)
    map.width = data.info.width
    map.height = data.info.height
    map.originx = data.info.origin.position.x
    map.originy = data.info.origin.position.y
    map.resolution = data.info.resolution
    # print(map.resolution, map.originx, map.originy)

def truncateCoordinate(x, y, resolution):
    tx = x
    ty = -y
    if(x != 0):
        tx = ((abs(x) // resolution) * resolution) * (x / abs(x))    
    if(y != 0):
        ty = ((abs(y) // resolution) * resolution) * (y / abs(y))
    # print(x, y)
    # print(round(tx, 1), round(ty, 1))
    return (round(tx, 1), round(ty, 1))

def getMapCoordinate(x, y, dimensions, resolution):
    x = (x / resolution) + (dimensions[0] / 2)
    y = (y / resolution) + (dimensions[1] / 2)
    return (x, y)

def getGazeboCoordinate(mapx, mapy, dimensions, resolution):
    #invert the calculation of getMapCoordinate
    x = (mapx - (dimensions[0] / 2)) * resolution
    y = (mapy - (dimensions[1] / 2)) * resolution
    return (round(x, 1), round(y, 1))

def planToGazeboCoordinate(px, py, radius):
    side = 4*radius/math.sqrt(2)
    ax = (px)*(side) + side/2
    ay = (py)*(side) + side/2
    return (ax, ay)
    

def gazebo_time_callback(data, time):
    time = data.header.stamp.secs + (data.header.stamp.nsecs * 1e-9)
    # print(time)
    return time
    
class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0