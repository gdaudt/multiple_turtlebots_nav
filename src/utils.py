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
    x = (x / resolution) + (dimensions[1] / 2)
    y = (y / resolution) + (dimensions[0] / 2)
    return (x, y)

def getGazeboCoordinate(mapx, mapy, dimensions, resolution):
    #invert the calculation of getMapCoordinate
    x = (mapx - (dimensions[0] / 2)) * resolution
    y = (mapy - (dimensions[1] / 2)) * resolution
    return (round(x, 1), round(y, 1))
    
class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0