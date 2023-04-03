import xml.etree.ElementTree as et
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)
from matplotlib import pyplot as plt
from PIL import Image as im
#x = 46, y = 70d
treemap = et.parse("../misc/map_example.xml")
treeroot = treemap.getroot()
# maparray = 
i = 0
j = 0
original_reso = 0.5
new_reso = 0.05
for data in treeroot[0]:
    print(data.attrib)
    width = data.attrib['width']
    print("width: ", int(width))
    height = data.attrib['height']
    print("height: ", int(height))
#initialize map array with width and height
maparray = np.zeros((int(height), int(width)))    
for data in treeroot[0].findall('grid'):
    #for each row in the map, fill the values in the map array with the values from the xml file
    for tile in data.findall('row'):
        for char in tile.text.split(' '):
            # print(char)
            if((i < int(height)) and (j < int(width))):
                #print i, j            
                maparray[i][j] = int(char)
                # print(i, j, maparray[i][j])
                j += 1
        i += 1
        j = 0
   
# print(maparray)
n = int(original_reso / new_reso)
rosmap = maparray
print("-------------------------------------")
rosmap = np.kron(rosmap, np.ones((n,n)))
for i in range(rosmap.shape[0]):
    for j in range(rosmap.shape[1]):
        if(rosmap[i][j] == 0):
            rosmap[i][j] = 255
        else:
            rosmap[i][j] = 0
rosmap = rosmap.astype(np.uint8)
# print(rosmap)
data = im.fromarray(rosmap)
data.convert('RGB')
rotated = data.rotate(90)
rotated.save('../worlds/map.pgm')

print(rosmap.shape)
#find out the center coordinates of the map through the resolution and the width and height of the map
centerx = (rosmap.shape[1] / 2) * new_reso
centery = (rosmap.shape[0] / 2) * new_reso
# originx = rosmap.shape[1]/40
# originy = rosmap.shape[0]/40
yamlFile = open('../worlds/map.yaml', 'w')
yamlFile.write('image: map.pgm' + '\n')
yamlFile.write('resolution: 0.050000' + '\n')
yamlFile.write('origin: [' + str(-centerx) + ', ' + str(-centery) + ', 0.0000]' + '\n')
yamlFile.write('negate: 0' + '\n')
yamlFile.write('occupied_thresh: 0.65' + '\n')
yamlFile.write('free_thresh: 0.196' + '\n')
yamlFile.close()
