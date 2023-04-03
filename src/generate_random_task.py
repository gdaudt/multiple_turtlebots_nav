import xml.etree.ElementTree as ET
import utils
import random
import sys

#read the xml file that describes the map, 0 being a free space and 1 being an obstacle which is modeled like this:
# <?xml version="1.0" ?>
# <root>
#     <map>
#         <width>10</width>
#         <height>10</height>
#         <grid>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 			<row>0 0 0 0 0 0 0 0 0 0</row>
# 		</grid>
#     </map>
# </root>
# then creates a new xml file, receiving as parameter the number of agents and selecting random non-repeating start and goal with the following structure:
# <?xml version="1.0" ?>
# <root>
#    <agent start_i="0" start_j="0" goal_i="7" goal_j="2"/>
#    <!-- <agent start_i="0" start_j="1" goal_i="7" goal_j="1"/> -->
#    <!-- <agent start_i="0" start_j="2" goal_i="7" goal_j="0"/> -->
# </root>

def generateRandomTask(robotNumber):
    tree = ET.parse('../misc/grid_map.xml')
    root = tree.getroot()
    width = int(root.find('map').find('width').text)
    height = int(root.find('map').find('height').text)
    grid = root.find('map').find('grid')
    rows = grid.findall('row')
    #print(rows[0].text)
    #print(len(rows))
    #print(len(rows[0].text.split(" ")))
    availablePos = []
    for i in range(0, len(rows)):
        for j in range(0, len(rows[0].text.split(" "))):
            if(rows[i].text.split(" ")[j] == '0'):
                availablePos.append((i,j))
    #print(availablePos)
    file = open('../misc/random_task.xml', 'w')
    file.write("<?xml version=\"1.0\" ?>\r")
    file.write("<root>\r")
    for i in range(0, robotNumber):
        start = availablePos.pop(random.randint(0, len(availablePos)-1))
        goal = availablePos.pop(random.randint(0, len(availablePos)-1))
        file.write("\t<agent start_i=\"" + str(start[0]) + "\" start_j=\"" + str(start[1]) + "\" goal_i=\"" + str(goal[0]) + "\" goal_j=\"" + str(goal[1]) + "\"/>\r")
    file.write("</root>")
    file.close()
    
rnumber = sys.argv[1]
generateRandomTask(int(rnumber))