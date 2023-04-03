import xml.etree.ElementTree as ET
import utils
import math

def steeringAngle(gx, gy, rx, ry):
    # print("steeringAngle: ", math.degrees(math.atan2(goal[1] - ry, goal[0] - rx))+ math.radians(90)) 
    return math.atan2(gy - ry, gx - rx)

robotNumber = 3
robotName = "robot"
xpos, ypos, apos = 0, 0, 0
iterator = 0
robot_radius = 0.15
map_size = 10

tree = ET.parse('../misc/grid_task_log.xml')
root = tree.getroot()
file = open('../misc/template.txt', 'r')
template = file.read()
aux = template
robots_init = open('../misc/robots_init.txt', 'r')
header = robots_init.read()
generatedFile = open('../launch/generated_robots.launch', 'w')
generatedFile.write(header + '\n')
i = 0
for agent in root.findall('agent'): 
    i += 1
    name = robotName + str(i)
    prefix = name + "_prefix"
    aux = aux.replace('robot_prefix', prefix)
    aux = aux.replace('robotname', name)
    start_i = agent.attrib['start_i']
    start_j = agent.attrib['start_j']
    goal_i = agent.attrib['goal_i']
    goal_j = agent.attrib['goal_j']    
    (xpos, ypos) = utils.planToGazeboCoordinate(float(start_i), float(start_j), robot_radius)
    aux = aux.replace('xpos', str(xpos))
    aux = aux.replace('ypos', str(ypos))
    yawpos = steeringAngle(float(goal_i), float(goal_j), float(start_i), float(start_j))
    if (yawpos > math.pi):
        yawpos -= 2 * math.pi
    elif (yawpos < -math.pi):
        yawpos += 2 * math.pi
    aux = aux.replace('yawpos', str(yawpos))  
    generatedFile.write(aux + '\n')
    aux = template
generatedFile.write("</launch>")
file.close()
robots_init.close()
generatedFile.close()