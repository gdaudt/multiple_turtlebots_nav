import os
import sys
import fileinput
import argparse

msg = "Generates files for multi-robot simulation. Arguments are listed in the -h command."

def parseCoordinate(coordinate):
    c = coordinate.split(',')
    c[0] = int(c[0])
    c[1] = int(c[1])
    return c

robotNumber = 3
robotName = "robot"
xpos, ypos, apos = 0, 0, 0
iterator = 0

parser = argparse.ArgumentParser(msg)
parser.add_argument("--robot", "-r", help="Base name for robots. Files will be created with concatenated name and number of the robot. Default is robot", default="robot") 
parser.add_argument("--number", "-n", help="Number of robots desired. Defaults as 1", type=int, default=1)
parser.add_argument("--positions", "-p", nargs=argparse.REMAINDER, help="Coordinates for the robots. Write as x,y and separated by spaces in order of the robots."
                    + "If less positions are specified, robots will be spawned by a 1m increment in the x axis after the last position. Default is this behavior at 0,0", default=['0,0'])
args = parser.parse_args()
robotName = args.robot
robotNumber = args.number
# generate the launch file for the robots
file = open('../misc/template.txt', 'r')
template = file.read()
aux = template
robots_init = open('../misc/robots_init.txt', 'r')
header = robots_init.read()
generatedFile = open('../launch/generated_robots.launch', 'w')
# write the initial header file
generatedFile.write(header + '\n')
# append the template files
for i in range(1, robotNumber+1):    
    robotName = robotName + str(i)
    prefix = robotName + "_prefix"
    aux = aux.replace('robot_prefix', prefix)
    aux = aux.replace('robotname', robotName)
    if(len(args.positions) >= i):
        coord = parseCoordinate(args.positions[i-1]) 
        xpos = coord[0]   
        ypos = coord[1]
    else:
        xpos = xpos+1
    aux = aux.replace('xpos', str(xpos))
    aux = aux.replace('ypos', str(ypos))
    aux = aux.replace('apos', str(apos))
    generatedFile.write(aux + '\n')
    aux = template
    robotName = args.robot
generatedFile.write("</launch>")
file.close()
robots_init.close()
generatedFile.close()

# generate the rviz file for visualization
# rviz_block1 -> rviz_TFtree 1(loop), 2, 3(loop) -> rviz_block2 -> rviz_group(loop) -> rviz_block3
rviz_file = open('../navigation/navigation.rviz', 'w')
rb1 = open('../misc/rviz_block1.rviz', 'r')
rvizblock1 = rb1.read()
aux = rvizblock1.replace('robot', robotName)
rviz_file.write(aux + '\n')
rb1.close()
rtf1 = open('../misc/rviz_TFtree1.rviz', 'r')
rviz_tf1 = rtf1.read()
aux = rviz_tf1
for i in range(1, robotNumber+1):
    robotName = robotName + str(i)
    aux = aux.replace('robot', robotName)
    rviz_file.write(aux + '\n')
    aux = rviz_tf1
    robotName = args.robot
rtf1.close()
rtf2 = open('../misc/rviz_TFtree2.rviz', 'r')
rviz_tf2 = rtf2.read()
rviz_file.write(rviz_tf2 + '\n')    
rtf2.close()
rtf3 = open('../misc/rviz_TFtree3.rviz', 'r')
rviz_tf3 = rtf3.read()
aux = rviz_tf3
for i in range(1, robotNumber+1):
    robotName = robotName + str(i)
    aux = aux.replace('robot', robotName)
    rviz_file.write(aux + '\n')
    aux = rviz_tf3
    robotName = args.robot
rtf3.close()
rb2 = open('../misc/rviz_block2.rviz', 'r')
rvizblock2 = rb2.read()
rviz_file.write(rvizblock2 + '\n')
rb2.close()
rg = open('../misc/rviz_group.rviz', 'r')
rvizgroup = rg.read()
aux = rvizgroup
for i in range(1, robotNumber+1):
    robotName = robotName + str(i)
    aux = aux.replace('robot', robotName)
    aux = aux.replace(robotName + '_description', 'robot_description')
    rviz_file.write(aux + '\n')
    aux = rvizgroup
    robotName = args.robot
rg.close()
rb3 = open('../misc/rviz_block3.rviz', 'r')
rvizblock3 = rb3.read()
rviz_file.write(rvizblock3 + '\n')
rb3.close()
rviz_file.close()


    
    
    
    

