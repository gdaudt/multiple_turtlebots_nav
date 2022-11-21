import rospy
import math
import actionlib
import numpy as np
import time
from enum import Enum

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid


class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0
        self.publisher = 0
        self.index = 0
        self.goal = (0.0,0.0)
        self.path = 0
        self.name = "robot"
        self.timer = 0
        self.moving = False
        self.rotation_init = True
        self.rotation_angle = 0

class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0

class Movement(Enum):
    kUp = 0
    kRight = 1
    kLeft = 2
    kDown = 3  

r1 = Robot()
r2 = Robot()
r3 = Robot()
map = Map()
path1 = [Movement.kUp, Movement.kRight, Movement.kUp, Movement.kLeft, Movement.kUp, Movement.kRight, Movement.kUp]
path2 = [Movement.kUp, Movement.kRight, Movement.kUp, Movement.kLeft, Movement.kUp, Movement.kRight, Movement.kUp]
path3 = [Movement.kUp, Movement.kRight, Movement.kUp, Movement.kLeft, Movement.kUp, Movement.kRight, Movement.kUp]
threshold = 0.3
setup = True
flag = False

def euclidianDistance(goal, rx, ry):
    return math.sqrt(pow((goal[0] - rx), 2) + pow((goal[1] - ry), 2))

def linearVel(goal, rx, ry, constant=0.4):
    return constant * euclidianDistance(goal, rx, ry)

def steeringAngle(goal, rx, ry):
    # print("steeringAngle: ", math.degrees(math.atan2(goal[0] - rx, goal[1] - ry))+ math.radians(90)) 
    return math.atan2(goal[1] - ry, goal[0] - rx)

def angularVel(goal, rx, ry, robotYaw, constant=2):
    # print("vel in degrees: ", math.degrees(constant * steeringAngle(goal, rx, ry) - robotYaw))
    # print("yaw in degrees: ", math.degrees(robotYaw))
    return constant * (steeringAngle(goal, rx, ry) - robotYaw)

def transformCoordinateOdomToMap(x, y): 
    j = y / map.resolution - map.originy / map.resolution
    i = x / map.resolution - map.originx / map.resolution
    return (i, j);  
  
def transformCoordinateMapToOdom(x, y):
    global map
    print(map.resolution, map.originx, map.originy)
    i = (x + map.originx / map.resolution) * map.resolution
    j = (y + map.originy / map.resolution) * map.resolution
    return (i, j)  

def setupRobots():
    global r1, r2, r3
    r1.publisher = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
    r2.publisher = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)
    r3.publisher = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=1)
    r1.goal = path1[r1.index]
    r2.goal = path2[r2.index]
    r3.goal = path3[r3.index]
    r1.path = path1
    r2.path = path2
    r3.path = path3
    r1.name = "robot1"
    r2.name = "robot2"
    r3.name = "robot3"

def moveRobot(robot):        
    # print("angle difference: ", math.degrees(steeringAngle(robot.goal, robot.x, robot.y)))
    # difference = math.degrees(abs(robot.yaw - steeringAngle(robot.goal, robot.x, robot.y)))
    # print("difference: ", difference)
    if(robot.path[robot.index] == Movement.kUp):
        if(robot.moving == False):
            robot.moving = True
            robot.timer = time.time() + 2.5
        if(moveForward(robot)):
            speed = Twist()
            speed.linear.x = 0
            robot.publisher.publish(speed)
            if robot.index < len(robot.path) - 1:
                rospy.loginfo(robot.name + " reached point " + str(robot.index))
                robot.index += 1
                robot.moving = False    
    elif(robot.path[robot.index] == Movement.kRight):
        if(rotate(robot, math.radians(90))):
            speed = Twist()
            speed.angular.z = 0
            robot.publisher.publish(speed)
            if robot.index < len(robot.path) - 1:
                rospy.loginfo(robot.name + " reached point " + str(robot.index))
                robot.index += 1     
                robot.rotation_init = True       
    elif(robot.path[robot.index] == Movement.kLeft):
        if(rotate(robot, math.radians(-90))):
            speed = Twist()
            speed.angular.z = 0
            robot.publisher.publish(speed)
            if robot.index < len(robot.path) - 1:
                rospy.loginfo(robot.name + " reached point " + str(robot.index))
                robot.index += 1              
                robot.rotation_init = True  

def moveForward(robot):
    speed = Twist()
    speed.linear.x = 0.2
    speed.linear.y = 0
    robot.publisher.publish(speed)
    if(robot.timer < time.time()):
        robot.moving = False
        return True
    else:
        return False

    
def rotate(robot, angle):
    kP = 0.8
    if(robot.rotation_init):
        robot.rotation_angle = robot.yaw + angle
        robot.rotation_init = False
    speed = Twist()
    speed.angular.z = kP * (robot.rotation_angle - robot.yaw)
    robot.publisher.publish(speed)
    if(robot.rotation_angle - robot.yaw < 0.03 and robot.rotation_angle - robot.yaw > -0.03):
        return True
    else:
        return False
    
# compares every robots current pose to the goal pose
# if the distance between the robot and the goal is less than a threshold, the robot is considered reached
# then it iterates the global index of the robot
def checkRobotGoalReached(robot):    
    speed = Twist()
    speed.linear.x = 0
    speed.angular.z = 0
    if euclidianDistance(robot.goal, robot.x, robot.y) < threshold:
        if robot.index < len(robot.path) - 1:
            rospy.loginfo(robot.name + " reached point " + str(robot.index))
            robot.index += 1            
        else:            
            robot.publisher.publish(speed)
            rospy.loginfo(robot.name + " reached goal")  
        
def cbpose1(data):
    global r1
    r1.x = data.pose.pose.position.x
    r1.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    r1.yaw = yaw
    
def cbpose2(data):
    global r2
    r2.x = data.pose.pose.position.x
    r2.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    r2.yaw = yaw
    
def cbpose3(data):
    global r3
    r3.x = data.pose.pose.position.x
    r3.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    r3.yaw = yaw

def map_callback(data):
    global map
    # print(data.info.width, data.info.height, data.info.resolution)
    map.width = data.info.width
    map.height = data.info.height
    map.originx = data.info.origin.position.x
    map.originy = data.info.origin.position.y
    map.resolution = 0.05
    
if __name__ == '__main__':
    rospy.init_node('path_demo')
    rate = rospy.Rate(10)    
    while not rospy.is_shutdown():
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe 
        map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)       
        sub1 = rospy.Subscriber('/robot1/odom', Odometry, cbpose1)
        sub2 = rospy.Subscriber('/robot2/odom', Odometry, cbpose2)
        sub3 = rospy.Subscriber('/robot3/odom', Odometry, cbpose3)        
        if setup:
            # path_demo()
            setupRobots()
            setup = False
        if(map.resolution != 0):
            moveRobot(r1)
            moveRobot(r2)
            moveRobot(r3)   
        rate.sleep()
    
    