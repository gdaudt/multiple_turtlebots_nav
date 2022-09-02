import rospy
import math
import actionlib
import numpy as np

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
    
    def stop(self):
        self.publisher.publish(Twist())

class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0

# robot1Pose = 0
# robot2Pose = 0
# robot3Pose = 0
# r1yaw, r2yaw, r3yaw = 0, 0, 0
# r1index, r2index, r3index = 0, 0, 0 
r1 = Robot()
r2 = Robot()
r3 = Robot()
robots = [r1, r2, r3]
map = Map()
path1 = np.array([[2.000000,-1.600000],[2.000000,-2.000000],[2.000000,-2.400000],[2.000000,-2.800000],[2.000000,-3.200000],[2.000000,-3.600000],[2.000000,-4.000000],[2.000000,-4.400000],[1.600000,-4.400000],[1.200000,-4.400000],[1.200000,-4.800000],[0.800000,-4.800000],[0.400000,-4.800000],[0.000000,-4.800000],[-0.400000,-4.800000],[-0.800000,-4.800000],[-1.200000,-4.800000],[-1.600000,-4.800000],[-2.000000,-4.800000],[-2.400000,-4.800000],[-2.800000,-4.800000],[-3.200000,-4.800000],[-3.600000,-4.800000],[-3.600000,-4.400000],[-4.000000,-4.400000],[-4.400000,-4.400000]])
path2 = np.array([[2.000000,-0.400000],[2.000000,-0.800000],[2.400000,-0.800000],[2.800000,-0.800000],[3.200000,-0.800000], [2.000000,-0.400000],[2.000000,-0.800000],[2.400000,-0.800000],[2.800000,-0.800000],[3.200000,-0.800000]])
path3 = np.array([[2.000000,0.800000],[2.000000,0.400000],[2.000000,0.000000],[2.000000,-0.400000],[2.000000,-0.800000],[2.000000,-1.200000],[2.000000,-1.600000],[1.600000,-1.600000],[1.200000,-1.600000],[1.200000,-2.000000],[0.800000,-2.000000],[0.400000,-2.000000],[0.000000,-2.000000],[-0.400000,-2.000000],[-0.800000,-2.000000],[-1.200000,-2.000000],[-1.600000,-2.000000],[-2.000000,-2.000000],[-2.400000,-2.000000],[-2.800000,-2.000000],[-3.200000,-2.000000],[-3.600000,-2.000000],[-3.600000,-1.600000],[-4.000000,-1.600000],[-4.400000,-1.600000]])
# copy all paths and replace all negative values with positive ones
threshold = 0.2
setup = True


def euclidianDistance(goal, rx, ry):
    return math.sqrt(pow((goal[0] - rx), 2) + pow((goal[1] - ry), 2))

def linearVel(goal, rx, ry, limit, constant=0.4):
    return np.clip(constant * euclidianDistance(goal, rx, ry), 0, limit)

def steeringAngle(goal, rx, ry):
    print("steeringAngle: ", math.degrees(math.atan2(goal[1] - ry, goal[0] - rx))+ math.radians(90)) 
    return math.atan2(goal[1] - ry, goal[0] - rx)

def angularVel(goal, rx, ry, robotYaw, limit, constant=2):
    print("vel in degrees: ", math.degrees(constant * steeringAngle(goal, rx, ry) - robotYaw))
    print("yaw in degrees: ", math.degrees(robotYaw))
    return np.clip(constant * (steeringAngle(goal, rx, ry) - robotYaw), -limit, limit)

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
    # if(abs(robot.yaw - steeringAngle(robot.goal, robot.x, robot.y)) > 0.02):
    #     angularSpeed(robot)
    #     #linearSpeed(robot)
    # else:
    #     angularSpeed(robot)                
    #     linearSpeed(robot)
    speed = Twist()
    robot.goal = (robot.path[robot.index][0], robot.path[robot.index][1])
    speed.linear.x = linearVel(robot.goal, robot.x, robot.y, 0.3)
    speed.angular.z = angularVel(robot.goal, robot.x, robot.y, robot.yaw, 1.2)
    robot.publisher.publish(speed)
    # print(robot.name, "speed: ", speed)
    print(robot.name, "going to: ", robot.goal)
    print(robot.name, "at: ", robot.x, ",", robot.y)            
    checkRobotGoalReached(robot)  
    
def linearSpeed(robot):
    speed = Twist()
    robot.goal = (robot.path[robot.index][0], robot.path[robot.index][1])
    goal = robot.goal
    if(euclidianDistance(goal, robot.x, robot.y) > threshold):
        vel = linearVel(goal, robot.x, robot.y)
        speed.linear.x = vel
        speed.linear.y = 0
        if(vel > 0.3):
            vel = 0.3
        # print("going forward at speed: ", vel)
    else:
        speed.linear.x = 0
    robot.publisher.publish(speed)
    
def angularSpeed(robot):
    speed = Twist()
    robot.goal = (robot.path[robot.index][0], robot.path[robot.index][1])
    goal = robot.goal
    ang = angularVel(goal, robot.x, robot.y, robot.yaw)
    if(ang > 1):
        ang = 1
    elif(ang < -1):
        ang = -1 
    speed.angular.z = ang 
    # print("rotating at speed: ", ang)
    robot.publisher.publish(speed)           
    
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
        sub1 = rospy.Subscriber('/robot1/ground_truth/state', Odometry, cbpose1)
        sub2 = rospy.Subscriber('/robot2/ground_truth/state', Odometry, cbpose2)
        sub3 = rospy.Subscriber('/robot3/ground_truth/state', Odometry, cbpose3)        
        if setup:
            # path_demo()
            setupRobots()
            setup = False
        if(map.resolution != 0):
            #iterate through all robots in robots vector, if they are not at their goal, move them
            for robot in robots:
                if(robot.index < len(robot.path) -1):
                    moveRobot(robot)
                else:
                    robot.stop()  
        rate.sleep()
    
    