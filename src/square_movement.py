import rospy
import math
import actionlib
import numpy as np
from enum import Enum

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid

class Movement(Enum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

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

class Map:
    def __init__(self):
        self.height = 0
        self.width = 0
        self.originx = 0
        self.originy = 0
        self.resolution = 0
r1 = Robot()
r2 = Robot()
r3 = Robot()
map = Map()
path1 = [Movement.UP, Movement.UP, Movement.UP, Movement.LEFT, Movement.UP, Movement.UP, Movement.UP]
path2 = [Movement.UP, Movement.UP, Movement.UP, Movement.UP, Movement.UP, Movement.UP, Movement.UP, Movement.RIGHT, Movement.UP, Movement.UP]
path3 = [Movement.UP, Movement.UP, Movement.UP, Movement.RIGHT, Movement.UP, Movement.UP, Movement.UP, Movement.LEFT, Movement.UP, Movement.UP, Movement.RIGHT, Movement.UP]
threshold = 0.3
kP = 0.5
setup = True

def euclidianDistance(goal, rx, ry):
    return math.sqrt(pow((goal[0] - rx), 2) + pow((goal[1] - ry), 2))

def linearVel(goal, rx, ry, constant=1):
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
    if(abs(robot.yaw - steeringAngle(robot.goal, robot.x, robot.y)) > 0.02):
        angularSpeed(robot)
        #linearSpeed(robot)
    else:
        angularSpeed(robot)                
        linearSpeed(robot)
    
    print(robot.name, "going to: ", robot.goal)
    print(robot.name, "at: ", robot.x, ",", robot.y)            
    checkRobotGoalReached(robot)    
    
# def testAngular(robot):
#     speed = Twist()
#     speed.angular.z = 0.1
#     robot.publisher.publish(speed)
    
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
 
def rotate_robot(robot, angle):
    target_rad = math.radians(angle)
    ang_speed = kP * (target_rad - robot.yaw)
    return ang_speed

# move up on 0.4 ms for 1 second
def move_up(robot):
    speed = Twist() 
    speed.linear.x = 0.4
    robot.publisher.publish(speed)
    rospy.sleep(1)
    speed.linear.x = 0.0
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
        

def setGoal(goal, x, y):    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y    
    goal.target_pose.pose.orientation.w = 1.0
    
def setGoal(goal, x, y, robot):    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y    
    goal.target_pose.pose.orientation.w = 1.0
    robot.goal = (x, y)
        
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
    
    