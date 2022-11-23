import rospy
import math
import actionlib
import numpy as np
import utils
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0
        self.publisher = 0
        self.id = 0
        self.goal = (0.0,0.0)
        self.path = []
        self.name = "robot" + str(self.id)
        self.actiontime = float('inf')
        self.total_time = 0
    #method to initialize all variables receiving them as parameters
    def init(self, index, name, publisher):
        self.id = index
        self.name = name
        self.publisher = publisher        
    #set the linear and angular velocity to 0, and publish using the publisher
    def stop(self):
        speed = Twist()
        speed.linear.x = 0
        speed.angular.z = 0
        self.publisher.publish(speed)

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
map = Map()
# path1 = np.array([[2.000000,-1.600000],[2.000000,-2.000000],[2.000000,-2.400000],[2.000000,-2.800000],[2.000000,-3.200000],[2.000000,-3.600000],[2.000000,-4.000000],[2.000000,-4.400000],[1.600000,-4.400000],[1.200000,-4.400000],[1.200000,-4.800000],[0.800000,-4.800000],[0.400000,-4.800000],[0.000000,-4.800000],[-0.400000,-4.800000],[-0.800000,-4.800000],[-1.200000,-4.800000],[-1.600000,-4.800000],[-2.000000,-4.800000],[-2.400000,-4.800000],[-2.800000,-4.800000],[-3.200000,-4.800000],[-3.600000,-4.800000],[-3.600000,-4.400000],[-4.000000,-4.400000],[-4.400000,-4.400000]])
# path2 = np.array([[2.000000,-0.400000],[2.000000,-0.800000],[2.400000,-0.800000],[2.800000,-0.800000],[3.200000,-0.800000]])
# path3 = np.array([[2.000000,0.800000],[2.000000,0.400000],[2.000000,0.000000],[2.000000,-0.400000],[2.000000,-0.800000],[2.000000,-1.200000],[2.000000,-1.600000],[1.600000,-1.600000],[1.200000,-1.600000],[1.200000,-2.000000],[0.800000,-2.000000],[0.400000,-2.000000],[0.000000,-2.000000],[-0.400000,-2.000000],[-0.800000,-2.000000],[-1.200000,-2.000000],[-1.600000,-2.000000],[-2.000000,-2.000000],[-2.400000,-2.000000],[-2.800000,-2.000000],[-3.200000,-2.000000],[-3.600000,-2.000000],[-3.600000,-1.600000],[-4.000000,-1.600000],[-4.400000,-1.600000]])

threshold = 0.3
# robot1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
# robot2 = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
# robot3 = actionlib.SimpleActionClient('robot3/move_base', MoveBaseAction)
# goal1 = MoveBaseGoal()
# goal2 = MoveBaseGoal()
# goal3 = MoveBaseGoal()
setup = True
robots = []
init_time = 0

def euclidianDistance(goal, rx, ry):
    return math.sqrt(pow((goal[0] - rx), 2) + pow((goal[1] - ry), 2))

def linearVel(goal, rx, ry, limit, constant=0.4):
    return np.clip(constant * euclidianDistance(goal, rx, ry), 0, limit)

def steeringAngle(goal, rx, ry):
    # print("steeringAngle: ", math.degrees(math.atan2(goal[1] - ry, goal[0] - rx))+ math.radians(90)) 
    return math.atan2(goal[1] - ry, goal[0] - rx)

def angularVel(goal, rx, ry, robotYaw, limit, constant=2):
    # print("vel in degrees: ", math.degrees(constant * steeringAngle(goal, rx, ry) - robotYaw))
    # print("yaw in degrees: ", math.degrees(robotYaw))
    return np.clip(constant * (steeringAngle(goal, rx, ry) - robotYaw), -limit, limit) 

def setupRobots():
    global r1, r2, r3
    r1.publisher = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=1)
    r2.publisher = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)
    r3.publisher = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=1)
    r1.path.append(utils.getGazeboCoordinate(0, 30, (70, 46), 0.4))
    r1.path.append(utils.getGazeboCoordinate(12, 21, (70, 46), 0.4)) 
    r1.path.append(utils.getGazeboCoordinate(28, 20, (70, 46), 0.4)) 
    r1.goal = r1.path[r1.id]   
    #append points to r2 path in gazebo coordinates
    #points are (13, 43), (57,40), (68,36)
    r2.path.append(utils.getGazeboCoordinate(13, 43, (70, 46), 0.4))
    r2.path.append(utils.getGazeboCoordinate(57, 40, (70, 46), 0.4))
    r2.path.append(utils.getGazeboCoordinate(68, 36, (70, 46), 0.4))
    r2.goal = r2.path[r2.id] 
    r1.name = "robot1"
    r2.name = "robot2"
    r3.name = "robot3"
    robots.append(r1)
    robots.append(r2)

def moveRobot(robot):          
    if robot.actiontime > time.time():
        robot.actiontime = time.time()
        print("starting at time: ", robot.actiontime - robot.actiontime)
    speed = Twist()
    robot.goal = (robot.path[robot.id][0], robot.path[robot.id][1])
    speed.linear.x = linearVel(robot.goal, robot.x, robot.y, 0.3)
    speed.angular.z = angularVel(robot.goal, robot.x, robot.y, robot.yaw, 1.2)
    robot.publisher.publish(speed)
    # print(robot.name, "speed: ", speed)
    # print(robot.name, "going to: ", robot.goal)
    # print(robot.name, "at: ", robot.x, ",", robot.y)            
    checkRobotGoalReached(robot)  
    
def linearSpeed(robot):
    speed = Twist()
    robot.goal = (robot.path[robot.id][0], robot.path[robot.id][1])
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
    robot.goal = (robot.path[robot.id][0], robot.path[robot.id][1])
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
        if robot.id < len(robot.path) - 1:
            rospy.loginfo(robot.name + " reached point " + str(robot.id))
            print("at time: ", abs(time.time() - robot.actiontime))
            robot.total_time += abs(time.time() - robot.actiontime)
            robot.actiontime = time.time()
        else:            
            robot.publisher.publish(speed)
            rospy.loginfo(robot.name + " reached point " + str(robot.id))
            print("at time: ", abs(time.time() - robot.actiontime))
            robot.total_time += abs(time.time() - robot.actiontime)
            rospy.loginfo(robot.name + " reached goal") 
            print("at time: ", robot.total_time)
        robot.id += 1           
                    
def odom_callback(data, robot):
    robot.x = data.pose.pose.position.x
    robot.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    robot.yaw = yaw

def laser_callback(data, robot):    
    # if data.ranges[0::15] < 0.2:
    #     print("obstacle detected")
    #     speed = Twist()
    #     speed.angular.z = 0.5
    #     robot.publisher.publish(speed)
    # elif data.ranges[344:359] < 0.2:
    #     print("obstacle detected")
    #     speed = Twist()
    #     speed.angular.z = -0.5
    #     robot.publisher.publish(speed)
    

    
if __name__ == '__main__':
    rospy.init_node('path_demo')
    rate = rospy.Rate(10)    
    init_time = time.time()
    while not rospy.is_shutdown():
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe 
        map_sub = rospy.Subscriber('/map', OccupancyGrid, utils.map_callback, map)   
        if setup and (map.resolution != 0 and map.originx != 0 and map.originy != 0 ):
            # path_demo() 
            setupRobots()
            setup = False
        if(map.resolution != 0 and map.originx != 0 and map.originy != 0 ):
            #iterate through all robots in robots vector, if they are not at their goal, move them
            for robot in robots:
                rospy.Subscriber(robot.name+'/ground_truth/state', Odometry, odom_callback, robot)
                rospy.Subscriber(robot.name+'/scan', LaserScan, laser_callback, robot)
                if(robot.id < len(robot.path)):
                    moveRobot(robot)
                    # print(len(robot.path))
                    # print(robot.id)
                else:
                    robot.stop()  
        rate.sleep()
    
    