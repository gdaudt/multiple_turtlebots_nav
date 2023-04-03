import rospy
import math
import actionlib
import numpy as np
import utils
import sys
import time

from xml_read import TaskReader
from xml_read import Task
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import PerformanceMetrics



class Robot:    
    #method to initialize all variables receiving them as parameters
    def __init__(self, index, publisher, path, name, init_time):
        self.id = index
        self.name = name
        self.publisher = publisher
        self.actiontime = float('inf') 
        self.init_time = init_time       
        self.path = path
        self.goalid = 2
        self.pathid = 0
        self.goal_duration = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0
        self.completed = False
        self.goal_reached = False
        self.newgoal = True
        self.speed = Twist()
        self.radius = 0.20
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
robot_radius = 0.15
map = Map()
gazebo_time = 0
threshold = 0.03
setup = True
flowtime = 0
makespan = 0
robots = []
init_time = 0
map_resolution = 0.5
lin_limit = 0.22

def gazebo_time_callback(data, time):
    global gazebo_time
    time = data.header.stamp.secs + (data.header.stamp.nsecs * 1e-9)
    gazebo_time = time

def euclidianDistance(goal, rx, ry):
    return math.sqrt(pow((goal[0] - rx), 2) + pow((goal[1] - ry), 2))

def linearVel(goal, rx, ry, limit, constant=1.6):
    return np.clip(constant * euclidianDistance(goal, rx, ry), 0, limit)

def steeringAngle(goal, rx, ry):
    # print("steeringAngle: ", math.degrees(math.atan2(goal[1] - ry, goal[0] - rx))+ math.radians(90)) 
    return math.atan2(goal[1] - ry, goal[0] - rx)

def angularVel(goal, rx, ry, robotYaw, limit, constant=1):
    #divide the goal(x, y) and the rx, ry difference between the four quadrants of the radians
    #adjust the resulting radians to the robot's yaw
    vel = steeringAngle(goal, rx, ry) - robotYaw
    # print("vel before adjustment: ", vel)
    if (vel > math.pi):
        vel -= 2 * math.pi
    elif (vel < -math.pi):
        vel += 2 * math.pi
    # print("vel after adjustment: ", vel)
    # print("angular vel: ", vel)
    # print("goal: ", goal)
    # print("rx, ry: ", rx, ry)
    # print("robotYaw: ", robotYaw)
    return np.clip(constant*vel, -limit, limit)

def setupRobots(tasks, robotid):
    global gazebo_time
    global robot_radius
    global flowtime
    global makespan
    i = 1 
    for task in tasks: 
        if(i != robotid):
            i+=1
            continue           
        id = i
        name = "robot" + str(i)
        publisher = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=1)    
        # print("robot name: ", name)
        # print("path duration: ",task.duration)
        # print("gazebo time: ", gazebo_time)
        path = []
        
        for p in task.path:
            # r.path.append(utils.getGazeboCoordinate(p[0], p[1], (70, 46), map_resolution))
            start = utils.planToGazeboCoordinate(p[0], p[1], robot_radius)
            goal = utils.planToGazeboCoordinate(p[2], p[3], robot_radius)            
            # print("start: ", p[0], p[1])
            # print("goal: ", p[2], p[3])
            # print("duration: ", p[4])
            if(float(p[4]) < 1.5):
                p = (start[0], start[1], goal[0], goal[1], float(p[4])*1.8)
            else:
                p = (start[0], start[1], goal[0], goal[1], float(p[4])*1.8)
            # print(p)
            path.append(p)
        init_time = gazebo_time
        robots.append(Robot(id, publisher, path, name, init_time))    
        i+=1  
        


def moveRobot(robot):       
    global gazebo_time   
    
    if robot.actiontime > gazebo_time:
        robot.actiontime = gazebo_time - init_time
        # print("starting at time: ", robot.actiontime - robot.actiontime)  
    if robot.newgoal:  
        robot.goal = (robot.path[robot.pathid][robot.goalid], robot.path[robot.pathid][robot.goalid+1])
        robot.goal_duration = robot.path[robot.pathid][4]
        robot.newgoal = False
    if(not robot.goal_reached):    
        robot.speed.angular.z = angularVel(robot.goal, robot.x, robot.y, robot.yaw, 2.5)
        # print("angular speed: ", robot.speed.angular.z)
        #maybe limit the linear speed so it can't be bigger than 10% of the angular speed
        if(robot.speed.angular.z < (lin_limit*-2.8) or robot.speed.angular.z > (lin_limit*2.8)):
            # print("limiting linear speed")
            robot.speed.linear.x = 0
        else:
            robot.speed.linear.x = linearVel(robot.goal, robot.x, robot.y, lin_limit)
        # if (robot.speed.angular.z < 0.1 and robot.speed.angular.z > -0.1):
        #     if(robot.speed.linear.x < 0.2):
        #         robot.speed.linear.x += 0.03
        #     robot.speed.angular.z = 0
        # else:
        #     robot.speed.linear.x = 0
        # robot.speed.linear.x = 0.2
        robot.publisher.publish(robot.speed)
        # print(robot.name, "speed: ", speed)
        # print(robot.name, "going to: ", robot.goal)
        # print(robot.name, "at: ", robot.x, ",", robot.y)            
    checkRobotGoalReached(robot)  
    
# compares every robots current pose to the goal pose
# if the distance between the robot and the goal is less than a threshold, the robot is considered reached
# then it iterates the global index of the robot
def checkRobotGoalReached(robot):  
    global gazebo_time   
    if euclidianDistance(robot.goal, robot.x, robot.y) < threshold:
        robot.goal_reached = True        
        if((abs(robot.actiontime - (gazebo_time - init_time))) > robot.goal_duration):
            if robot.pathid+1 < len(robot.path):
                # rospy.loginfo(robot.name + " reached point " + str(robot.pathid) + " (" + str(robot.goal[0]) + ", " + str(robot.goal[1]) + ")")
                # print("at time: ", (abs(gazebo_time - robot.actiontime) - init_time), " expected duration: ", robot.goal_duration)
                robot.actiontime = gazebo_time - init_time
                robot.pathid += 1
                robot.newgoal = True
                robot.goal_reached = False
            else:            
                robot.stop()
                # rospy.loginfo(robot.name + " reached point " + str(robot.pathid))
                # print("at time: ", abs(gazebo_time - robot.actiontime))
                # print(robot.name + " reached goal") 
                # print("at time: ", abs(init_time - robot.actiontime))
                print(abs(robot.init_time - gazebo_time) - init_time)  
                robot.pathid += 1
        else:            
            # print(robot.name + " waiting at point " + str(robot.pathid) + " for " + str(robot.goal_duration - abs(gazebo_time - robot.actiontime)) + " seconds")            
            if(robot.pathid+1 < len(robot.path)):                
                robot.speed.angular.z = angularVel((robot.path[robot.pathid+1][2], robot.path[robot.pathid+1][3]), robot.x, robot.y, robot.yaw, 2.5)
            robot.speed.linear.x = 0
            robot.publisher.publish(robot.speed)
                        
                    
def odom_callback(data, robot):
    robot.x = data.pose.pose.position.x
    robot.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    #turn negative yaw values into positive ones
    yaw = yaw % (2 * math.pi)
    robot.yaw = yaw

    
if __name__ == '__main__':
    filepath = "../misc/grid_task_log.xml"
    robotid = int(sys.argv[1])
    tr = TaskReader()
    tasks, agents = tr.read_xml(filepath)
    rospy.init_node('path_demo' + sys.argv[1])
    rate = rospy.Rate(10)    
    while not rospy.is_shutdown():
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe 
        map_sub = rospy.Subscriber('/map', OccupancyGrid, utils.map_callback, map) 
        gazebo_time_sub = rospy.Subscriber('/gazebo/performance_metrics', PerformanceMetrics, gazebo_time_callback, gazebo_time)  
        if(init_time == 0):
            init_time = gazebo_time
        # print("gazebo time: ", gazebo_time)
        # if setup and (map.resolution != 0 and map.originx != 0 and map.originy != 0 ):
        if setup:
            setupRobots(tasks, robotid)
            # path_demo() 
            setup = False
        if(map.resolution != 0 and map.originx != 0 and map.originy != 0 ):
            #iterate through all robots in robots vector, if they are not at their goal, move them
            for robot in robots:
                rospy.Subscriber(robot.name+'/ground_truth/state', Odometry, odom_callback, robot)
                if(robot.pathid < len(robot.path)):
                    moveRobot(robot)
                    # print(len(robot.path))
                    # print(robot.id)
                else:
                    robot.stop()
                    quit() 
        rate.sleep()
    
    