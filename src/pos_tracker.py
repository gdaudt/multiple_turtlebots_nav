import rospy
import utils
from move_demo import Robot
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid

#vector of robots in the simulation
robots = []

def odom_callback(data, robot):
    robot.x = data.pose.pose.position.x
    robot.y = data.pose.pose.position.y
    (roll, pitch, yaw) = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    robot.yaw = yaw

if __name__ == '__main__':
    rospy.init_node('mission_node', anonymous=True)
    mapResolution = 0.4
    map = utils.Map()
    r1 = Robot()
    r2 = Robot()
    r3 = Robot()
    r2.name = "robot2"
    r3.name = "robot3"
    r1.name = "robot1"
    r1.id = 0
    r2.id = 1
    r3.id = 2
    robots.append(r1)
    # robots.append(r2)
    # robots.append(r3)    
    rate = rospy.Rate(10)  
    posvec = [(-999, -999), (-999, -999), (-999, -999)]  
    while not rospy.is_shutdown():
        map_sub = rospy.Subscriber('/map', OccupancyGrid, utils.map_callback, map) 
        print(utils.getGazeboCoordinate(7, 35, (70, 46), mapResolution))
        #print the map info
        # print("MAP GLOBAL VARIABLE", map.width, map.height, map.resolution)
        #loops through all the robots, creating a subscriber to each of the robots/odom topic
        #each topic is named robot+vector_index/odom and the callback function is odom_callback 
        for robot in robots:
            rospy.Subscriber(robot.name+'/ground_truth/state', Odometry, odom_callback, robot)
            # print(robot.name, robot.x, robot.y, robot.yaw)
            if(map.resolution != 0 and map.originx != 0 and map.originy != 0 ):
                # print("robot odom to map: ", utils.transformCoordinateOdomToMap(robot.x, robot.y, map))
                #adjust and print the robot's coordinates from the 5cm resolution map to the 40cm resolution map 
                # print(robot.id)
                # print("posvec: ", (posvec[robot.id][0], posvec[robot.id][1]))
                # print("robot odom to map: ", utils.truncateCoordinate(robot.x, robot.y, 0.4))
                # print(robot.name)
                truncated = utils.truncateCoordinate(robot.x, robot.y, mapResolution)          
                test = (posvec[robot.id][0] != truncated[0] or posvec[robot.id][1] != truncated[1])
                # print("vecpos: ", posvec[robot.id])
                # print(test)
                if(test):
                    posvec[robot.id] = utils.truncateCoordinate(robot.x, robot.y, mapResolution)
                    print(robot.name + " truncated coordinates: ", (posvec[robot.id][0], posvec[robot.id][1]))
                    #resolution is 0.4m
                    abstractCoordinate = utils.getMapCoordinate(posvec[robot.id][0], posvec[robot.id][1], (46, 70), mapResolution)
                    print(robot.name + " abstract coordinates: ", abstractCoordinate)
                    print(robot.name + " gazebo coordinates: ", utils.getGazeboCoordinate(abstractCoordinate[1], abstractCoordinate[0], (46, 70), mapResolution))
        rate.sleep()
        now = rospy.get_rostime()
        # rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
    