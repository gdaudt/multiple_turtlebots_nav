import requests
import argparse
import cherrypy
import rospy
from threading import Thread
from queue import *
from time import sleep
from geometry_msgs.msg import Pose
from multiple_turtlebots_nav.msg import mission


class Robot:

    function_queue = Queue()
    
    def __init__(self, server, robot_id):
        self.server_ip = server.split(":")[0]
        self.server_port = int(server.split(":")[1])
        self.url = 'http://{}:{}/process_mission'.format(self.server_ip, self.server_port)

        self.id = robot_id
        self.status = 'running'
        self.battery = 10
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        

#  mission: int,
#             operation_type: string,
#             task: int
#             pos x: float,
#             pos y: float,
#             heading: float,



    def send_mission_complete(self, mission_id):
        self.url = 'http://{}:{}/mission_complete'.format(self.server_ip, self.server_port)        
        data = {
            'id_mission': mission_id
        }
        r = requests.post(self.url, timeout=10.0, json=data)

    def send_mission_interrupted(self, mission_id):
        self.url = 'http://{}:{}/mission_interrupted'.format(self.server_ip, self.server_port)  
        data = {
            'id_mission': mission_id
        } 
        r = requests.post(self.url, timeout=10.0, json=data)

    def send_charging_battery(self, mission_id):
        self.url = 'http://{}:{}/charging_battery'.format(self.server_ip, self.server_port)  
        data = {
            'id_mission': mission_id
        } 
        r = requests.post(self.url, timeout=10.0, json=data)
            
    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def mission_resume(self):
        data = cherrypy.request.json
        sleep(6)
        self.send_mission_complete(data['id_mission'])    

    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def mission_receive(self):
        '''
        data = {
            'id_mission': mission.id,
            'type_mission': mission.type_m,
            'origin_mission': mission.origin.position,
            'destination_mission': mission.destination.position
        }
        '''
        data = cherrypy.request.json
        self.x = data['destination_mission'][0]
        self.y = data['destination_mission'][1]
        self.function_queue.put((self.publish_mission, data))
        print(self.function_queue)
        print(data)

    def publish_mission(self, data):
        # print('publish_mission')
        # print(data)        
        pub_mission = mission()    
        pub_mission.mission_id = data['id_mission']
        pub_mission.mission_type.data = data['type_mission']    
        pub_mission.origin.pose.position.x = data['origin_mission'][0]
        pub_mission.origin.pose.position.y = data['origin_mission'][1]
        pub_mission.destination.pose.position.x = data['destination_mission'][0]
        pub_mission.destination.pose.position.y = data['destination_mission'][1]
        print(pub_mission)        
        rate = rospy.Rate(10)
        count = 0
        while count < 5:
            pub = rospy.Publisher('mission_publisher', mission, queue_size=10)   
            pub.publish(pub_mission)
            count += 1  
            rate.sleep()               
        
    @cherrypy.expose
    @cherrypy.tools.json_out()
    @cherrypy.tools.json_in()
    def mission_cancel(self):
        data = cherrypy.request.json
        print("mission canceled: ", data['id_mission'])


def main():
    parser = argparse.ArgumentParser(description='A simple example of web service')

    # Robot's ip and port
    parser.add_argument('--host_ip', type=str, default='0.0.0.0')
    parser.add_argument('--host_port', type=int, default=8081)
    parser.add_argument('--robot_id', type=int, default=1)

    # Server's IP and port
    parser.add_argument('--server', type=str, default='0.0.0.0:8080')

    args = parser.parse_args()
    
    config = {'server.socket_host': args.host_ip,
              'server.socket_port': args.host_port}

    cherrypy.config.update(config)
    robot = Robot(args.server, args.robot_id)
    thread = Thread(target=cherrypy.quickstart, args=(robot,))
    thread.setDaemon(True)   
    thread.start()    
    return robot
    
def ros_loop(robot):
    if not robot.function_queue.empty():
        print("queue has items")
        item = robot.function_queue.get()
        func = item[0]
        args = item[1]
        func(args)
        
   # print("ros loop")
        
if __name__ == '__main__':
    robot = main()
    while not rospy.is_shutdown():        
        rospy.init_node('publisher', anonymous=True)
        rate = rospy.Rate(10)
        ros_thread = Thread(target=ros_loop, args=(robot,))        
        ros_thread.setDaemon(True)
        ros_thread.start()