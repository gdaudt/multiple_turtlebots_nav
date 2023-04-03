import xml.etree.ElementTree as ET

class Task:
    path = []
    duration = 0
    flowtime = 0
    makespan = 0
    def __init__(self) -> None:
        pass
    
    def __init__(self, p, dur, flow, make):
        self.path = p
        self.duration = dur
        self.flowtime = flow
        self.makespan = make

class TaskReader:
    agents = []
    tasks = []
    def __init__(self) -> None:
        pass
    
    def read_xml(self, file_path):
        tree = ET.parse(file_path)
        root = tree.getroot()    
        # iterate through the log in root and find all the agent tags
        for log in root.findall('log'):
            flowtime = log.find('summary').attrib['flowtime']  
            makespan = log.find('summary').attrib['makespan']  
            # print("flowtime: " + flowtime)
            for agent in log.findall('agent'):
                number = int(agent.attrib['number'])
                self.agents.append(number) 
                for path in agent.findall('path'):
                    duration = path.attrib['duration']  
                    # print ("duration: " + duration)
                    # j = 0
                    # print("path " + str(number))  
                    p = []                  
                    for section in path.findall('section'):                        
                        # print("section: " + number)
                        start_i = section.attrib['start_i']
                        start_j = section.attrib['start_j']
                        goal_i = section.attrib['goal_i']
                        goal_j = section.attrib['goal_j']
                        dur = section.attrib['duration']
                        p.append((float(start_i), float(start_j), float(goal_i), float(goal_j), float(dur)))
                        # print("start_i: " + start_i, 
                        #       "start_j: " + start_j, 
                        #       "goal_i: " + goal_i, 
                        #       "goal_j: " + goal_j, 
                        #       "duration: " + duration)
                        # j+=1
                    self.tasks.append(Task(p, duration, flowtime, makespan)) 
        return self.tasks, self.agents

