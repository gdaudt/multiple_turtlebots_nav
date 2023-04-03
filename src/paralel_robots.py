import multiprocessing
import os      
import sys     
from xml_read import TaskReader                                                    
 
# Creating the tuple of all the processes
robot_number = int(sys.argv[1]) + 1 
all_processes = []

for i in range(1, robot_number):
    all_processes.append('move_demo_single.py ' + str(i))

print(all_processes)                                
                                                  
# This block of code enables us to call the script from command line.                                                                                
def execute(process):                                                             
    os.system(f'python3 {process}')                                       
                                                                                
                                                                                
process_pool = multiprocessing.Pool(processes = robot_number)                                                        
process_pool.map(execute, all_processes)

filepath = "../misc/grid_task_log.xml"
tr = TaskReader()
tasks, agents = tr.read_xml(filepath)
print(tasks[0].flowtime)
print(tasks[0].makespan)


