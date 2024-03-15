import sys
import os
import subprocess
import time
import traci
import re

sumoBinary = "/usr/local/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "../../clones/sumo/tools/homburg_vehicles_only/osm.sumocfg"]
traci.start(sumoCmd)

file_handle = open("trajectory.txt", "a")
global trajectoryList   
global max_num_veh 
global dict_vehicles
dict_vehicles = {}
max_num_veh = 0
simTime = 1000
trajectoryList = list(range(0,1000))

step = 0

class Vehicle():
    def __init__(self): 
        self.veh_id   = ' '
    
    
def get_vehicles_in_simulation():

    lst_departed_vehicles = traci.vehicle.getIDList()
    global dict_vehicles
 
    for veh_id in lst_departed_vehicles:
        dict_vehicles[veh_id] = Vehicle()
        dict_vehicles[veh_id].veh_id = veh_id

if __name__ == "__main__":
    
    while step < 1000:
        traci.simulationStep()
        lst_departed_vehicles = traci.vehicle.getIDList()        
    
        if len(lst_departed_vehicles) > 0:
            get_vehicles_in_simulation()
    
        if len(dict_vehicles) > 0: 
            for veh in dict_vehicles:
                max_num_veh = int(re.search("\d+", veh)[0]) if max_num_veh < int(re.search("\d+", veh)[0]) else max_num_veh
                                
                if veh not in lst_departed_vehicles:
                        continue
    
                else:
                    print("CarID is = %s " % veh)
                    print("Position = %s, %s" % traci.vehicle.getPosition(veh))
                    [xp, xy] =  traci.vehicle.getPosition(veh)
                    xp = str(xp)
                    xy = str(xy)
                    speed = traci.vehicle.getSpeed(veh)
                    msTimer = traci.simulation.getTime() 
                    s =  msTimer
    
                    s = str(s)                    
                    id = int(re.search("\d+", veh)[0])
                    tmp = str(trajectoryList[id])                    
                    trajectoryList[id] = tmp+ " "+s+":"+str(speed)+":"+xp+":"+xy                    
                    print("      ")
                
            msTimer = traci.simulation.getTime() 
            s =  msTimer 
            cnt = 0;       
    
            if s == simTime : 
                for id1 in trajectoryList :
                    id1 = str(id1)
                    id1 = id1 + " \n"                    
                    file_handle.write(id1)                    
                    cnt +=1
                    if cnt == (max_num_veh+1):
                        file_handle.close()                        
                        sys.stdout.flush()
                        traci.close()
                        break
        
        step = step + 1
            
    
