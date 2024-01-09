import sys
import os
import subprocess
import time
import traci
import re

sumoBinary = "/usr/local/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "/home/dfkiterminal/clones/sumo/tools/homburg_vehicles_only/osm.sumocfg"]
# sumoCmd = [sumoBinary, "-c", "/home/dfkiterminal/clones/sumo/tools/homburg_pedestrians_only/osm.sumocfg"]
traci.start(sumoCmd)

fo = open("trajectory.txt", "a")
global trajectoryList   
global max_num_veh 
# global VehinMap
global dict_vehicles
# VehinMap = {}
dict_vehicles = {}
max_num_veh = 0
simTime = 1000
trajectoryList = list(range(0,1000))

step = 0

class Vehicle():
    def __init__(self): 
        self.veh_id   = ' '
    
    
def get_vehicles_in_simulation():

    DepartVeh = traci.vehicle.getIDList()
    # global VehinMap
    global dict_vehicles
 
    for veh_id in DepartVeh:
        dict_vehicles[veh_id] = Vehicle()
        dict_vehicles[veh_id].veh_id = veh_id
        # VehinMap[VehId] = Vehicle()
        # VehinMap[VehId].VehId = VehId


if __name__ == "__main__":

    # DepartVeh = traci.simulation.getDepartedIDList()
    while step < 1000:
        traci.simulationStep()
        DepartVeh = traci.vehicle.getIDList()
        AllDepartVeh = traci.simulation.getDepartedIDList()
    
        if len(DepartVeh) > 0:
            get_vehicles_in_simulation()
    
        if len(dict_vehicles) > 0: 
            for veh in dict_vehicles:
                max_num_veh = int(re.search("\d+", veh)[0]) if max_num_veh < int(re.search("\d+", veh)[0]) else max_num_veh
                                
                if veh not in DepartVeh:
                        continue
    
                else:
                    print("CarID is = %s " % veh)
                    print("Position = %s, %s" % traci.vehicle.getPosition(veh))
                    [xp, xy] =  traci.vehicle.getPosition(veh)
                    xp = str(xp)
                    xy = str(xy)
                    speed = traci.vehicle.getSpeed(veh)
                    msTimer = traci.simulation.getTime() #getCurrentTime()
                    s =  msTimer # /1000 
                    # print("Current time = %s" % s)
    
                    s = str(s)
                    # id = int (VehId)
                    id = int(re.search("\d+", veh)[0])
                    tmp = str(trajectoryList[id])

                    # trajectoryList[id] = tmp+ " "+s+":"+xp+":"+xy
                    trajectoryList[id] = tmp+ " "+s+":"+str(speed)+":"+xp+":"+xy
                    # print(trajectoryList[id])
                    print("      ")
                
            msTimer = traci.simulation.getTime() #getCurrentTime()
            s =  msTimer # /1000 
            cnt = 0;       

            # print("STEP 0000 >> ", step, s)
    
            if s == simTime :  #output time
                for id1 in trajectoryList :
                    id1 = str(id1)
                    id1 = id1 + " \n"
                    #os.write(fo, id1)
                    fo.write(id1)
                    #print(id1)
                    cnt +=1
                    if cnt == (max_num_veh+1):
                        fo.close()
                        print("###################", str(max_num_veh+1))
                        #print(RunNs3)
                        # sumohome = os.environ['SUMO_HOME']
                        #subprocess.call(["bash",sumohome+"/tools/RunNs3.sh","simTime" , str(simTime), "enbTxPowerDbm", str(enbTxPowerDbm), "dlpacketSize", str(dlpacketSize), "dlpacketsInterval", str(dlpacketsInterval), "ulpacketSize", str(ulpacketSize), "ulpacketsInterval", str(ulpacketsInterval), "fadingModel", str(fadingModel)])

                        sys.stdout.flush()
                        traci.close()
                        break
        
        step = step + 1
            
    
