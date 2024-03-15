# ns3_sumo
NS3+SUMO integration and functionality customization for QoS Sustainability prediction in V2X/UE networks 

With these extensions to NS-3 functionality, it is now possible to simulate mobility use-cases 
like V2X setups. The mobility profile for each device in a selected cite is topographically 
captured by Open Street Map and emulated using the SUMO tool. The output of this emulation is then
fed into the TraCI library to record the devices' trajectories as a text file.
An arbitrarily pre-defined file constituent of the base station locations plus this trajectories file
are then the input to the NS-3 tooling such that the throughput measures of each device plus the 
standard connectivity metrics are written as a dataset for self-organizing the Radio Access Network
using ML algorithms.
