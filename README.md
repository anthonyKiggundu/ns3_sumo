**Get Started**
NS3+SUMO integration and functionality customization for QoS Sustainability prediction in V2X/UE networks 


**Dependencies**
- Compiled using _gcc version 9.4.0_
- On _ubuntu1~20.04.2_
- 
With these extensions to NS-3 functionality, it is now possible to simulate mobility use-cases 
like V2X setups. The mobility profile for each device in a selected cite is topographically 
captured by Open Street Map and emulated using the SUMO tool. The output of this emulation is then
fed into the TraCI library to record the devices' trajectories as a text file.
An arbitrarily pre-defined file constituent of the base station locations plus this trajectories file
are then the input to the NS-3 tooling such that the throughput measures of each device plus the 
standard connectivity metrics are written as a dataset for self-organizing the Radio Access Network
using ML algorithms.

- A brief guide on getting the topographic city map captured using osm can be followed here.
- The output above is an _osmsumo.cfg_ file from which trajcetories are read  (TraCI) by the _scratch/binder.py_ file.
- The trajectories are stored in the _trajectories.txt_ and gNodeB poses in the _scratch/enbLocations.txt_
- All enhancements were done in the _scratch/overly.cc_ file
- The metrics are recoreded into a _.csv_ file using the _metric_writer.py_ file.

**Notes**
- Ping-Pong effect: As documented in our findings, setting the jockeying threshold to a low value resulted in the behaviour that
  packets jumps were so often leading to a intractable system. Getting out of this system state then required application termination
  and we are working to improving the handling of these cases in code.

**Contact**
- For pull requests please send an email to:

**Acknowledgements**
- This work was done under the auspice of the AINET-ANTILLAS Project.

**Citations**
- Please use the `Cite this repository` link on the right pane in case you if you intend to cite the tooling for experimental use-cases.  

**License**

**ToDo**
- Extensions for compatibility to **NR-LENA**
- Code refactoring
