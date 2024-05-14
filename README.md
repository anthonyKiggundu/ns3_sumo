**Get Started**

ns3+SUMO integration and functionality customization for QoS Sustainability datasets generation in V2X/UE networks 


**Dependencies**
- Compiled using _gcc version 9.4.0_
- On _ubuntu1~20.04.2_
  
With these extensions to NS-3 functionality, it is now possible to simulate mobility use-cases 
like V2X setups. The mobility profile for each device in a selected cite is topographically 
captured by Open Street Map and emulated using the SUMO tool. The output of this emulation is then
fed into the TraCI library to record the devices' trajectories as a text file.
An arbitrarily pre-defined file constituent of the base station locations plus this trajectories file
are then the input to the NS-3 tooling such that the throughput measures of each device plus the 
standard connectivity metrics are written as a dataset for self-organizing the Radio Access Network
using ML algorithms.

- A brief guide on getting the topographic city map captured using osm can be followed [here](https://sumo.dlr.de/docs/Tutorials/OSMWebWizard.html).
- The output from _osmWebWizard.py_ above is an _osmsumo.cfg_ file from which trajectories are read  (using TraCI) by the _scratch/binder.py_ file.
- The trajectories are stored in the _trajectories.txt_ and gNodeB poses in the _scratch/enbs.txt_
- All enhancements were done in the _scratch/overly.cc_ file
- The metrics like distance between eNB and UE, how many UEs are attached to an eNB are recorded into a _.csv_ file using the _metric_writer.py_ file.

**Notes**

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
