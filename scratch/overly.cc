/*
###################################################################################################
With these extensions to NS-3 functionality, it is now possible to simulate mobility use-cases 
like V2X setups. The mobility profile for each device in a selected cite is topographically 
captured by Open Street Map and emulated using the SUMO tool. The output of this emulation is then
fed into the TraCI library to record the devices' trajectories as a text file.
An arbitrarily pre-defined file constituent of the base station locations plus this trajectories file
are then the input to the NS-3 tooling such that the throughput measures of each device plus the 
standard connectivity metrics are written as a dataset for self-organizing the Radio Access Network
using ML algorithms.
###################################################################################################
*/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include "ns3/waypoint.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-classifier.h"
#include "ns3/flow-probe.h"
#include "ns3/gnuplot.h"
#include "ns3/config-store-module.h"
#include "ns3/ipv4.h"
#include <boost/type_index.hpp>
#include <ns3/lte-common.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-helper.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-phy.h>
#include <ns3/lte-ue-rrc.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/combine.hpp>

#include <math.h>
#include <iterator>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <libsumo/libtraci.h>
#include <exception>
#include <dirent.h>
#include <stdexcept>
#include <stdio.h>
#include <cstring>
#include <map>
#include <algorithm>

using namespace libtraci;
using namespace ns3;
using namespace std;

// Struct to hold blob of each ue's details as it moves during the simulation
struct Ues {
   string ue_id;
   float ue_speed;
   vector<float> qos_params;
   vector<float> pose;
};

typedef struct {
  double x;
  double y;
} Point;

NS_LOG_COMPONENT_DEFINE ("SumoNS3UesTrajectoriesHandoverMeasures");

Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

void attachinfuture(NetDeviceContainer& uesdev, NetDeviceContainer& enbsdev){
  // lteHelper->Attach (ues.Get(ueid)); 
  lteHelper->AttachToClosestEnb(uesdev, enbsdev);
}


/*
###################################################################################################
Some global variables and functions
###################################################################################################
*/

string trunct; // , current_ue;
vector< pair <string, int> > original_trajectory_sizes;
NodeContainer ueNodes;
NodeContainer enbNodes;
NetDeviceContainer ueLteDevs, enbLteDevs;
vector<string> speeds, nodes, processed_nodes, all_node_list;
vector<int> speed_sizes;
vector<string> ue_handover_list;
vector<string> handover_lst_rntis;
vector<string> newUe_lst_rntis; // , lst_ips;
std::map<std::string, string> motional;
vector< pair <string, string> > coordinates;
vector< pair <string, string> > directional_speeds;
vector< pair <string, int> > node_trajectory_sizes;
vector< pair <string, int> > trajectory_sizes;
vector< pair <string, string> > in_handover_mode_ues;
map< string, string>  ipaddr_macaddr_mapping;
map<string, string> vect_ip_pose;

typedef map<string, string> TStrStrMap;
typedef pair<string, string> TStrStrPair;
TStrStrMap uernti_ips_to_enbips_mapping;
TStrStrMap connected_ips_to_enbips_mapping;
TStrStrMap connected_imsi_ips_to_enbips_mapping;
TStrStrMap handed_over_ues;
TStrStrMap vect_rsrq_rsrp;

map<string, string> cellid_ueip_mapping;

int runner = 0, counter = 0;
double dist_to_enb = 0.0;
vector<string> connected_ues;
vector<string> current_profiles_ue_add;

/*
###################################################################################################
Templates
###################################################################################################
*/

template < typename Type > std::string to_str (const Type & t)
{
  std::ostringstream os;
  os << t;
  return os.str ();
}

// templating for printing out all vector elements at once
template <typename S> ostream& operator<<(ostream& os, const vector<S>& vector)
{
    // Printing all the elements
    // using <<
    for (auto element : vector) {
        os << element << " ";
       //*** std::cout<<"\n "<< element; 
    }
    return os;
}


template<typename T>
inline void remove(std::vector<T> & v, const T & item)
{
    // Note: if you omit the v.end() parameter to v.erase, the
    // code will compile but only a single item will be removed.

    v.erase(std::remove(v.begin(), v.end(), item), v.end());
}

/*
###################################################################################################
End Templates
###################################################################################################
*/

/*
###################################################################################################
 Set of functions that count the number of similar elements in key:value
 map. Here the values in the map are counted and returned.
 Other functions do comparisons to extract required values from the mappings
###################################################################################################
*/
 
struct Compare {
    std::string str;
    Compare(const std::string& str) : str(str) {}
};
bool operator==(const std::pair<string, std::string>&p, const Compare& c) {
    return c.str == p.second;
}
bool operator==(const Compare& c, const std::pair<string, std::string>&p) {
    return c.str == p.second;
}	

struct PartialMatch
{
  std::string s;
  PartialMatch(const std::string& str) : s(str) {}

  bool operator()(const std::string& in)
  {
    size_t pos = in.find(s);
    return pos != std::string::npos;
  }

};

int GetCountEnBEntries (map< string, string> m, string enb_ip) {

    int count = std::count(m.begin(), m.end(), Compare(enb_ip));

    return count;
}


TStrStrMap::const_iterator FindPrefix(const TStrStrMap& map, const string& search_for) {
    TStrStrMap::const_iterator i = map.lower_bound(search_for);
    if (i != map.end()) {
        const string& key = i->first;
        if (key.compare(0, search_for.size(), search_for) == 0) // Really a prefix?
            return i;
    }
    return map.end();
}


string GetIpFromMap (const TStrStrMap& m, const string& cell_rnti) {

  string ip_addr;
  auto i = FindPrefix(m, cell_rnti);
  if (i != m.end())
     ip_addr = i->first;

  return ip_addr;
}


string GetEnBFromMap(const TStrStrMap& uernti_ips_to_enbips_mapping, const string& id_rnti_cellid) {

  string enb_ip_addr;
  auto i = FindPrefix(uernti_ips_to_enbips_mapping, id_rnti_cellid);
  if (i != uernti_ips_to_enbips_mapping.end())
     enb_ip_addr = i->second;

  return enb_ip_addr;
}


map<string, string> setEnBIPToPoseMap (string filename) {

    vector<string> v_enb;
    string s;

    fstream enbFile, uesFile;
    enbFile.open(filename, ios::in);

    while(getline(enbFile, s)){
       v_enb.push_back(s);
    }

    string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";

    string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));

    Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);
    
    string enbmacaddr, enb_ip_addr;
    std::map<string, string> vect_ip_pose;
    int k = 0;

    for (uint32_t t = 0; t < match.GetN (); t++) {
       Ptr<Object> enbNetDevice = match.Get (t);

       Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
       Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
       Ptr<LteEnbMac> enbMac = enbLteDevice->GetMac ();
       ostringstream os_str_macaddr;
       os_str_macaddr << enbMac;
       enbmacaddr = os_str_macaddr.str();

       std::map<string, string>::iterator it = ipaddr_macaddr_mapping.find(enbmacaddr);
       enb_ip_addr = it->second;

       vect_ip_pose.insert(std::pair<string, string>(enb_ip_addr, v_enb[t]));
       k++;
    }

    return vect_ip_pose;
}


vector <pair <string, string>> Difference(map<string, string> tempSet, map<string, string> currentSet) {

    std::vector <pair <string, string>> result;
    std::set_symmetric_difference(tempSet.begin(), tempSet.end(), currentSet.begin(), currentSet.end(), std::back_inserter(result));
   
  /*   
    for (auto p : result) {
        std::cout << p.first << " => " << p.second << "\n";
    }
  */

    return result;
}


string StartHandoverHandler ( uint16_t rnti) {
   ostringstream os_str_ueaddr;
   string enb_ip_addr, enbmacaddr;
   string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";
   string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));
   Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);
   
   for (uint16_t t = 0; t < match.GetN (); t++) {
      Ptr<Object> enbNetDevice = match.Get (t);

      Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
      Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
      Ptr<LteEnbMac> enbMac = enbLteDevice->GetMac ();

      ostringstream os_str_macaddr;

      bool ueManagerFound = enbRrc->HasUeManager (rnti);

      if (ueManagerFound) { // does bool = 1 mean true? confirm
         os_str_macaddr << enbMac;
         enbmacaddr = os_str_macaddr.str();
         std::map<string, string>::iterator it = ipaddr_macaddr_mapping.find(enbmacaddr);
         enb_ip_addr = it->second;
      }
      else {
         continue;
      }
   }

   return enb_ip_addr;
 
}

// function that finds the trajectory matching that of the ue passed parameter
// then if this trajectory has come to an end, remove the ue from the map of
// connected ues [connected_imsi_ips_to_enbips_mapping]

void HouseKeeper(string finished_ue) {
  auto it = connected_imsi_ips_to_enbips_mapping.find(finished_ue);

  if (it != connected_imsi_ips_to_enbips_mapping.end()) {
     connected_imsi_ips_to_enbips_mapping.erase(it);
  }

}


string GetValueInMapMatchingKey (string key) {

   map<string, string>::iterator itr = connected_imsi_ips_to_enbips_mapping.begin();
    
   string str_key, str_value;

   while (itr != connected_imsi_ips_to_enbips_mapping.end()) {
       ostringstream os_str1, os_str2;
       os_str1 << itr->first;
       str_key = os_str1.str();
       if (str_key.find(key) != std::string::npos) {
	   os_str2 << itr->second;
	   str_value = os_str2.str();
       }
       ++itr;
   }

   return str_value;
}


string GetCellID (string ip_add) {
   map<string, string>::iterator itr = connected_imsi_ips_to_enbips_mapping.begin();

  string str_key, existing_cell_id;

   while (itr != connected_imsi_ips_to_enbips_mapping.end()) {
       ostringstream os_str1, os_str2;
       os_str1 << itr->first;
       str_key = os_str1.str();

       std::size_t n = str_key.rfind("_");       

       string actual_ip = str_key.substr(n+1, str_key.size());

       if (actual_ip == ip_add ) {
	   
	  std::size_t found = str_key.find_first_of("_");
          if (found != std::string::npos) {
             existing_cell_id = str_key.substr(found+1, 1);
          }
       }
          ++itr;
   }

   return existing_cell_id;

}

/*
###################################################################################################
 ================== End of function set ==========================
###################################################################################################
*/

/*
###################################################################################################
Handover Trace Functions
###################################################################################################
*/


//IMSI = International Mobile Subscriber Identitiy
//RNTI = Radio Network Temporary Identifier
void NotifyConnectionEstablishedUe (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{

    string ip_add;   
   
      string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);
      ip_add = GetIpFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);

      if (ip_add.length() > 0) {

         string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, ip_add);         
         string imsi_rnti_ipadd = to_string(imsi)+"_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;

         connected_ips_to_enbips_mapping.insert(TStrStrPair( ip_add, enb_ip_addr));
	 connected_imsi_ips_to_enbips_mapping.insert(TStrStrPair( imsi_rnti_ipadd, enb_ip_addr));

	 int count = GetCountEnBEntries (connected_ips_to_enbips_mapping, enb_ip_addr);

	 cout << endl << Simulator::Now ().GetSeconds () << " " << "Connection Established Ue: "
            << " UE IMSI " << imsi
            << " with IP: " << ip_add // id_rnti_cellid
            << " connected to CellId " << cellid
            << " with RNTI" << rnti
            << ". " << count << " UEs are now connected to EnB with IP: "
            << enb_ip_addr
            << endl;

      }

     else {
	std::vector <pair <string, string >>  in_handover_mode_ues = Difference(connected_ips_to_enbips_mapping, uernti_ips_to_enbips_mapping);
        string actual_cell_id = "CellID_"+to_string(cellid);
	string seg_one;
        string actual_cellid_rnti = "CellID_"+to_string(cellid)+"_"+to_string(rnti);;

        for (auto p : in_handover_mode_ues) {
	   ostringstream _cellid_rnti;
	   _cellid_rnti << p.first;
	   string str_cell_rnti = _cellid_rnti.str();
	   std::size_t found = str_cell_rnti.find_first_of("_"); //rfind("_");

	   std::size_t n = str_cell_rnti.rfind("_"); 
	   seg_one = str_cell_rnti.substr(0, n); 
	   ip_add = str_cell_rnti.substr(n+1, str_cell_rnti.length());

           if (found != std::string::npos) {            
               string existing_cell_id = str_cell_rnti.substr(0, found+2);  //, i.size());
               if (existing_cell_id == actual_cell_id) {          
	          std::map<string, string>::iterator iter = connected_ips_to_enbips_mapping.find(seg_one); 
	          string new_rnti_comb = actual_cellid_rnti+"_"+ip_add;

                  if (iter == connected_ips_to_enbips_mapping.end()) {
	             ostringstream output_for_enb;
                     string str_enb;		 
                     output_for_enb << p.second;

                     string enb_ip_addr =  output_for_enb.str(); //iter->second;

		     string imsi_rnti_ipadd = to_string(imsi)+"_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
                     connected_ips_to_enbips_mapping.insert(TStrStrPair( new_rnti_comb, enb_ip_addr));

                     connected_imsi_ips_to_enbips_mapping.insert(TStrStrPair( imsi_rnti_ipadd, enb_ip_addr));
	             int count = GetCountEnBEntries (connected_ips_to_enbips_mapping, enb_ip_addr);

                     cout << endl << Simulator::Now ().GetSeconds () << " " << "Connection Established Ue: "
                        << " UE IMSI " << imsi
                        << " with IP: " << new_rnti_comb //ip_add // id_rnti_cellid
                        << " connected to CellId " << cellid
                        << " with RNTI" << rnti
                        << ". " << count << " UEs are now connected to EnB with IP: "
                        << enb_ip_addr
                        << endl;
	          }
	       }
	   }
        }
    }

}


void NotifyHandoverStartUe (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId)
{

  string moving_ue = to_string(imsi)+"_"+to_string(cellid)+"_"+to_string(rnti);
  string ip_add = GetIpFromMap(connected_imsi_ips_to_enbips_mapping, moving_ue);
  string enb_ip_addr = GetValueInMapMatchingKey (moving_ue);
  
  handed_over_ues.insert((TStrStrPair( ip_add, enb_ip_addr)));

  cout << Simulator::Now ().GetSeconds () << " " << "Handover Start Ue: "
        << " UE IMSI " << imsi
        << " with IP: " << ip_add
        << " previously connected to CellId " << cellid
	<< " on ENB: " << enb_ip_addr
        << " with RNTI " << rnti
        << ", doing handover to CellId " << targetCellId
        << endl;
}


void NotifyHandoverEndOkUe (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    string str_imsi = to_string(imsi);
    string ip_add, enb_ip_addr; //, existing_handed_over_imsi;
    std::map<string, string>::iterator it = handed_over_ues.begin();

    while (it != handed_over_ues.end()) {
       ostringstream _imsi;
       _imsi << it->first;
       string old_str_imsi = _imsi.str();
       // std::size_t found = old_str_imsi.find_first_of("_");
       connected_imsi_ips_to_enbips_mapping.erase(old_str_imsi);

       std::size_t n = old_str_imsi.rfind("_");
       ip_add = old_str_imsi.substr(n+1, old_str_imsi.length());

       ostringstream output_for_enb;
       string str_enb;
       output_for_enb << it->second;

       enb_ip_addr =  output_for_enb.str();

       it++;
    }

    string new_enb_ip = StartHandoverHandler ( rnti);

    string new_str_ue_cell_rnti = to_string(imsi)+"_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
    connected_imsi_ips_to_enbips_mapping.insert(TStrStrPair( new_str_ue_cell_rnti, new_enb_ip));

    int count = GetCountEnBEntries (connected_imsi_ips_to_enbips_mapping, new_enb_ip);
    int _count = GetCountEnBEntries (connected_imsi_ips_to_enbips_mapping, enb_ip_addr);
	
        cout << Simulator::Now ().GetSeconds () << " " << "Handover End Ue: "
            << " UE IMSI " << imsi
	    << " with IP: " << ip_add
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
	    << " on ENB with IP: " << new_enb_ip
	    << ". Total UEs attached after handover on Current ENB: " << count
	    << ". Total UEs attached to previous ENB: " << _count 
            << endl;

   handed_over_ues.clear();
}


void NotifyConnectionEstablishedEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{     
   string id_rnti_cellid = to_string(imsi)+"_"+to_string(cellid)+"_"+to_string(rnti);
   string ip_add = GetIpFromMap(connected_imsi_ips_to_enbips_mapping, id_rnti_cellid);

   std::size_t n = ip_add.rfind("_");
   ip_add = ip_add.substr(n+1, ip_add.length());
   string enb_ip_addr = GetValueInMapMatchingKey (id_rnti_cellid);

   int count = GetCountEnBEntries (connected_imsi_ips_to_enbips_mapping, enb_ip_addr);  

   cout << Simulator::Now ().GetSeconds () << " " << "Connection Established eNB: "
         << " eNB with IP: " << enb_ip_addr
         << " CellId " << cellid
         << " successful connection of UE with IMSI " << imsi 
         << " and IP: " << ip_add
         << " RNTI " << rnti
         << ". Total UEs attached:  " << count
         << endl;   
}


void NotifyHandoverStartEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId)
{
  cout << Simulator::Now ().GetSeconds () << " " << "Handover Start eNB: "
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << endl;

}


void NotifyHandoverEndOkEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    string str_imsi = to_string(imsi);
    string old_enb_ip_addr, ip_add, existing_handed_over_imsi; 

    vector< pair <string, string>>::iterator it = in_handover_mode_ues.begin();

    while (it != in_handover_mode_ues.end()) {
       ostringstream _imsi;
       _imsi << it->first;
       string old_str_imsi = _imsi.str();
       std::size_t found = old_str_imsi.find_first_of("_");

       existing_handed_over_imsi = old_str_imsi.substr(0, found+2);

       std::size_t n = old_str_imsi.rfind("_");
       ip_add = old_str_imsi.substr(n+1, old_str_imsi.length());

       ostringstream output_for_enb;
       string str_enb;
       output_for_enb << it->second;

       old_enb_ip_addr =  output_for_enb.str();
    }

    if (existing_handed_over_imsi == str_imsi) {
  
        string enb_ip_addr = StartHandoverHandler (rnti);

        // int count = GetCountEnBEntries (connected_imsi_ips_to_enbips_mapping, enb_ip_addr);

        cout << Simulator::Now ().GetSeconds () << " " << "Handover End eNB: "
	    << " from: " << old_enb_ip_addr
            << " to eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
	    << " with IP: " << ip_add
            << " RNTI " << rnti
	    << " after handover to ENB with IP: " << enb_ip_addr
	    << " total attached UEs is now " << rnti // it->second 
            << endl;
    }
}


void NotifyUeReport (string context, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, bool servingCell, uint8_t carrierid) 
{   
   if (servingCell == 1) {      
       map<string, string>::iterator itr = connected_imsi_ips_to_enbips_mapping.begin();
       string str_key, existing_cell_id, exising_rnti;

       while (itr != connected_imsi_ips_to_enbips_mapping.end()) {
          ostringstream os_str1, os_str2;
          os_str1 << itr->first;
          str_key = os_str1.str();

          std::size_t found = str_key.find_first_of("_");
          existing_cell_id = str_key.substr(found+1, 1); // found+1
	  exising_rnti = str_key.substr(found+3, 1 );
          std::size_t n = str_key.rfind("_");
          string ip_add = str_key.substr(n+1, str_key.size());

	  for (auto& it : current_profiles_ue_add) 
	  {
              if ( it == ip_add ) {
                  if ((to_string(cellId) == existing_cell_id) && (to_string(rnti) == exising_rnti)) {
		       string value = to_string(rsrq)+"_"+to_string(rsrp);
                       // std::cout << " RNTI = " << rnti
		       //	         << " IP = " << ip_add
                       //          << " CellID = " << cellId
                       //          << " RSRQ = " << rsrq
                       //          << " RSRP = " << rsrp
                             //   << " SERVINGCELL = " << servingCell
                             // << " CARRIER ID = " << carrierid
                       //          << endl;

		        auto it = vect_rsrq_rsrp.find(ip_add); 
  
                        if (it != vect_rsrq_rsrp.end()) { 
                           vect_rsrq_rsrp.erase(it); 
                        } 
			else {
		           vect_rsrq_rsrp.insert(TStrStrPair(ip_add, value));
	                }

                  }
              }
	  }

          ++itr;
      }
   }
}


void NotifyNewUeContext (string context, uint16_t cellid, uint16_t rnti) 
{
   
   ostringstream os_str_ueaddr;
   string enb_ip_addr, enbmacaddr; // umsi_plus_rnti, ue_addr, enbmacaddr;
   string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";

   string uePath = "/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/$ns3::LteUeNetDevice";
 
   string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));

   string ueNetDevicePath = uePath.substr(0, uePath.find ("/LteEnbRrc"));

   Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);

   Config::MatchContainer ue_match = Config::LookupMatches (ueNetDevicePath);

  
   if (counter < int(all_node_list.size())) {
       string curr_ue = all_node_list[counter];
       string uernti_ip;

       for (uint16_t t = 0; t < match.GetN (); t++) {
          Ptr<Object> enbNetDevice = match.Get (t);

          Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
          Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
          Ptr<LteEnbMac> enbMac = enbLteDevice->GetMac ();

          ostringstream os_str_macaddr;

          bool ueManagerFound = enbRrc->HasUeManager (rnti);

          if (ueManagerFound) { // does bool = 1 mean true? confirm
             os_str_macaddr << enbMac;
             enbmacaddr = os_str_macaddr.str();

	     std::map<string, string>::iterator it = ipaddr_macaddr_mapping.find(enbmacaddr);
             enb_ip_addr = it->second;
	    
          }
          else {
             continue;
          }

       }
     
       uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+curr_ue;
       string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);
       cellid_ueip_mapping.insert(std::pair<string, string>("CellID_"+to_string(cellid), curr_ue));
       uernti_ips_to_enbips_mapping.insert(TStrStrPair(uernti_ip, enb_ip_addr));
       string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);
       string ip_add = GetIpFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);

       int count = GetCountEnBEntries (uernti_ips_to_enbips_mapping, enb_ip_addr);
   
       cout << Simulator::Now ().GetSeconds () << " " << "New UE Context: "
             << " eNB CellId " << cellid
             << " RNTI " << rnti
             << " with IP " << curr_ue
             << " Total UEs attached to " << enb_ip_addr << " is "
             << count
             << endl;
   
   } 
   
   counter = counter + 1;

}


int GetNumberOfEnbsInFile (string filename) 
{

  vector<string> v_enb;
  string s;

  fstream enbFile, uesFile;
  enbFile.open(filename, ios::in);
  
  while(getline(enbFile, s)){
     v_enb.push_back(s);
  }

  enbFile.close();

  return v_enb.size();
}

int GetNumberOfUesInFile (string filename)     
{
  // Read from the sumo trajectories files which are saved in the format:
  // vehID Time:locationX:locationY

  vector<string> v_ues;
  string s;

  fstream  uesFile;
  uesFile.open(filename, ios::in);

  while(getline(uesFile, s)){
     v_ues.push_back(s);
  }

  uesFile.close();

  return v_ues.size();
}


string GetUesIds(string line) {

   string left_part, right_part;
   int start = 0, index_of_space_delim;

   index_of_space_delim = line.find(" ", start);

   left_part = line.substr(start, index_of_space_delim);
   right_part = line.substr(index_of_space_delim, line.size());

   return left_part;
}


struct BearerData
  {
    uint32_t bid;
    Ptr<PacketSink> dlSink;
    Ptr<PacketSink> ulSink;
    uint32_t dlOldTotalRx;
    uint32_t ulOldTotalRx;
  };

struct UeData
  {
    uint32_t id;
    std::list<BearerData> bearerDataList;
  };


int getIndex(vector<string> v, string K)
{
    auto it = find(v.begin(), v.end(), K);

    // If element was found
    if (it != v.end())
    {
      // At what index is the node in the list of nodes
       int index = it - v.begin();
       return index;
    }
    else {
       // If the element is not present in the vector
       return -1;
    }
}

/*
void setupStationaryUes (MobilityHelper ueMobHelper, NodeContainer ueNodes, int num_ues, NetDeviceContainer enbDevs) {

    ueMobHelper.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                       "X", DoubleValue (20.0),
                                      "Y", DoubleValue (0.0),
                                       "rho", DoubleValue (radius));
    ueMobHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

   
    for (uint16_t i = 0; i < num_ues; i++) {
       ueMobHelper.Install (ueNodes[i]);
  
       NetDeviceContainer ueDevs+"_"+i;
       ueDevs+"_"+i = lteHelper->InstallUeDevice (ueNodes[i]);
       // ueDevs2 = lteHelper->InstallUeDevice (ueNodes2);
  
       // Attach UEs to a eNB
       // lteHelper->Attach (ueDevs1, enbDevs);
       lteHelper->Attach (ueDevs+"_"+i, enbDevs);
   
       // Activate an EPS bearer on all UEs
       enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
       EpsBearer bearer (q);
       lteHelper->ActivateEpsBearer (ueDevs+"_"+i, bearer, EpcTft::Default ());
       // lteHelper->ActivateEpsBearer (ueDevs2, bearer, EpcTft::Default ());
    }
}
*/

// This function will solve the TODO bbelow where we setup boot ip.
// The ips are available in the vector returned by this function
std::vector<string> getNodesList ( NodeContainer ueNodes) {
  string s, node_id, right_part;
  int index_of_space_delim, j = 0, start;
  vector<string> nodes_list;
  fstream uesfile;
  uesfile.open("scratch/trajectory.txt",ios::in);

  while(getline(uesfile, s)){

     start = 0;
     index_of_space_delim = s.find(" ", start);
     right_part = s.substr(index_of_space_delim, s.size());
     istringstream ss(right_part);
     string  real_ip; // trunct
     ostringstream str_add;

     str_add << ueNodes.Get(j)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
     node_id = str_add.str();

     nodes_list.push_back(node_id);
     j++;
  }
  
  uesfile.close();
  return nodes_list;

}


std::map<string, string> getIpAddMacAddMapping (NodeContainer enbNodes) {

    string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";

    string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));

    Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);

    for (uint16_t t = 0; t < match.GetN (); t++) {

       string node_ip, enbmacaddr;
       ostringstream os_str_add;
       ostringstream os_str_macaddr;

       Ptr<Object> enbNetDevice = match.Get (t);
       Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
       Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
       Ptr<LteEnbMac> enbMac = enbLteDevice->GetMac ();

       os_str_macaddr << enbMac;
       os_str_add << enbNodes.Get(t)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
       enbmacaddr = os_str_macaddr.str();
       node_ip = os_str_add.str();
       ipaddr_macaddr_mapping.insert({enbmacaddr, node_ip});
       // std::cout<<"** TEST *** "<<enbmacaddr<<" **** "<<node_ip<<"\n";
    }

  /*
    for (auto i = ipaddr_macaddr_mapping.begin(); i!= ipaddr_macaddr_mapping.end(); i++) {
       std::cout <<"IP-MAC MAPPING: "<< i->first << "\t" << i->second << endl;
    }
  */  

    return ipaddr_macaddr_mapping;
 }	 

/*
###################################################################################################
 Algorithm: As nodes get thru the flowmonitor we need to track when their 
 trajectories have come to the end so that we move the node out of the list
 of flow monitor proble candidates.
 We store the ips and the sizes of the trajectories in a map so that at each 
 sweep for a course change, the size of the trajectory is reduced by one for 
 each node. Then if the size shrinks to 0, remove the node from the list of 
 flow monitor probing.
###################################################################################################
*/


int updateTrajectorySizes (vector< pair <string, int> >& node_trajectory_sizes, string dest_ip, vector<string> curr_nodes) {

   int curr_size; //  = 0;

   for(int i =0; i<int(node_trajectory_sizes.size()); i++)
    { 
	if ((node_trajectory_sizes[i].first == dest_ip ) && (std::find(curr_nodes.begin(), curr_nodes.end(), dest_ip) != curr_nodes.end())) {
           // std::cout<<"\n ======= "<<dest_ip <<" ====>> "<<node_trajectory_sizes[i].second;
	   curr_size = node_trajectory_sizes[i].second - 1;
           node_trajectory_sizes[i].second = curr_size;
	   break;
	}

    }

   return curr_size;

}

vector< pair <string, int> > getNodesTrajectorySizes (vector<int> speed_sizes, vector<string> nodes) {

   vector< pair <string, int> >  node_trajectory_sizes;

   // assert(speed_sizes.size() == nodes.size());

   for (int i = 0; i < int(nodes.size()); i++) {
      node_trajectory_sizes.push_back(make_pair(nodes[i], speed_sizes[i])); 
   }

   return node_trajectory_sizes;
}


void setNodesTrajectorySizes (vector<int> speed_sizes, vector<string> nodes) {

   vector< pair <string, int> >  node_trajectory_sizes;

   for (int i = 0; i < int(nodes.size()); i++) {
      original_trajectory_sizes.push_back(make_pair(nodes[i], speed_sizes[i]));
   }

}


int getOriginalTrajectorySize (vector< pair <string, int> > original_trajectory_sizes, string ipadd) {

   int original_size;

   auto it = std::find_if(original_trajectory_sizes.begin(), original_trajectory_sizes.end(), [&ipadd](std::pair<string, int> i) { return i.first == ipadd; });

   if(it != original_trajectory_sizes.end())
   {
      original_size = (*it).second;
      // std::cout<<"\n - - - - "<< original_size;
   }

   return original_size;
}


// Calculate the distance between two nodes given their 
// Cartesian coordinates

double CalculateDistance(Point a, Point b) {
  double sum = sqrt(pow((b.x - a.x), 2) + pow((a.y - b.y), 2));
  return sum;
}


Point GetUEAttachedToEnBXYCoords (string ueip) {

	map<string, string> vect_ip_pose;
        vect_ip_pose = setEnBIPToPoseMap ("scratch/enbs.txt");
	std::string enb_xy_location;

        map<string, string>::iterator it = connected_ips_to_enbips_mapping.begin();
        map<string, string>::iterator itr = vect_ip_pose.begin();

        while (it != connected_ips_to_enbips_mapping.end()) {
	  
          ostringstream ss, tt, strstrm;

          ss << (*it).first;
          tt << (*it).second;

          string simulation_ueip = ss.str();
          string simulation_enbip = tt.str();

          for (itr = vect_ip_pose.begin(); itr != vect_ip_pose.end(); ++itr) {

             ostringstream v_ss, v_tt;
             v_ss << (*itr).first;
             string static_enbip = v_ss.str();

             if (simulation_enbip == static_enbip) {

		if (simulation_ueip.find(ueip) != std::string::npos) {		               
		    v_tt << (*itr).second;
                    enb_xy_location = v_tt.str();
		}
             }

             v_ss.clear();
             v_tt.clear();
          }

          ss.clear();
          tt.clear();
          ++it;
        }
      
      size_t found;
      Point enb_xy_coords;

      if ((found = enb_xy_location.find(" ")) != string::npos) {
	 std::string t = enb_xy_location.substr(0,found);
         std::string q = enb_xy_location.substr(found+1, enb_xy_location.size()-1);
         enb_xy_coords.x = atof(t.c_str()); 
	 enb_xy_coords.y = atof(q.c_str()); 
      }

     return enb_xy_coords;
}


void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon,Gnuplot2dDataset DataSet,  string ip_add, vector<string> speeds) // , NetDeviceContainer ueDevs, NetDeviceContainer enbDevs)
  {
     double localThrou=0;
     
     std::vector<string> curr_node_list;
     int node_counter = 0;
     int curr_node_size;
      
     for (int t = 0;  t < int(motional.size()); t++) {
        curr_node_list.push_back(all_node_list[node_counter]);
        node_counter++;
     }    
    
     std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
     Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());  
     for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
      {
         int counter = motional.size();
        	
         // string command = "./metric_writer.py ";
         string src_ipadd, dest_ipadd, tx, rx, thruput, avg_delay, avg_jitter, lst_pkts, x, y;
         ostringstream str_src, str_dest;

         Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
         str_src << fiveTuple.sourceAddress;
         str_dest << fiveTuple.destinationAddress;
         src_ipadd = str_src.str();
         dest_ipadd = str_dest.str();
	 
	 if (counter == 0) {
            break;
         }
         else {          
            int index_of_destip = getIndex(curr_node_list, dest_ipadd);          
 	    string x, y, vel_x, vel_y;
	    int index = 0; // std::distance(vec_nodes.begin(), it);
	    string command = "./metric_writer.py ";

	    if (index > int(curr_node_list.size())) {
               break;
            }

 	    else {
	       if (index_of_destip >= 0) {
		  x = coordinates[index_of_destip].first;
		  y = coordinates[index_of_destip].second;
		  vel_x = directional_speeds[index_of_destip].first;
		  vel_y = directional_speeds[index_of_destip].second;

	          int original_node_trajectory_size;

                  original_node_trajectory_size = getOriginalTrajectorySize (original_trajectory_sizes, dest_ipadd);

                  if (trajectory_sizes.size() > 0 ) {
                     curr_node_size = updateTrajectorySizes (trajectory_sizes, dest_ipadd, all_node_list);
                     int diff_size = original_node_trajectory_size - curr_node_size;

                     if (diff_size == original_node_trajectory_size) {
                         remove(all_node_list, dest_ipadd);
                         processed_nodes.push_back(dest_ipadd);

                         if (all_node_list.size() == 0) {
                             Simulator::Stop (Seconds (0.1));
                         }
                     }


		     if (fiveTuple.sourceAddress == Ipv4Address("1.0.0.2") && (dest_ipadd == curr_node_list[index_of_destip])) // (dest_ipadd == vec_nodes[index_of_destip])) 
                      {
			current_profiles_ue_add.push_back(dest_ipadd);
			string enb_ip_addr = GetValueInMapMatchingKey(dest_ipadd); // GetValueInMapMatchingKey (string key)

                        string cellID_attached_to = GetCellID(dest_ipadd);

                        int num_connected_ues = GetCountEnBEntries (connected_imsi_ips_to_enbips_mapping, enb_ip_addr); 

			Point  enb_attachedto_location = GetUEAttachedToEnBXYCoords(dest_ipadd);

	                tx = to_str(stats->second.txPackets);
                        rx = to_str(stats->second.rxPackets);

			// convert x and y to double
			double x_ = atof(x.c_str());
			double y_ = atof(y.c_str());
			
			Point ue_pose;
			ue_pose.x = x_;
			ue_pose.y = y_;

			double dist_btn_enb_and_ue = CalculateDistance(ue_pose, enb_attachedto_location);

                        thruput = to_str(stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024);
                        avg_delay = to_str(stats->second.delaySum.GetSeconds()/stats->second.rxPackets);
                        avg_jitter = to_str(stats->second.jitterSum.GetSeconds()/(stats->second.rxPackets));
                        lst_pkts = to_str(stats->second.lostPackets);

			if(!vect_rsrq_rsrp.empty()) {
                           std::map<string, string>::iterator it;
        		   it = vect_rsrq_rsrp.find(dest_ipadd);
			   if(it != vect_rsrq_rsrp.end()) {
			      ostringstream rsrq_rsrp_value;
 			      rsrq_rsrp_value << it->second; // std::prev(vect_rsrq_rsrp.end())->second;
			      string str_value = rsrq_rsrp_value.str();
                              
			      string str_rsrq, str_rsrp;
                              size_t found = str_value.find_first_of("_");

				 str_rsrq = str_value.substr(0, found);
				 str_rsrp = str_value.substr(found+1, str_value.length());

	                         command += x+":"+y+":"+vel_x+":"+vel_y+":"+src_ipadd+":"+dest_ipadd+":"+tx+":"+rx+":"+thruput+":"+avg_delay+":"+avg_jitter+":"+lst_pkts+":"+to_string(dist_btn_enb_and_ue)+":"+cellID_attached_to+":"+enb_ip_addr+":"+to_string(num_connected_ues)+":"+str_rsrq+":"+str_rsrp;
                                 std::cout<<"\n ++++ "<<command <<endl;
	                         system(command.c_str());
		 	   }
		        }
	      
                        if (motional.size()>= 1) {
		           x.erase();
                           y.erase();
                           tx.erase();
                           rx.erase();
                           thruput.erase();
                           avg_delay.erase();
                           avg_jitter.erase();
                           lst_pkts.erase();                              
                        }
	                command.erase(); 
	                index++;               
                      }

                  }
		 
		 // Take care of the care that the vehicle trajectory came to an end i.e.vehicle left the simulation
		 if (trajectory_sizes.size() == 0 ) {
		    HouseKeeper(dest_ipadd);	            		 
	         }
 	       }
	    }

            localThrou=(stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024);
            // updata gnuplot data
            DataSet.Add((double)Simulator::Now().GetSeconds(),(double) localThrou);       
         }

     }
    Simulator::Schedule(Seconds(0.5),&ThroughputMonitor, fmhelper, flowMon,DataSet, ip_add, speeds); //  ueDevs, enbDevs);
    flowMon->SerializeToXmlFile ("ThroughputMonitor.xml", true, true);
    motional.clear();
    coordinates.clear();
    directional_speeds.clear();

  }


//  Prints actual position and velocity when a course change event occurs

static void
CourseChange (std::string foo, Ptr<const MobilityModel> ueMobility)
{
   Vector pos = ueMobility->GetPosition (); // Get position
   Vector vel = ueMobility->GetVelocity (); // Get velocity
   // string command = "./pose_writer.py ";
   ostringstream x_pos, y_pos, x_speed, y_speed;
   x_pos << pos.x;
   y_pos << pos.y;
   x_speed << vel.x;
   y_speed << vel.y;
   string x = x_pos.str();
   string y = y_pos.str();
   string xsp = x_speed.str();
   string ysp = y_speed.str();

   Vector2D  uepos, enbpos;
   uepos.x = pos.x;
   uepos.y = pos.y;

  // system(command.c_str());
  // cout<<"\n =======>> "<< command;
   //std::cout <<"\n => "<< Simulator::Now ().GetSeconds() << "\n POS: x=" << pos.x << ", y=" << pos.y
   //    << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
   //    << ", z=" << vel.z << std::endl;
   
   motional[x] = y;   
   coordinates.push_back( make_pair(x, y));
   directional_speeds.push_back(make_pair(xsp, ysp));

}


int main (int argc, char *argv[])
{

/*
###################################################################################################
Parameters
###################################################################################################
*/
  //Transmission Power
  double enbTxPowerDbm = 46.0; // About 500 meters (46)

  //Number of Bearers per Ue
  uint32_t numOfBearers = 1;

  //Simulation time thus a total of 51.2
  int sim_time = 1200.0; //51.2;

  vector<string> v_enb, v_ue;
  string s;

  // uint16_t port = 8000;

  // static vector<string> nodes;

  int64_t stream = 1;

  // assing central frequencies for each enb
  bool m_admitHo = true;
  // vector<Ues> ueMetrics;
  vector<float> ueMetrics;

  vector<float> qos_params;

  fstream enbFile, uesFile;

  enbFile.open("scratch/enbs.txt",ios::in);

  uesFile.open("scratch/trajectory.txt",ios::in);

  // I need to get access to the config store so that 
  // I can trap trace source sinks

  ConfigStore config;
  config.ConfigureDefaults ();

  uint16_t numberOfUes = GetNumberOfUesInFile("scratch/trajectory.txt");
  uint16_t numberOfEnbs = GetNumberOfEnbsInFile("scratch/enbs.txt");

  std::vector<UeData> m_ueDataVector;

/*
###################################################################################################
Set fading model 
###################################################################################################
*/
  int fadingmodel = 0;
  if (fadingmodel != 0){
    lteHelper->SetFadingModel("ns3::TraceFadingLossModel");
    switch (fadingmodel)
    {
    case 1/* constant-expression */:
      lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
      break;

    case 2:
      lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_ETU_3kmph.fad"));
      break;

    case 3:
      lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_EVA_3kmph.fad"));
      break;

    default:
      break;
    }  
    lteHelper->SetFadingModelAttribute ("TraceLength", TimeValue (Seconds (10.0)));
    lteHelper->SetFadingModelAttribute ("SamplesNum", UintegerValue (10000));
    lteHelper->SetFadingModelAttribute ("WindowSize", TimeValue (Seconds (0.5)));
    lteHelper->SetFadingModelAttribute ("RbNum", UintegerValue (100));
  }

/*
###################################################################################################
LTE Architecture 
###################################################################################################
*/

  //Creating the LTE helper object and setting EPC helper to it
  //eNodeB connets to EPC which is consisted of SGW, PDN, MME and HSS

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper); 
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler"); //RoundRobin Scheduler

  //Handover algorithm to be set. This is highly likely to be default in LTE network. 
  lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold", UintegerValue (30));
  lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",  UintegerValue (1));

  //Packet Delivery Network GateWay or simply PGW is the gateway to communicate to outside world. 
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

/*
###################################################################################################
Topology 
###################################################################################################
*/

  // Create a single RemoteHost and install internet on it.
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  //Creates a P2P helper and sets the attributes
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500)); //Maximum Transmission Unit
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));

  Ptr<RateErrorModel> em = CreateObject<RateErrorModel> ();
  em->SetAttribute ("ErrorRate", DoubleValue (0.00001));

  //Creates a net device container and adds PGW and remote host to it
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  for  (uint32_t t = 0; t < ueNodes.GetN(); ++t ) {
     internetDevices.Get (t)->SetAttribute ("ReceiveErrorModel", PointerValue (em));
  }

  // Uses Ipv4AddressHelper to make available IPs out of a base IP address and then assigns them to net devices, therefore each of the net devices (PGW and remote host) will have different IPs. (internetIpIfaces.GetAddress 1 being remote host and 0 being PGW)
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());

  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // NodeContainer ueNodes;
  // NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (numberOfUes);
  // internet.Install(ueNodes);

 /*
###################################################################################################
Set positions of enbs as read from the enbLocations.txt file
###################################################################################################
*/
  
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();

  while(getline(enbFile, s)){
         int start = 0, index_of_delim;
         string x_pose, y_pose;
         index_of_delim = s.find(" ", start);
         x_pose = s.substr(start, index_of_delim);
         y_pose = s.substr(index_of_delim, s.size());

	 Vector enbPosition (stof(x_pose), stof(y_pose), 0);
	 enbPositionAlloc->Add (enbPosition);

  }

  enbFile.close();

  MobilityHelper enbMobility;

  enbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator (enbPositionAlloc);
  enbMobility.Install (enbNodes);

// Add some stationed nodes into the simulation...
//   MobilityHelper staticNodes;
//  uint16_t num_ues = 20
//  setupStationaryUes (staticNodes, ueNodes, num_ues, enbLteDevs);

/*
###################################################################################################
Set dynamic ue motion as read from the sumo path data and UE Mobility Model
###################################################################################################
*/

  // Install Mobility Model in UE
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::WaypointMobilityModel");
  ueMobility.Install (ueNodes);
  
/*
###################################################################################################
Installing Internet
###################################################################################################
*/
  // Install LTE Devices in eNB and UEs
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));
  // NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);

  stream += lteHelper->AssignStreams(enbLteDevs, stream);
  for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it)
    {
      Ptr<LteEnbRrc> enbRrc = (*it)->GetObject<LteEnbNetDevice>()->GetRrc();
      enbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(m_admitHo));
    }

  // NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);
  ueLteDevs = lteHelper->InstallUeDevice (ueNodes);
  stream += lteHelper->AssignStreams(ueLteDevs, stream);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  // ipv4h.SetBase ("10.1.1.0", "255.255.255.0");
  ipv4h.SetBase ("192.168.1.0", "255.255.255.0");
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
 
  //ToDo::  Add some stationed nodes into the simulation...
  // MobilityHelper staticNodes;
  // uint16_t num_ues = 20
  // setupStationaryUes (staticNodes, ueNodes, num_ues, enbLteDevs);
 
  string veh_time, veh_speed, node_list; //, trunct;

  string x_pose, y_pose, left_part, right_part, right_part_left, word, x_y_poses, all_cmd, dot_delim, left_over_chunk;
  int start, j = 0, delim, next_delim, index_of_space_delim, next_next_delim;
  vector<string> poses;
  
   Gnuplot2dDataset dataset;
   dataset.SetTitle ("Flow Monitor");
   dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);

   FlowMonitorHelper fmHelper;
   Ptr<FlowMonitor> flowMonitor = fmHelper.InstallAll();
   flowMonitor->CheckForLostPackets ();

   nodes = getNodesList(ueNodes);

   while(getline(uesFile, s)){
 
     start = 0;
     index_of_space_delim = s.find(" ", start);
     // ToDo:: stripping of the space (index_of_space_delim-1) using inbuilt string processing libraries...
     // left_part = s.substr(start, index_of_space_delim);
     right_part = s.substr(index_of_space_delim, s.size());
     // get the count on the last metric in a file
     istringstream ss(right_part);
     string  real_ip; // trunct
     ostringstream str_add, os_speed;

     str_add << ueNodes.Get(j)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
     trunct = str_add.str();
     dot_delim = trunct.find(".", start);

     string command = "./csv_writer.py "+trunct+" ";
     static bool first_time = true;

     while (ss >> word)
     {
	// string command = "./csv_writer.py ";
        delim = word.find(":", start);
        veh_time = word.substr(start,delim);
	left_over_chunk = word.substr(delim+1,word.size());
	next_delim = left_over_chunk.find(":",start);
	veh_speed = left_over_chunk.substr(start,next_delim);
        x_y_poses = left_over_chunk.substr(next_delim+1, left_over_chunk.size());
        next_next_delim = x_y_poses.find(":", start);
        x_pose = x_y_poses.substr(start,next_next_delim);
        y_pose = x_y_poses.substr(next_next_delim+1, x_y_poses.size());

	ueNodes.Get(j)->GetObject<WaypointMobilityModel>()->AddWaypoint(Waypoint(Seconds(stof(veh_time)),Vector(stof(x_pose),stof(y_pose),0)));
        //  Excuse my laziness here,  but can someone save these details using c++ instead of
	//  calling in an extra python script to write to CSV
        speeds.push_back(veh_speed);

     }     
     
      Simulator::Schedule(Seconds(1.0), &attachinfuture, ueLteDevs, enbLteDevs); // j);

      if(first_time) {
         first_time = false;
         Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));
      }
 
      speed_sizes.push_back(speeds.size());     

      ThroughputMonitor(&fmHelper, flowMonitor, dataset, trunct, speeds); // , ueLteDevs, enbLteDevs); //, nodes);      

      veh_time.erase();
      veh_speed.erase();
      x_pose.erase();
      y_pose.erase();

     // write the speeds to csv file here
      os_speed << speeds;
      command += os_speed.str();
      system(command.c_str());
      command.erase();
      speeds.clear();

      j = j + 1;  
     trunct.erase();
    }
  
    uesFile.close();    

    trajectory_sizes = getNodesTrajectorySizes(speed_sizes, nodes);

    setNodesTrajectorySizes(speed_sizes, nodes);

    all_node_list = getNodesList (ueNodes);

    getIpAddMacAddMapping (enbNodes);

     Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                     MakeCallback (&NotifyConnectionEstablishedEnb));
     Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                      MakeCallback (&NotifyConnectionEstablishedUe));
     Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                     MakeCallback (&NotifyHandoverStartEnb));
     Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                     MakeCallback (&NotifyHandoverStartUe));
     Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                      MakeCallback (&NotifyHandoverEndOkEnb));
     Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                     MakeCallback (&NotifyHandoverEndOkUe));
     Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/NewUeContext",		     
                     MakeCallback (&NotifyNewUeContext)); 
     Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteUeNetDevice/ComponentCarrierMapUe/*/LteUePhy/ReportUeMeasurements",
      		     MakeCallback (&NotifyUeReport));
     
/*
###################################################################################################
Application
###################################################################################################
*/

  NS_LOG_LOGIC ("setting up applications");

  //Configurations
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (1))); 
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000000)); 
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true)); //RRC = Radio Resource Control

  uint16_t port = 10000;

  // *********** OnAndOff Application OPTION 1: ***************** //
  /*
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

   Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
   Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate)); 

   OnOffHelper onoff1("ns3::UdpSocketFactory", Address());

   onOffHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
   onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

   for (int i = 0; i < ueNodes.GetN(); i++)
    {
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
        onOffHelper.SetAttribute("Remote", remoteHostAddr);
        ApplicationContainer app = sink.Install (ueNodes.Get(i))

        app = onOffHelper.Install (ueNodes.Get (i));
        // Start the application
        app.Start (Seconds (1.0));
        app.Stop (Seconds (100.0));
    }
 
  */

  // *********** OnAndOff Application OPTION 2: ***************** //

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  lteHelper->AddX2Interface (enbNodes);

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));
 
 
   for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {

       Ptr<Node> ue = ueNodes.Get (u);
       // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        UeData ueData;

        for (uint32_t b = 0; b < numOfBearers; ++b)
         {
            ++dlPort;
            ++ulPort;

            ApplicationContainer clientApps;
            ApplicationContainer serverApps;
            BearerData bearerData = BearerData();

            NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
            {
              UdpClientHelper dlClientHelper (ueIpIface.GetAddress (u), dlPort);
              clientApps.Add (dlClientHelper.Install (remoteHost));
              PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), dlPort));
              ApplicationContainer sinkContainer = dlPacketSinkHelper.Install(ue);
	      bearerData.dlSink = sinkContainer.Get(0)->GetObject<PacketSink>();
              // serverApps.Add (dlPacketSinkHelper.Install (ue));
              // bearerData.dlSink = sinkContainer.Get(0)->GetObject<PacketSink>();
              serverApps.Add(sinkContainer);
            }

            NS_LOG_LOGIC ("installing UDP UL app for UE " << u);

            {
              UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
              clientApps.Add (ulClientHelper.Install (ue));
              PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
              ApplicationContainer sinkContainer = ulPacketSinkHelper.Install(remoteHost);
              bearerData.ulSink = sinkContainer.Get(0)->GetObject<PacketSink>();
              // serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
	      // bearerData.ulSink = sinkContainer.Get(0)->GetObject<PacketSink>();
              serverApps.Add(sinkContainer);
            }  

          Ptr<EpcTft> tft = Create<EpcTft> ();
          // always true: if (epcDl)
          {
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            tft->Add (dlpf);
          }
          // always true: if (epcUl)
          {
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            tft->Add (ulpf);
          }

          // always true: if (epcDl || epcUl)
          EpsBearer bearer (EpsBearer::GBR_CONV_VOICE);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);
 
          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);
         
          ueData.bearerDataList.push_back(bearerData);
        }

       m_ueDataVector.push_back(ueData);

    }

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();

  // Ptr<PhyStatsCalculator> phyStats = lteHelper->GetPhyStats();
  // phyStats->SetAttribute("EpochDuration", TimeValue(Seconds(1.0)));
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));


/*
###################################################################################################
Netanim 
###################################################################################################
*/

/* 
  AnimationInterface anim ("OverlyAnimLTE.xml"); // Mandatory
   
  for (uint32_t i = 0; i < enbNodes.GetN (); ++i)
  {
    anim.UpdateNodeDescription (enbNodes.Get (i), "eNB"); // Optional
    anim.UpdateNodeColor (enbNodes.Get (i), 255, 0, 0); // Optional
    Ptr<MobilityModel> mob = enbNodes.Get(i)->GetObject<MobilityModel>();
    Vector position = mob->GetPosition();
    anim.SetConstantPosition (enbNodes.Get(i), position.x, position.y);
    cout<<"Positions of ENB Nodes: "<<position.x <<" "<<position.y<<endl;
  }

  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
  {
    anim.UpdateNodeDescription (ueNodes.Get (i), "Vehicle"+std::to_string(i));
    anim.UpdateNodeColor (ueNodes.Get (i), 0, 0, 255); // Optional 
  } 

  // anim.EnablePacketMetadata (); // Optional
  anim.SetMaxPktsPerTraceFile (9999999999999);

  // Interesting idea, we could use the topographic capture from SUMO here........
  // anim.SetBackgroundImage ("/home/elemenohpi/Desktop/map.jpg", 0.0, 0.0, 1.0  , 1.0, 1.0);
  anim.SetMobilityPollInterval (Seconds (0.1));
 
*/

/*
###################################################################################################
PCAP && ASCII Tracing
###################################################################################################
*/

  // Uncomment to enable PCAP tracing
  p2ph.EnablePcapAll("thesis_pcap/lena-x2-handover-measures");

  // Set up tracing
  // AsciiTraceHelper ascii;
  // p2ph.EnableAsciiAll (ascii.CreateFileStream ("thesis_ascii/lte-tcp-single-flow.tr"));
  // p2ph.EnablePcapAll ("lte-tcp-single-flow", false);
  // Setup tracing for cwnd

/*
###################################################################################################
Simulation Start
###################################################################################################
*/

  Simulator::Stop (Seconds (sim_time));
  Simulator::Run ();
  
/*
###################################################################################################
Custom Performance Trace
###################################################################################################
*/
/*
  Ptr<PacketSink> sink;

  //Recieved Ue
  sink = DynamicCast<PacketSink> (serverApps.Get(0));
  uint64_t rx_ue = sink->GetTotalRx();

  //Recieved Remote Host
  sink = DynamicCast<PacketSink> (serverApps.Get(1));
  uint64_t rx_rh = sink->GetTotalRx();

  cout<<"UE recieved count: "<< rx_ue<< endl;
  cout<<"RH recieved count: "<< rx_rh <<endl;

  // std::cout << "\nTotal Throughput (Mbps): dev "<< i << ": "<<sink[i]->GetTotalRx ()*8/sim_time/1000000 << std::endl;
*/
/*
###################################################################################################
Simulation End
###################################################################################################
*/
  Simulator::Destroy ();

  return 0;
}

