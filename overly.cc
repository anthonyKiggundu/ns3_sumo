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
#include "ns3/evalvid-client-server-helper.h"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/combine.hpp>

// #include "ns3/epc-sgw-pgw-application.h"

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

NS_LOG_COMPONENT_DEFINE ("SumoNS3UesTrajectoriesHandoverMeasures");

Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();

// void attachinfuture(NetDeviceContainer& ues, int ueid){
void attachinfuture(NetDeviceContainer& uesdev, NetDeviceContainer& enbsdev){
  // lteHelper->Attach (ues.Get(ueid)); 
  // ToDo:: Try to do the counting of how many nodes are attached to 
  // given ENB at this point and at the end of a handover
  // In short: find the enbdevice that is used here for attaching th
  // ue, what methods does the NetDeviceContainer expose that can be used?
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
vector<string> handover_lst_rntis;
vector<string> newUe_lst_rntis; // , lst_ips;
std::map<std::string, string> motional;
vector< pair <string, string> > coordinates;
vector< pair <string, string> > directional_speeds;
vector< pair <string, int> > node_trajectory_sizes;
vector< pair <string, int> > trajectory_sizes;
map< string, string>  ipaddr_macaddr_mapping;
map<string, string> vect_ip_pose;

typedef map<string, string> TStrStrMap;
typedef pair<string, string> TStrStrPair;
TStrStrMap uernti_ips_to_enbips_mapping;
TStrStrMap connected_ips_to_enbips_mapping;

map<string, string> cellid_ueip_mapping;
int runner = 0, counter = 0;
double dist_to_enb = 0.0;
vector<string> connected_ues;


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
        std::cout<<"\n "<< element; 
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


int GetCountEnBEntries (map< string, string> m, string enb_ip) {

    int count = std::count(m.begin(), m.end(), Compare(enb_ip));

    // std::cout << count << "\n";

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


string SimpleGetIpFromMap (map<string, string> cellid_ueip_mapping, string cellid) {

    // map<string, string>::iterator itr = cellid_ueip_mapping.begin();

    // while (itr != cellid_ueip_mapping.end()) {
    //    cout << "Key: " << itr->first
    //        << ", Value: " << itr->second << endl;
    //   ++itr;
    //}

    string ue_ipaddr;

    map<string, string>::iterator it; // = cellid_ueip_mapping.begin();
   
    it = cellid_ueip_mapping.find(cellid);

    if (it != cellid_ueip_mapping.end()) {
        ue_ipaddr = it->second;
    }

    return ue_ipaddr;
   
}

void setEnBIPToPoseMap (string filename) {

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

    // string imsi_cell_rnti = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+to_string(imsi);
    // map<string, string>::iterator it = uernti_ips_to_enbips_mapping.begin();
    
    // map<string, string>::iterator it = uernti_ips_to_enbips_mapping.begin();

    // while (it != uernti_ips_to_enbips_mapping.end()) {
    //   cout << "Key: " << it->first
    //        << ", Value: " << it->second << endl;
    //   ++it;
    //}

    string ip_add;   
   
    // ostringstream os_str_ueaddr;
    // string enb_ip_addr, enbmacaddr;
    // string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";

    // string uePath = "/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/$ns3::LteUeNetDevice";

    // string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));

    // string ueNetDevicePath = uePath.substr(0, uePath.find ("/LteEnbRrc"));

    // Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);

    // Config::MatchContainer ue_match = Config::LookupMatches (ueNetDevicePath);
   
  /*
    for  (uint16_t k = 0; k < ue_match.GetN (); k++) {
	Ptr<NetDevice> ueDevice = ueLteDevs.Get(k);
        Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
        Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();
        uint16_t curr_cellId = ueRrc->GetCellId ();
        uint16_t curr_rnti = ueRrc->GetRnti ();
	cout<<endl<<" ---------->> "<<curr_rnti<<" - --- "<<curr_cellId<<endl;
     }
  */

  /*
    if (counter < int(all_node_list.size())) {
      string curr_ue = all_node_list[counter];
      string uernti_ip = "IMSI_"+to_string(imsi)+"_"+to_string(rnti)+"_"+curr_ue;

      for (uint16_t t = 0; t < match.GetN (); t++) {

         Ptr<Object> enbNetDevice = match.Get (t);

         Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
         Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
         Ptr<LteEnbMac> enbMac = enbLteDevice->GetMac ();
         
         Ptr<NetDevice> ueDevice = ueLteDevs.Get(t);
         Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
         Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();

	 // curr_cellId = ueRrc->GetCellId ();
         // curr_rnti = ueRrc->GetRnti ();

         ostringstream os_str_macaddr;

         bool ueManagerFound = enbRrc->HasUeManager (rnti);
	string imsi_plus_ip = to_string(imsi)+"_"+curr_ue;

         if (ueManagerFound) { // && (cellid == curr_cellId)) { // does bool = 1 mean true? confirm
            os_str_macaddr << enbMac;
            enbmacaddr = os_str_macaddr.str();
            cout<<endl<<" ****** "<< rnti <<" ****** "<<cellid<<" **** "<<imsi <<endl;
            std::map<string, string>::iterator it = ipaddr_macaddr_mapping.find(enbmacaddr);
            enb_ip_addr = it->second;
          }

          // else {
          //   continue;
         // }
       }
        
      // string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+curr_ue;
  
      // Ptr<UeManager> ueManager; //  = GetUeManager(rnti);
      // uint16_t curr_imsi = ueManager->GetImsi();
      // cout<<"\n ===========> "<<curr_imsi<<endl;

   */
      // uernti_ips_to_enbips_mapping.insert(TStrStrPair(uernti_ip, enb_ip_addr));

      
      string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);
      // string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);      
      ip_add = GetIpFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);

      // string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;

      if (ip_add == "") {
	 string id_rnti_cellid = "CellID_"+to_string(cellid);
	 ip_add = SimpleGetIpFromMap(cellid_ueip_mapping, id_rnti_cellid);

	 for (auto i: connected_ues) {
            std::cout << i << ' ';
         }

	 string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
	 string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);
	 // string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
         connected_ips_to_enbips_mapping.insert(TStrStrPair( uernti_ip, enb_ip_addr));
         int count = GetCountEnBEntries (uernti_ips_to_enbips_mapping, enb_ip_addr);
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
	  string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);
	  string ip_add = GetIpFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);
	  connected_ues.push_back(ip_add);
	  string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
	  string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);
	  connected_ips_to_enbips_mapping.insert(TStrStrPair( uernti_ip, enb_ip_addr));
	  int count = GetCountEnBEntries (uernti_ips_to_enbips_mapping, enb_ip_addr);
	  cout << endl << Simulator::Now ().GetSeconds () << " " << "Connection Established Ue: "
            << " UE IMSI " << imsi
            << " with IP: " << ip_add // id_rnti_cellid
            << " connected to CellId " << cellid
            << " with RNTI" << rnti
            << ". " << count << " UEs are now connected to EnB with IP: "
            << enb_ip_addr
            << endl;

      }

      for (auto i: connected_ues) {
         std::cout << i << ' ';
      }

      // int count = GetCountEnBEntries (uernti_ips_to_enbips_mapping, enb_ip_addr);
      /*
      cout << endl << Simulator::Now ().GetSeconds () << " " << "Connection Established Ue: "
            << " UE IMSI " << imsi
	    << " with IP: " << ip_add // id_rnti_cellid
            << " connected to CellId " << cellid
            << " with RNTI" << rnti
	    << ". " << count << " UEs are now connected to EnB with IP: "
	    << enb_ip_addr
	    << endl;
      */
      // string uernti_ip = "IMSI_"+to_string(imsi)+"_"+"CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;
      // string uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+ip_add;

      // connected_ips_to_enbips_mapping.insert(TStrStrPair( uernti_ip, enb_ip_addr));

   // }

   // counter = counter + 1;

}


void NotifyHandoverStartUe (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId)
{
  cout << Simulator::Now ().GetSeconds () << " " << "Handover Start Ue: "
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << endl;
}

void NotifyHandoverEndOkUe (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  cout << Simulator::Now ().GetSeconds () << " " << "Handover End Ue: "
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << endl;
}

void NotifyConnectionEstablishedEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
       // map<string, string>::iterator it = connected_ips_to_enbips_mapping.begin();
       // while (it != connected_ips_to_enbips_mapping.end()) {
       //   cout << "Key: " << it->first
       //        << ", Value: " << it->second << endl;
       //   ++it;
       // }


       map<string, string>::iterator itr = connected_ips_to_enbips_mapping.begin();

       while (itr != connected_ips_to_enbips_mapping.end()) {
	  if (itr->second == "") {
              cout << "Key::::::::: " << itr->first<<endl;
               // << ", Value: " << itr->second << endl;
	  }
          ++itr;
       }


	   string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);

	   string ip_add = GetIpFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);

	   string enb_ip_addr = GetEnBFromMap(uernti_ips_to_enbips_mapping, id_rnti_cellid);

	   int count = GetCountEnBEntries (uernti_ips_to_enbips_mapping, enb_ip_addr);
   
	   cout << Simulator::Now ().GetSeconds () << " " << "Connection Established eNB: "
		   << " eNB with IP: " << enb_ip_addr
		   << " CellId " << cellid
		   << " successful connection of UE with IMSI " << imsi 
		   << " and IP: " << ip_add
		   << " RNTI " << rnti
		   << ". Total UEs attached to " << count
		   << endl;   

}


void NotifyHandoverStartEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId)
{

  // Here we remove the ue from the list of those attached to this enb 

  cout << Simulator::Now ().GetSeconds () << " " << "Handover Start eNB: "
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << endl;

  // Then update the map with the new cellid_ue_rnti_ip
}


void NotifyHandoverEndOkEnb (string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{

  cout << Simulator::Now ().GetSeconds () << " " << "Handover End eNB: "
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
	    << " after handover to ENB with IP " // << enb_ip_addr
	    << " total attached UEs is now " << rnti // it->second 
            << endl;
}


void NotifyNewUeContext (string context, uint16_t cellid, uint16_t rnti) 
{
   
   //  vector<string> lst_ips;
   ostringstream os_str_ueaddr;
   string enb_ip_addr, enbmacaddr; // umsi_plus_rnti, ue_addr, enbmacaddr;
   string path = "/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice";
   // map< string, int>  cellid_rnti_count_mapping;

   string uePath = "/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/$ns3::LteUeNetDevice";
 
   string enbNetDevicePath = path.substr (0, path.find ("/LteEnbRrc"));

   string ueNetDevicePath = uePath.substr(0, uePath.find ("/LteEnbRrc"));

   Config::MatchContainer match = Config::LookupMatches (enbNetDevicePath);

   Config::MatchContainer ue_match = Config::LookupMatches (ueNetDevicePath);

  
    // Ptr<NetDevice> ueDevice = ueLteDevs.Get(0);
    // Ptr<LteUeNetDevice> ueLteDevice = ueDevice->GetObject<LteUeNetDevice> ();
    // Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();
    //      ueRrc->GetTargetEnb();
    // uint16_t cellId = ueRrc->GetCellId ();
    // uint16_t rnti = ueRrc->GetRnti ();

    // for (std::vector<Ptr<NetDevice> >::const_iterator enbDevIt = enbDevices.Begin (); enbDevIt != enbDevices.End (); ++enbDevIt)
    // {
    //  if (((*enbDevIt)->GetObject<LteEnbNetDevice> ())->HasCellId (cellId))

   // for  (uint16_t t = 0; t < ue_match.GetN (); t++) {
    //   Ptr<Object> ueNetDevice = ue_match.Get (t);
     // Ptr<LteUeNetDevice> enbUeDevice = DynamicCast<LteUeNetDevice> (ueNetDevice);
      // os_str_ueaddr << ueNetDevice;
      // ue_addr = os_str_ueaddr.str();
      // Ptr<LteEnbNetDevice> ueEnbTarget = enbUeDevice->GetTargetEnb();
      // umsirnti_code_mapping.insert({umsi_plus_rnti, os_str_ueaddr.str()});
      // umsirnti_code_mapping.insert(pair<string, string> (umsi_plus_rnti, ue_addr));
      // cout<<"\n *********** "<<ueNetDevice<<" ******** "<<all_node_list[t]<<"\n";

      // Ptr<LteEnbNetDevice> enbLteDevice = DynamicCast<LteEnbNetDevice> (enbNetDevice);
      // Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc ();
   // }

   // cellid_rnti_count_mapping.insert(pair<string, int> (to_string(cellid+"_"+enb_ip_addr), rnti));
   // vector<string> v2 = std::vector<string>(all_node_list.begin() + counter, all_node_list.end());
   // string curr_ue = all_node_list[counter];
 
   // cout<<"\n ---> "<<counter<<" ------ "<<all_node_list[counter]<<"\n ";
   //

 
   if (counter < int(all_node_list.size())) {
       // cout<<"\n ---> "<<counter<<" ------ "<<all_node_list[counter]<<"\n ";
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
            // newUe_lst_rntis.push_back(to_string(rnti));	 
            // TRYING TO UNDERSTAND WHAT IS GOING ON HERE UNTIL THE END OF THE COMMENT

	     std::map<string, string>::iterator it = ipaddr_macaddr_mapping.find(enbmacaddr);
             enb_ip_addr = it->second;
	     // lst_ips.push_back(enb_ip_addr);
	     // bool exists_ip = (std::find(lst_ips.begin(), lst_ips.end(), enb_ip_addr) != lst_ips.end());
             // uernti_ip = curr_ue+"_"+to_string(rnti);
	     // uernti_ips_to_enbips_mapping.insert(std::pair<string, string>(uernti_ip, enb_ip_addr));
	     // if (lst_ips.size() > 1) { //(!exists_ip) {
             //    newUe_lst_rntis.clear();
	     //   newUe_lst_rntis.push_back(to_string(rnti));
             // }

	     // else {
             //    newUe_lst_rntis.push_back(to_string(rnti));
	     // }
	    
          }
          else {
             continue;
          }

      // cellid_rnti_count_mapping.insert(pair<string, int> (to_string(cellid+"_"+enb_ip_addr), rnti));
     
      // std::map<string, int>::iterator it = cellid_rnti_count_mapping.find(enb_ip_addr);
      // if (it != cellid_rnti_count_mapping.end()) {
      //    it->second = rnti;
      // }
      // cellid_rnti_count_mapping.insert(pair<string, int> (enb_ip_addr, newUe_lst_rntis.size()));

       }
     
       uernti_ip = "CellID_"+to_string(cellid)+"_"+to_string(rnti)+"_"+curr_ue;
       string id_rnti_cellid = "CellID_"+to_string(cellid)+"_"+to_string(rnti);
       // uernti_ips_to_enbips_mapping.insert(std::pair<string, string>(uernti_ip, enb_ip_addr));

       cellid_ueip_mapping.insert(TStrStrPair("CellID_"+to_string(cellid), curr_ue));
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
   

       // map<string, string>::iterator it = uernti_ips_to_enbips_mapping.begin();
       // while (it != uernti_ips_to_enbips_mapping.end()) {
       //  cout << "Key: " << it->first
       //      << ", Value: " << it->second << endl;
       //  ++it;
       // }
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


double  CalculateDistance (const Vector2D &a, const Vector2D &b) {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double distance = std::sqrt (dx * dx + dy * dy);
  return distance;
}


/*
void InstallCBRApp ( string nodeip) {

   // 7. Install applications: two CBR streams each saturating the channel
    ApplicationContainer cbrApps;
    uint16_t cbrPort = 12345;
    OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address (nodeip), cbrPort));
    onOffHelper.SetAttribute ("PacketSize", UintegerValue (1400));
    onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
   
    // flow 1:  node 0 -> node 1
    onOffHelper.SetAttribute ("DataRate", StringValue ("3000000bps"));
    onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.000000)));
    cbrApps.Add (onOffHelper.Install (nodes.Get (0)));
   
    uint16_t  echoPort = 9;
    UdpEchoClientHelper echoClientHelper (Ipv4Address ("10.0.0.2"), echoPort);
    echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
    echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
    echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
    ApplicationContainer pingApps;
   
    // again using different start times to workaround Bug 388 and Bug 912
    echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
    pingApps.Add (echoClientHelper.Install (nodes.Get (0)));
}
*/


// Return the distance between node and enb
double GetDistanceFrom (Ptr<Node> node1, Ptr<Node> node2)
{
	Ptr<MobilityModel> model1 = node1->GetObject<MobilityModel>();
	Ptr<MobilityModel> model2 = node2->GetObject<MobilityModel>();
	double distance = model1->GetDistanceFrom (model2);
	return distance;
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

	 // ToDo:: get enb coordinates that a ue is attached to from file
	 // That is, use uernti_ips_to_enbips_mapping and vect_ip_pose
	 // i.e find enb ip to which dest_ipadd is mapped and extract x-y pose value 
	 // the enb. Then use x and y coords of ue below as input to function calculateDistance()

	 if (counter == 0) {
            break;
         }
         else {          
            int index_of_destip = getIndex(curr_node_list, dest_ipadd);          
 	    string x, y, vel_x, vel_y;
	    int index = 0; // std::distance(vec_nodes.begin(), it);
	    string command = "./metric_writer.py ";

	    // std::cout<<"\n -------->> "<<curr_node_list<< " - - - - "<<dest_ipadd;

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
                         // std::cout<<"\n =====INSIDE THE DIFF LOOP ====== "<<curr_node_list<<"------- "<<all_node_list<<" ------ "<<processed_nodes;
                         if (all_node_list.size() == 0) {
                             Simulator::Stop (Seconds (0.1));
                         }
                     }



    	             if (fiveTuple.sourceAddress == Ipv4Address("1.0.0.2") && (dest_ipadd == curr_node_list[index_of_destip])) // (dest_ipadd == vec_nodes[index_of_destip])) 
                      {
	                tx = to_str(stats->second.txPackets);
                        rx = to_str(stats->second.rxPackets);
                        thruput = to_str(stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024);
                        avg_delay = to_str(stats->second.delaySum.GetSeconds()/stats->second.rxPackets);
                        avg_jitter = to_str(stats->second.jitterSum.GetSeconds()/(stats->second.rxPackets));
                        lst_pkts = to_str(stats->second.lostPackets);

	                command += x+":"+y+":"+vel_x+":"+vel_y+":"+src_ipadd+":"+dest_ipadd+":"+tx+":"+rx+":"+thruput+":"+avg_delay+":"+avg_jitter+":"+lst_pkts;
	                // count_run++;
                        // std::cout<<"\n ++++ "<<command;
	                // system(command.c_str());
	      
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

	              // if (int(curr_node_list.size()) > 1) {
                      //  ip_add = curr_node_list[index];
                      //}
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

    // count_run = 0;

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
   // std::cout<<"\n ------- "<<xsp<<" ------ "<<ysp;
  // command +=x.str()+":"+y.str();
  // system(command.c_str());
  // cout<<"\n =======>> "<< command;
   //std::cout <<"\n => "<< Simulator::Now ().GetSeconds() << "\n POS: x=" << pos.x << ", y=" << pos.y
   //    << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
   //    << ", z=" << vel.z << std::endl;
   
   // std::cout<<"\n =-----------------"<<x<<" ----------- "<<y; //<<" - - - - - - "<<ip;

   // ToDo::
   // Here we shall simpy get the current ip address of the enb that the ue is attached to
   // and get the enb poses from the enblocations.txt.
   // Steps 1: Get enb IP ue is attached to
   //       2: Read pose corresponding to enb IP above [enb_x_pos, enb_y_pos]
   //       3: enbpos.x = enb_x_pos;
   //          enbpos.y = enb_y_pos;
   //         
   
   // dist_to_enb = CalculateDistance (uepos, enbpos);
   // what or where would you guys love the distance to be saved? 
   // As part of the dataset or?
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

  uint16_t port = 8000;

  // static vector<string> nodes;

  int64_t stream = 1;

  // assing central frequencies for each enb
  bool m_admitHo = true;
  // vector<Ues> ueMetrics;
  vector<float> ueMetrics;

  vector<float> qos_params;

  fstream enbFile, uesFile;

  enbFile.open("scratch/enbLocations.txt",ios::in);

  uesFile.open("scratch/trajectory.txt",ios::in);
  
  //BS parameters - 3GPP TR 36.942 V14.0.0 but with 500m
  // double beamwidth = 65.0;
  // double maxAttenuation = 20.0;
  // double interSiteDistance = 500;
  // double macroEnbTxPowerDbm = 46.0;

  //Other parameters
  // bool enableTraces = true;
  // bool useUdp = false;
  // bool epcDl = true;
  // bool epcUl = false;

  // ues and enbs
  uint16_t numberOfUes = GetNumberOfUesInFile("scratch/trajectory.txt");
  uint16_t numberOfEnbs = GetNumberOfEnbsInFile("scratch/enbLocations.txt");

  std::vector<UeData> m_ueDataVector;

  //937.5 MB
  //Official documentation: https://support.google.com/youtube/answer/2853702?hl=en
  // uint64_t bytesToTransfer = 9.375E7;

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
  //Ptr<LteHelper> lteHelper = CreateObject<LteHelper> (); // already global above 

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper); 
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler"); //RoundRobin Scheduler

  //Handover algorithm to be set. This is highly likely to be default in LTE network. 
  lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold",
                                            UintegerValue (30));
  lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset",
                                            UintegerValue (1));

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
  // 69399
  //Creates a net device container and adds PGW and remote host to it
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  //Uses Ipv4AddressHelper to make available IPs out of a base IP address and then assigns them to net devices, therefore each of the net devices (PGW and remote host) will have different IPs. (internetIpIfaces.GetAddress 1 being remote host and 0 being PGW)
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

     // node_id = trunct.substr(start,7);
     // nodes.push_back(trunct);
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

     // CheckConnected( ueLteDevs, enbLteDevs); // , enbNodes);

     if(first_time) {
        first_time = false;
        Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange", MakeCallback (&CourseChange));
     }

     speed_sizes.push_back(speeds.size());     

     ThroughputMonitor(&fmHelper, flowMonitor, dataset, trunct, speeds); // , ueLteDevs, enbLteDevs); //, nodes); 
     
     // CheckConnected( ueLteDevs, enbLteDevs);

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

    setEnBIPToPoseMap ("scratch/enbLocations.txt");

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
     
    // cout<<"\n ......COUNT ON NODES ATTACHED NOW ........ "<<ueips_to_enbips_mapping<<"\n ";
    //
    //Here get rid of the old approach of using the rnti as the count for the number of ues 
    // attached to the enb. We create a vector mapping and do a grouping based 
    // on similar keys then for each key count the number of the values:
    // Struct like <enb_ip:list[ue ips]>
    // So at each required time we simple extrac the enb of interest and give a count 
    // on the size of the values vector as the number of ues attached at that point in time.
    //
    // FixMe:: Move to the handovers, pass this mapping structure such that when a ue is handed 
    // over to another enb the list is updates bi-directionally, from source and destination.
    //
    // for (const auto & foo : ueips_to_enbips_mapping) foos_by_x[foo.x].push_back(foo);

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

  // Install and start applications on UEs and remote host
   uint16_t dlPort = 10000;
   uint16_t ulPort = 20000;

  lteHelper->AddX2Interface (enbNodes);

// Create two udpServer applications on node one.

  uint16_t server_port = 4000;
  UdpServerHelper server (server_port);
  // for (int k = 0; k < numEnbNodes; k++) {
     ApplicationContainer apps = server.Install (enbNodes.Get (0));    
  //   ApplicationContainer apps = server.Install (enbNodes.Get (k));
  //   apps.Start (Seconds (1.0));
  //   apps.Stop (Seconds (10.0));
  // }

  //
// Create one UdpTraceClient application to send UDP datagrams from enb to selected UEs
//
  uint32_t MaxPacketSize = 1472;  // Back off 20 (IP) + 8 (UDP) bytes from MTU
  UdpTraceClientHelper client (remoteHostAddr, port,"");
  client.SetAttribute ("MaxPacketSize", UintegerValue (MaxPacketSize));


 // 7. Install applications: one CBR stream saturating the channel
  ApplicationContainer cbrApps;
  uint16_t cbrPort = 12345;
  OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address ("1.0.0.2"), cbrPort));
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (1400));
  onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

  onOffHelper.SetAttribute ("DataRate", StringValue ("3000000bps"));
  onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.000000)));

  // for (int t = 0; t < numEnbNodes; t++ ) {
  //   cbrApps.Add (onOffHelper.Install (enbNodes.Get (t)));
  // }

  uint16_t  echoPort = 9;
  UdpEchoClientHelper echoClientHelper (Ipv4Address ("1.0.0.2"), echoPort);
  echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
  echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
  // again using different start times to workaround Bug 388 and Bug 912
  echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
  ApplicationContainer pingApps;

  for (uint32_t count = 0; count < numberOfUes; count+=2)
  {
    apps = client.Install (ueNodes.Get (count));
    apps.Start (Seconds (2.0));
    apps.Stop (Seconds (sim_time));

    // pingApps.Add (echoClientHelper.Install (ueNodes.Get (count)));
  }


  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
    startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
    startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

  // assign IP address to UEs     
  
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
              bearerData.dlSink = sinkContainer.Get(0)->GetObject<PacketSink>();
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
              bearerData.ulSink = sinkContainer.Get(0)->GetObject<PacketSink>();
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
Custom Handover Trace
###################################################################################################
*/

  // Connect to trace sources in UEs
 
 // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/StateTransition",
 //                 MakeCallback (&LteCellSelectionTestCase::StateTransitionCallback, this));
 // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/InitialCellSelectionEndOk",
 //                 MakeCallback (&LteCellSelectionTestCase::InitialCellSelectionEndOkCallback, this));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/InitialCellSelectionEndError",
//                   MakeCallback (&LteCellSelectionTestCase::InitialCellSelectionEndErrorCallback, this));
//   Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
//                   MakeCallback (&LteCellSelectionTestCase::ConnectionEstablishedCallback, this));

 //  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
 //                 MakeCallback (&NotifyMeasureMentReport));
 //    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
 //               MakeCallback(&NotifyHandoverFailureNoPreamble));
 //   Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
 //               MakeCallback(&NotifyHandoverFailureMaxRach));
 //   Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
 //               MakeCallback(&NotifyHandoverFailureLeaving));
 //   Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
 //               MakeCallback(&NotifyHandoverFailureJoining));

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
//   p2ph.EnablePcapAll("thesis_pcap/lena-x2-handover-measures");

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
  // os.close (); // close log file

  return 0;
}

