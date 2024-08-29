/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-classifier.h"
#include "ns3/flow-probe.h"
#include "ns3/gnuplot.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/radio-bearer-stats-calculator.h"

#include <ns3/net-device-container.h>
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-helper.h>
#include <ns3/double.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-ue-phy.h>
#include <ns3/config.h>
#include <ns3/boolean.h>
#include <ns3/ff-mac-scheduler.h>
#include <ns3/packet.h>
#include <ns3/ptr.h>
#include <iostream>   

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("LenaX2HandoverMeasures");

Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
uint8_t lcid;

void attachinfuture(NetDeviceContainer& uesdev, NetDeviceContainer& enbsdev){
  lteHelper->AttachToClosestEnb(uesdev, enbsdev);
}

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void ReceivePacket (std::string context, Ptr<const Packet> packet) 
{
  LteRadioBearerTag tag;
  packet->PeekPacketTag (tag);
  lcid = tag.GetLcid ();
  std::cout << context	  
            << " Packet size: " << packet->GetSize()
            << " RNTI " << tag.GetRnti ()
	    << " LCID " << lcid
            << std::endl;
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



/**
 * Sample simulation script for an automatic X2-based handover based on the RSRQ measures.
 * It instantiates two eNodeB, attaches one UE to the 'source' eNB.
 * The UE moves between both eNBs, it reports measures to the serving eNB and
 * the 'source' (serving) eNB triggers the handover of the UE towards
 * the 'target' eNB when it considers it is a better eNB.
 */
int
main (int argc, char *argv[])
{
  uint16_t numBearersPerUe = 2;
  double speed = 20;       // m/s
  double simTime = 75; // (double)(numberOfEnbs + 1) * distance / speed; // 1500 m / 20 m/s = 75 secs
  double enbTxPowerDbm = 46.0;
  double m_berRef = 0.00003;
  int64_t stream = 1;
  bool m_admitHo = true;

  fstream enbFile, uesFile;

  enbFile.open("scratch/enbs.txt",ios::in);

  uesFile.open("scratch/trajectory.txt",ios::in);

  uint16_t numberOfUes = GetNumberOfUesInFile("scratch/trajectory.txt");
  uint16_t numberOfEnbs = GetNumberOfEnbsInFile("scratch/enbs.txt");

  // change some default attributes so that they are reasonable for
  // this scenario, but do this before processing command line
  // arguments, so that the user is allowed to override these settings
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (10)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (m_berRef));

  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));  
  Config::SetDefault("ns3::LteUePhy::NoiseFigure", DoubleValue(7));
  Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(5));

  // Command line arguments
  CommandLine cmd (__FILE__);
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.AddValue ("speed", "Speed of the UE (default = 20 m/s)", speed);
  cmd.AddValue ("enbTxPowerDbm", "TX power [dBm] used by HeNBs (default = 46.0)", enbTxPowerDbm);

  cmd.Parse (argc, argv);

  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");

  lteHelper->SetHandoverAlgorithmType ("ns3::A2A4RsrqHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute ("ServingCellThreshold", UintegerValue (30));
  lteHelper->SetHandoverAlgorithmAttribute ("NeighbourCellOffset", UintegerValue (1));
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);


  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (numberOfUes);
 
  // Install Mobility Model in eNB
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  string s;

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


  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|2000|0|2000"));

  // Install Mobility Model in UE
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("100.0"),
                                 "Y", StringValue ("100.0"),
                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=30]"));
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2s"),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                             "Bounds", StringValue ("0|2000|0|2000"));
  mobility.Install(ueNodes);

  /*
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  ueMobility.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, yForUe, 0));
  ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0));
  */

  // Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));
  
  // Install LTE Devices in eNB and UEs
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);

  stream += lteHelper->AssignStreams(enbLteDevs, stream);
  for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it)
    {
      Ptr<LteEnbRrc> enbRrc = (*it)->GetObject<LteEnbNetDevice>()->GetRrc();
      enbRrc->SetAttribute("AdmitHandoverRequest", BooleanValue(m_admitHo));
  }
  for (uint32_t t = 0; t < enbNodes.GetN(); ++t) {
      Ptr<LteEnbNetDevice> lteEnbDev = enbLteDevs.Get (t)->GetObject<LteEnbNetDevice> ();
      Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();
      enbPhy->SetAttribute ("TxPower", DoubleValue (43.0));
      enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));
    }


  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);
  stream += lteHelper->AssignStreams(ueLteDevs, stream);

  for (uint16_t i = 0; i < numberOfUes; i++) {
     Ptr<LteUeNetDevice> lteUeDev = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ();
     Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
     uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
     uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
  }

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  Simulator::Schedule(Seconds(1.0), &attachinfuture, ueLteDevs, enbLteDevs);

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
          ++dlPort;
          ++ulPort;

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
          clientApps.Add (dlClientHelper.Install (remoteHost));
          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), dlPort));
          serverApps.Add (dlPacketSinkHelper.Install (ue));

          NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
          UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
          clientApps.Add (ulClientHelper.Install (ue));
          PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), ulPort));
          serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort;
          dlpf.localPortEnd = dlPort;
          tft->Add (dlpf);
          EpcTft::PacketFilter ulpf;
          ulpf.remotePortStart = ulPort;
          ulpf.remotePortEnd = ulPort;
          tft->Add (ulpf);
          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }


  // Add X2 interface
  lteHelper->AddX2Interface (enbNodes);


  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();

  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));

  // X2-based Handover
  //lteHelper->HandoverRequest (Seconds (0.100), ueLteDevs.Get (0), enbLteDevs.Get (0), enbLteDevs.Get (1));

  // Uncomment to enable PCAP tracing
  // p2ph.EnablePcapAll("lena-x2-handover-measures");

  // connect custom trace sinks for RRC connection establishment and handover notification
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
  // Config::Connect ("/NodeList/*/DeviceList/*/UanNetDevice/Phy/UanPhy/PhyRxEnd", MakeCallback (&ReceivePacket));
  // Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice/ComponentCarrierMap/*/LteEnbPhy/UlSpectrumPhy/RxEndOk", MakeCallback (&ReceivePacket));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteNetDevice/$ns3::LteEnbNetDevice/ComponentCarrierMap/*/LteEnbPhy/DlSpectrumPhy/RxEndOk", MakeCallback (&ReceivePacket));

  std::vector <uint64_t> dlDataRxed;

  for (int i = 0; i < numberOfUes; i++)
    {
      // get the imsi
      uint64_t imsi = ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ()->GetImsi ();
      // get the lcId
      // ueLteDevs.Get (i)->GetObject<LteUeNetDevice> ()->GetRrc ()->GetLcIdVector ().at (0);
      dlDataRxed.push_back (rlcStats->GetDlRxData (imsi, lcid));
      double txed = rlcStats->GetDlTxData (imsi, lcid);
      double ber = 1.0 - ((double)dlDataRxed.at (i)/txed);
      cout << "\n\tUser " << i << " imsi " << imsi << " bytes rxed " << (double)dlDataRxed.at (i) << " txed " << txed <<" LCID "<<lcid
           << " BER " << ber << " Err " << fabs (m_berRef - ber)<<endl;
     // NS_TEST_ASSERT_MSG_EQ_TOL (ber, m_berRef, 0.1, " Unexpected BER distribution!");
   }


  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  Gnuplot2dDataset dataset;
  dataset.SetTitle ("Flow Monitor");
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);


  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
   {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      
     if (t.sourceAddress == Ipv4Address("1.0.0.2")) {
        std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        // std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / simTime.GetSeconds () / 1000 / 1000  << " Mbps\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        // std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / simTime.GetSeconds () / 1000 / 1000  << " Mbps\n";
        std::cout << "  Lost Packets: " << i->second.lostPackets <<"\n";
     }
   }

  // GtkConfigStore config;
  // config.ConfigureAttributes ();

  AnimationInterface anim ("LTEAnimLTE.xml"); // Mandatory

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

  anim.SetMaxPktsPerTraceFile (9999999999999);

  // Interesting idea, we could use the topographic capture from SUMO here........
  anim.SetMobilityPollInterval (Seconds (0.1));


  Simulator::Destroy ();
  return 0;

}
