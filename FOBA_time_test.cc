//--- Core (Ptr, Time, Creatobject...) ---
#include "ns3/core-module.h"
//--- mobility (helper) ---
#include "ns3/mobility-module.h"
//--- application (UDP) ---
#include "ns3/applications-module.h"
//--- Communication (Netdevice, Layer3, Layer4) ---
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/udp-client-server-helper.h"
//--- Buildings (helper) ---
#include <ns3/buildings-helper.h>
#include <ns3/first-order-buildings-aware-propagation-loss-model.h>

#include "ns3/building-list.h"
//--- trace output ---
#include "ns3/trace-helper.h"
//--- Routing ---
#include "ns3/aodv-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/olsr-helper.h"

//---Other---
#include <cmath>
#include <vector> 
#include <iostream>
#include <utility>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <ctime>
#include <string>

using namespace ns3;

std::vector<Ptr<Building>> makeBuildings(double n_build) {
    double z_min = 0.0;
    double z_max = 15.0;
    double building_size = 140.0;
    double spacing = 150.0;
    double start_x = -100.0;
    double start_y = -100.0;

    std::vector<Ptr<Building>> LBuildings;

    for (int i = 0; i < n_build; ++i) {
        // Calculate the center of the building
        double center_x = start_x + spacing / 2 ;
        double center_y = start_y + i * spacing + spacing / 2 ;

        // Calculate the boundaries of the building
        double x_min = center_x - building_size / 2;
        double x_max = center_x + building_size / 2;
        double y_min = center_y - building_size / 2;
        double y_max = center_y + building_size / 2;

        // Create the building
        Ptr<Building> building = CreateObject<Building>();
        building->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
        building->SetBuildingType(Building::Residential);
        building->SetExtWallsType(Building::ConcreteWithWindows);
        LBuildings.push_back(building);
    }

    return LBuildings;
}

int main (int argc, char *argv[])
{   
    int n_build = 1;
    int n_nodes = 15;
    std::string RA = "aodv";
    std::string proto = "UDP";
    CommandLine cmd;
    cmd.AddValue("n_build", "Number of building to test FOBA with", n_build);
    cmd.AddValue("n_nodes", "Number of nodes to test FOBA with", n_nodes);
    cmd.Parse(argc, argv);

    double StartTime = 5;
    double EndTime = 400;

    NodeContainer nodes;
    nodes.Create(n_nodes);

    MobilityHelper mobility;
    // setup the grid itself: objects are layed out
    // started from (-100,-100) with 25 objects per row, 
    // the x interval between each object is 150 meters
    // and the y interval between each object is 150 meters
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (-100.0),
                                   "MinY", DoubleValue (-100.0),
                                   "DeltaX", DoubleValue (150.0),
                                   "DeltaY", DoubleValue (150.0),
                                   "GridWidth", UintegerValue (2),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);
    // Adjust the z position to 5m for each node
    for (NodeContainer::Iterator it = nodes.Begin(); it != nodes.End(); ++it) {
        Ptr<MobilityModel> mobilityModel = (*it)->GetObject<MobilityModel>();
        Vector position = mobilityModel->GetPosition();
        position.z = 5.0; // Set z to 5m
        mobilityModel->SetPosition(position);
    }

    std::vector<Ptr<Building>> Buildings = makeBuildings(n_build);

    
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel"); /*, // FirstOrderBuildingsAwarePropagationLossModel  FriisPropagationLossModel  HybridBuildingsPropagationLossModel*/
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    //wifiPhy.Set("TxPowerStart", DoubleValue(18.0));  // Starting transmission power (dBm)
    //wifiPhy.Set("TxPowerEnd", DoubleValue(18.0));    // Ending transmission power (dBm)
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac"
                    //"Ssid", SsidValue(ssid),
                    //"QosSupported", BooleanValue(true)
                    //"BeaconGeneration", BooleanValue(true),
                    //"BeaconInterval", TimeValue(Seconds(0.1024))
                    );
    NetDeviceContainer devices;   // Container for network devices
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

    
    if (RA == "olsr")
    {
        InternetStackHelper stack;
        Ipv4StaticRoutingHelper staticRouting;
        OlsrHelper olsr;    
        Ipv4ListRoutingHelper list;
        list.Add(staticRouting, 0);
        list.Add(olsr, 10);
        stack.SetRoutingHelper(list);
        stack.Install (nodes);
    }

    if (RA == "aodv")
    {
        InternetStackHelper stack;
        AodvHelper aodv;
        stack.SetRoutingHelper(aodv);
        if (true)
        {
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>("./scratch/aodv.routes", std::ios::out);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(5), routingStream);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(35), routingStream);
        }
        stack.Install (nodes);
    }
    if (RA == "dsdv")
    {
        InternetStackHelper stack;
        DsdvHelper dsdv;
        dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(5)));
        dsdv.Set("SettlingTime", TimeValue(Seconds(5)));
        stack.SetRoutingHelper(dsdv); // has effect on the next Install ()

        if (true)
        {
            Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper>(("./scratch/dsdv.routes"), std::ios::out);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(5), routingStream);
            Ipv4RoutingHelper::PrintRoutingTableAllAt(Seconds(35), routingStream);

        }
        stack.Install (nodes);
    }


    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0"); 
    Ipv4InterfaceContainer interfaces;
    interfaces = address.Assign(devices);

    Ptr<Node> destination;
    Ipv4Address dst_ip;
    Ptr<Node> source;
    Ipv4Address src_ip;

    destination = nodes.Get(0);
    dst_ip = destination->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    Ptr<MobilityModel> conmob5 = destination->GetObject<MobilityModel> ();
    source = nodes.Get(n_nodes-1);
    src_ip = source->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    Ptr<MobilityModel> conmob1 = source->GetObject<MobilityModel> ();
    

    // Source
    if (proto == "UDP"){
        //Destination
        uint16_t port = 4000;
        UdpServerHelper server (port);
        ApplicationContainer apps = server.Install (destination);
        apps.Start (Seconds (1.0));
        apps.Stop (Seconds (EndTime));
        //Source
        uint32_t MaxPacketSize = 2048;
        Time interPacketInterval = Seconds (0.02); // 50 packets per second
        UdpClientHelper client (dst_ip, port);
        client.SetAttribute ("MaxPackets", UintegerValue (0));
        client.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client.SetAttribute ("PacketSize", UintegerValue (MaxPacketSize));
        apps = client.Install(source);
        apps.Start(Seconds(StartTime));
        apps.Stop(Seconds(EndTime));
    }
    if (proto == "TCP"){
        //Destination
        uint16_t port = 50000;
        Address hubLocalAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
        PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", hubLocalAddress);
        ApplicationContainer hubApp = packetSinkHelper.Install (destination);
        hubApp.Start (Seconds (1.0));
        hubApp.Stop (Seconds (EndTime));
        //Source
        ApplicationContainer spokeApps;
        OnOffHelper onOffHelper ("ns3::TcpSocketFactory", Address ());
        onOffHelper.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        AddressValue remoteAddress (InetSocketAddress (dst_ip, port));
        onOffHelper.SetAttribute ("Remote", remoteAddress);
        spokeApps.Add (onOffHelper.Install (source));
        spokeApps.Start (Seconds (StartTime));
        spokeApps.Stop (Seconds (EndTime));
    }

    // Tracing
    wifiPhy.EnablePcap("./scratch/pcap-sender.pcap", source->GetDevice(0),false, true);
    wifiPhy.EnablePcap("./scratch/pcap-receiver.pcap", destination->GetDevice(0),false, true);
    

    Simulator::Stop(Seconds(EndTime));
    std::cout << "Running..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    Simulator::Run ();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << " microsecondes" << "\n";
    Simulator::Destroy ();


    return 0;
}
