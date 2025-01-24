//--- Core (Ptr, Time, Creatobject...) ---
#include "ns3/core-module.h"
//--- mobility (helper) ---
#include "ns3/mobility-module.h"
//--- Buildings (helper) ---
#include <ns3/buildings-helper.h>
#include <ns3/first-order-building-aware-propagation-loss-model.h>

#include "ns3/building-list.h"
//--- VoxelGrid ----
//#include "ns3/VoxelNLOS.h"
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

void savePathlossData(double current_time, double pathloss) {
    try {
        std::string filename = "pathloss.csv";
        std::filesystem::path file_path = std::filesystem::path(filename);

        // Open the file for writing
        std::fstream file(file_path, std::ios::out | std::ios::app);

        if (!file.is_open()) {
            std::cerr << "Error: Could not open file: " << file_path << std::endl;
            return;
        }

        // Write the header if the file is newly created
        if (std::filesystem::file_size(file_path) == 0) {
            file << "current_time,pathloss\n";
        }

        // Write the data
        file << current_time << "," << pathloss << "\n"; // ERROR HERE

        file.flush();
        file.close();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}


void printPathloss(Time period, Ptr<FirstOrderBuildingAwarePropagationLossModel> model, Ptr<ns3::Node> sender, Ptr<ns3::Node> receiver){
    
    double loss;
    loss = model->GetLoss(sender->GetObject<MobilityModel>(), receiver->GetObject<MobilityModel>());
    double current_time = ns3::Simulator::Now().GetSeconds();
    savePathlossData(current_time, loss);
    Simulator::Schedule(period,
                        &printPathloss,
                        period,
                        model,
                        sender,
                        receiver);
}

void selfsetWaypoints(Ptr<Node> node, std::deque<Waypoint> waypoints)
{
    MobilityHelper mob;
    mob.SetMobilityModel("ns3::WaypointMobilityModel");
    mob.Install(node);
    Ptr<WaypointMobilityModel> wayMobility;
    wayMobility = node->GetObject<WaypointMobilityModel>();
    for (std::deque<Waypoint>::iterator w = waypoints.begin (); w != waypoints.end (); ++w )
    {
        wayMobility->AddWaypoint(*w);
    }
}


int main (int argc, char *argv[])
{   
    auto start = std::chrono::high_resolution_clock::now();
    double z = 10.0; // can't be z >= 0 because of buildings

    double t1 = 0;
    double t2 = 22;
    double t3 = 42; 
    double t4 = 62;
    double t5 = 82;


    NodeContainer nodes;
    nodes.Create(2);
    
    std::deque<Waypoint> waypoints;
    Waypoint waypointS1  (Seconds (t1), Vector(15, 30, z)); 
    Waypoint waypointS2  (Seconds (t2), Vector(15, 30, z));
    Waypoint waypointS3  (Seconds (t3), Vector(15, 30, z));
    Waypoint waypointS4  (Seconds (t4), Vector(15, 30, z));
    Waypoint waypointS5  (Seconds (t5), Vector(15, 30, z));
    waypoints.push_back (waypointS1);
    waypoints.push_back (waypointS2);
    waypoints.push_back (waypointS3);
    waypoints.push_back (waypointS4);
    waypoints.push_back (waypointS5);

    selfsetWaypoints(nodes.Get(0), waypoints);
    waypoints.clear();
    
    Waypoint waypointD1  (Seconds (t1), Vector(15, 25, z)); 
    Waypoint waypointD2  (Seconds (t2), Vector(15, 15, z));
    Waypoint waypointD3  (Seconds (t3), Vector(30, 15, z));
    Waypoint waypointD4  (Seconds (t4), Vector(30, 30, z));
    Waypoint waypointD5  (Seconds (t5), Vector(20, 30, z));
    waypoints.push_back (waypointD1);
    waypoints.push_back (waypointD2);
    waypoints.push_back (waypointD3);
    waypoints.push_back (waypointD4);
    waypoints.push_back (waypointD5);

    selfsetWaypoints(nodes.Get(1), waypoints);

    double x_min = 20.0;
    double x_max = 25.0;
    double y_min = 20.0;
    double y_max = 25.0;
    double z_min = 0.0;
    double z_max = 15.0;

    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
    b1->SetBuildingType(Building::Residential);
    b1->SetExtWallsType(Building::ConcreteWithWindows);

    /*
    double x_min2 = 30.0;
    double y_min2 = 20.0;
    double y_max2 = 25.0;
    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(x_min2, x_max, y_min2, y_max2, z_min, z_max));
    b2->SetBuildingType(Building::Residential);
    b2->SetExtWallsType(Building::ConcreteWithWindows);

    double x_min3 = 40.0;
    double y_min3 = 25.0;
    double y_max3 = 30.0;
    Ptr<Building> b3 = CreateObject<Building>();
    b3->SetBoundaries(Box(x_min3, x_max, y_min3, y_max3, z_min, z_max));
    b3->SetBuildingType(Building::Residential);
    b3->SetExtWallsType(Building::ConcreteWithWindows);
    */
    
    Ptr<FirstOrderBuildingAwarePropagationLossModel> FOpropagationLossModel = CreateObject<FirstOrderBuildingAwarePropagationLossModel>();
   
    Simulator::Schedule(Seconds(0),
                                &printPathloss,
                                Seconds(0.2),
                                FOpropagationLossModel,
                                nodes.Get(0),
                                nodes.Get(1));

    Simulator::Stop(Seconds(t5+2));
    Simulator::Run ();
    Simulator::Destroy ();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << " microsecondes" << "\n";
}
