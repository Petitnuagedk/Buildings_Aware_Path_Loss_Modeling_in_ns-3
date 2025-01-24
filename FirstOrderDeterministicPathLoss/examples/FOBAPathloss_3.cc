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
    double t2 = 10;
    double t3 = 20; 


    NodeContainer nodes;
    nodes.Create(2);
    
    std::deque<Waypoint> waypoints;
    Waypoint waypointS1  (Seconds (t1), Vector(5, 32, z)); 
    Waypoint waypointS2  (Seconds (t2), Vector(5, 32, z));
    Waypoint waypointS3  (Seconds (t3), Vector(5, 37, z));
    waypoints.push_back (waypointS1);
    waypoints.push_back (waypointS2);
    waypoints.push_back (waypointS3);

    selfsetWaypoints(nodes.Get(0), waypoints);
    waypoints.clear();
    
    Waypoint waypointD1  (Seconds (t1), Vector(35, 32, z)); 
    Waypoint waypointD2  (Seconds (t2), Vector(35, 32, z));
    Waypoint waypointD3  (Seconds (t3), Vector(35, 37, z));
    waypoints.push_back (waypointD1);
    waypoints.push_back (waypointD2);
    waypoints.push_back (waypointD3);

    selfsetWaypoints(nodes.Get(1), waypoints);

    double x_min = 10.0;
    double x_max = 30.0;
    double y_min = 20.0;
    double y_max = 30.0;
    double z_min = 0.0;
    double z_max = 15.0;

    Ptr<Building> b1 = CreateObject<Building>();
    b1->SetBoundaries(Box(x_min, x_max, y_min, y_max, z_min, z_max));
    b1->SetBuildingType(Building::Residential);
    b1->SetExtWallsType(Building::ConcreteWithWindows);

    
    double y_min2 = 45.0;
    double y_max2 = 50.0;
    Ptr<Building> b2 = CreateObject<Building>();
    b2->SetBoundaries(Box(x_min, x_max, y_min2, y_max2, z_min, z_max));
    b2->SetBuildingType(Building::Residential);
    b2->SetExtWallsType(Building::ConcreteWithWindows);

    double x_max3 = 13.0;
    double y_min3 = 35.0;
    double y_max3 = 45.0;
    Ptr<Building> b3 = CreateObject<Building>();
    b3->SetBoundaries(Box(x_min, x_max3, y_min3, y_max3, z_min, z_max));
    b3->SetBuildingType(Building::Residential);
    b3->SetExtWallsType(Building::ConcreteWithWindows);
    
    double x_min4 = 16.0;
    double x_max4 = 19.0;
    Ptr<Building> b4 = CreateObject<Building>();
    b4->SetBoundaries(Box(x_min4, x_max4, y_min3, y_max3, z_min, z_max));
    b4->SetBuildingType(Building::Residential);
    b4->SetExtWallsType(Building::ConcreteWithWindows);
    
    double x_min5 = 22.0;
    double x_max5 = 25.0;
    Ptr<Building> b5 = CreateObject<Building>();
    b5->SetBoundaries(Box(x_min5, x_max5, y_min3, y_max3, z_min, z_max));
    b5->SetBuildingType(Building::Residential);
    b5->SetExtWallsType(Building::ConcreteWithWindows);

    double x_min6 = 28.0;
    double x_max6 = 30.0;
    Ptr<Building> b6 = CreateObject<Building>();
    b6->SetBoundaries(Box(x_min6, x_max6, y_min3, y_max3, z_min, z_max));
    b6->SetBuildingType(Building::Residential);
    b6->SetExtWallsType(Building::ConcreteWithWindows);

    Ptr<FirstOrderBuildingAwarePropagationLossModel> FOpropagationLossModel = CreateObject<FirstOrderBuildingAwarePropagationLossModel>();
   
    Simulator::Schedule(Seconds(0),
                                &printPathloss,
                                Seconds(0.2),
                                FOpropagationLossModel,
                                nodes.Get(0),
                                nodes.Get(1));

    Simulator::Stop(Seconds(t3+2));
    Simulator::Run ();
    Simulator::Destroy ();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << " microsecondes" << "\n";
}
