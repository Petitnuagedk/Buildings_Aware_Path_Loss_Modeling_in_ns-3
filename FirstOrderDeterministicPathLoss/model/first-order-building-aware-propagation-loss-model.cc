/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#include "first-order-building-aware-propagation-loss-model.h"

#include "ns3/mobility-building-info.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/pointer.h"
#include "ns3/building.h"
#include "ns3/building-list.h"

// Loss models
#include "ns3/itu-r-1411-los-propagation-loss-model.h"

#include <cmath>
#include <random>
#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("FirstOrderBuildingAwarePropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(FirstOrderBuildingAwarePropagationLossModel);

FirstOrderBuildingAwarePropagationLossModel::FirstOrderBuildingAwarePropagationLossModel()
{
    m_ituR1411Los = CreateObject<ItuR1411LosPropagationLossModel>();
    m_assess = CreateObject<NLOSassess>();
}

FirstOrderBuildingAwarePropagationLossModel::~FirstOrderBuildingAwarePropagationLossModel()
{
}

TypeId
FirstOrderBuildingAwarePropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::FirstOrderBuildingAwarePropagationLossModel")
            .SetParent<BuildingsPropagationLossModel>()
            .AddConstructor<FirstOrderBuildingAwarePropagationLossModel>()
            .SetGroupName("Buildings")
        ;

    return tid;
}

double
FirstOrderBuildingAwarePropagationLossModel::GetLoss(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const
{
    NS_ASSERT_MSG(
        (rx->GetPosition().z >= 0) && (tx->GetPosition().z >= 0),
        "HybridBuildingsPropagationLossModel does not support underground nodes (placed at z < 0)");

    // double dist_rx_tx = rx->GetDistanceFrom(tx);

    double loss = 0.0;

    // Need to check if both node are outside

    // For now singular loss model ITU-R-1411
    loss = ItuR1411(rx, tx);
    std::cout << "init loss : " << loss << "\n";
    std::vector<Ptr<Building>> NLOSBuildings;
    std::vector<Ptr<Building>> AllBuildings;
    Vector rxPos = rx->GetPosition();
    Vector txPos = tx->GetPosition();
    int limit = BuildingList::GetNBuildings();
    for (int index = 0; index < limit; ++index)
    {
        Ptr<Building> CurBu = BuildingList::GetBuilding(index);
        if (CurBu->IsIntersect(rxPos,txPos))
        {
            NLOSBuildings.push_back(CurBu);
        }
        AllBuildings.push_back(CurBu);
    }

    // if there is NLOS -> first order process
    if (!NLOSBuildings.empty())
    {
        double direct_path_loss = loss + PenetrationLoss(NLOSBuildings);
        double diffracted_path_loss = loss + DiffractionLoss(NLOSBuildings, AllBuildings, rx, tx);
        double reflected_path_loss  = ReflectionLoss(AllBuildings, rx, tx);
        std::cout << "reflacted : " << reflected_path_loss << "\n";
        loss = std::min(std::min(direct_path_loss, diffracted_path_loss), reflected_path_loss); // #include <algorithm>
        std::cout << "out loss : " << loss << "\n";
        std::cout << "-------------------------------\n";
    }
    loss += Noise(loss);
    return loss;
}

double
FirstOrderBuildingAwarePropagationLossModel::PenetrationLoss(std::vector<Ptr<Building>> NLOSBuildings) const
{
    double loss = 0;
    for (size_t i = 0; i < NLOSBuildings.size(); ++i)
    {
        if (NLOSBuildings[i]->GetExtWallsType() == Building::Wood)
        {
            loss += 2 * 4;
        }
        if (NLOSBuildings[i]->GetExtWallsType() == Building::ConcreteWithWindows)
        {
            loss += 2 * 7;
        }
        if (NLOSBuildings[i]->GetExtWallsType() == Building::ConcreteWithoutWindows)
        {
            loss += 2 * 15;
        }
        if (NLOSBuildings[i]->GetExtWallsType() == Building::StoneBlocks)
        {
            loss += 2 * 12;
        }
        else
        {
            NS_LOG_ERROR(this << " Unkwnon Wall Type");
        }
    }
    std::cout << "direct : " << loss << "\n";
    return loss;
}

double
FirstOrderBuildingAwarePropagationLossModel::DiffractionLoss(std::vector<Ptr<Building>> NLOSBuildings, std::vector<Ptr<Building>> AllBuildings, Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const
{
    auto CreateTempMobilityModel = [](Vector position)
    {
        Ptr<ConstantPositionMobilityModel> tempMobility = CreateObject<ConstantPositionMobilityModel>();
        tempMobility->SetPosition(position);
        return tempMobility;
    };

    for (size_t i = 0; i < NLOSBuildings.size(); ++i)
    {
        std::vector<Vector> CornersPos = m_assess->GetCorner(NLOSBuildings[i], rx, tx);
        const size_t size_cor = CornersPos.size();
        if (size_cor == 1)
        {
            Ptr<MobilityModel> corner_pos = CreateTempMobilityModel(CornersPos[0]);
            std::vector<Ptr<Building>> NLOScorner = m_assess->assesNLOS(corner_pos, tx, AllBuildings);
            if (NLOScorner.empty())
            {
                double theta = calculateAngle(tx,CornersPos[0],rx);
                //std::cout << "theta : " << theta << "\n";
                //std::cout << "DiffFunct : " << DiffFunct(theta) << "\n";
                return DiffFunct(theta);
            }
        }
        if (size_cor == 2)
        {
            Ptr<MobilityModel> corner_pos_1 = CreateTempMobilityModel(CornersPos[0]);
            Ptr<MobilityModel> corner_pos_2 = CreateTempMobilityModel(CornersPos[1]);
            std::vector<Ptr<Building>> NLOScorner_1 = m_assess->assesNLOS(corner_pos_1, tx, AllBuildings);
            std::vector<Ptr<Building>> NLOScorner_2 = m_assess->assesNLOS(corner_pos_2, tx, AllBuildings);
            if (NLOScorner_1.empty() || NLOScorner_2.empty())
            {
                double theta_1 = calculateAngle(tx,CornersPos[0],rx);
                double theta_2 = calculateAngle(tx,CornersPos[1],rx);
                double loss_1 = DiffFunct(theta_1);
                double loss_2 = DiffFunct(theta_2);
                return std::min(loss_1,loss_2);
            }
        }
        if (size_cor > 2)
        {
            NS_LOG_ERROR(this << "Unexpected amount of corners");
            return std::numeric_limits<double>::infinity();;
        }
    }
    return std::numeric_limits<double>::infinity();;
}

double
FirstOrderBuildingAwarePropagationLossModel::ReflectionLoss(std::vector<Ptr<Building>> AllBuildings, Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const
{
    std::vector<double> refl_loss;
    double refl_coef = 0;

    auto CreateTempMobilityModel = [](Vector position) {
        Ptr<ConstantPositionMobilityModel> tempMobility = CreateObject<ConstantPositionMobilityModel>();
        tempMobility->SetPosition(position);
        return tempMobility;
    };

    for (size_t i = 0; i < AllBuildings.size(); ++i)
    {
        std::optional<Vector> reflection_point = m_assess->Getreflectionpoint(AllBuildings[i], rx, tx);
        if (reflection_point)
        {
            std::cout << "reflection point : " << *reflection_point << "\n";
            // Check if NLOS conditions are met
            Ptr<MobilityModel> reflectionMobility = CreateTempMobilityModel(*reflection_point);
            if (m_assess->assesNLOS(reflectionMobility, rx, {AllBuildings[i]}).empty() &&
                m_assess->assesNLOS(reflectionMobility, tx, {AllBuildings[i]}).empty())
            {
                // Assign reflection coefficient based on wall type
                switch (AllBuildings[i]->GetExtWallsType())
                {
                case Building::Wood:
                    refl_coef = 4;
                    break;
                case Building::ConcreteWithWindows:
                    refl_coef = 7;
                    break;
                case Building::ConcreteWithoutWindows:
                    refl_coef = 15;
                    break;
                case Building::StoneBlocks:
                    refl_coef = 12;
                    break;
                default:
                    NS_LOG_ERROR(this << " Unknown Wall Type");
                    continue;
                }
                // Calculate loss
                std::cout << "first half : " << ItuR1411(tx, reflectionMobility) << "\n";
                std::cout << "second half : " << ItuR1411(reflectionMobility, rx) << "\n";
                double loss = ItuR1411(tx, reflectionMobility) + 
                              ItuR1411(reflectionMobility, rx) + 
                              refl_coef;
                refl_loss.push_back(loss);
            }
        }
    }

    // Return the minimum reflection loss
    if (!refl_loss.empty())
    {
        return *std::min_element(refl_loss.begin(), refl_loss.end());
    }
    else
    {
        return std::numeric_limits<double>::infinity(); // No valid reflections
    }
}

double
FirstOrderBuildingAwarePropagationLossModel::Noise(double loss) const
{
    double y = 0.25*loss + 5;
    double top = y*1.1;
    double bot = y*(1-.1);
    return getRandomDouble(bot, top);
}

double
FirstOrderBuildingAwarePropagationLossModel::getRandomDouble(double min, double max) const
{ // #include <random>
    std::random_device rd;  // Seed the random number generator
    std::mt19937 gen(rd()); // Mersenne Twister engine
    std::uniform_real_distribution<double> dis(min, max); // Distribution in the range [min, max]

    return dis(gen); // Generate and return a random number
}

double
FirstOrderBuildingAwarePropagationLossModel::calculateAngle(Ptr<MobilityModel> rx, Vector B, Ptr<MobilityModel> tx) const
{ // Test available at Angletest.cc

    Vector A = rx->GetPosition();
    Vector C = tx->GetPosition();
    // Vector AB
    double ABx = B.x - A.x;
    double ABy = B.y - A.y;

    // Vector BC
    double BCx = C.x - B.x;
    double BCy = C.y - B.y;

    // Dot product of AB and BC
    double dotProduct = (ABx * BCx) + (ABy * BCy);

    // Magnitudes of AB and BC
    double magnitudeAB = std::sqrt(ABx * ABx + ABy * ABy);
    double magnitudeBC = std::sqrt(BCx * BCx + BCy * BCy);

    // Cosine of the angle
    double cosTheta = dotProduct / (magnitudeAB * magnitudeBC);

    // Return the angle in degrees
    return std::acos(cosTheta) * 180.0 / M_PI;
}

double
FirstOrderBuildingAwarePropagationLossModel::DiffFunct(double angle) const
{
    double a = 0.82;
    double b = 23.2;
    double c = 3.555;
    double d = 32.0;
    return -a/(exp((angle/b)-c))+d;
}

double
FirstOrderBuildingAwarePropagationLossModel::ItuR1411(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const
{
    return m_ituR1411Los->GetLoss(rx, tx);
}

} // namespace ns3
