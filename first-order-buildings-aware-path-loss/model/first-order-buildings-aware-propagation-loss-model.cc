/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#include "first-order-buildings-aware-propagation-loss-model.h"

#include "ns3/building-list.h"
#include "ns3/building.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-list.h"
#include "ns3/pointer.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-phy.h"

// Loss models
#include "ns3/itu-r-1411-los-propagation-loss-model.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("FirstOrderBuildingsAwarePropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED(FirstOrderBuildingsAwarePropagationLossModel);

FirstOrderBuildingsAwarePropagationLossModel::FirstOrderBuildingsAwarePropagationLossModel()
{
    m_ituR1411Los = CreateObject<ItuR1411LosPropagationLossModel>();
    m_assess = CreateObject<NLOSassess>();
    m_frequency = 2160e6;
    txGain = 25;
    uni_rdm = CreateObject<UniformRandomVariable>();
}

FirstOrderBuildingsAwarePropagationLossModel::~FirstOrderBuildingsAwarePropagationLossModel()
{
}

TypeId
FirstOrderBuildingsAwarePropagationLossModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::FirstOrderBuildingsAwarePropagationLossModel")
            .SetParent<PropagationLossModel>()
            .AddConstructor<FirstOrderBuildingsAwarePropagationLossModel>()
            .SetGroupName("Propagation")
            .AddAttribute(
                "Frequency",
                "The Frequency  (default is 2.106 GHz).",
                DoubleValue(2160e6),
                MakeDoubleAccessor(&FirstOrderBuildingsAwarePropagationLossModel::SetFrequency),
                MakeDoubleChecker<double>())
            .AddAttribute(
                "TxGain",
                "Emminting Power (default 20 dBm)",
                DoubleValue(20),
                MakeDoubleAccessor(&FirstOrderBuildingsAwarePropagationLossModel::SetGain),
                MakeDoubleChecker<double>());

    return tid;
}

void
FirstOrderBuildingsAwarePropagationLossModel::SetFrequency(double freq)
{
    NS_LOG_FUNCTION(this);

    m_ituR1411Los->SetAttribute("Frequency", DoubleValue(freq));
    m_frequency = freq;
}

void
FirstOrderBuildingsAwarePropagationLossModel::SetGain(double gain)
{
    NS_LOG_FUNCTION(this);
    txGain = gain;
}

double
FirstOrderBuildingsAwarePropagationLossModel::GetLoss(Ptr<MobilityModel> rx,
                                                      Ptr<MobilityModel> tx) const
{
    NS_LOG_FUNCTION(this);

    NS_ASSERT_MSG((rx->GetPosition().z >= 0) && (tx->GetPosition().z >= 0),
                  "FirstOrderBuildingsAwarePropagationLossModel does not support underground nodes "
                  "(placed at z < 0)");

    double loss = 0.0;


    // For now singular loss model ITU-R-1411
    loss = ItuR1411(rx, tx);
    NS_LOG_DEBUG("Initial loss (before first order path loss) : " << loss);
    std::vector<Ptr<Building>> NLOSBuildings;
    std::vector<Ptr<Building>> AllBuildings;
    Vector rxPos = rx->GetPosition();
    Vector txPos = tx->GetPosition();
    int limit = BuildingList::GetNBuildings();
    for (int index = 0; index < limit; ++index)
    {
        Ptr<Building> CurBu = BuildingList::GetBuilding(index);
        if (CurBu->IsIntersect(rxPos, txPos))
        {
            NLOSBuildings.push_back(CurBu);
        }
        AllBuildings.push_back(CurBu);
    }
    if (loss > 90)
    {
        loss += Noise(loss);
    	return loss;
    }
    
    if (!NLOSBuildings.empty())
    {
        double direct_path_loss = loss + PenetrationLoss(NLOSBuildings);
        NS_LOG_DEBUG("NLOS first order buildings aware, direct path loss : " << direct_path_loss);
        double diffracted_path_loss =
            loss + NLOSDiffractionLoss(NLOSBuildings, AllBuildings, rx, tx);
        NS_LOG_DEBUG(
            "NLOS first order buildings aware, diffracted path loss : " << diffracted_path_loss);
        double reflected_path_loss = ReflectionLoss(AllBuildings, rx, tx);
        NS_LOG_DEBUG(
            "NLOS first order buildings aware, reflected path loss : " << reflected_path_loss);
        loss = std::min(std::min(direct_path_loss, diffracted_path_loss),
                        reflected_path_loss);
        NS_LOG_INFO(this << " ------------------------- 0-0 NLOS first order buildings aware loss : " << loss);
        loss += Noise(loss);
        //if (diffracted_path_loss > 1000){std::cout <<"tik"<<std::endl;std::cout << diffracted_path_loss<<std::endl;std::exit(1);}
        return loss;
    }
    loss += LOSDiffractionLoss(AllBuildings, rx, tx);
    NS_LOG_INFO(this << " 0-0 LOS first order buildings aware loss : " << loss);
    loss += Noise(loss);
    return loss;
}

double
FirstOrderBuildingsAwarePropagationLossModel::DoCalcRxPower(double txPowerDbm,
                                                            Ptr<MobilityModel> a,
                                                            Ptr<MobilityModel> b) const
{
    double rxPow = txPowerDbm;
    rxPow -= GetLoss(a, b);
    return rxPow;
}

int64_t
FirstOrderBuildingsAwarePropagationLossModel::DoAssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this);

    uni_rdm->SetStream(stream);

    return 1;
}

double
FirstOrderBuildingsAwarePropagationLossModel::PenetrationLoss(
    const std::vector<Ptr<Building>>& NLOSBuildings) const
{
    NS_LOG_FUNCTION(this);

    double loss = 0;
    for (size_t i = 0; i < NLOSBuildings.size(); ++i)
    {
        if (NLOSBuildings[i]->GetExtWallsType() == Building::Wood)
        {
            loss += 2 * 20;
        }
        else if (NLOSBuildings[i]->GetExtWallsType() == Building::ConcreteWithWindows)
        {
            loss += 2 * 30;
        }
        else if (NLOSBuildings[i]->GetExtWallsType() == Building::ConcreteWithoutWindows)
        {
            loss += 2 * 30;
        }
        else if (NLOSBuildings[i]->GetExtWallsType() == Building::StoneBlocks)
        {
            loss += 2 * 40;
        }
        else
        {
            NS_LOG_ERROR(this << " Unkwnon Wall Type");
        }
    }
    return loss;
}

double
FirstOrderBuildingsAwarePropagationLossModel::NLOSDiffractionLoss(
    const std::vector<Ptr<Building>>& NLOSBuildings,
    const std::vector<Ptr<Building>>& AllBuildings,
    Ptr<MobilityModel> rx,
    Ptr<MobilityModel> tx) const
{
    NS_LOG_FUNCTION(this);

    auto CreateTempMobilityModel = [](Vector position) {
        Ptr<ConstantPositionMobilityModel> tempMobility =
            CreateObject<ConstantPositionMobilityModel>();
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
            std::vector<Ptr<Building>> NLOScorner =
                m_assess->GetBuildingsBetween(corner_pos, tx, AllBuildings);
            if (NLOScorner.empty())
            {
                double theta = calculateAngle(tx, CornersPos[0], rx);
                NS_LOG_DEBUG("NLOS diffraction, theta : " << theta << " on corner "
                                                          << CornersPos[0]);
                return DiffFunct(theta);
            }
        }
        if (size_cor == 2)
        {
            Ptr<MobilityModel> corner_pos_1 = CreateTempMobilityModel(CornersPos[0]);
            Ptr<MobilityModel> corner_pos_2 = CreateTempMobilityModel(CornersPos[1]);
            std::vector<Ptr<Building>> NLOScorner_1 =
                m_assess->GetBuildingsBetween(corner_pos_1, tx, AllBuildings);
            std::vector<Ptr<Building>> NLOScorner_2 =
                m_assess->GetBuildingsBetween(corner_pos_2, tx, AllBuildings);
            if (NLOScorner_1.empty() || NLOScorner_2.empty())
            {
                double theta_1 = calculateAngle(tx, CornersPos[0], rx);
                double theta_2 = calculateAngle(tx, CornersPos[1], rx);
                NS_LOG_DEBUG("NLOS diffraction, theta_1 : "
                             << theta_1 << " on corner " << CornersPos[0]
                             << " theta_2 : " << theta_2 << " on corner " << CornersPos[1]);
                double loss_1 = DiffFunct(theta_1);
                double loss_2 = DiffFunct(theta_2);
                return std::min(loss_1, loss_2);
            }
        }
        if (size_cor > 2)
        {
            NS_LOG_ERROR(this << "Unexpected amount of corners");
            return std::numeric_limits<double>::infinity();
            ;
        }
    }
    return std::numeric_limits<double>::infinity();
}

double
FirstOrderBuildingsAwarePropagationLossModel::LOSDiffractionLoss(
    const std::vector<Ptr<Building>>& AllBuildings,
    Ptr<MobilityModel> rx,
    Ptr<MobilityModel> tx) const
{
    NS_LOG_FUNCTION(this);

    auto CreateTempMobilityModel = [](Vector position) {
        Ptr<ConstantPositionMobilityModel> tempMobility =
            CreateObject<ConstantPositionMobilityModel>();
        tempMobility->SetPosition(position);
        return tempMobility;
    };

    std::vector<double> losses;
    for (size_t j = 0; j < AllBuildings.size(); ++j)
    {
        std::vector<Vector> CornersPos = m_assess->GetCorner(AllBuildings[j], rx, tx);
        const size_t size_cor = CornersPos.size();
        if (size_cor == 1)
        {
            Ptr<MobilityModel> corner_pos = CreateTempMobilityModel(CornersPos[0]);
            std::vector<Ptr<Building>> NLOScorner =
                m_assess->GetBuildingsBetween(corner_pos, tx, AllBuildings);
            if (NLOScorner.empty())
            {
                double theta = -calculateAngle(tx, CornersPos[0], rx);
                NS_LOG_DEBUG("NLOS diffraction, theta : " << theta << " on corner "
                                                          << CornersPos[0]);
                losses.push_back(DiffFunct(theta));
            }
        }
        if (size_cor > 1)
        {
            NS_LOG_ERROR(
                this
                << "In LOS, a given building should at most be source of one (1) difffraction");
            return 0.0;
            ;
        }
    }

    if (!losses.empty())
    {
        auto maxIt =
            std::max_element(losses.begin(), losses.end());
        double maxL = *maxIt;
        if (maxL >= 0)
        {
            return maxL;
        }
        return 0.0;
    }
    return 0.0;
}

double
FirstOrderBuildingsAwarePropagationLossModel::ReflectionLoss(
    const std::vector<Ptr<Building>>& AllBuildings,
    Ptr<MobilityModel> rx,
    Ptr<MobilityModel> tx) const
{
    NS_LOG_FUNCTION(this);

    std::vector<double> refl_loss;
    double refl_coef = 0;

    auto CreateTempMobilityModel = [](Vector position) {
        Ptr<ConstantPositionMobilityModel> tempMobility =
            CreateObject<ConstantPositionMobilityModel>();
        tempMobility->SetPosition(position);
        return tempMobility;
    };

    for (size_t i = 0; i < AllBuildings.size(); ++i)
    {
        std::optional<Vector> reflection_point =
            m_assess->Getreflectionpoint(AllBuildings[i], rx, tx);
        if (reflection_point)
        {
            // Check if NLOS conditions are met
            Ptr<MobilityModel> reflectionMobility = CreateTempMobilityModel(*reflection_point);
            if (m_assess->GetBuildingsBetween(reflectionMobility, rx, {AllBuildings[i]}).empty() &&
                m_assess->GetBuildingsBetween(reflectionMobility, tx, {AllBuildings[i]}).empty())
            {
                // Assign reflection coefficient based on wall type
                switch (AllBuildings[i]->GetExtWallsType())
                {
                case Building::Wood:
                    refl_coef = 0.4;
                    break;
                case Building::ConcreteWithWindows:
                    refl_coef = 0.6;
                    break;
                case Building::ConcreteWithoutWindows:
                    refl_coef = 0.61;
                    break;
                case Building::StoneBlocks:
                    refl_coef = 0.9;
                    break;
                default:
                    NS_LOG_ERROR(this << " Unknown Wall Type");
                    continue;
                }
                // Calculate loss
                NS_LOG_DEBUG("NLOS reflection at : "
                             << *reflection_point
                             << " Tx-reflection-point loss : " << ItuR1411(tx, reflectionMobility)
                             << " reflection-point-Rx loss : " << ItuR1411(reflectionMobility, rx));
                // Calculate the 'first half'
                double first_half = txGain - ItuR1411(tx, reflectionMobility);
                // Apply attenuation coefficient
                double rxGain =
                    (first_half > 0)
                        ? (first_half * refl_coef - ItuR1411(reflectionMobility, rx))
                        : (first_half * (1 + (1 - refl_coef)) - ItuR1411(reflectionMobility, rx));
                double loss = txGain - rxGain;
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
FirstOrderBuildingsAwarePropagationLossModel::Noise(double loss) const
{
    NS_LOG_FUNCTION(this);

    double y = 0.25 * loss + 5;
    double top = y * 1.1;
    double bot = y * (1 - .1);
    double borne = std::abs(top - bot);
    uni_rdm->SetAttribute("Min", DoubleValue(-borne));
    uni_rdm->SetAttribute("Max", DoubleValue(+borne));
    return uni_rdm->GetValue();
}

double
FirstOrderBuildingsAwarePropagationLossModel::calculateAngle(Ptr<MobilityModel> rx,
                                                             Vector B,
                                                             Ptr<MobilityModel> tx) const
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
FirstOrderBuildingsAwarePropagationLossModel::DiffFunct(double angle) const
{
    NS_LOG_FUNCTION(this);

    double a = 0.70;
    double b = 24.9;
    double c = 3.555;
    double d = 31.7;
    return -a / (exp((angle / b) - c)) + d;
}

double
FirstOrderBuildingsAwarePropagationLossModel::ItuR1411(Ptr<MobilityModel> rx,
                                                       Ptr<MobilityModel> tx) const
{
    NS_LOG_FUNCTION(this);

    return m_ituR1411Los->GetLoss(rx, tx);
}

} // namespace ns3
