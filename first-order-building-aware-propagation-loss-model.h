/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#ifndef FIRSTORDERDETERMINISTICPATHLOSS_H
#define FIRSTORDERDETERMINISTICPATHLOSS_H

#include "NLOSassess.h"
#include "buildings-propagation-loss-model.h"

#include "ns3/propagation-environment.h"

namespace ns3
{

class ItuR1411LosPropagationLossModel;

/**
 * @ingroup buildings
 * @ingroup propagation
 *
 * @brief The FirstOrderBuildingAwarePropagationLossModel takes into account the presence
 * or not of buildings in the sight of nodes to compute loss. To limit the complexity and
 * computation overhead, we limit the reflection of signal to one and consider only one
 * diffraction effect (thus 'First Order').
 *
 *  @warning This model is not meant to simulate realistic loss.
 *
 */

class FirstOrderBuildingAwarePropagationLossModel : public BuildingsPropagationLossModel
{
  public:
    /**
     * @brief Get the type ID.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();
    FirstOrderBuildingAwarePropagationLossModel();
    ~FirstOrderBuildingAwarePropagationLossModel() override;

    /**
     * set the propagation frequency
     *
     * @param freq
     */
    void SetFrequency(double freq);

    /**
     * @brief Compute the path loss according to the nodes position
     * and the presence or not of buildings in between.
     *
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the propagation loss (in dB)
     */
    double GetLoss(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const override;

  private:
    /**
     * @brief Compute the path loss that goes throught the building(s)
     *
     * @param NLOSBuildings the buildings between the sight of the two nodes
     * @returns the penetration loss (in dB)
     */
    double PenetrationLoss(std::vector<Ptr<Building>> NLOSBuildings) const;

    /**
     * @brief Compute the path loss that is diffracted by the building(s) with positive angles
     *
     * @param NLOSBuildings the buildings between the sight of the two nodes
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the diffraction loss (in dB)
     */
    double NLOSDiffractionLoss(std::vector<Ptr<Building>> NLOSBuildings,
                               std::vector<Ptr<Building>> AllBuildings,
                               Ptr<MobilityModel> rx,
                               Ptr<MobilityModel> tx) const;

    /**
     * @brief Compute the path loss that is diffracted by the building(s) with negative angles
     *
     * @param NLOSBuildings the buildings between the sight of the two nodes
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the diffraction loss (in dB)
     */
    double LOSDiffractionLoss(std::vector<Ptr<Building>> AllBuildings,
                              Ptr<MobilityModel> rx,
                              Ptr<MobilityModel> tx) const;

    /**
     * @brief Compute the path loss that is reflected on the building(s)
     *
     * @param AllBuildings All the buildings in the simulation
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the reflection loss (in dB)
     */
    double ReflectionLoss(std::vector<Ptr<Building>> AllBuildings,
                          Ptr<MobilityModel> rx,
                          Ptr<MobilityModel> tx) const;

    /**
     * @brief Adds noise to the loss, proportionnaly to it's strength
     *
     * @param loss the loss to apply to the signal
     * @returns the propagation loss (in dB)
     */
    double Noise(double loss) const;

    /**
     * @brief Random double generator
     *
     * @param min lower bound of the random double to get
     * @param max upper bound of the random double to get
     * @returns a random double value between min and max
     */
    double getRandomDouble(double min, double max) const;

    /**
     * @brief Calculate the angle between AB and BC on the x-y plan
     *
     * @param A a 3D point
     * @param B a 3D point
     * @param C a 3D point
     * @returns The angle (in degrees) between AB and BC
     */
    double calculateAngle(Ptr<MobilityModel> rx, Vector B, Ptr<MobilityModel> tx) const;

    /**
     * @brief Signal attenuation as a function of the shadowing angle
     *
     * @param angle angle of the shadow between tx, the corner and rx
     * @returns loss (in dB)
     */
    double DiffFunct(double angle) const;

    /**
     * @brief Get the loss between two node according to ItuR1411
     *
     * @param angle angle of the shadow between tx, the corner and rx
     * @returns loss (in dB)
     */
    double ItuR1411(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const;

    /// ItuR1411LosPropagationLossModel
    Ptr<ItuR1411LosPropagationLossModel> m_ituR1411Los;
    Ptr<NLOSassess> m_assess;
    double m_frequency;
};

} // namespace ns3

#endif /* FIRSTORDERDETERMINISTICPATHLOSS_H */
