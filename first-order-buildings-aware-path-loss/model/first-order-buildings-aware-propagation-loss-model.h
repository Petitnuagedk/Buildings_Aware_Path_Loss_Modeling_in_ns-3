/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#ifndef FIRST_ORDER_DETERMINISTIC_PATHLOSS_H
#define FIRST_ORDER_DETERMINISTIC_PATHLOSS_H

#include "foba-toolbox.h"

#include "ns3/propagation-environment.h"
#include "ns3/propagation-loss-model.h"

namespace ns3
{

class ItuR1411LosPropagationLossModel;

/**
 * @ingroup buildings
 *
 * @brief The FirstOrderBuildingsAwarePropagationLossModel takes into account the presence
 * or not of buildings in the sight of nodes to compute loss. To limit the complexity and
 * computation overhead, we limit the reflection of signal to one and consider only one
 * diffraction effect (thus 'First Order'). Based on dominant path method, we compute only
 * the strongest diffraction and reflection, then use the least loss amoung LOS, penetration,
 * difraction and reflection to use as loss to apply to the signal.
 *
 * @warning This model is meant to introduce realistic path loss by considering interaction
 * of the signal with buildings, however, it has a level of abstraction, it does not reflect
 * the exact behavior that the signal would have in real life.
 *
 */

class FirstOrderBuildingsAwarePropagationLossModel : public PropagationLossModel
{
  public:
    /**
     * @brief Get the type ID.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();
    FirstOrderBuildingsAwarePropagationLossModel();
    ~FirstOrderBuildingsAwarePropagationLossModel() override;

    /**
     * set the propagation frequency
     *
     * @param freq frequency in GHz
     */
    void SetFrequency(double freq);

    /**
     * set the emmittsing power
     *
     * @param gain Gain in dB
     */
    void SetGain(double gain);

    /**
     * @brief Compute the path loss according to the nodes position
     * and the presence or not of buildings in between.
     *
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the propagation loss (in dB)
     */
    double GetLoss(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const;

  private:
    /**
     * Computes the received power by applying the pathloss model
     *
     * @param txPowerDbm tx power in dBm
     * @param a tx mobility model
     * @param b rx mobility model
     * @return the rx power in dBm
     */
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override;

    /**
     * Assign a fixed random variable stream number to the random variables used by this model.
     *
     * Subclasses must implement this; those not using random variables
     * can return zero.
     *
     * @param stream first stream index to use
     * @return the number of stream indices assigned by this model
     */
    int64_t DoAssignStreams(int64_t stream) override;

    /**
     * @brief Compute the path loss with additionnal loss for all walls traversed.
     *
     * @param NLOSBuildings the buildings between the sight of the two nodes
     * @returns the penetration loss (in dB)
     */
    double PenetrationLoss(const std::vector<Ptr<Building>>& NLOSBuildings) const;

    /**
     * @brief Compute the path loss that is diffracted by a building with positive angles.
     *
     * Since we only consider one diffraction phenomenon, only one building will be responsible for
     * this type of loss, however, all buildings are evaluated. if there are multiple valide
     * diffraction, only the building that generates the least loss is kept.
     *
     * When assesing the loss, if we accounter a scenario where the the NLOSDiffractionLoss has
     * been called but where there are no valid diffraction, we need to invalidate the result. To
     * do so, we return a +infinity loss value so that when it is compare to penetration and
     * reflection we are sure it wont be selected.
     *
     * @param NLOSBuildings the buildings between the sight of the two nodes
     * @param AllBuildings List of all the buildings in the simulation
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the diffraction loss (in dB)
     */
    double NLOSDiffractionLoss(const std::vector<Ptr<Building>>& NLOSBuildings,
                               const std::vector<Ptr<Building>>& AllBuildings,
                               Ptr<MobilityModel> rx,
                               Ptr<MobilityModel> tx) const;

    /**
     * @brief Compute the path loss that is diffracted by the building(s) with negative angles
     *
     * @param AllBuildings the buildings between the sight of the two nodes
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns the diffraction loss (in dB)
     */
    double LOSDiffractionLoss(const std::vector<Ptr<Building>>& AllBuildings,
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
    double ReflectionLoss(const std::vector<Ptr<Building>>& AllBuildings,
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
     * @brief Calculate the angle between AB and BC on the x-y plan
     *
     * @param rx a 3D point
     * @param B a 3D point
     * @param tx a 3D point
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
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @returns loss (in dB)
     */
    double ItuR1411(Ptr<MobilityModel> rx, Ptr<MobilityModel> tx) const;

    Ptr<ItuR1411LosPropagationLossModel>
        m_ituR1411Los;                  ///< ItuR1411LosPropagationLossModel variable holder
    Ptr<NLOSassess> m_assess;           ///< FOBA toolbox
    double m_frequency;                 ///< Operating frequency
    double txGain;                      ///< Emiting gain
    Ptr<UniformRandomVariable> uni_rdm; ///< RandomVariable object
};

} // namespace ns3

#endif /* FIRST_ORDER_DETERMINISTIC_PATHLOSS_H */
