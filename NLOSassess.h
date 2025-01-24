/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#ifndef NLOSASSESS_H
#define NLOSASSESS_H

#include "building.h"

#include "ns3/mobility-module.h"
#include "ns3/object.h"
#include "ns3/pointer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

namespace ns3
{

class Building;

/**
 * @brief Asses if the 3D straight line between to points intersect a building.
 *
 * This class provide an optimization tool to make quick decision on Line of
 * Sight (LoS) between two point and a given building.
 */
class NLOSassess : public Object
{
  public:
    /**
     * @brief Get the type ID.
     * @return The object TypeId.
     */
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    NLOSassess();
    ~NLOSassess() override;

    /**
     * @brief Assesses the number of buildings that cause NLOS.
     *
     * @param eva first point of the line to evaluate.
     * @param ave second point of the line to evaluate.
     * @param buildings contains the buildings to evaluate.
     * @return the buildings that intersect the line between the two points.
     */
    std::vector<Ptr<Building>> assesNLOS(Ptr<MobilityModel> eva,
                                         Ptr<MobilityModel> ave,
                                         std::vector<Ptr<Building>> buildings);

    /**
     * @brief Gives the corners that may produce diffraction between Rx and Tx
     *
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @param CurrBuild Current Building to evaluate
     * @return the corners of buildings that may produce a diffraction
     */
    std::vector<Vector> GetCorner(Ptr<Building> CurrBuild,
                                  Ptr<MobilityModel> rx,
                                  Ptr<MobilityModel> tx);

    /**
     * @brief Gives the corners that may produce reflection between Rx and Tx
     *
     * @param rx the mobility model of the destination
     * @param tx the mobility model of the source
     * @param Building Current Building to evaluate
     * @return the coordinates on the surface that may produce a diffraction
     */
    std::optional<Vector> Getreflectionpoint(Ptr<Building> Building,
                                             Ptr<MobilityModel> rx,
                                             Ptr<MobilityModel> tx);

  private:
    /** @brief The point is allocated to one of the zone detailed in the figure bellow.
     *
     *        A   |   B    |   C
     *     -------+--------+-------
     *        H   |building|   D
     *     -------+--------+-------
     *        G   |   F    |   E
     *
     * @param mob point to locate relatively to the building.
     * @param b building that will be used to categorize the point.
     * @return the zone in which the point belong relatively to the building.
     */
    char zone(Ptr<MobilityModel> mob, Ptr<Building> b);

    /**
     * @brief Assesses if the building causes NLOS.
     *
     * @param eva first point of the line to evaluate.
     * @param ave second point of the line to evaluate.
     * @param b building to evaluate.
     * @return true if the building causes a NLOS, false if there is LOS
     * between eva and ave.
     */
    bool NLOSplan(Ptr<MobilityModel> eva, Ptr<MobilityModel> ave, Ptr<Building> b);
};

} // namespace ns3
#endif
