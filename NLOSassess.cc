/*
 * Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Hugo LE DIRACH  <hugo.le_dirach@onera.fr>
 */

#include "NLOSassess.h"

#include "building.h"

#include "ns3/mobility-module.h"
#include "ns3/object.h"
#include "ns3/pointer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NLOSassess");

TypeId
NLOSassess::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NLOSassess").SetParent<Object>().SetGroupName("Buildings")
        //.AddConstructor<NLOSassess> ()
        ;
    return tid;
}

TypeId
NLOSassess::GetInstanceTypeId() const
{
    return NLOSassess::GetTypeId();
}

NLOSassess::NLOSassess()
{
}

NLOSassess::~NLOSassess()
{
}

char
NLOSassess::zone(Ptr<MobilityModel> mob, Ptr<Building> b)
{
    double x = mob->GetPosition().x;
    double y = mob->GetPosition().y;

    if (((x < b->GetBoundaries().xMax) &&
         (x > b->GetBoundaries()
                  .xMin)) && // since the comparaison is stric, mob on bound is consider outside
        ((y < b->GetBoundaries().yMax) && (y > b->GetBoundaries().yMin)))
    {
        return 'Z'; // Node in walls
    }

    if (x <= b->GetBoundaries().xMin)
    {
        if (y >= b->GetBoundaries().yMax)
        {
            return 'A';
        }
        if (y <= b->GetBoundaries().yMin)
        {
            return 'G';
        }
        return 'H';
    }
    if (x >= b->GetBoundaries().xMax)
    {
        if (y >= b->GetBoundaries().yMax)
        {
            return 'C';
        }
        if (y <= b->GetBoundaries().yMin)
        {
            return 'E';
        }
        return 'D';
    }
    else
    {
        if (y >= b->GetBoundaries().yMax)
        {
            return 'B';
        }
        if (y <= b->GetBoundaries().yMin)
        {
            return 'F';
        }
    }
    return 'Z'; // Undefined zone
}

/*
return True for NLOS
*/
bool
NLOSassess::NLOSplan(Ptr<MobilityModel> eva, Ptr<MobilityModel> ave, Ptr<Building> b)
{
    double eva_x = eva->GetPosition().x;
    double eva_y = eva->GetPosition().y;
    double eva_z = eva->GetPosition().z;
    double ave_x = ave->GetPosition().x;
    double ave_y = ave->GetPosition().y;
    double ave_z = ave->GetPosition().z;

    double BxMax = b->GetBoundaries().xMax;
    double ByMax = b->GetBoundaries().yMax;
    double BzMax = b->GetBoundaries().zMax;
    double BxMin = b->GetBoundaries().xMin;
    double ByMin = b->GetBoundaries().yMin;

    double z1_diff = eva_z - BzMax;
    double z2_diff = ave_z - BzMax;

    if ((z1_diff > 0) && (z2_diff > 0))
    { // both node are strictly over roof top height
        return false;
    }
    if (z1_diff * z2_diff <= 0)
    { // one of the node is below or at roof height

        // Check if the line crosses the box on each plan, if so -> NLOS
        // Parameters for the X-Z plan
        double alpha_x = (eva_x - ave_x) / (eva_z - ave_z);
        double beta_x = eva_z - alpha_x * eva_x;
        // Parameters for the Y-Z plan
        double alpha_y = (eva_y - ave_y) / (eva_z - ave_z);
        double beta_y = eva_z - alpha_y * eva_y;
        // Parameters for the X-Y plan
        double alpha_z = (eva_y - ave_y) / (eva_x - ave_x);
        double beta_z = eva_y - alpha_z * eva_x;

        if (((alpha_x * BxMin - beta_x < BzMax) || (alpha_x * BxMax - beta_x < BzMax)) &&
            ((alpha_y * ByMin - beta_y < BzMax) || (alpha_y * ByMax - beta_y < BzMax)))
        {
            if (((eva_x == ave_x) && ((eva_x == BxMax) || (eva_x == BxMin))) ||
                ((eva_y == ave_y) && ((eva_y == ByMax) || (eva_y == ByMin))))
            {
                return true;
            }

            if (((alpha_z * BxMin - beta_z <= ByMax) && (alpha_z * BxMin - beta_z >= ByMin)) ||
                ((alpha_z * BxMax - beta_z <= ByMax) && (alpha_z * BxMax - beta_z >= ByMin)))
            {
                return true;
            }
        }
    }
    return false; // LOS by default
}

std::vector<Ptr<Building>>
NLOSassess::assesNLOS(Ptr<MobilityModel> eva,
                      Ptr<MobilityModel> ave,
                      std::vector<Ptr<Building>> buildings)
{
    /*
     * Buildings in NS3 are rectangles that are orthogonally aligned with the axis of the
     * environment, taking advantage of this model, we label the area surrounding a building and
     * avoid non-necessary calculation. For example, if the nodes are in respectively zone A and
     * zone G, their specific position does not matter, they will have LOS no matter the shape of
     * the building. However, if the nodes are in zone A and F, we need to evaluate if the link
     * crosses the building, which implies calculations.
     *
     * To assess if we are in a NLOS configuration we take the following steps:
     * 1. Determine the zone
     * 2. Making a quick decision based on default cases
     * 3. If uncertainty persists, compute the linear function between the nodes and check if it
     * crosses the building box
     *
     *      A   |   B    |   C
     *   -------+--------+-------
     *      H   |building|   D
     *   -------+--------+-------
     *      G   |   F    |   E
     */
    // All LOS cases that are automatic LOS --> no assesment needed
    std::vector<std::string> default_LOS = {"AA", "BB", "CC", "DD", "EE", "FF", "GG", "HH",
                                            "AB", "BA", "AC", "CA", "AH", "HA", "BC", "CB",
                                            "CD", "DC", "CE", "EC", "DE", "ED", "EF", "FE",
                                            "EG", "GE", "FG", "GF", "GH", "HG", "AG", "GA"};
    // All LOS cases that are automatic NLOS
    std::vector<std::string> default_NLOS = {"HD", "DH", "BF", "FB"};
    // Zone from which we evaluate the NLOS
    std::vector<char> evaluator = {'A', 'B', 'F', 'G', 'H'};

    char zone_a;
    char zone_b;
    std::string zone_comb;
    int limit = buildings.size();
    std::vector<Ptr<Building>> NLOSbuildings;

    for (int index = 0; index < limit; index++)
    {
        Ptr<Building> building = buildings[index];
        double building_zMax = building->GetBoundaries().zMax;
        double a_z = eva->GetPosition().z;
        double b_z = ave->GetPosition().z;
        zone_a = zone(eva, building);
        zone_b = zone(ave, building);
        zone_comb = std::string(1, zone_a) + zone_b;
        NS_ASSERT_MSG(zone_a != 'Z', "Undefined zone, check if node is note in the walls");
        NS_ASSERT_MSG(zone_b != 'Z', "Undefined zone, check if node is note in the walls");

        if (std::find(default_LOS.begin(), default_LOS.end(), zone_comb) != default_LOS.end())
        {
            // We have LOS for this building
            continue;
        }
        if (std::find(default_NLOS.begin(), default_NLOS.end(), zone_comb) != default_NLOS.end())
        {
            // We have NLOS for this building
            NLOSbuildings.push_back(building);
            continue;
        }
        if ((a_z >= building_zMax) && (b_z >= building_zMax))
        {
            // We have LOS for this building
            continue;
        }
        if (std::find(evaluator.begin(), evaluator.end(), zone_a) != evaluator.end())
        {
            if (NLOSplan(eva, ave, building))
            {
                NLOSbuildings.push_back(building);
            }
            continue;
        }
        NS_LOG_DEBUG("Could not asses NLOS");
    }
    return NLOSbuildings;
}

std::vector<Vector>
NLOSassess::GetCorner(Ptr<Building> CurrBuild, Ptr<MobilityModel> rx, Ptr<MobilityModel> tx)
{
    // Area where the diffraction happens in top left corner
    std::vector<std::string> Top_left = {"BG", "GB", "HB", "BH", "HC", "CH"};
    // Area where the diffraction happens in top right corner
    std::vector<std::string> Top_right = {"BE", "EB", "DB", "BD", "DA", "AD"};
    // Area where the diffraction happens in bottom left corner
    std::vector<std::string> Bot_left = {"HE", "EH", "FH", "HF", "FA", "AF"};
    // Area where the diffraction happens in bottom left corner
    std::vector<std::string> Bot_right = {"DG", "GD", "FD", "DF", "FC", "CF"};

    Vector Corner_pos;
    std::vector<Vector> Corners;
    char zone_a = zone(rx, CurrBuild);
    char zone_b = zone(tx, CurrBuild);
    std::string zone_comb = std::string(1, zone_a) + zone_b;

    if (std::find(Top_left.begin(), Top_left.end(), zone_comb) != Top_left.end())
    {
        Corner_pos.x = CurrBuild->GetBoundaries().xMin;
        Corner_pos.y = CurrBuild->GetBoundaries().yMax;
        Corners.push_back(Corner_pos);
        return Corners;
    }
    if (std::find(Top_right.begin(), Top_right.end(), zone_comb) != Top_right.end())
    {
        Corner_pos.x = CurrBuild->GetBoundaries().xMax;
        Corner_pos.y = CurrBuild->GetBoundaries().yMax;
        Corners.push_back(Corner_pos);
        return Corners;
    }
    if (std::find(Bot_left.begin(), Bot_left.end(), zone_comb) != Bot_left.end())
    {
        Corner_pos.x = CurrBuild->GetBoundaries().xMin;
        Corner_pos.y = CurrBuild->GetBoundaries().yMin;
        Corners.push_back(Corner_pos);
        return Corners;
    }
    if (std::find(Bot_right.begin(), Bot_right.end(), zone_comb) != Bot_right.end())
    {
        Corner_pos.x = CurrBuild->GetBoundaries().xMin;
        Corner_pos.y = CurrBuild->GetBoundaries().yMax;
        Corners.push_back(Corner_pos);
        return Corners;
    }
    if ((zone_comb == "CG") || (zone_comb == "GC"))
    {
        Vector Corner_pos_2;
        Corner_pos.x = CurrBuild->GetBoundaries().xMin;
        Corner_pos.y = CurrBuild->GetBoundaries().yMax;
        Corner_pos_2.x = CurrBuild->GetBoundaries().xMax;
        Corner_pos_2.y = CurrBuild->GetBoundaries().yMin;
        Corners.push_back(Corner_pos);
        Corners.push_back(Corner_pos_2);
        return Corners;
    }
    if ((zone_comb == "AE") || (zone_comb == "EA"))
    {
        Vector Corner_pos_2;
        Corner_pos.x = CurrBuild->GetBoundaries().xMin;
        Corner_pos.y = CurrBuild->GetBoundaries().yMin;
        Corner_pos_2.x = CurrBuild->GetBoundaries().xMax;
        Corner_pos_2.y = CurrBuild->GetBoundaries().yMax;
        Corners.push_back(Corner_pos);
        Corners.push_back(Corner_pos_2);
        return Corners;
    }

    return Corners;
}

std::optional<Vector>
NLOSassess::Getreflectionpoint(Ptr<Building> Building, Ptr<MobilityModel> rx, Ptr<MobilityModel> tx)
{
    // y_min areas
    std::vector<std::string> y_min = {"GF", "FG", "FE", "EF", "EG", "GE", "FF"};
    // y_max areas
    std::vector<std::string> y_max = {"AB", "BA", "BC", "CB", "AC", "CA", "BB"};
    // y_min areas
    std::vector<std::string> x_min = {"AH", "HA", "HG", "GH", "GA", "AG", "HH"};
    // y_max areas
    std::vector<std::string> x_max = {"CD", "DC", "DE", "ED", "EC", "CE", "DD"};

    char zone_a = zone(rx, Building);
    char zone_b = zone(tx, Building);
    std::string zone_comb = std::string(1, zone_a) + zone_b;

    if (std::find(y_min.begin(), y_min.end(), zone_comb) != y_min.end())
    {
        double y_refl = Building->GetBoundaries().yMin;
        double rx_x = rx->GetPosition().x;
        double rx_y = rx->GetPosition().y;
        double tx_x = tx->GetPosition().x;
        double tx_y = tx->GetPosition().y;
        double x_refl =
            (rx_x * (y_refl - tx_y) - tx_x * (rx_y - y_refl)) / ((y_refl - tx_y) - (rx_y - y_refl));
        return Vector(x_refl, y_refl, 1);
    }
    if (std::find(y_max.begin(), y_max.end(), zone_comb) != y_max.end())
    {
        double y_refl = Building->GetBoundaries().yMax;
        double rx_x = rx->GetPosition().x;
        double rx_y = rx->GetPosition().y;
        double tx_x = tx->GetPosition().x;
        double tx_y = tx->GetPosition().y;
        double x_refl =
            (rx_x * (y_refl - tx_y) - tx_x * (rx_y - y_refl)) / ((y_refl - tx_y) - (rx_y - y_refl));
        return Vector(x_refl, y_refl, 1);
    }
    if (std::find(x_min.begin(), x_min.end(), zone_comb) != x_min.end())
    {
        double x_refl = Building->GetBoundaries().xMin;
        double rx_x = rx->GetPosition().x;
        double rx_y = rx->GetPosition().y;
        double tx_x = tx->GetPosition().x;
        double tx_y = tx->GetPosition().y;
        double y_refl =
            (rx_y * (x_refl - tx_x) + tx_y * (x_refl - rx_x)) / ((x_refl - tx_y) + (x_refl - rx_x));
        return Vector(x_refl, y_refl, 1);
    }
    if (std::find(x_max.begin(), x_max.end(), zone_comb) != x_max.end())
    {
        double x_refl = Building->GetBoundaries().xMax;
        double rx_x = rx->GetPosition().x;
        double rx_y = rx->GetPosition().y;
        double tx_x = tx->GetPosition().x;
        double tx_y = tx->GetPosition().y;
        double y_refl =
            (rx_y * (x_refl - tx_x) + tx_y * (x_refl - rx_x)) / ((x_refl - tx_y) + (x_refl - rx_x));
        return Vector(x_refl, y_refl, 1);
    }
    return std::nullopt;
}

} // namespace ns3
