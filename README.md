# First Order Buildings Aware Propagation Loss Model

**Language:** C++
**Library/Framework:** ns-3 (Network Simulator 3)

## Overview
This project implements a first-order buildings-aware propagation loss model for simulating wireless communication networks in urban environments. The model takes into account the presence of buildings and their impact on signal propagation, including penetration, diffraction, and reflection losses.
It is based on the concept of **dominant path models** which, unlike exhaustive ray tracing or radiosity calculations, identifies and only simulates the most influential paths (e.g., line-of-sight, first-order reflections, and possibly some stronger diffractions) between the transmitter and receiver. By focusing on these dominant signal propagation paths, the method achieves a more manageable computational load while still providing reliable predictions for signal behavior in specific scenarios, especially where detailed interaction with the environment's geometry is crucial.

The full paper that detail this model is under review and will be posted here at some point.


## Code architecture
![Screenshot from 2025-01-20 08-56-01](https://github.com/user-attachments/assets/270b27d7-77dc-40b2-9f12-158878ddacd2)

### Key Features

* **Building Awareness**: Accounts for the presence of buildings between transmitter and receiver nodes
* **First-Order Approximation**: Limits signal reflections to one and considers only one diffraction effect and one reflection to reduce computational complexity
* **Loss Calculation**: Computes path loss based on node positions, building presence, and signal interaction with buildings
* **Integration with ns-3**: Utilizes ns-3's `BuildingsPropagationLossModel` and `MobilityModel` classes

## Usage

To use this tool, simply download the "first-order-buildings-aware-path-loss" folder and place it in the _contrib_ folder in your ns-3.xx repository.

In your code:
* write `#include <ns3/first-order-buildings-aware-propagation-loss-model.h>` along the other header files
* When setting up the channel condition use `.AddPropagationLoss("ns3::FirstOrderBuildingAwarePropagationLossModel")`

## Notes

* This model is **not intended for absolute realistic loss simulations** (as per the warning in the code comments), it's goal it to bring some coherence in the interaction of the signal with the environment.
* This model uses a noise component based on the RNG of ns-3, to desable it in a simulation configuration script you can do : `lossModel->SetAttribute("NoiseEnabled", BooleanValue(false));` or `wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel", "NoiseEnabled", BooleanValue(false));`.
* The `ItuR1411LosPropagationLossModel` class is referenced but not included in this repository.
* For more information on NS-3 propagation models, refer to the official [NS-3 documentation](https://www.nsnam.org/docs/).

## Authors

Hugo Le Dirach 

## License
Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
SPDX-License-Identifier: GPL-2.0-only
