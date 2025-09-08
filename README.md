# First Order Buildings Aware (FOBA) Propagation Loss Model

**Language:** C++
**Library/Framework:** ns-3 (Network Simulator 3)

This version (1.0) has been tested with ns-3.44 release.

## Overview
This project implements a first-order buildings-aware propagation loss model for simulating wireless communication networks in urban environments.

The model takes into account the presence of buildings and their impact on signal propagation, including penetration, diffraction, and reflection losses.
It is based on the concept of **dominant path models** which, unlike exhaustive ray tracing or radiosity calculations, identifies and only simulates the most influential paths (e.g., line-of-sight, first-order reflections, and possibly some stronger diffractions) between the transmitter and receiver. By focusing on these dominant signal propagation paths, the method achieves a more manageable computational load while still providing reliable predictions for signal behavior in specific scenarios, especially where detailed interaction with the environment's geometry is crucial.

The "Slides of explanation.pdf" file will give a good overview of the module, how it works, what it does and it's limitations. We strongly advise to parse it before usage.

The full paper that detail this model is available with in the last section.


## Code architecture

<img width="1588" height="538" alt="Screenshot from 2025-09-08 10-11-41" src="https://github.com/user-attachments/assets/1a407cf9-cd76-4cdb-853e-f23134afa679" />

### Key Features

* **Buildings Awareness**: Accounts for the presence of buildings between transmitter and receiver nodes
* **First-Order Approximation**: Limits signal reflections to one and considers only one diffraction effect and one reflection to reduce computational complexity
* **Loss Calculation**: Computes path loss based on node positions, building presence, and signal interaction with buildings
* **Integration with ns-3**: Utilizes ns-3's `BuildingsPropagationLossModel` and `MobilityModel` classes

## Usage

To use this tool, simply download the repository as .zip (or by cloning it) and place the "first-order-buildings-aware-path-loss" folder in the _contrib_ folder in your ns-3.xx repository.

In your code:
* write `#include <ns3/first-order-buildings-aware-propagation-loss-model.h>` along the other header files
* When setting up the channel condition use `.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel")`

## Notes

* This model is **not intended for absolute realistic loss simulations** (as per the warning in the code comments), it's goal it to bring some coherence in the interaction of the signal with the environment.
* This model uses a noise component based on the RNG of ns-3, to disable it in a simulation configuration script you can do : `lossModel->SetAttribute("NoiseEnabled", BooleanValue(false));` or `wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel", "NoiseEnabled", BooleanValue(false));`.
* The `ItuR1411LosPropagationLossModel` class is referenced but not included in this repository.
* For more information on ns-3 propagation models, refer to the official [ns-3 documentation](https://www.nsnam.org/docs/).

## Authors

Hugo Le Dirach 

## Reference

Hugo Le Dirach, Marc Boyer, and Emmanuel Lochin. 2025. Building-Aware Path Loss Modeling in ns-3. In Proceedings of the 2025 International Conference on ns-3 (ICNS3 '25). Association for Computing Machinery, New York, NY, USA, 143–152. https://doi.org/10.1145/3747204.3747220

## License
Copyright (c) 2024 Office National d'Etude et de Recherche Aérospatiale (ONERA)
SPDX-License-Identifier: GPL-2.0-only
