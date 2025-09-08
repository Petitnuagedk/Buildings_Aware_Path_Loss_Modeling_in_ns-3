.. include:: replace.txt
.. highlight:: cpp
.. highlight:: bash

First Order Buildings Aware Path loss model (draft v0.2)
====================================================

This chapter describes the implementation of the |ns3| model for a buildings
aware path loss model.

The model described in this document is not based on a certified standard (such as the 3GPP or ITU-R standards). This model aims to integrate the position and nature of buildings and consider them in the calculation of path loss between two nodes. This model incorporates signal loss based on three components:

- Direct signal path through buildings
- Diffraction
- Reflection.

Although not intended to be fully realistic, the model aims to improve the consistency of signal path loss in short-distance environments obstructed by objects.

The source code for the model lives in the directory ``src/buildings/model/``.

.. _fig-MP:

.. figure:: figures/MP.*

Simplified schematic of the signal paths when the line-of-sight is obstructed. Three phenomena are represented: penetration (attenuation of the signal when traversing different mediums), diffraction (attenuation occurs when the signal is diverted), and reflection (depending on the material nature, the signal is partly absorbed and partly diffused in all directions).

The model is based on the dominant path method, a hybrid approach that
balances computational efficiency with acceptable accuracy. Unlike
exhaustive ray tracing or radiosity calculations, this method identifies
and simulates only the most influential paths—such as line-of-sight,
first-order reflections, and potentially stronger diffractions—between
the transmitter and receiver. By focusing on these dominant signal propagation
paths, it maintains a manageable computational load while still delivering
reliable predictions of signal behavior, particularly in scenarios where
specific interaction with the environment’s geometry is crucial.

Scope and Limitations
---------------------

- The model is tailored for short-range (less than 1 km) buildings obstructed settings.
- The frequency will impact the loss value but is limited between 0.1 and 80 GHz.
- The model has a level of abstraction, it does not reflect the exact behavior of the loss as it would happen in real life. It will work without building but is not made for such usage.
- The magic numbers used in the model reflect trends but are not standardized.
- This model is not as strict as ``HybridBuildingsPropagationLossModel`` which requires every node to be linked with a building but since the ``FirstOrderBuildingsAwarePropagationLossModel`` evaluate every building in the simulation, it may dig into performance compared to stochastic models.


Propagation Model Design
------------------------

The model inherits from ``BuildingsPropagationLossModel`` and will most likely only be
called through the ``GetLoss()`` method. The figure below describes the overall workflow.
Let's go through it together to better understand each component.
You can open ``first-order-buildings-aware-propagation-loss-model.cc`` to follow the process.


.. _fig-CodeArc:

.. figure:: figures/CodeArc.*

    Simplified schematic of the penetration, diffraction, and reflection phenomenon.

First, the physical layer (e.g., YansWifiPhy) will call the ``GetLoss()`` method from the propagation loss
model that has been configured by the user script, for this example purpose, we will
consider the loss model to be ``FirstOrderBuildingsAwarePropagationLossModel``.

The ``GetLoss()`` method full appelation is ``double GetLoss(Ptr<MobilityModel> rx,
Ptr<MobilityModel> tx) const``, it returns a loss value of type double, and the
method is constant, meaning it will not modify the values of passed arguments.

The arguments are, by order, the mobility model of the receiver (Rx) and the mobility
model of the emitter (Tx) (although, since the path is symmetric/reversible, this order does not
matter from a result perspective, it is there to better distinguish the two nodes).

We use the ITU-R 1411 norm as the LOS path loss and basis for any additional loss, so the
first computation is this LOS loss. Then, we call ``GetBuildingsBetween()``, a method
from the toolbox that will provide the buildings, if any, that obstruct the LOS between
the nodes. There we have two cases: LOS and NLOS.

**LOS case**: An interesting fact about diffraction is that the presence of an object
near the LOS of two nodes will create interferences. To account for this, we need to
check the proximity of every building and then apply the appropriate loss if necessary.

**NLOS case**: If one or more building (s) obstruct the LOS of two nodes, three phenomena may occur.

1. Penetration loss: the signal loses strength for every medium (ex: walls) it traverses.
2. NLOS diffraction: the signal loses strength by being deviated.
3. Reflection loss: the signal is partly reflected and partly absorbed by a surface.

All three of these are computed, then the ``LossCombinator()`` takes the least loss (as the signal would be received as the strongest from the least loss path).

**Noise**: At the end, a noise is added to account for real-life, unpredictable interferences. ``GetLoss()`` then provides the loss value in return and the process is complete.

Physical behavior
~~~~~~~~~~~~~~~~~

.. _fig-PDR:

.. figure:: figures/PDR.*

    Simplified schematic of the penetration, diffraction, and reflection phenomenon.

This model is based on three real-life phenomena shown in the figure above.

For the penetration loss, for each wall traversed, we add a loss according to the material.

The diffraction loss depends on the angle of obstruction. For LOS the angle is φ on the figure, for NLOS it is θ.
As φ increases, the building/object is further from the line, therefore the loss is reduced.
As θ increases, the building/object is further obstructing the LOS, therefore the loss is increased.

The reflection calculation is slightly different, we need to compute the loss of the
first 'half' (from Tx to reflection point) then apply an attenuation coefficient to the
power at this point, then apply the loss of the rest of the path (from the  reflection point
to Rx).

Usage
-----

This model is a subclass of the ``BuildingsPropagationLossModel``, it is as easy to use as any other propagation model.

If used along a physical layer, it is used as follows::

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel;
    wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel");

If the user wants to summon the ``GetLoss()`` by itself, outside the automatic loop this can be used::

    Ptr<FirstOrderBuildingsAwarePropagationLossModel>
    firstOrderPropagationLossModel = CreateObject<FirstOrderBuildingsAwarePropagationLossModel>();

Then the user can call ``FOpropagationLossModel.GetLoss(mob1, mob2)``.

Attributes
~~~~~~~~~~

This model list of attributes :

- The frequency: The operating frequency for wireless communications).
- The emitting power: Gain of the sending nodes.

To configure them ::

    wifiChannel.AddPropagationLoss("ns3::FirstOrderBuildingsAwarePropagationLossModel"
                                   "Frequency",
                                   DoubleValue(5e9),
                                   "TxGain",
                                   DoubleValue(20));

Or ::

    Ptr<FirstOrderBuildingsAwarePropagationLossModel>
    FOpropagationLossModel = CreateObject<FirstOrderBuildingsAwarePropagationLossModel>();
    FOpropagationLossModel->SetAttribute("TxGain", DoubleValue(22.0));

Output: The model generates a loss value of type ``double``. The logging info will give more
context to what is happening (Initial loss value, loss value for each phenomenon, noise level, ...).

Examples and Tests
~~~~~~~~~~~~~~~~~~

There are two examples available in ``src/propagation/examples/``.
The test file is to be found in ``src/propagation/test``

Validation
----------

No formal verification has been done to compare the result in the simulator with real-life behavior.

References
----------

[`1 <https://ieeexplore.ieee.org/abstract/document/6966147?casa_token=HsOL3mm298UAAAAA:qrEJZ1_ZSPxk-8ocND5kDqVg-zgAXcAjAemVKGWTIYfnnm9As2zYNCbd2MkbIlUVsTD7l1iblK9->`_] Ignacio Rodriguez, Huan C. Nguyen, Niels T. K. Jorgensen, Troels B. Sorensen,
and Preben Mogensen. Radio propagation into modern buildings: Attenuation measurements in the range from 800 MHz to 18 GHz. In 2014 IEEE 80th Vehicular Technology Conference (VTC2014-Fall), page 1–5, Vancouver, BC, Canada, September 2014. IEEE

[`1 <https://ieeexplore.ieee.org/abstract/document/7841898?casa_token=cVMkdpKqujkAAAAA:E8kuoqIC-oeRg478fmhXAOB5ykaVfuSuzdeQ_VRb0KIBFdednjsejYn6_JoWN5QS1rBZ0GBpnrfr>`_] Sijia Deng, Geoge R. MacCartney, and Theodore S. Rappaport. Indoor and outdoor 5g diffraction measurements and models at 10, 20, and 26 GHz. page 1–7, Washington, DC, USA, December 2016. IEEE.

[`3 <https://ieeexplore.ieee.org/abstract/document/477526?casa_token=7BbgDhFCziAAAAAA:dA1ch6wztw6CvFVeY9_zq64nSrtKGsKV8kh1nX-jB49fxKpxzRKb63S8WoNf4CABkqpl9GLZyCWK>`_] K. Sato, T. Manabe, J. Polivka, T. Ihara, Y. Kasashima, and K. Yamaki, “Measurement of the complex refractive index of concrete at 57.5 GHz,” IEEE Trans. Antennas Propagat., vol. 44, no. 1, pp. 35–40, Jan. 1996, doi: 10.1109/8.477526.
