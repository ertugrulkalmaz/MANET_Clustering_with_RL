# Reinforcement Learning Based LEACH Clustering in OMNeT++

This project extends the LEACH clustering protocol with **reinforcement learning (RL)** to improve cluster-head (CH) selection in mobile ad hoc and sensor-style networks. The implementation integrates **tabular Q-learning and SARSA** into a LEACH routing module using **OMNeT++ 6.0** and **INET 4.5**, and evaluates performance under a comprehensive simulation campaign.

The goal is to investigate whether lightweight, distributed RL can improve **cluster stability, energy fairness, and delivery performance** under mobility and dynamic conditions without increasing control overhead.

---

## Key Features

- LEACH clustering implementation for INET 4.5
- **Q-learning** and **SARSA** for CH selection
- Compact discrete state representation (160 states)
- Binary action space: *become CH* / *remain non-CH*
- Reward shaping based on energy, delivery activity, connectivity, and distance to base station
- Extensive simulation campaign with topology, mobility, energy, and PHY/MAC sweeps
- Automated repetition with independent seeds and confidence intervals

---

## System Requirements

- **OMNeT++**: 6.0  
- **INET Framework**: 4.5  
- Linux or Windows

---

## Project Structure

```
inet4.5/
├── examples/LeachProtocolSimulation
   ├── address.xml
   ├── bonnmotion.movements
   ├── LeachProtocolNetwork.ned
   ├── nodePos.csv
   ├── obstacles.xml
   ├── omnetpp.ini
   ├── omnetpp_campaign.ini
   ├── omnetpp_original.ini
   ├── omnetpp_qlearning.ini
   ├── omnetpp_sarsa.ini
   ├── run_comparison.bat
   ├── run_comparison.sh
   ├── turtle.xml
├── src/inet
   ├── node/LEACHnode
      ├── LEACHbs.ned
      ├── LEACHnode.ned
   ├── routing/leach
      ├── Leach.cc
      ├── Leach.h
      ├── Leach.ned
      ├── LeachBS.cc
      ├── LeachBS.h
      ├── LeachBS.ned
      ├── LeachPkts.msg
      ├── LeachPkts_m.cc
      ├── LeachPkts_m.h
```

---

## Reinforcement Learning Model

Each node learns independently using a compact tabular RL formulation with:
- 160 discrete states
- 2 actions (CH / non-CH)
- ε-greedy exploration
- Q-learning or SARSA updates

---

## Running the Simulation

1. Open OMNeT++ 6.0
2. Download INET 4.5
3. Copy the project files to INET 4.5
4. Open `examples/LeachProtocolSimulation/omnetpp_campaign.ini`
5. Choose a configuration and run