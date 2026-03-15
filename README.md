# Distributive Perimetral Queue Balancing Mechanisms: Towards Equitable Urban Traffic Gating and Fair Perimeter Control

## Introduction

This is the online repository of *"Distributive Perimetral Queue Balancing Mechanisms: Towards Equitable Urban Traffic Gating and Fair Perimeter Control"*. This repository contains a Python-implementation of a traffic microsimulation to demonstrate the potential of the fair perimeter controller based on queue balancing mechanisms. The repository is based on [SUMO (provided by DLR)](https://eclipse.dev/sumo/).

<table>
    <tr>
        <td><img src="figures/SanFrancisco_Model_Documentation.png/SanFrancisco_Model_Documentation-0001.png"  width="200"/></td>
        <td><img src="figures/SanFrancisco_Model_Documentation.png/SanFrancisco_Model_Documentation-0002.png"  width="200"/></td>
        <td><img src="figures/SanFrancisco_Model_Documentation.png/SanFrancisco_Model_Documentation-0003.png"  width="200"/></td>
        <td><img src="figures/SanFrancisco_Model_Documentation.png/SanFrancisco_Model_Documentation-0004.png"  width="200"/></td>
    </tr>
    <tr>
        <td><center>Case Study Overview</center></td>
        <td><center>Zone 1</center></td>
        <td><center>Zone 2</center></td>
        <td><center>Zone 3</center></td>
    </tr>
</table>

## Abstract

<table>
    <tr><td>
Perimeter control is an effective, urban traffic management strategy that regulates inflow to congested urban regions using aggregate network dynamics. While existing approaches primarily optimize system-level efficiency, such as total travel time or network throughput, they often overlook equity considerations, leading to uneven delay distributions across entry points and user groups. 

This work integrates fairness objectives into perimeter control design through explicit queue balancing mechanisms.
A large-scale, microscopic case study of the Financial District in the San Francisco urban network is used to evaluate both performance and implementation challenges. 
The results demonstrate conventional perimeter control not only reduces total and internal delays but can also improve fairness metrics, including the Gini coefficient and Jain’s fairness index. 
Building on this observation, queue balancing strategies which yield measurable fairness improvements in heterogeneous demand scenarios, where congestion is unevenly distributed across entry points. 

Ultimately, the proposed framework contributes toward equitable control design for emerging, intelligent transportation systems, and higher user acceptance for those. 
        </td></tr><tr>
        <td>
<img src="figures/Figure_Animation.gif" style="height:250px" />
<table><tr><td><img src="figures/Figure_1.png" style="height:50px" /></td></tr><tr><td><img src="figures/Figure_3.png" style="height:50px" /></td></tr></table>
        </td>
    </tr>
</table>





## What you will find in this repository

This repository contains the simulation model and source code to reproduce the findings of our study.
The folder contains following information:

```
./
├── code/
│   ├── fpc_constants.py
│   ├── fpc_library.py
│   ├── RunSimulation_control_multi_fpc.py
│   ├── RunSimulation_control_multi_region.py
│   ├── RunSimulation_control_single_region.py
│   └── RunSimulation_control_no_control.py
├── figures/
│   └── ...
├── logs/
│   ├── logs_uncontrolled/
│   ├── logs_multizones/
│   ├── logs_fcp_maxmin/
│   └── logs_fcp_prop/
└── model/
    ├── Configuration.sumocfg
    ├── SFO.net.xml
    └── ...
```

- The source code for this study can be found in folder *code/*.
- Some of the figures used in the paper can be found in folder *figures/*.
- The log files used for the analysis in this study can be foud in folder *logs/*.
- The SUMO model and all related files can be found in folder folder *model/*.



## Installation & Run Instructions
```
pip install -r requirements.txt
python code/RunSimulation_control_multi_fpc.py
```
(Please do not forget to update SUMO_PATH in the script.)

### Log Files
This will trigger to run the simulation and create log files in `../model/logs/`:
- `log_region_flow.txt` (this is a log file tha tracks flow and vehicle accumulation / number of vehicles seperately for each zone)
- `basic_summary.xml` (this is a SUMO log file including a basic summary of the whole networks flow, speed, and density)
- `basic_tripinfo.xml` (this is a SUMO log file including information about single individual trips, their delays, and travel times)
- `teleports.pkl` (a list of all vehicles that were teleported by SUMO, these need to be removed when calculating individual travel times, delays, and their distributions)
- `fpc_recorder.pkl` (a history of control variables from the fair peterimeter controller FPC based on queue balancing mechanism)
- `queue_recorder.pkl` (a history of queue lengths and loss times for all queues at all perimetral intersections of the three zones)
- `control_history.pkl` (a history of control variables from the feedback-based gating algorithm, which is the PI controller and serves as a foundation also for the FPC controller)

## Citation
If you found this repository helpful, please cite our work:
```
Kevin Riehl, Lea Künslter, Ying-Chuan Ni, Shaimaa K. El-Baklish, Anastasia Psarou, Anastasios Kouvelas, Michail A. Makridis, Rafał Kucharski
"Distributive Perimetral Queue Balancing Mechanisms: Towards Equitable Urban Traffic Gating and Fair Perimeter Control", 2026.
Submitted to CDC2026: 65th IEEE Conference on Decision and Control, Honolulu, Hawaii.
```

