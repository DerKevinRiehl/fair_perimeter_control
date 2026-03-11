"""
Distributive Perimetral Queue Balancing Mechanisms: towards Equitable Urban Traffic Gating and Fair Perimeter Control [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

# #############################################################################
# ## IMPORTS
# #############################################################################
import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import time
import pandas as pd
import fpc_library
import fpc_constants
import matplotlib.pyplot as plt




# #############################################################################
# ## PARAMETERS
# #############################################################################
#### SUMO-SPECIFIC
SUMO_BINARY = "C:/Users/kriehl/AppData/Local/sumo-1.19.0/bin/sumo-gui.exe"
        # SUMO_BINARY = "C:/Program Files (x86)/Eclipse/Sumo/bin/sumo-gui.exe"
        # SUMO_BINARY = "C:/Users/kriehl/Desktop/sumo-1.19.0/bin/sumo.exe"
SUMO_CONFIG = "../model/Configuration.sumocfg" 
SUMO_DEMAND_FILE = "../model/demand_scenarios/demand_2136_reduced.zip"
#### SIMULATION-SPECIFIC
SIMULATION_WAIT_TIME = 0.0 # 0.1
SIMULATION_DURATION = 86400 + 7000 # 0.1
#### LOGGING OPTIONS
SHOULD_LOG_FLOW = True
LOG_FILE_FLOW = "../model/logs/log_region_flow.txt"
#### CONTROL-SPECIFIC
region = "region_1"




# #############################################################################
# ## RUN SIMULATION
# #############################################################################
#### LOAD DATA
demand_model = fpc_library.loadDemandModel(SUMO_DEMAND_FILE)
system_state = fpc_library.init_system_state()
control_state = fpc_library.init_control_state(region)
control_history = fpc_library.init_control_history(region)
#### START SUMO
traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--time-to-teleport", "-1"])
fpc_library.createFlowLogHead(SHOULD_LOG_FLOW, LOG_FILE_FLOW)
for time_ctr in range(0, SIMULATION_DURATION):
    # SPAWN CARS
    fpc_library.spawn_vehicles(demand_model, time_ctr)            
    # RUN SIMULATION FOR ONE SECOND
    traci.simulationStep()
    time.sleep(SIMULATION_WAIT_TIME)
    # DETERMINE STATE (SYSTEM)
    system_state, flow_information = fpc_library.measure_system_state(system_state)
    fpc_library.generateFlowLogEntry(SHOULD_LOG_FLOW, LOG_FILE_FLOW, time_ctr, system_state, flow_information)
    # DETERMINE STATE (CONTROL)
    if (time_ctr-0) % fpc_constants.sensor_params["SENSOR_PERIOD"]==0:
        control_state, control_history = fpc_library.update_control_state(control_state, control_history, region, system_state, time_ctr)
    # EXECUTE CONTROL
    if time_ctr % fpc_constants.control_params["CONTROL_PERIOD"] == 0:
        # PI-CONTROL LAW
        unbounded_entering_rate = control_state[region]["entering_rate"] - fpc_constants.control_params[region]["K_i"] * (control_state[region]["n_vehicle"] - control_state[region]["last_n_vehicle"]) + fpc_constants.control_params[region]["K_p"] * (fpc_constants.control_params[region]["crit_n"] - control_state[region]["n_vehicle"])
        control_state[region]["entering_rate"] = max(fpc_constants.control_params["GREEN_MIN"], min(fpc_constants.control_params["GREEN_MAX"], unbounded_entering_rate))
        print(">>", time_ctr, "\tGreen Share: ", unbounded_entering_rate, control_state[region]["entering_rate"], fpc_constants.control_params[region]["crit_n"], control_state[region]["n_vehicle"])
        # UPDATE HISTORY
        control_state, control_history = fpc_library.update_control_history(control_state, control_history, region, time_ctr)
        # ACTUATE
        fpc_library.actuate_entering_rates_in_region(region, control_state)
# STOP SUMO
traci.close()




# #############################################################################
# ## VISUALIZATION
# #############################################################################
n_vehicles = pd.DataFrame(control_history[region]["entering_rates"], columns=["time", "inflow", "outflow", "n_vehicles", "n_vehicles_true"])
entering_rates = pd.DataFrame(control_history[region]["n_vehicles"], columns=["time", "green_share"])

plt.subplot(1,2,1)
plt.step(n_vehicles["time"], n_vehicles["n_vehicles"], label="measure with e1 sensors")
plt.step(n_vehicles["time"], n_vehicles["n_vehicles_true"], label="true number")
plt.axhline(y=fpc_constants.control_params[region]["crit_n"], color="black")
plt.ylim(0,150)
plt.legend()
plt.subplot(1,2,2)
plt.step(entering_rates["time"], entering_rates["green_share"])