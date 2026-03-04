# #############################################################################
# ## IMPORTS
# #############################################################################
import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import time
import fpc_library




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
SHOULD_LOG_FLOW = False
FLOW_LOG_FILE = "../model/logs/log_region_flow.txt"




# #############################################################################
# ## RUN SIMULATION
# #############################################################################
#### LOAD DATA
demand_model = fpc_library.loadDemandModel(SUMO_DEMAND_FILE)
system_state = fpc_library.init_system_state()
#### START SUMO
traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--time-to-teleport", "-1"])
fpc_library.createFlowLogHead(SHOULD_LOG_FLOW, FLOW_LOG_FILE)
for time_ctr in range(0, SIMULATION_DURATION):
    # PRINT TO CONSOLE
    if time_ctr%100==0:
        print(time_ctr)
    # SPAWN CARS          
    fpc_library.spawn_vehicles(demand_model, time_ctr)
    # RUN SIMULATION FOR ONE SECOND
    traci.simulationStep()
    time.sleep(SIMULATION_WAIT_TIME)
    # DETERMINE STATE (FLOW)
    system_state, flow_information = fpc_library.measure_system_state(system_state)
    fpc_library.generateFlowLogEntry(SHOULD_LOG_FLOW, FLOW_LOG_FILE, time_ctr, system_state, flow_information)
            
# STOP SUMO
traci.close()
