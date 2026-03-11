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
import pickle
import fpc_library
import fpc_constants




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
SIMULATION_DURATION = 3600#86400 + 7000 # 0.1
#### LOGGING OPTIONS
SHOULD_LOG_FLOW = True
LOG_FILE_FLOW = "../model/logs/log_region_flow.txt"
LOG_FILE_QUEUE_STATS = "../model/logs/queue_recorder.pkl"
LOG_FILE_SUMMARY = "../model/logs/basic_summary.xml"
LOG_FILE_TRIP = "../model/logs/basic_tripinfo.xml"
LOG_FILE_TELEP = "../model/logs/teleports.pkl"




# #############################################################################
# ## RUN SIMULATION
# #############################################################################
#### LOAD DATA
demand_model = fpc_library.loadDemandModel(SUMO_DEMAND_FILE)
system_state = fpc_library.init_system_state()
queue_recording = []
teleporting_ids = []
#### START SUMO
traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--time-to-teleport", "-1",  "--tripinfo-output", LOG_FILE_TRIP, "--summary-output", LOG_FILE_SUMMARY])
fpc_library.createFlowLogHead(SHOULD_LOG_FLOW, LOG_FILE_FLOW)
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
    fpc_library.generateFlowLogEntry(SHOULD_LOG_FLOW, LOG_FILE_FLOW, time_ctr, system_state, flow_information)
    # TELEPORT LOGGING
    for id in traci.simulation.getEndingTeleportIDList():
        teleporting_ids.append(id)
    # QUEUE RECORDING
    if (time_ctr-0) % fpc_constants.sensor_params["QUEUE_SENSING"] == 0:
        max_queue_lengths, mean_speeds, time_losses = fpc_library.measure_queue_state()
        queue_recording.append([time_ctr, mean_speeds, max_queue_lengths, time_losses])
# STOP SUMO
traci.close()

### STORE LOG FILES
with open(LOG_FILE_TELEP, "wb") as tele_f:
    pickle.dump(teleporting_ids, tele_f)
with open(LOG_FILE_QUEUE_STATS, "wb") as rec_f:
    pickle.dump(queue_recording, rec_f)
