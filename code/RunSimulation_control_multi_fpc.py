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
#### SUMO PARAMETERS
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
LOG_FILE_SUMMARY = "../model/logs/basic_summary.xml"
LOG_FILE_TRIP = "../model/logs/basic_tripinfo.xml"
LOG_FILE_TELEP = "../model/logs/teleports.pkl"
LOG_FILE_RECORD = "../model/logs/fpc_recorder.pkl"
LOG_FILE_QUEUE_STATS = "../model/logs/queue_recorder.pkl"
LOG_FILE_CONTR_HIST = "../model/logs/control_history.pkl"
#### CONTROLLER-SPECIFIC
allocation_rule = "proportional"
     
                
     

# #############################################################################
# ## RUN SIMULATION
# #############################################################################
#### LOAD DATA
demand_model = fpc_library.loadDemandModel(SUMO_DEMAND_FILE)
system_state = fpc_library.init_system_state()
control_state = fpc_library.init_control_state("all")
control_history = fpc_library.init_control_history("all")
fairness_state = fpc_library.init_fairness_state()
queue_recording = []
recorder_log = []
intermediate_act = {}
teleporting_ids = []
fpc_constants.actuators["multi_control_intersection"] = fpc_library.determine_multi_zone_intersection()

#### START SUMO
traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--time-to-teleport", "-1",  "--tripinfo-output", LOG_FILE_TRIP, "--summary-output", LOG_FILE_SUMMARY])
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
    # EXECUTE CONTROL
    if (time_ctr-0) % fpc_constants.control_params["CONTROL_PERIOD"] == 0:
        for region in fpc_constants.regions:
            # PI-CONTROL LAW
            unbounded_entering_rate = control_state[region]["entering_rate"] - fpc_constants.control_params[region]["K_i"] * (control_state[region]["n_vehicle"] - control_state[region]["last_n_vehicle"]) + fpc_constants.control_params[region]["K_p"] * (fpc_constants.control_params[region]["crit_n"] - control_state[region]["n_vehicle"])
            control_state[region]["entering_rate"] = max(fpc_constants.control_params["GREEN_MIN"], min(fpc_constants.control_params["GREEN_MAX"], unbounded_entering_rate))
            # UPDATE HISTORY
            control_state, control_history = fpc_library.update_control_history(control_state, control_history, region, time_ctr)
            # ACTUATE
        fair_act, intermediate_act, fairness_state = fpc_library.distribution_fair(fpc_constants.regions, control_state, fairness_state, allocation_rule)
        for region in fpc_constants.regions:
            fpc_library.actuate_allocative_entering_rates_in_region(region, fpc_constants.control_params["CONTROL_PERIOD"], fair_act)
        recorder_log.append(control_state)                    
        print(">>", time_ctr, "\t", end="")
        for region in fpc_constants.regions:
            print(control_state[region]["entering_rate"], "\t", end="")
        print("")
    elif (time_ctr-0) % fpc_constants.control_params["CONTROL_PERIOD"] == 45 and intermediate_act: 
        fair_act, intermediate_act, fairness_state = fpc_library.distribution_intermediate(intermediate_act, fairness_state, allocation_rule)
        for region in fair_act:
            fpc_library.actuate_allocative_entering_rates_in_region(region, fpc_constants.control_params["CONTROL_PERIOD"], fair_act)
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
with open(LOG_FILE_RECORD, "wb") as rec_f:
    pickle.dump(recorder_log, rec_f)
with open(LOG_FILE_QUEUE_STATS, "wb") as rec_f:
    pickle.dump(queue_recording, rec_f)
with open(LOG_FILE_CONTR_HIST, "wb") as ctr_f:
    pickle.dump(control_history, ctr_f)
