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
SHOULD_LOG_FLOW = False
FLOW_LOG_FILE = "../model/logs/log_region_flow.txt"
SUMMARY_FILE = "../model/logs/basic_summary.xml"
TRIP_FILE = "../model/logs/basic_tripinfo.xml"
TELEPORT_FILE = "../model/logs/prop_tele.pkl"
RECORDER_FILE = "../model/logs/prop_recorder.pkl"
#### CONTROLLER-SPECIFIC
allocation_rule = "proportional"




# #############################################################################
# ## METHODS
# #############################################################################

def fairDistribution(regions):
    # dict to keep data for intermediate actuation
    # basically just keeps track of how much greetimeshare is still allowed
    intermediate_actuation = {}
    intersection_green = {} # assigned greenshare per intersection in region
    for region in regions:       
        intermediate_act_region = {}
        alloc_dict = {}
        # if max greenshare -> assign all at beginning
        if control_state[region]["entering_rate"] == fpc_constants.control_params["GREEN_MAX"]:
            for intersection in fpc_constants.actuators[f"{region}_relevance"]:
                alloc_dict[intersection] = fpc_constants.control_params["GREEN_MAX"]
                intermediate_act_region[intersection] = {'free': 0}
            
            inter_greenshare[region]["entering_rate"] = 0
            intersection_green[region] = alloc_dict
            intermediate_actuation[region] = {}
            continue
        rel_inter = []
        for intersection in fpc_constants.actuators[f"{region}_relevance"]:
            # if multicontrolintersection -> max
            if fpc_constants.actuators[f"{region}_relevance"][intersection] and intersection not in fpc_constants.actuators["multi_control_intersection"]:
                rel_inter.append(intersection)
            elif fpc_constants.actuators[f"{region}_relevance"][intersection] and intersection in fpc_constants.actuators["multi_control_intersection"]:
                alloc_dict[intersection] = control_state[region]["entering_rate"]
            queue_list = []
            inter_list = []
            n_queues = 0 # number queues in region
            lines_intersection = {} #number of queues at an intersection
        for intersection in rel_inter:
            # number of queues at intersection rel_inter
            inter_queues = len(fpc_constants.sensors['queue_length'][region][intersection])
            n_queues += inter_queues
            lines_intersection[intersection] = inter_queues
            # determine max lenght at itersection -> all traffic lines at this intersection request the maximum
            max_length = 0
            for i in range(inter_queues):
                inter_list.append(intersection)
                det_id = fpc_constants.sensors['queue_length'][region][intersection][i]
                queue_length = traci.lanearea.getLastStepHaltingNumber(det_id)
                # if whole detector is occupied -> request all
                if queue_length >= traci.lanearea.getLength(det_id) / 7.0: # car length is 5 but include distance between them
                    queue_length = 26
                max_length = max(max_length, queue_length)
            # adds intersection max_length times to
            queue_list += [fpc_constants.queue_perc_map[max_length]]*lines_intersection[intersection]
        total_durs = n_queues * control_state[region]["entering_rate"]
        alloc_green = fpc_library.allocation_rule(allocation_rule, queue_list, total_durs)
        total_alloc = sum(alloc_green)
        for i, intersection in enumerate(inter_list):
            if intersection not in alloc_dict:
                green_share = alloc_green[i]
                alloc_dict[intersection] = green_share
                # checks if intermediate activation is possible
                if intersection not in fpc_constants.actuators["multi_control_intersection"]:
                    intermediate_act_region[intersection] = {'free': fpc_constants.control_params["GREEN_MAX"] - green_share}
        # saves greenshares and leftover greenshare
        intersection_green[region] = alloc_dict
        #inter_greenshare[region]["entering_rate"] = (total_durs-total_alloc)/n_queues
        inter_greenshare[region]["entering_rate"] = total_durs-total_alloc
        intermediate_actuation[region] = intermediate_act_region
    return intersection_green, intermediate_actuation        

def intermediateDistribution(allowed_act):
	intersection_green = {}
	for region in allowed_act:
		if inter_greenshare[region]["entering_rate"] <= 0: # region does not have greenshare left
			continue
		n_queues = 0
		queue_list = []
		inter_list = []
		lines_intersection = {}
		alloc_dict = {}
		for intersection in  allowed_act[region]:
			#print(allowed_act)
			if allowed_act[region][intersection]['free'] <= 0: # no greeshare left for this intersection
				continue
			#determine which phase is red:
			''' if 45 sec checks -> no light shcoud be in greenphase anymore :)
			if intersection in actuators["phase_shift"][region]:
				r = 2
			else:
				r = 0
			#print(traci.trafficlight.getPhase(actuators[region][intersection]) == r)
			if traci.trafficlight.getPhase(actuators[region][intersection]) != r: # traffic light currently in greenphase
				continue
            '''
			inter_queues = len(fpc_constants.sensors['queue_length'][region][intersection])
			n_queues += inter_queues
			lines_intersection[intersection] = inter_queues
			max_length = 0
			for i in range(inter_queues):
				inter_list.append(intersection)
				det_id = fpc_constants.sensors['queue_length'][region][intersection][i]
				queue_length = traci.lanearea.getLastStepHaltingNumber(det_id)
				# if whole detector is occupied -> request all
				if queue_length >= traci.lanearea.getLength(det_id) / 7.0: # car length is 5 but include distance between them
					queue_length = 26
				max_length = max(max_length, queue_length)
			req_perc = min(fpc_constants.queue_perc_map[max_length], allowed_act[region][intersection]['free']) # adjusts requested to maximum allowed
			queue_list += [req_perc]*lines_intersection[intersection]
		alloc_green = fpc_library.allocation_rule(allocation_rule, queue_list, inter_greenshare[region]["entering_rate"])
		total_alloc = sum(alloc_green)
		for i, intersection in enumerate(inter_list):
			if intersection not in alloc_dict:
				green_share = alloc_green[i]
				alloc_dict[intersection] = green_share
				# checks if intermediate activation is possible
				if intersection not in fpc_constants.actuators["multi_control_intersection"]:
					allowed_act[region][intersection]['free'] -= green_share
		intersection_green[region] = alloc_dict
		inter_greenshare[region]["entering_rate"] -= total_alloc
	return intersection_green, allowed_act
            
def fairActuate(region, period_duration, alloc_dict):
    for intersection in alloc_dict[region]:
        if fpc_constants.actuators[f"{region}_relevance"][intersection]:
            if not intersection in fpc_constants.actuators["multi_control_intersection"]:
                green_share = alloc_dict[region][intersection]
                fpc_library.set_traffic_light_green_time_share(
                    region=region, 
                    period_duration=period_duration, 
                    controller=fpc_constants.actuators[region][intersection], 
                    green_share=green_share, 
                )
            else:
                region_a = fpc_constants.actuators["multi_control_intersection"][intersection][0]
                region_b = fpc_constants.actuators["multi_control_intersection"][intersection][1]
                if intersection in fpc_constants.actuators["phase_shift"][region_b]:
                    green_share_a = alloc_dict[region_a][intersection]
                    green_share_b = alloc_dict[region_b][intersection]
                else:
                    green_share_a = alloc_dict[region_b][intersection]
                    green_share_b = alloc_dict[region_a][intersection]
                fpc_library.set_traffic_light_green_time_share(
                    region=region, 
                    period_duration=period_duration, 
                    controller=fpc_constants.actuators[region][intersection], 
                    green_share=green_share_a, 
                    green_share2=green_share_b
                )
                
                
                

# #############################################################################
# ## RUN SIMULATION
# #############################################################################
#### LOAD DATA
demand_model = fpc_library.loadDemandModel(SUMO_DEMAND_FILE)
system_state = fpc_library.init_system_state()
control_state = fpc_library.init_control_state("all")
control_history = fpc_library.init_control_history("all")
fpc_constants.actuators["multi_control_intersection"] = fpc_library.determine_multi_zone_intersection()

recorder_log = []
inter_greenshare = {
    "region_1": {"entering_rate" : 0},
    "region_2": {"entering_rate" : 0},
    "region_3": {"entering_rate" : 0},
}
intermediate_act = {}
teleporting_ids = []

#### START SUMO
traci.start([SUMO_BINARY, "-c", SUMO_CONFIG, "--start", "--quit-on-end", "--time-to-teleport", "-1",  "--tripinfo-output", TRIP_FILE, "--summary-output", SUMMARY_FILE])
fpc_library.createFlowLogHead(SHOULD_LOG_FLOW, FLOW_LOG_FILE)
for time_ctr in range(0, SIMULATION_DURATION):
    # SPAWN CARS
    fpc_library.spawn_vehicles(demand_model, time_ctr)               
    # RUN SIMULATION FOR ONE SECOND
    traci.simulationStep()
    time.sleep(SIMULATION_WAIT_TIME)
    # DETERMINE STATE (SYSTEM)
    system_state, flow_information = fpc_library.measure_system_state(system_state)
    fpc_library.generateFlowLogEntry(SHOULD_LOG_FLOW, FLOW_LOG_FILE, time_ctr, system_state, flow_information)
    # EXECUTE CONTROL
    if (time_ctr-0) % fpc_constants.control_params["CONTROL_PERIOD"] == 0:
        for region in fpc_constants.regions:
            # PI-CONTROL LAW
            unbounded_entering_rate = control_state[region]["entering_rate"] - fpc_constants.control_params[region]["K_i"] * (control_state[region]["n_vehicle"] - control_state[region]["last_n_vehicle"]) + fpc_constants.control_params[region]["K_p"] * (fpc_constants.control_params[region]["crit_n"] - control_state[region]["n_vehicle"])
            control_state[region]["entering_rate"] = max(fpc_constants.control_params["GREEN_MIN"], min(fpc_constants.control_params["GREEN_MAX"], unbounded_entering_rate))
            # UPDATE HISTORY
            control_state, control_history = fpc_library.update_control_history(control_state, control_history, region, time_ctr)
            # ACTUATE
        fair_act, intermediate_act = fairDistribution(fpc_constants.regions)
        for region in fpc_constants.regions:
            fairActuate(region, fpc_constants.control_params["CONTROL_PERIOD"], fair_act)
        recorder_log.append(control_state)                    
        print(">>", time_ctr, "\t", end="")
        for region in fpc_constants.regions:
            print(control_state[region]["entering_rate"], "\t", end="")
        print("")
    elif (time_ctr-0) % fpc_constants.control_params["CONTROL_PERIOD"] == 45 and intermediate_act: 
        fair_act, intermediate_act = intermediateDistribution(intermediate_act)
        for region in fair_act:
            fairActuate(region, fpc_constants.control_params["CONTROL_PERIOD"], fair_act)
    for id in traci.simulation.getEndingTeleportIDList():
        teleporting_ids.append(id)
    # STOP SUMO
traci.close()

with open(TELEPORT_FILE, "wb") as tele_f:
    pickle.dump(teleporting_ids, tele_f)
    
with open(RECORDER_FILE, "wb") as rec_f:
    pickle.dump(recorder_log, rec_f)
