"""
Distributive Perimetral Queue Balancing Mechanisms: towards Equitable Urban Traffic Gating and Fair Perimeter Control [2026]
Authors: Kevin Riehl <kriehl@ethz.ch>
Organisation: ETH Zürich, Institute for Transport Planning and Systems (IVT)
"""

###############################################################################
#### IMPORTS
###############################################################################
import traci
import pandas as pd
import numpy as np
import zipfile
import io
import ast 
import fpc_constants




###############################################################################
#### LOADERS
###############################################################################

def loadDemandModel(DEMAND_FILE):
    with zipfile.ZipFile(DEMAND_FILE, 'r') as z:
        csv_file = z.namelist()[0]  # Gets the only file in the ZIP
        with z.open(csv_file) as f:
            df = pd.read_csv(io.BytesIO(f.read()), sep="@", names=["vehicle_id", "edges", "times"])
    #df = pd.read_csv(DEMAND_FILE, sep="@", names=["vehicle_id", "edges", "times"])
    df["launchTime"] = df["times"].str.split(",").str[0].str.strip("[]").astype(float)
    df['launchTime'] = df['launchTime'].replace(-1, np.nan)
    df['launchTime'] = df['launchTime'].ffill()
    df['launchTime'] = df['launchTime'].bfill()
    df = df.sort_values(by='launchTime')
    return df




###############################################################################
#### LOGGERS
###############################################################################

def createFlowLogHead(log_activated, log_file):
    if log_activated:
        with open(log_file, "a") as file:
            file.write("time\tn_1\tn_2\tn_3\tflow_1\tflow_2\tflow_3\n")

def generateFlowLogEntry(log_activated, log_file, time_ctr, system_state, flow_information):
    if log_activated:
        with open(log_file, "a") as file:
            file.write(str(time_ctr))
            file.write("\t")
            file.write(str(len(system_state["last_vehicles_region_1"])))
            file.write("\t")
            file.write(str(len(system_state["last_vehicles_region_2"])))
            file.write("\t")
            file.write(str(len(system_state["last_vehicles_region_3"])))
            file.write("\t")
            file.write(str(flow_information["flow_region_1"]))
            file.write("\t")
            file.write(str(flow_information["flow_region_2"]))
            file.write("\t")
            file.write(str(flow_information["flow_region_3"]))
            file.write("\n")




###############################################################################
#### SUMO METHODS
###############################################################################

def determine_multi_zone_intersection():
    # Determine intersection per region
    relevant_ints1 = []
    for key in fpc_constants.actuators["region_1"]:
        if fpc_constants.actuators["region_1_relevance"][key]:
            relevant_ints1.append(key)
    relevant_ints2 = []
    for key in fpc_constants.actuators["region_2"]:
        if fpc_constants.actuators["region_2_relevance"][key]:
            relevant_ints2.append(key)
    relevant_ints3 = []
    for key in fpc_constants.actuators["region_3"]:
        if fpc_constants.actuators["region_3_relevance"][key]:
            relevant_ints3.append(key)
    # Create sets from the lists
    set1 = set(relevant_ints1)
    set2 = set(relevant_ints2)
    set3 = set(relevant_ints3)
    # Combine all sets to find unique elements across all regions
    all_elements = set1.union(set2).union(set3)
    # Create a dictionary to track the regions where each element appears
    element_regions = {}
    for element in all_elements:
        regions = []
        if element in set1:
            regions.append("region_1")
        if element in set2:
            regions.append("region_2")
        if element in set3:
            regions.append("region_3")
        # Only include elements appearing in more than one region
        if len(regions) > 1:
            element_regions[element] = regions
    # exception as this is not same
    # del element_regions["INT16"] # excluded here
    return element_regions

def spawn_vehicles(demand_model, time_ctr):
    df_spawn = demand_model[demand_model["launchTime"]==time_ctr]
    if len(df_spawn)>0:
        for idx, row in df_spawn.iterrows():
            vehicle_id = row["vehicle_id"]
            edge_str = row["edges"]
            edge_list = ast.literal_eval(edge_str)
            insert_vehicle(vehicle_id, edge_list)         

def insert_vehicle(vehicle_id, edge_list):
    traci.vehicle.add(vehicle_id, "route_0")
    traci.vehicle.setRoute(vehicle_id, edge_list)
    traci.vehicle.moveTo(vehicle_id, edge_list[0]+"_0", pos=0.0)

def determine_vehicles_in_region(edges):
    vehicle_ids = []
    for edge in edges:
        vehicle_ids += traci.edge.getLastStepVehicleIDs(edge)
    return vehicle_ids

def determine_last_step_flow(last_vehicle_ids, vehicle_ids):
    flow = 0
    for veh_id in last_vehicle_ids:
        if not veh_id in vehicle_ids:
            flow += 1
    return flow

def determine_region_flows(sensors_in, sensors_out):
    inflow_region = 0
    outflow_region = 0
    for sensor in sensors_in:
        inflow_region += traci.inductionloop.getIntervalVehicleNumber(sensor)
    for sensor in sensors_out: 
        outflow_region += traci.inductionloop.getIntervalVehicleNumber(sensor)
    return inflow_region, outflow_region

def set_traffic_light_green_time_share(region, period_duration, controller, green_share, green_share2=None):
    # Return the current traffic light progtam logic for the intersection
    ft_logic = traci.trafficlight.getAllProgramLogics(controller)[0]
    # traffic light controller might start to dynamically adjust duration on its own
    # therefor also set minDur and maxDur to lock the durationtion in
    if green_share2 is None: # CASE 1: SINGLE ZONE INTERSECTION
        for ph_id in range(0,len(ft_logic.phases)):
            ph_id_g = 2
            ph_id_r = 0
            if controller in fpc_constants.actuators["phase_shift"][region]: # it is ensured, that phase_shift only exists for intersection without multi zones
                ph_id_g = 0
                ph_id_r = 2
            if ph_id==ph_id_g: # Green light phase
                ft_logic.phases[ph_id].minDur   = int(green_share*period_duration)
                ft_logic.phases[ph_id].maxDur   = int(green_share*period_duration)
                ft_logic.phases[ph_id].duration = int(green_share*period_duration)
            elif ph_id==ph_id_r: # Red light phase
                ft_logic.phases[ph_id].minDur   = period_duration - int(green_share*period_duration)
                ft_logic.phases[ph_id].maxDur   = period_duration - int(green_share*period_duration)
                ft_logic.phases[ph_id].duration = period_duration - int(green_share*period_duration)
            else: # Yellow phases
                ft_logic.phases[ph_id].minDur   = 0
                ft_logic.phases[ph_id].maxDur   = 0
                ft_logic.phases[ph_id].duration = 0
    else: # CASE 2: MULTI ZONE INTERSECTION
        green_share_a = green_share / (green_share+green_share2)
        green_share_a = max(fpc_constants.control_params["GREEN_MIN"], min(fpc_constants.control_params["GREEN_MAX"], green_share_a))
        green_share_b = 1 - green_share_a
        ft_logic.phases[2].minDur   = int(green_share_a*period_duration)
        ft_logic.phases[2].maxDur   = int(green_share_a*period_duration)
        ft_logic.phases[2].duration = int(green_share_a*period_duration)
        ft_logic.phases[0].minDur   = int(green_share_b*period_duration)
        ft_logic.phases[0].maxDur   = int(green_share_b*period_duration)
        ft_logic.phases[0].duration = int(green_share_b*period_duration)
        ft_logic.phases[1].minDur   = 0
        ft_logic.phases[1].maxDur   = 0
        ft_logic.phases[1].duration = 0
        ft_logic.phases[3].minDur   = 0
        ft_logic.phases[3].maxDur   = 0
        ft_logic.phases[3].duration = 0
    # sets a new program for the given intersection
    traci.trafficlight.setProgramLogic(controller, ft_logic)
    # switches to the phase with the given index (0-> start with the green phase) in the list of all phases for the current program
    traci.trafficlight.setPhase(controller, 0)
    
def actuate_entering_rates_in_region(region, control_state):
    relevance = fpc_constants.actuators[region + "_relevance"]
    multi_control = fpc_constants.actuators.get("multi_control_intersection", {})
    for intersection, enabled in relevance.items():
        if not enabled:
            continue
        controller = fpc_constants.actuators[region][intersection]
        if intersection in multi_control:
            region_a, region_b = multi_control[intersection]
            # Determine if phase shift applies for region_b
            phase_shift = fpc_constants.actuators.get("phase_shift", {})
            has_phase_shift = intersection in phase_shift.get(region_b, {})
            green_share_a = control_state[region_a if has_phase_shift else region_b]["entering_rate"]
            green_share_b = control_state[region_b if has_phase_shift else region_a]["entering_rate"]
            set_traffic_light_green_time_share(
                region="",
                period_duration=fpc_constants.control_params["CONTROL_PERIOD"],
                controller=controller,
                green_share=green_share_a,
                green_share2=green_share_b
            )
        else:
            green_share = control_state[region]["entering_rate"]
            set_traffic_light_green_time_share(
                region=region,
                period_duration=fpc_constants.control_params["CONTROL_PERIOD"],
                controller=controller,
                green_share=green_share
            )

def actuate_allocative_entering_rates_in_region(region, period_duration, alloc_dict):
    region_alloc = alloc_dict[region]
    actuators = fpc_constants.actuators
    for intersection, green_share in region_alloc.items():
        # skip irrelevant intersections
        if not actuators[f"{region}_relevance"][intersection]:
            continue
        # simple (single-control) case
        if intersection not in actuators["multi_control_intersection"]:
            controller = actuators[region][intersection]
            set_traffic_light_green_time_share(
                region=region,
                period_duration=period_duration,
                controller=controller,
                green_share=green_share,
            )
            continue
        # multi-control case
        region_a, region_b = actuators["multi_control_intersection"][intersection]
        if intersection in actuators["phase_shift"][region_b]:
            green_share_a = alloc_dict[region_a][intersection]
            green_share_b = alloc_dict[region_b][intersection]
        else:
            green_share_a = alloc_dict[region_b][intersection]
            green_share_b = alloc_dict[region_a][intersection]
        controller = actuators[region][intersection]
        set_traffic_light_green_time_share(
            region=region,
            period_duration=period_duration,
            controller=controller,
            green_share=green_share_a,
            green_share2=green_share_b,
        )
                
def init_system_state():
    system_state = {}
    for region in fpc_constants.regions:
        system_state["last_vehicles_"+region] = []
    return system_state
    
def init_control_state(region):
    base = {
        "n_vehicle": 0,
        "last_n_vehicle": 0,
        "entering_rate": fpc_constants.control_params["GREEN_MAX"],
    }
    if region == "all":
        return {r: base.copy() for r in fpc_constants.regions}
    else:
        return {region: base.copy()}

def init_control_history(region):
    base = {
        "n_vehicles": [],
        "entering_rates": [[0, fpc_constants.control_params["GREEN_MAX"]]],
    }
    if region == "all":
        return {r: base.copy() for r in fpc_constants.regions}
    else:
        return {region: base.copy()}

def init_fairness_state():
    inter_greenshare = {
        "region_1": {"entering_rate" : 0},
        "region_2": {"entering_rate" : 0},
        "region_3": {"entering_rate" : 0},
    }    
    return inter_greenshare

def measure_system_state(system_state):
    flow_information = {}
    for region in fpc_constants.regions:
        vehicles_region = determine_vehicles_in_region(fpc_constants.edges[region])
        flow_information["flow_"+region] = determine_last_step_flow(system_state["last_vehicles_"+region], vehicles_region)
        system_state["last_vehicles_"+region] = vehicles_region.copy()
    return system_state, flow_information

def update_control_state(control_state, control_history, region, system_state, time_ctr):
    inflow_region, outflow_region = determine_region_flows(sensors_in=fpc_constants.sensors["entrance"][region], sensors_out=fpc_constants.sensors["exit"][region])
    control_state[region]["n_vehicle"] = control_state[region]["n_vehicle"] + inflow_region - outflow_region
    vehicles_region = system_state["last_vehicles_"+region]
    control_history[region]["entering_rates"].append([time_ctr, inflow_region, outflow_region, control_state[region]["n_vehicle"], len(vehicles_region)])
    control_state[region]["n_vehicle"] = len(vehicles_region) # take true value instead of sensor value because it is not so accurate
    return control_state, control_history

def update_control_history(control_state, control_history, region, time_ctr):
    control_state[region]["last_n_vehicle"] = control_state[region]["n_vehicle"]
    control_history[region]["n_vehicles"].append([time_ctr, control_state[region]["entering_rate"]])     
    return control_state, control_history

def measure_queue_state():
    max_queue_lengths = {}
    mean_speeds = {}
    time_losses = {}
    for region in fpc_constants.regions:
        if region not in mean_speeds:
            max_queue_lengths[region] = []
            mean_speeds[region] = []
            time_losses[region] = []
        qles = []
        spds = []
        lsss = []
        for inter in fpc_constants.sensors["queue_lenght"][region]:
            for sensor in fpc_constants.sensors["queue_lenght"][region][inter]:
                qles.append(traci.lanearea.getLastIntervalMaxJamLengthInMeters(sensor))
                spds.append(traci.lanearea.getLastIntervalMeanSpeed(sensor))
                vehnum = traci.lanearea.getLastIntervalVehicleNumber(sensor)
                occupa = traci.lanearea.getLastIntervalOccupancy(sensor)
                lsss.append(occupa * 3600 / vehnum if vehnum > 0 else 0.0)#(traci.lanearea.getLastIntervalMeanTimeLoss(sensor))
        max_queue_lengths[region].append(qles)
        mean_speeds[region].append(spds)
        time_losses[region].append(lsss)
    return max_queue_lengths, mean_speeds, time_losses
    



###############################################################################
#### DISTRIBUTIVE ALLOCATION RULES
###############################################################################

"""
For each of the folling methods:
Args:   req ([float]): requested green light percentage for each traffic light !! =< 1.0
        avail (float): allowed green light percentage
Returns:    [float] : list with allowed percentage
"""

def allocation_rule(rule, reqs, avail):
    if rule=="proportional":
        return proportional(reqs, avail)
    else: # if rule=="maxmin"
        return maxmin(reqs, avail)

def proportional(reqs, avail):
    n = len(reqs)
    # case : no resources left
    if avail <= 0:
        return [0] * n   
    total_res = avail
    #total_res = n * avail
    total_req = sum(reqs)
    # case : all requests can be fullfilled
    if total_req <= total_res:
        return reqs
    # case : proportional (to the requests) allocation
    allo = [(req / total_req) * total_res for req in reqs]
    return allo

def maxmin(reqs, avail):
    n = len(reqs)
    if avail <= 0:
        return [0.0] * n
    
    total_req = sum(reqs)
    if total_req <= avail:
        return [float(r) for r in reqs]
    
    # Step 1: Sort requests with indices (ascending)
    remains = sorted([(float(reqs[i]), i) for i in range(n)])
    allo = [0.0] * n
    total_res = float(avail)
    
    while remains:
        # Step 2a: Equal share of remaining budget
        num_remains = len(remains)
        rho = total_res / num_remains
        
        # Step 2b: Satisfy all where request <= rho
        satisfied = []
        for req, i in remains:
            if req <= rho:
                allo[i] = req  # Full request
                total_res -= req
                satisfied.append((req, i))
            else:
                # Keep remaining (those > rho)
                break
        
        # Remove satisfied ones
        remains = remains[len(satisfied):]
        
        # Step 2c: If none satisfied, give rho to all remaining and stop
        if not satisfied and remains:
            for req, i in remains:
                allo[i] = rho
            total_res = 0
            break
    
    return allo
    
"""
def maxmin(reqs, avail):
    n = len(reqs)
    allo = [0] * n
    if avail <= 0:
        return allo
    # TODO: don't forget to change it !!!
    total_res = avail
    #total_res = n*avail
    total_req = sum(reqs)
    allo = [0]*n # start with 0 res for all and then fill it up
    if total_req <= total_res:
        return reqs
    # additional index is required to map sorted list back to the junction
    remains = [(req, i) for i, req in enumerate(reqs)]
    remains.sort() # sorts the reiming requests by requests
    # now iterate as long as there is resources left 
    # always giving req resources to the ones that are below their share
    while total_res > 0:
        num_remains = len(remains)
        share = total_res / num_remains
        found = 0
        for req, i in  remains:
            if req <= share:
                allo[i] = req
                found += 1
                total_res -= req
            else:
                break        
        # iterate again
        if found > 0:
            remains = remains[found:]
        # already in this round no one smaller than their share
        else:
            for rem in remains:
                allo[rem[1]] = share
            total_res = 0
    return allo
"""

def distribution_fair(regions, control_state, inter_greenshare, al_rule):
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
        alloc_green = allocation_rule(al_rule, queue_list, total_durs)
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
    return intersection_green, intermediate_actuation, inter_greenshare

def distribution_intermediate(allowed_act, inter_greenshare, al_rule):
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
		alloc_green = allocation_rule(al_rule, queue_list, inter_greenshare[region]["entering_rate"])
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
	return intersection_green, allowed_act, inter_greenshare