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