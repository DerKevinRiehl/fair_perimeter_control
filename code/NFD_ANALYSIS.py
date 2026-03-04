import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


network_length_1 = 6549.33/1000
network_length_2 = 18983.96/1000
network_length_3 = 18514.79/1000

scenarios = ["2136", "2137", "2138", "2139", "2140", "2141", "2142", "2143", "2144", "2145"]
scenarios = ["2136", "2137", "2138", "2139", "2140", "2141", "2142", "2143", "2144", "2145"]
# scenarios = ["2136",]

edge_logs_folder = "../logs/uncontrolled/"
scenarios = ["2136", "2137", "2138", "2139", "2140", "2141", "2142", "2143", "2144", "2145"]

# edge_logs_folder = "edge_logs/control_region_1/"
# scenarios = ["2136", "2138", "2139", "2140", "2141", "2142", "2143"]

# edge_logs_folder = "edge_logs/control_region_2/"
# scenarios = ["2136", "2137", "2138", "2139", "2140", "2141", "2142", "2143", "2145"]

# edge_logs_folder = "edge_logs/control_region_3/"
# scenarios = ["2136", "2137", "2138", "2139", "2140", "2141", "2142", "2143", "2145"]

# edge_logs_folder = "edge_logs/control_region_123/"
# scenarios = ["2136", "2138", "2139", "2140", "2141", "2142", "2143", "2145"]

def loadMFDFile(file, network_length):
    f = open(file, "r")
    content = f.read()
    f.close()
    
    groups = content.split("<interval")
    groups = groups[1:]
    
    data = []
    for group in groups:
        time = float(group.split("begin=\"")[1].split("\"")[0])
        lines = group.split("</interval>")[0].split("\n")
        lines = [line.strip() for line in lines]
        lines = lines[1:-1]
        
        relevant_lines = [line for line in lines if "sampledSeconds=\"0.00\"" not in line]
        stats = [time, -1, -1, -1, -1]
        if len(relevant_lines)>0:
            speeds = [float(line.split("speed=\"")[1].split("\"")[0]) for line in relevant_lines]
            densities = [float(line.split("density=\"")[1].split("\"")[0]) for line in relevant_lines]
            flows = [float(line.split("left=\"")[1].split("\"")[0]) for line in relevant_lines]
            num_vehicles = np.mean(densities)*network_length
            stats = [time, np.mean(speeds)*3.6, np.mean(densities), np.mean(flows)/300*3600, num_vehicles]
            data.append(stats)
    
    data = pd.DataFrame(data, columns=["time", "speed", "density", "flow", "num_vehicles"])
    return data

def loadMFD(file, network_length):
    mfd_data = None
    for scenario in scenarios:
        mfd_sub = loadMFDFile(edge_logs_folder+"/scenario_"+scenario+"/"+file, network_length)
        mfd_sub["scenario"] = scenario
        if mfd_data is None:
            mfd_data = mfd_sub.copy()
        else:
            mfd_data = pd.concat([mfd_data, mfd_sub])
    return mfd_data

def loadFlows():
    flow_data = None
    for scenario in scenarios:
        print(scenario)
        df = pd.read_csv(edge_logs_folder+"/scenario_"+scenario+"/log_region_flow.txt", sep="\t")
        df['time_bin'] = (df['time'] // 300) * 300
        aggregated_df = df.groupby('time_bin')[['flow_1', 'flow_2', 'flow_3']].sum().reset_index()
        aggregated_df["scenario"] = scenario 
        if flow_data is None:
            flow_data = aggregated_df.copy()
        else:
            flow_data = pd.concat([flow_data, aggregated_df])
    return flow_data

flowz = loadFlows()

mfd_1 = loadMFD("log_region1_edges.xml", network_length_1)
mfd_2 = loadMFD("log_region2_edges.xml", network_length_2)
mfd_3 = loadMFD("log_region3_edges.xml", network_length_3)

mfd_1 = mfd_1.merge(flowz, left_on=["time", "scenario"], right_on=["time_bin", "scenario"], how="left")
mfd_1["flow_correct"] = mfd_1["flow_1"]

mfd_2 = mfd_2.merge(flowz, left_on=["time", "scenario"], right_on=["time_bin", "scenario"], how="left")
mfd_2["flow_correct"] = mfd_2["flow_2"]

mfd_3 = mfd_3.merge(flowz, left_on=["time", "scenario"], right_on=["time_bin", "scenario"], how="left")
mfd_3["flow_correct"] = mfd_3["flow_3"]


df_mdf_all = mfd_1.copy()
df_mdf_all = df_mdf_all.merge(mfd_2, on="time", how="left").merge(mfd_3, on="time", how="left")
df_mfd_all = df_mdf_all.rename(columns={'speed': "speed_z", 'density': "density_z", 'flow': "flow_z", 'num_vehicles': "num_vehicles_z", 'scenario': "scenario_z", 'time_bin': "time_bin_z", 'flow_1':"flow_1_z", 'flow_2':"flow_2_z", 'flow_3':"flow_3_z", 'flow_correct':"flow_correct_z"})
df_mfd_all["speed"] = df_mfd_all["num_vehicles_x"]*df_mfd_all["speed_x"] + df_mfd_all["num_vehicles_y"]*df_mfd_all["speed_y"] + df_mfd_all["num_vehicles_z"]*df_mfd_all["speed_z"]
df_mfd_all["speed"] = df_mfd_all["speed"] / (df_mfd_all["num_vehicles_x"] + df_mfd_all["num_vehicles_y"] + df_mfd_all["num_vehicles_z"])
df_mfd_all["num_vehicles"] = df_mfd_all["num_vehicles_x"] + df_mfd_all["num_vehicles_y"] + df_mfd_all["num_vehicles_z"]
df_mfd_all["flow_correct"] = df_mfd_all["flow_correct_x"] + df_mfd_all["flow_correct_y"] + df_mfd_all["flow_correct_z"]
df_mfd_all["density"] = df_mfd_all["density_x"] + df_mfd_all["density_y"] + df_mfd_all["density_z"]



def plot_poly_regression(x, y, ax, color='r', degree=2):
    coefficients = np.polyfit(x, y, degree)
    poly = np.poly1d(coefficients)
    x_smooth = np.linspace(min(x), max(x), 200)
    plt.plot(x_smooth, poly(x_smooth), color='red', label='Fitted curve', linestyle="--")


relevant_x_label = "num_vehicles"

OBSERVATION_LIMIT = 30000
OBSERVATION_LIMIT = 100000
plt.figure()
plt.suptitle("MFD Estimation "+edge_logs_folder)
plt.subplot(2,4,1)
plt.title("Region 1")
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Flow [veh/h]")
plt.scatter(mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT]["flow_correct"], alpha=0.2)
plot_poly_regression(mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT]["flow_correct"], plt.gca(), degree=2)
# plt.xlim(0,40000)
# plt.xlim(0,120)
# plt.ylim(0,300)
plt.subplot(2,4,2)
plt.title("Region 2")
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Flow [veh/h]")
plt.scatter(mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT]["flow_correct"], alpha=0.2)
plot_poly_regression(mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT]["flow_correct"], plt.gca(), degree=2)
# plt.xlim(0,OBSERVATION_LIMIT0)
plt.subplot(2,4,3)
plt.title("Region 3")
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Flow [veh/h]")
plt.scatter(mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT]["flow_correct"], alpha=0.2)
plot_poly_regression(mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT]["flow_correct"], plt.gca(), degree=2)
# plt.xlim(0,OBSERVATION_LIMIT0)
plt.subplot(2,4,4)
plt.title("Region ALL")
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Flow [veh/h]")
plt.scatter(df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT][relevant_x_label], df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT]["flow_correct"], alpha=0.2)
plot_poly_regression(df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT][relevant_x_label], df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT]["flow_correct"], plt.gca(), degree=2)
# plt.xlim(0,OBSERVATION_LIMIT0)

plt.subplot(2,4,1+4)
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Speed [km/h]")
plt.scatter(mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_1[mfd_1["time"]<=OBSERVATION_LIMIT]["speed"], alpha=0.2)
# plot_poly_regression(mfd_1["num_vehicles"], mfd_1["speed"], plt.gca(), degree=1)
plt.subplot(2,4,2+4)
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Speed [km/h]")
plt.scatter(mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_2[mfd_2["time"]<=OBSERVATION_LIMIT]["speed"], alpha=0.2)
# plot_poly_regression(mfd_2["num_vehicles"], mfd_2["speed"], plt.gca(), degree=1)
plt.subplot(2,4,3+4)
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Speed [km/h]")
plt.scatter(mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT][relevant_x_label], mfd_3[mfd_3["time"]<=OBSERVATION_LIMIT]["speed"], alpha=0.2)
# plot_poly_regression(mfd_3["num_vehicles"], mfd_3["speed"], plt.gca(), degree=1)
plt.subplot(2,4,4+4)
# plt.xlabel("Density [veh/km]")
plt.xlabel("Number Of Vehicles [veh]")
plt.ylabel("Speed [km/h]")
plt.scatter(df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT][relevant_x_label], df_mfd_all[df_mfd_all["time"]<=OBSERVATION_LIMIT]["speed"], alpha=0.2)
# plot_poly_regression(mfd_3["num_vehicles"], mfd_3["speed"], plt.gca(), degree=1)

plt.tight_layout()


"""
plt.figure()
df_aggregated = df_mfd_all.groupby('time').mean().reset_index()
# plt.plot(df_aggregated["time"], df_aggregated["num_vehicles"])
# plt.ylim(0,600)
plt.plot(df_aggregated["time"], df_aggregated["flow_correct"])
plt.ylim(0,1600)
"""




"""
plt.figure()
mfd_1_agg = mfd_1.groupby('time')[['speed', 'density', 'flow_correct']].mean().reset_index()
mfd_2_agg = mfd_2.groupby('time')[['speed', 'density', 'flow_correct']].mean().reset_index()
mfd_3_agg = mfd_3.groupby('time')[['speed', 'density', 'flow_correct']].mean().reset_index()
plt.suptitle("Traffic States Over Time")
plt.subplot(3,4,1)
plt.title("Density [veh/km]")
plt.ylim(0,25)
plt.ylabel("Region 1")
for scenario in scenarios:
    plt.plot(mfd_1[mfd_1["scenario"]==scenario]["time"], mfd_1[mfd_1["scenario"]==scenario]["density"].tolist(), alpha=0.2)
plt.plot(mfd_1_agg["time"], mfd_1_agg["density"], "--", color="black")
plt.subplot(3,4,1+4)
plt.ylabel("Region 2")
plt.title("Density [veh/km]")
plt.ylim(0,25)
for scenario in scenarios:
    plt.plot(mfd_2[mfd_2["scenario"]==scenario]["time"], mfd_2[mfd_2["scenario"]==scenario]["density"], alpha=0.2)
plt.plot(mfd_2_agg["time"], mfd_2_agg["density"], "--", color="black")
plt.subplot(3,4,1+4*2)
plt.ylabel("Region 3")
plt.title("Density [veh/km]")
plt.ylim(0,25)
for scenario in scenarios:
    plt.plot(mfd_3[mfd_3["scenario"]==scenario]["time"], mfd_3[mfd_3["scenario"]==scenario]["density"], alpha=0.2)
plt.plot(mfd_3_agg["time"], mfd_3_agg["density"], "--", color="black")
plt.subplot(3,4,2)
plt.title("Flow [veh/h]")
plt.ylim(0,1500)
for scenario in scenarios:
    plt.plot(mfd_1[mfd_1["scenario"]==scenario]["time"], mfd_1[mfd_1["scenario"]==scenario]["flow_correct"], alpha=0.2)
plt.plot(mfd_1_agg["time"], mfd_1_agg["flow_correct"], "--", color="black")
plt.subplot(3,4,2+4)
plt.title("Flow [veh/h]")
plt.ylim(0,1500)
for scenario in scenarios:
    plt.plot(mfd_2[mfd_2["scenario"]==scenario]["time"], mfd_2[mfd_2["scenario"]==scenario]["flow_correct"], alpha=0.2)
plt.plot(mfd_2_agg["time"], mfd_2_agg["flow_correct"], "--", color="black")
plt.subplot(3,4,2+4*2)
plt.title("Flow [veh/h]")
plt.ylim(0,1500)
for scenario in scenarios:
    plt.plot(mfd_3[mfd_3["scenario"]==scenario]["time"], mfd_3[mfd_3["scenario"]==scenario]["flow_correct"], alpha=0.2)
plt.plot(mfd_3_agg["time"], mfd_3_agg["flow_correct"], "--", color="black")
plt.subplot(3,4,3)
plt.title("Speed [km/h]")
plt.ylim(0,50)
for scenario in scenarios:
    plt.plot(mfd_1[mfd_1["scenario"]==scenario]["time"], mfd_1[mfd_1["scenario"]==scenario]["speed"], alpha=0.2)
plt.plot(mfd_1_agg["time"], mfd_1_agg["speed"], "--", color="black")
plt.subplot(3,4,3+4)
plt.title("Speed [km/h]")
plt.ylim(0,50)
for scenario in scenarios:
    plt.plot(mfd_2[mfd_2["scenario"]==scenario]["time"], mfd_2[mfd_2["scenario"]==scenario]["speed"], alpha=0.2)
plt.plot(mfd_2_agg["time"], mfd_2_agg["speed"], "--", color="black")
plt.subplot(3,4,3+4*2)
plt.title("Speed [km/h]")
plt.ylim(0,50)
for scenario in scenarios:
    plt.plot(mfd_3[mfd_3["scenario"]==scenario]["time"], mfd_3[mfd_3["scenario"]==scenario]["speed"], label=scenario)#, alpha=0.2)
plt.plot(mfd_3_agg["time"], mfd_3_agg["speed"], "--", color="black")
plt.legend()

plt.subplot(3,4,4)
plt.ylim(0,4000)
plt.title("Traffic Demand / Trips Launched Every 5 Minutes")
demand = []
for scenario in scenarios:
    df_demand = pd.read_csv("demand_scenarios/demand_"+scenario+"_reduced.xml", sep="@", names=["vehid", "route", "times"], header=None)
    df_demand["spawn_time"] = df_demand["times"].str.split(",").str[0].str.strip("[]").astype(float)
    df_demand['spawn_time'] = df_demand['spawn_time'].replace(-1, np.nan)
    df_demand['spawn_time'] = df_demand['spawn_time'].ffill()
    df_demand['spawn_time'] = df_demand['spawn_time'].bfill()
    df_demand = df_demand.sort_values(by="spawn_time", ascending=True)
    df_demand = df_demand[["spawn_time"]]
    df_demand['time_bin'] = df_demand['spawn_time'] // 300
    row_counts = df_demand.groupby('time_bin').size().reset_index(name='count')
    row_counts['time_range'] = row_counts['time_bin'].apply(lambda x: x*300)
    plt.plot(row_counts["time_range"], row_counts["count"], alpha=0.2)
    demand.append(row_counts["count"].tolist())
min_n = np.min([len(d) for d in demand])
demand = [d[0:min_n] for d in demand]
demand = np.asarray(demand)
demand = np.mean(demand, axis=0)
plt.plot(row_counts["time_range"].tolist()[0:min_n], demand, "--", color="black")

plt.tight_layout()
"""