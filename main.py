# built-in
import os
import sys
import copy
import pickle
import logging
import argparse
import numpy as np
import gurobipy as gp
from gurobipy import *
from tabulate import tabulate

# my own
from planner import *
from visualization import visualization

# initialize the logger
logger = logging.getLogger("Multi objective mod system")
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}

def create_data_model(func, num_veh, pudo, penalty, alpha=1, beta=0, gamma=0):
    """Stores the data for the problem."""
    logger.info("Problem formulation with alpha %s, beta %s, gamma %s", alpha, beta, gamma)

    # planner
    planner = func(alpha, beta, gamma)

    # data
    data = {}

    # pick-up and drop-off
    data["pickups_deliveries"] = pudo   # raw pick-up and drop-off pairs
    data["user_id_pudo"] = {}           # user id -> (pick-up location, drop-off location)
    data["user_id_pu"] = {}             # user id -> pick-up location
    data["user_id_do"] = {}             # user id -> drop-off location
    data["pu_user_id"] = {}             # pick-up location -> user id
    data["do_user_id"] = {}             # drop-off location -> user id
    data["index_id_stop"] = {}          # index -> (user id, stop)
    idx = 1
    for id, pu, do in pudo:
        data["user_id_pudo"][id] = (pu, do)
        data["user_id_pu"][id] = pu
        data["index_id_stop"][idx] = (id, pu)
        idx += 1
        data["user_id_do"][id] = do
        data["index_id_stop"][idx] = (id, do)
        idx += 1
        data["pu_user_id"][pu] = id
        data["do_user_id"][do] = id
    data["id_stop_index"] = {idpu: idx for idx, idpu in data["index_id_stop"].items()}

    # others
    data["num_vehicles"] = num_veh
    data["penalty"] = penalty
    data["num_stops"] = len(data["pickups_deliveries"]) * 2
    data["stops"] = list(np.array(data["pickups_deliveries"])[:, 1:].reshape((data["num_stops"],)))
    data["locations"] = set(data["stops"])
    data["num_locations"] = len(data["locations"])
    data["cost"] = np.zeros((data["num_locations"], data["num_locations"]))

    data["index_stop"] = {}  # index -> stop
    data["stop_index"] = {}  # stop -> index
    for idx, loc in enumerate(data["locations"]):
        data["index_stop"][idx+1] = loc
        data["stop_index"][loc] = idx+1

    # calculate cost
    logger.info("Start calculating cost")
    for i in range(data["num_locations"]):
        for j in range(data["num_locations"]):
            planner.init()
            data["cost"][i][j] = planner.astar(data["index_stop"][i+1], data["index_stop"][j+1])
    logger.info("Complete cost calculation")

    # set depot (artificial location)
    depot = 0
    data["num_stops"] = data["num_stops"] + 1
    data["num_locations"] = data["num_locations"] + 1
    data["locations"].add(depot)
    data["index_stop"][0] = depot
    data["stop_index"][depot] = 0
    data["index_id_stop"][0] = (0, 0)
    data["id_stop_index"][(0, 0)] = 0

    # arch's cost for a depot
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.vstack((c_0, data["cost"]))
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.concatenate((c_0.T, data["cost"]), axis=1)

    return data, planner

def optimize(data, working_time=None, capacity=None):
    # define and initialize the optimal model
    m = gp.Model()
    m.Params.outputFlag = False

    num_v = data["num_vehicles"]
    n = data["num_stops"]
    users = list(data["user_id_pudo"].keys())

    # re-definite distance matrix
    dist = {}
    dist_c = {}
    for i in range(n):
        for j in range(n):
            for v in range(num_v):
                if (i != j):
                    dist[(i, j, v)] = data["cost"][data["stop_index"][data["index_id_stop"][i][1]]][data["stop_index"][data["index_id_stop"][j][1]]]
                    dist_c[(i, j, v)] = 0

    # prize and penalty
    p = {}
    p_c = {}
    for i in range(n):
        for v in range(num_v):
            p[(i, v)] = -1
        p_c[i] = penalty
    p_c[0] = 0

    # edges
    e_vars = m.addVars(dist_c.keys(), obj=dist_c, vtype=GRB.BINARY, name="e")

    # passengers
    p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")
    # penalty
    pc_vars = m.addVars(p_c.keys(), obj=p_c, vtype=GRB.BINARY, name="pc")
    # sequences
    s_vars = m.addVars(np.arange(1, data["num_stops"] + 1), lb=1, ub=data["num_stops"], vtype=GRB.INTEGER, name="seq")

    # Constraint 1: only one vehicle can visit one stop except for the depot.
    cons1 = m.addConstrs(p_vars.sum(i, "*") <= 1 for i in range(n) if i != 0)
    # Constraint 2: visited node i must have an outgoing edge.
    cons2 = m.addConstrs(e_vars.sum(i, "*", v) == p_vars[(i, v)] for i in range(n) for v in range(num_v))
    # Constraint 3: visited node j must have an ingoing edge.
    cons3 = m.addConstrs(e_vars.sum("*", j, v) == p_vars[(j, v)] for j in range(n) for v in range(num_v))
    # Constraint 4: considering the origin.
    cons4 = m.addConstr(p_vars.sum(0, "*") == num_v)
    # Constraint 5: working time limit, capacity limit, penalty
    if working_time:
        cons5 = m.addConstrs(gp.quicksum(e_vars[i, j, v] * dist[(i, j, v)]
                                         for i in range(n) for j in range(n) if i != j) <= working_time
                             for v in range(num_v))
    if capacity:
        cons6 = m.addConstrs(gp.quicksum(p_vars[data["id_stop_index"][(user, data["user_id_pu"][user])] , v] for user in users) <= capacity for v in range(num_v))
    # Constraint 6: penalty
    cons7 = m.addConstrs(1 - p_vars.sum(i, "*") == pc_vars[i] for i in range(n) if i != 0)
    # Constraint 7: pickup-dropoff pairs
    cons8 = m.addConstrs(p_vars[data["id_stop_index"][(user, data["user_id_pu"][user])], v] == p_vars[data["id_stop_index"][(user, data["user_id_do"][user])], v]
                         for user in users for v in range(num_v))
    # Constraint 8: sequences
    cons9_1 = m.addConstrs(
        s_vars[i] <= s_vars[j] + data["num_stops"] * (1 - e_vars[(i, j, k)]) - 1 for i, j, k in e_vars.keys()
        if i != 0 and j != 0)
    cons9_2 = m.addConstrs(s_vars[data["id_stop_index"][(user, data["user_id_pu"][user])]] + 1 <=
                           s_vars[data["id_stop_index"][(user, data["user_id_do"][user])]] for user in users)

    def subtourlim(model, where):
        if where == GRB.Callback.MIPSOL:
            # make a list of edges selected in the solution
            vals = model.cbGetSolution(model._vars)
            selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)
            # find the shortest cycle in the selected edge list
            tour = subtour(selected)
            for v in range(num_v):
                if tour[v]:
                    for tv in tour[v]:
                        if len(tv) < n:
                            # add subtour elimination constraint for every pair of cities in tour
                            model.cbLazy(gp.quicksum(model._vars[i, j, v] for i, j in itertools.permutations(tv, 2))
                                         <= len(tv) - 1)

    def subtour(edges, exclude_depot=True):
        cycle = [[] for v in range(num_v)]

        for v in range(num_v):
            unvisited = list(np.arange(0, n))

            while unvisited:  # true if list is non-empty
                this_cycle = []
                neighbors = unvisited

                while neighbors:
                    current = neighbors[0]
                    this_cycle.append(current)
                    unvisited.remove(current)
                    neighbors = [j for i, j, k in edges.select(current, '*', '*') if (j in unvisited) and (k == v)]

                if len(this_cycle) > 1:
                    if exclude_depot:
                        if not (0 in this_cycle):
                            cycle[v].append(this_cycle)
        return cycle

    # optimize model
    m._vars = e_vars
    m._dvars = p_vars
    m._ddvars = pc_vars
    m._svars = s_vars
    m.Params.lazyConstraints = 1
    m.optimize(subtourlim)

    # status
    logger.info("Solved (%s)", status_dict[m.status])

    if m.status != 2:
        sys.exit("There is no solution. Check constraints again.")

    e_vals = m.getAttr('x', e_vars)
    s_vals = m.getAttr('x', s_vars)

    # get solutions
    sol = {}
    for car in range(num_v):
        sol[car] = {}
        for i, j, k in e_vals.keys():
            if (e_vals[i, j, k] > 0.5) and (k == car):
                sol[k][i] = j

    routes = []
    capacities = []
    passengers = []
    travel_times = []
    pickups = [data["id_stop_index"][(id, pu)] for id, pu in data["user_id_pu"].items()]
    for car in range(num_v):
        route = sol[car]
        station = 0
        travel_time = 0
        cap = 0
        path = []
        users = []
        while True:
            station_ = copy.copy(station)
            station = route[station]
            travel_time += data["cost"][data["stop_index"][data["index_id_stop"][station_][1]]][data["stop_index"][data["index_id_stop"][station][1]]]
            if station == 0:
                break
            if station in pickups:
                cap += 1
                users.append(data["index_id_stop"][station][0])
            path.append(data["index_id_stop"][station][1])
        routes.append(path)
        capacities.append(cap)
        passengers.append(users)
        travel_times.append(round(travel_time, 2))

    return routes, capacities, passengers, travel_times

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write the number of vehicles, "
                                                 "three weights (alpha, beta, gamma), "
                                                 "passengers requests (user id, pick-up location, drop-off location), "
                                                 "working time of delivers, "
                                                 "capacity of vehicles for delivery "
                                                 "and penalty given for not visiting a location.")
    parser.add_argument("-v", "--num_vehicle", type=int, required=True, help="number of vehicles")
    parser.add_argument("-w", "--weight", type=int, required=True, nargs="+", help="alpha, beta, gamma")
    parser.add_argument("-pudo", "--pudo", type=int, required=True, nargs="+", action="append",
                        help="passenger request (user id, pick-up location, drop-off location)")
    parser.add_argument("-t", "--time", type=int, required=False,
                        help="working time of delivers in seconds")
    parser.add_argument("-c", "--capacity", type=int, required=False,
                        help="capacity of vehicles for delivery")
    parser.add_argument("-p", "--penalty", type=int, required=True, help="penalty for unvisited stations")

    args = parser.parse_args()

    # make problem
    num_veh = args.num_vehicle  # number of vehicles
    penalty = args.penalty      # penalty give for not visiting a location

    # problem setting
    data, planner = create_data_model(func=RoutingPlanner, num_veh=num_veh, pudo=args.pudo, penalty=penalty,
                                      alpha=args.weight[0], beta=args.weight[1], gamma=args.weight[2])

    routes, capacities, passengers, travel_times = optimize(data, working_time=args.time, capacity=args.capacity)

    # passengers' calls
    print("[Passengers' Calls]")
    print_pudo = {"user ID": list(map(lambda x: x[0], data["pickups_deliveries"])),
                  "pickup location": list(map(lambda x: x[1], data["pickups_deliveries"])),
                  "dropoff location": list(map(lambda x: x[2], data["pickups_deliveries"]))}
    print(tabulate(print_pudo, headers="keys", tablefmt="fancy_grid", missingval="N/A"))

    # result
    print("[Result]")
    # problem description
    print("* There is a working time limit (%s seconds) for each delivers." % (args.time))
    print("* There is a capacity limit (%s) for vehicles." % (args.capacity))
    print("* A penalty (%s) is given for the number of locations that cannot be visited." % (penalty))
    print_route = {"car": list(np.arange(num_veh)), "route": routes, "users": passengers,
                   "capacity": capacities, "travel time": travel_times}
    total_users = set(print_pudo["user ID"])
    for i in range(num_veh):
        total_users = total_users - set(passengers[i])
    print(tabulate(print_route, headers="keys", tablefmt="fancy_grid", missingval="N/A"))
    print("unserviced users: ", list(total_users))