from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


# =====================================================
# GENERIC VRP SOLVER
# =====================================================

def solve_vrp(distance_matrix,
              num_vehicles=1,
              depot=0,
              max_distance=None):

    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        num_vehicles,
        depot
    )

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):

        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)

        return distance_matrix[from_node][to_node]

    transit_callback_index = (
        routing.RegisterTransitCallback(
            distance_callback
        )
    )

    routing.SetArcCostEvaluatorOfAllVehicles(
        transit_callback_index
    )

    if max_distance:

        routing.AddDimension(
            transit_callback_index,
            0,
            max_distance,
            True,
            "Distance"
        )

    search_parameters = (
        pywrapcp.DefaultRoutingSearchParameters()
    )

    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy
        .PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(
        search_parameters
    )

    routes = {}

    if solution:

        for vehicle_id in range(num_vehicles):

            route = []

            index = routing.Start(vehicle_id)

            while not routing.IsEnd(index):

                route.append(
                    manager.IndexToNode(index)
                )

                index = solution.Value(
                    routing.NextVar(index)
                )

            route.append(
                manager.IndexToNode(index)
            )

            routes[vehicle_id] = route

    return routes


# =====================================================
# INITIAL DATA
# =====================================================

distance_matrix = [
    [0,10,15,20,18,22],
    [10,0,12,25,17,30],
    [15,12,0,10,20,14],
    [20,25,10,0,15,11],
    [18,17,20,15,0,16],
    [22,30,14,11,16,0]
]

num_vehicles = 1

print("\nINITIAL OPTIMIZATION")

routes = solve_vrp(
    distance_matrix,
    num_vehicles=num_vehicles,
    depot=0,
    max_distance=100
)

print(routes)

# Example output:
# {0: [0,1,2,5,3,4,0]}


# =====================================================
# SIMULATE VEHICLE PROGRESS
# =====================================================

vehicle_route = routes[0]

completed_nodes = []

current_time = 0

print("\nSIMULATING EXECUTION")

# vehicle completed first two customers

completed_nodes.append(vehicle_route[1])

current_time += distance_matrix[
    vehicle_route[0]
][
    vehicle_route[1]
]

completed_nodes.append(vehicle_route[2])

current_time += distance_matrix[
    vehicle_route[1]
][
    vehicle_route[2]
]

current_location = vehicle_route[2]

remaining_nodes = []

for node in vehicle_route[3:]:

    if node != 0:
        remaining_nodes.append(node)

print("Completed:", completed_nodes)
print("Current Location:", current_location)
print("Remaining:", remaining_nodes)
print("Current Time:", current_time)


# =====================================================
# NEW ORDER ARRIVES
# =====================================================

print("\nNEW ORDER ARRIVED")

new_order = 6

print("New customer:", new_order)


# =====================================================
# EXPAND DISTANCE MATRIX
# =====================================================

expanded_matrix = [
    [0,10,15,20,18,22,19],
    [10,0,12,25,17,30,14],
    [15,12,0,10,20,14,8],
    [20,25,10,0,15,11,7],
    [18,17,20,15,0,16,12],
    [22,30,14,11,16,0,9],
    [19,14,8,7,12,9,0]
]


# =====================================================
# BUILD REROUTING SUBPROBLEM
# =====================================================

active_nodes = (
    [current_location]
    + remaining_nodes
    + [new_order]
)

print("Nodes for rerouting:", active_nodes)

# active_nodes might become:
# [2,5,3,4,6]


# =====================================================
# CREATE SMALL MATRIX
# =====================================================

mapping = [0] + active_nodes

small_matrix = []

for i in mapping:

    row = []

    for j in mapping:

        row.append(
            expanded_matrix[i][j]
        )

    small_matrix.append(row)


# =====================================================
# REROUTE
# =====================================================

print("\nREROUTING")

new_routes = solve_vrp(
    small_matrix,
    num_vehicles=1,
    depot=0,
    max_distance=100
)

print("Local Route:", new_routes)

# convert back to original ids

rerouted_path = []

for node in new_routes[0]:

    rerouted_path.append(
        mapping[node]
    )

print("\nFINAL REROUTED PATH")

print(rerouted_path)

print("\nSTATE")

print({
    "completed_nodes": completed_nodes,
    "current_location": current_location,
    "current_time": current_time,
    "remaining_nodes": remaining_nodes,
    "new_order": new_order
})