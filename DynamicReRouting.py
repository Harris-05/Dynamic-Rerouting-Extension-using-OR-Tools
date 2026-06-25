from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def solve_vrp(
    distance_matrix,
    num_vehicles=1,
    starts=[0],
    ends=[0],  
    max_distance=9999,
    initial_routes=None
):

    
    manager = pywrapcp.RoutingIndexManager(
        len(distance_matrix),
        num_vehicles,
        starts,
        ends
    )

    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        0,
        max_distance,
        True,
        "Distance"
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.seconds = 2

    #Warm Start if Solution Already exists
    if initial_routes is not None:
        initial_assignment = routing.ReadAssignmentFromRoutes(
            initial_routes,
            True
        )
        solution = routing.SolveFromAssignmentWithParameters(
            initial_assignment,
            search_parameters
        )
        #If no solution exists make a new one normally
    else:
        solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        return None

    routes = {}
    for vehicle_id in range(num_vehicles):
        route = []
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        routes[vehicle_id] = route

    return routes


#Simulating Data

distance_matrix = [
    [0,10,15,20,18,22],
    [10,0,12,25,17,30],
    [15,12,0,10,20,14],
    [20,25,10,0,15,11],
    [18,17,20,15,0,16],
    [22,30,14,11,16,0]
]

print("=" * 60)
print("INITIAL OPTIMIZATION")
print("=" * 60)

routes = solve_vrp(
    distance_matrix=distance_matrix,
    num_vehicles=1,
    starts=[0],
    ends=[0],
    max_distance=100
)

print("Initial Route:")
print(routes)


#Simulate Execution

vehicle_route = routes[0]
completed_nodes = []
current_time = 0

print("\n" + "=" * 60)
print("SIMULATING VEHICLE MOVEMENT")
print("=" * 60)

# Vehicle completed first customer
completed_nodes.append(vehicle_route[1])
current_time += distance_matrix[vehicle_route[0]][vehicle_route[1]]

# Vehicle completed second customer
completed_nodes.append(vehicle_route[2])
current_time += distance_matrix[vehicle_route[1]][vehicle_route[2]]

current_location = vehicle_route[2]

remaining_nodes = []
for node in vehicle_route[3:]:
    if node != 0:
        remaining_nodes.append(node)

print("Completed Nodes:", completed_nodes)
print("Current Location:", current_location)
print("Remaining Nodes:", remaining_nodes)
print("Current Time:", current_time)


# NEW ORDER ARRIVES


print("\n" + "=" * 60)
print("NEW ORDER ARRIVED")
print("=" * 60)

new_customer = 6
print(f"New Customer = {new_customer}")

# Expanded matrix including new customer
expanded_matrix = [
    [0,10,15,20,18,22,19],
    [10,0,12,25,17,30,14],
    [15,12,0,10,20,14,8],
    [20,25,10,0,15,11,7],
    [18,17,20,15,0,16,12],
    [22,30,14,11,16,0,9],
    [19,14,8,7,12,9,0]
]

#rerouting

# Fix: Mapping starts at current location and explicitly ends at the original depot (0)
mapping = [current_location] + remaining_nodes + [new_customer] + [0]

print("Nodes To Re-Optimize (Mapping):", mapping)

small_matrix = []
for i in mapping:
    row = []
    for j in mapping:
        row.append(expanded_matrix[i][j])
    small_matrix.append(row)


#Warm Start

old_remaining_route = []
for node in remaining_nodes:
    # Use the new mapping to get the correct internal index for the warm start
    old_remaining_route.append(mapping.index(node))

print("\nWarm Start Route (Internal Indices):", old_remaining_route)

#Dynamic Rerouting

print("\n" + "=" * 60)
print("DYNAMIC RE-ROUTING")
print("=" * 60)

new_routes = solve_vrp(
    distance_matrix=small_matrix,
    num_vehicles=1,
    starts=[0], # Starts at local index 0 (current_location)
    ends=[len(mapping) - 1], # Ends at local index - 1 (the original depot)
    max_distance=100,
    initial_routes=[old_remaining_route]
)

print("Local Route (Mapped Indices):")
print(new_routes)

final_route = []
for local_node_idx in new_routes[0]:
    final_route.append(mapping[local_node_idx])

print("\nFinal Dynamic Route (Global IDs):")
print(final_route)

print("\n" + "=" * 60)
print("FINAL STATE")
print("=" * 60)

print({
    "completed_nodes": completed_nodes,
    "current_location": current_location,
    "current_time": current_time,
    "remaining_nodes": remaining_nodes,
    "new_customer": new_customer,
    "rerouted_path": final_route
})