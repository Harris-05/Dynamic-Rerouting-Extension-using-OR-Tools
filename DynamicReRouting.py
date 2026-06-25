from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# -------------------------
# DATA
# -------------------------

distance_matrix = [
    [0, 10, 15, 20, 18],
    [10, 0, 12, 25, 17],
    [15, 12, 0, 10, 20],
    [20, 25, 10, 0, 15],
    [18, 17, 20, 15, 0]
]
max_travel= [
    70,
    40,
    30,
    20,
    90
]
demands = [
    0,  # depot
    4,
    3,
    5,
    6
]

vehicle_capacities = [10, 10]
completed_nodes=[]
num_vehicles = 2
depot = 0

# -------------------------
# MANAGER
# -------------------------

manager = pywrapcp.RoutingIndexManager(
    len(distance_matrix),
    num_vehicles,
    depot
)

# -------------------------
# ROUTING MODEL
# -------------------------

routing = pywrapcp.RoutingModel(manager)

# -------------------------
# DISTANCE CALLBACK
# -------------------------

def distance_callback(from_index, to_index):

    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    completed_nodes.append(from_node)
    return distance_matrix[from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(
    distance_callback
)

routing.SetArcCostEvaluatorOfAllVehicles(
    transit_callback_index
)

# -------------------------
# DEMAND CALLBACK
# -------------------------

def demand_callback(from_index):

    from_node = manager.IndexToNode(from_index)

    return demands[from_node]


demand_callback_index = (
    routing.RegisterUnaryTransitCallback(
        demand_callback
    )
)

# -------------------------
# CAPACITY CONSTRAINT
# -------------------------
def max_travel_callback(from_index):
    from_node = manager.IndexToNode(from_index)
    return max_travel[from_node]



routing.AddDimension(
    transit_callback_index,
    0,                      # slack
    50,
    True,
    "MaxTravel"
)
# -------------------------
# SEARCH PARAMETERS
# -------------------------

search_parameters = (
    pywrapcp.DefaultRoutingSearchParameters()
)

search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)

# -------------------------
# SOLVE
# -------------------------

solution = routing.SolveWithParameters(
    search_parameters
)

# -------------------------
# PRINT SOLUTION
# -------------------------

if solution:

    capacity_dimension = routing.GetDimensionOrDie(
        "MaxTravel"

    )

    for vehicle_id in range(num_vehicles):

        print("\n" + "=" * 40)
        print(f"Vehicle {vehicle_id}")

        index = routing.Start(vehicle_id)

        route_distance = 0

        while not routing.IsEnd(index):

            node = manager.IndexToNode(index)

            load = solution.Value(
                capacity_dimension.CumulVar(index)
            )

            print(
                f"Node {node} "
                f"(Load={load})"
            )

            previous_index = index

            index = solution.Value(
                routing.NextVar(index)
            )

            route_distance += (
                routing.GetArcCostForVehicle(
                    previous_index,
                    index,
                    vehicle_id
                )
            )

        end_load = solution.Value(
            capacity_dimension.CumulVar(index)
        )

        print(
            f"Node {manager.IndexToNode(index)} "
            f"(Load={end_load})"
        )

        print(
            f"Route Distance: {route_distance}"
        )

else:
    print("No solution found")