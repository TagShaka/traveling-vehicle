import json
import math
from datetime import datetime, timedelta
from flask import Flask, request, jsonify
from flask_cors import CORS
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

@app.route('/test', methods=['GET'])
def test():
    return "Server is running!"

# Function to calculate the Euclidean distance between two points
def calculate_distance(coord1, coord2):
    return math.sqrt((coord1['lat'] - coord2['lat'])**2 + (coord1['lng'] - coord2['lng'])**2)

# Function to create the data model for OR-Tools
def create_data_model(requests):
    data = {}
    locations = [request['coordinates'] for request in requests]
    distance_matrix = [[calculate_distance(loc1, loc2) for loc2 in locations] for loc1 in locations]
    
    # Convert pickup and delivery times to manageable units
    time_windows = []
    for request in requests:
        pickup_time = datetime.fromisoformat(request['pickup'][:-1])
        delivery_time = datetime.fromisoformat(request['delivery'][:-1])
        # Expand the time window slightly to avoid tight constraints
        time_window = (int((pickup_time - datetime(1970, 1, 1)).total_seconds()) - 3600,  # 1 hour before
                       int((delivery_time - datetime(1970, 1, 1)).total_seconds()) + 3600)  # 1 hour after
        time_windows.append(time_window)

    # Define pickup and delivery pairs with relaxed constraints
    pickups_deliveries = [(i, i + 1) for i in range(0, len(requests), 2)]

    # Find the index of the request with the earliest pickup time
    earliest_pickup_index = min(range(len(requests)), key=lambda i: datetime.fromisoformat(requests[i]['pickup'][:-1]))

    data['distance_matrix'] = distance_matrix
    data['time_windows'] = time_windows
    data['pickups_deliveries'] = pickups_deliveries
    data['num_vehicles'] = 1
    data['depot'] = earliest_pickup_index
    return data

# Function to solve the VRP problem with flexible pickup and delivery constraints
def solve_vrp(requests):
    data = create_data_model(requests)
    print(f"Data model created: {data}")  # Logging data model

    try:
        manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
        print("RoutingIndexManager created successfully.")  # Log successful creation
    except Exception as e:
        print(f"Exception creating RoutingIndexManager: {str(e)}")
        return []

    try:
        routing = pywrapcp.RoutingModel(manager)
        print("RoutingModel created successfully.")  # Log successful creation
    except Exception as e:
        print(f"Exception creating RoutingModel: {str(e)}")
        return []

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    try:
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        print("Transit callback and cost evaluator set successfully.")  # Log successful setup
    except Exception as e:
        print(f"Exception setting transit callback: {str(e)}")
        return []

    time = 'time'
    try:
        routing.AddDimension(
            transit_callback_index,
            100000,  # allow larger waiting time
            100000,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time)
        time_dimension = routing.GetDimensionOrDie(time)
        print("Time dimension added successfully.")  # Log successful dimension addition
    except Exception as e:
        print(f"Exception adding time dimension: {str(e)}")
        return []

    # Set adjusted time windows for all locations
    try:
        for location_idx, (start, end) in enumerate(data['time_windows']):
            index = manager.NodeToIndex(location_idx)
            start_readable = datetime.fromtimestamp(start).isoformat()
            end_readable = datetime.fromtimestamp(end).isoformat()
            print(f"Setting adjusted time window for location {location_idx}: {start} ({start_readable}) to {end} ({end_readable})")
            try:
                time_dimension.CumulVar(index).SetRange(start, end)
                print(f"Time window for location {location_idx} set successfully.")
            except Exception as e:
                print(f"Exception setting time window for location {location_idx}: {str(e)}")
        print("Adjusted time windows set successfully.")
    except Exception as e:
        print(f"Exception during setting adjusted time windows: {str(e)}")
        return []

    # Add pickup and delivery constraints with relaxed sequence
    try:
        for pickup, delivery in data['pickups_deliveries']:
            print(f"Adding pickup and delivery constraints for pickup {pickup} and delivery {delivery}")
            routing.AddPickupAndDelivery(manager.NodeToIndex(pickup), manager.NodeToIndex(delivery))
            routing.solver().Add(routing.VehicleVar(manager.NodeToIndex(pickup)) == routing.VehicleVar(manager.NodeToIndex(delivery)))
            routing.solver().Add(time_dimension.CumulVar(manager.NodeToIndex(pickup)) <= time_dimension.CumulVar(manager.NodeToIndex(delivery)))
        print("Pickup and delivery constraints added successfully.")
    except Exception as e:
        print(f"Exception in adding constraints for pickup {pickup} and delivery {delivery}: {str(e)}")
        return []

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.time_limit.seconds = 30

    print("Attempting to solve...")  # Log before solving
    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        print("No solution found!")
        return []

    print("Solution found, processing route...")  # Log successful solution finding
    index = routing.Start(0)
    plan_output = []
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        plan_output.append(requests[node_index])
        index = solution.Value(routing.NextVar(index))

    return plan_output

# Function to process the requests and return the sorted array
def process_requests(requests):
    sorted_requests = solve_vrp(requests)
    return sorted_requests

@app.route('/optimize_route', methods=['POST'])
def optimize_route():
    try:
        requests = request.json
        print("Received requests: ", requests)  # Logging the received requests
        sorted_requests = process_requests(requests)
        if not sorted_requests:
            return jsonify({"error": "No valid route found with the given constraints."}), 400
        return jsonify(sorted_requests)
    except Exception as e:
        print("Exception: ", str(e))  # Logging the exception
        return jsonify({"error": str(e)}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
