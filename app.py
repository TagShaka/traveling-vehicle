import logging
from flask import Flask, request, jsonify
from flask_cors import CORS
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from geopy.distance import geodesic
from datetime import datetime, timezone, timedelta

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

@app.route('/')
def home():
    logger.info("Home route accessed")
    return "Traveling Vehicle Problem Solver"

@app.route('/solve_vrp', methods=['POST'])
def solve_vrp():
    try:
        data = request.json
        logger.debug(f"Received data: {data}")
        locations, deadlines, names = parse_input(data)
        distance_matrix = calculate_distance_matrix(locations)
        solution = optimize_routes(locations, deadlines, names, distance_matrix)
        return jsonify(solution)
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        return jsonify({'error': str(e)}), 500

def parse_input(data):
    locations = []
    deadlines = []
    names = []
    
    # Find the earliest maxTime in the data
    earliest_entry = min(data, key=lambda x: datetime.fromisoformat(x['maxTime']).replace(tzinfo=timezone.utc))
    earliest_time = datetime.fromisoformat(earliest_entry['maxTime']).replace(tzinfo=timezone.utc)
    logger.debug(f"Earliest maxTime: {earliest_time.isoformat()}")
    
    # Reorder locations so that the earliest deadline is the first location
    sorted_data = sorted(data, key=lambda x: datetime.fromisoformat(x['maxTime']).replace(tzinfo=timezone.utc))
    
    for entry in sorted_data:
        coords = entry['coordinates']
        max_time = datetime.fromisoformat(entry['maxTime']).replace(tzinfo=timezone.utc)
        name = entry['name']
        logger.debug(f"Processing location {coords} with maxTime {max_time.isoformat()} and name {name}")
        deadline_minutes = (max_time - earliest_time).total_seconds() / 60
        locations.append((coords['lat'], coords['lng']))
        deadlines.append(deadline_minutes)
        names.append(name)
    logger.debug(f"Processed locations: {locations}")
    logger.debug(f"Processed deadlines (in minutes): {deadlines}")
    logger.debug(f"Processed names: {names}")
    return locations, deadlines, names

def calculate_distance_matrix(locations):
    matrix = []
    for loc1 in locations:
        row = []
        for loc2 in locations:
            distance = geodesic(loc1, loc2).kilometers
            travel_time_minutes = (distance / 70) * 60  # Assuming average travel speed of 70 km/h.
            # Add buffer to travel time to account for delays.
            travel_time_minutes += 200
            row.append(travel_time_minutes)
        matrix.append(row)
    logger.debug(f"Distance matrix (in minutes): {matrix}")
    return matrix

def optimize_routes(locations, deadlines, names, distance_matrix):
    num_locations = len(locations)
    num_vehicles = 1
    depot = 0  # Always start from the location with the earliest deadline

    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return int(distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    max_deadline = max(deadlines)
    logger.debug(f"Max deadline: {max_deadline}")
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        int(max_deadline * 2),  # set a large enough max time
        True,  # force start cumul to zero
        "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    for idx, deadline in enumerate(deadlines):
        index = manager.NodeToIndex(idx)
        start_time = 0
        end_time = int(deadline)
        if idx == 0:
            end_time += 1440  # Add buffer to the first location's deadline
        else:
            end_time += 1440  # Add buffer to each subsequent location
        logger.debug(f"Setting time window for location {index}: [{start_time}, {end_time}]")
        time_dimension.CumulVar(index).SetRange(start_time, end_time)

    # Ensure the vehicle doesn't need to return to the start point
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(0)))

    # Add precedence constraints to enforce visiting order based on deadlines
    for i in range(len(deadlines) - 1):
        routing.solver().Add(time_dimension.CumulVar(manager.NodeToIndex(i + 1)) >= time_dimension.CumulVar(manager.NodeToIndex(i)))

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        return extract_solution(manager, routing, solution, time_dimension, locations, names)
    logger.error("Solver failed to find a solution.")
    return {"status": "No solution found"}

def extract_solution(manager, routing, solution, time_dimension, locations, names):
    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        route.append({
            'location': locations[node_index],
            'name': names[node_index],
            'index': node_index,
            'time': solution.Min(time_dimension.CumulVar(index))
        })
        index = solution.Value(routing.NextVar(index))
    return {'status': 'Solution found', 'route': route}

if __name__ == '__main__':
    app.run(debug=True, port=5001)  # Changed port to avoid 403 Forbidden error
