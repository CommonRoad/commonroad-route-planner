import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_route_planner.RoutePlanner import RoutePlanner
from commonroad_route_planner.utils_visualization import draw_route

if __name__ == "__main__":
    scenario_path = 'example_scenarios/USA_Peach-2_1_T-1.xml'

    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    # create route planner
    route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
    # route candidate holder holds all feasible route candidates
    route_planner.plan_routes()
    print(f"Found route candidates: {route_planner.num_route_candidates}")

    # retrieve the best route by orientation metric
    # route = route_planner.retrieve_best_route_by_orientation()
    routes, _ = route_planner.retrieve_all_routes()
    route = routes[0]
    draw_route(route, draw_route_lanelets=True, draw_reference_path=True)
    plt.show()

    # # obtain the navigator object of the route
    # navigator = route.navigator
    #
    # # Query the distance until lane change is required
    # states = [planning_problem.initial_state]
    # distances_until_lane_change = [navigator.get_lane_change_distance(state) for state in states]
    # long_lat_distances = [navigator.get_long_lat_distance_to_goal(state.position) for state in states]
    # print(f"Distances until lane change: {distances_until_lane_change}")
    # print(f"Long-lat distances: {long_lat_distances}")
    #
    # draw_navigator(navigator)
