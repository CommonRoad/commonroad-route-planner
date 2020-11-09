import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from route_planner.RoutePlanner import RoutePlanner
from route_planner.utils_visualization import draw_route
import HelperFunctions as hf

if __name__ == "__main__":
    scenario_path = '../scenarios/DEU_Gar-3_2_T-1.xml'

    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    # create route planner
    route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

    # plan routes by calling this function
    route_planner.plan_routes()
    print(f"Feasible route candidates: {route_planner.num_route_candidates}")

    # retrieve the planned routes
    # option 1: retrieve the best route by orientation metric
    # route = route_planner.retrieve_best_route_by_orientation()
    # option 2: retrieve all route and manually select the route by indexing
    routes, num_route_candidates = route_planner.retrieve_all_routes()
    route = routes[0]

    # retrieve plot limits for better visualization. we can either use help from the route lanelet or the reference path
    plot_limits = hf.get_plot_limits_from_reference_path(route)
    # plot_limits = hf.get_plot_limits_from_routes(route, scenario)
    size_x = 20
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
    fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
    fig.gca().axis('equal')

    # draw and show
    draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
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
