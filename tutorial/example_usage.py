import os
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from route_planner.route_planner import RoutePlanner
from route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes

if __name__ == "__main__":
    path_notebook = os.getcwd()
    path_scenario = os.path.join(path_notebook, "../scenarios/")
    id_scenario = 'USA_Peach-2_1_T-1'

    # read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario + id_scenario + '.xml').open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # instantiate a route planner
    route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

    # plan routes
    route_planner.plan_routes()

    # retrieve a route
    # option 1: retrieve all routes and manually select a route by its index
    routes, num_route_candidates = route_planner.retrieve_all_routes()
    print(f"Feasible route candidates: {num_route_candidates}")
    # the index should not exceed (num_route_candidates - 1)
    route = routes[0]

    # option 2: retrieve the best route by orientation metric
    # route = route_planner.retrieve_best_route_by_orientation()

    # retrieve plot limits for better visualization. we can either use help from the route lanelet or the reference path
    plot_limits = get_plot_limits_from_reference_path(route)
    # plot_limits = get_plot_limits_from_routes(route)
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
