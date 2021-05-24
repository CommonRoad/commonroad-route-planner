import os

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader

from route_planner.route_planner import RoutePlanner
from route_planner.utils_visualization import draw_route, obtain_plot_limits_from_reference_path

if __name__ == "__main__":
    # ========== initialization =========== #
    path_scenarios = os.path.join(os.getcwd(), "../scenarios/")
    id_scenario = 'USA_Peach-2_1_T-1'
    # read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(f"{path_scenarios}{id_scenario}.xml").open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # ========== route planning =========== #
    # instantiate a route planner with the scenario and the planning problem
    route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
    # plan routes, and save the routes in a route candidate holder
    candidate_holder = route_planner.plan_routes()

    # ========== retrieving routes =========== #
    # option 1: retrieve all routes
    list_routes, num_route_candidates = candidate_holder.retrieve_all_routes()
    print(f"Number of route candidates: {num_route_candidates}")
    # here we retrieve the first route in the list, this is equivalent to: route = list_routes[0]
    route = candidate_holder.retrieve_first_route()

    # option 2: retrieve the best route by orientation metric
    # route = candidate_holder.retrieve_best_route_by_orientation()

    # ========== visualization =========== #
    # obtain plot limits for a better visualization. we can obtain them through the lanelets or the reference path
    plot_limits = obtain_plot_limits_from_reference_path(route)
    # plot_limits = obtain_plot_limits_from_routes(route)

    # set the figure size and ratio
    size_x = 20
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
    fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
    fig.gca().axis('equal')

    # draw and plot
    draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
    plt.show()
