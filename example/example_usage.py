from commonroad_route_planner.util import plot_found_routes, plot_route_environment
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner


if __name__ == "__main__":
    scenario_path = 'example_scenarios/USA_Peach-2_1_T-1.xml'

    print(f"Loading scenario {scenario_path}")

    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    print(f"Scenario {scenario_path} loaded!")

    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    # with the heuristic A* we do not want to solve cooperative scenarios so in all cases we will have
    # only one planning problem
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    route_planner = RoutePlanner(scenario, planning_problem, backend="networkx")
    routes = route_planner.search_alg()

    # plot_found_routes(scenario, planning_problem, routes)

    # you can get the envrionment of the found route
    chosen_route = routes[0]
    route_environment = \
        route_planner.get_sectionized_environment_from_route(chosen_route, is_opposite_direction_allowed=False)

    plot_route_environment(scenario, planning_problem, route_environment)
