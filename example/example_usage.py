import matplotlib as mpl

from commonroad_route_planner.util import plot_found_routes

mpl.use('Qt5Agg')

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner


if __name__ == "__main__":
    scenario_path = 'example_scenarios/DEU_Gar-3_2_T-1.xml'

    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    # with the heuristic A* we do not want to solve cooperative scenarios so in all cases we will have
    # only one planning problem
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    route_planner = RoutePlanner(scenario, planning_problem, backend="networkx")
    routes = route_planner.search_alg()

    plot_found_routes(scenario, planning_problem, routes)
