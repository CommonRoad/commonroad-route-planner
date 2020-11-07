import numpy as np
import matplotlib.pyplot as plt
from commonroad.scenario.trajectory import State

from commonroad_route_planner.utils_visualization import plot_found_routes, plot_route_environment, draw_navigator, draw_state
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.RoutePlanner import RoutePlanner


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

    route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)

    route_candidates = route_planner.plan_routes()
    print(f"Found route candidates: {route_candidates}")

    route = route_candidates.get_most_likely_route_by_orientation()
    # plot_found_routes(scenario, planning_problem, [route.route])

    # Query the distance until lane change is required
    navigator = route.navigator

    # states = [planning_problem.initial_state,
    #           State(position=np.array([-6.5, 6.5]), orientation=np.pi),
    #           State(position=np.array([-6.5, 20]), orientation=np.pi/2),
    #           State(position=np.array([-6.5, 60]), orientation=np.pi/2),
    #           State(position=np.array([-12.5, 20]), orientation=np.pi/2),
    #           State(position=np.array([-12.5, 60]), orientation=np.pi/2)]
    states = [planning_problem.initial_state]

    distances_until_lane_change = [navigator.get_lane_change_distance(state) for state in states]
    long_lat_distances = [navigator.get_long_lat_distance_to_goal(state.position) for state in states]
    print(f"Distances until lane change: {distances_until_lane_change}")
    print(f"Long-lat distances: {long_lat_distances}")

    draw_navigator(navigator)
    plt.show()

    # state_to_draw = State(position=np.array([-6.5, 20]), orientation=np.pi / 2)
    # draw_state(state_to_draw)
    # plt.show()