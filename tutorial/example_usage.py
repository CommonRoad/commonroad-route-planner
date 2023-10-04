import os
from pathlib import Path

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route


# typing
from typing import List


def main():
    # ========== initialization =========== #
    path_scenarios = Path(__file__).parents[1] / "scenarios"

    ignored_scenarios: List = [
        "USA_US101-3_4_T-1",
        "ZAM_Zip-1_6_T-1",
        "USA_Lanker-2_18_T-1",
        "DEU_Muc-1_2_T-1",
        "DEU_Test-1_1_T-1",
        "USA_Lanker-2_26_T-1",
        "USA_US101-4_3_T-1",
        "USA_US101-32_1_T-1"
    ]

    for filename in os.listdir(path_scenarios):
        id_scenario = filename.split('.')[0]
        if(id_scenario in ignored_scenarios):
            continue
        print(f'Testing scenario {filename}')
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(
            f"{path_scenarios / id_scenario}.xml"
        ).open()
        # retrieve the first planning problem in the problem set
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        # ========== route planning =========== #
        # instantiate a route planner with the scenario and the planning problem
        route_planner = RoutePlanner(
            scenario,
            planning_problem,
            backend=RoutePlanner.Backend.NETWORKX_REVERSED,
            use_predecessors_to_pass_through_goal_state=False,
        )
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
        visualize_route(route, draw_route_lanelets=True, draw_reference_path=True)


if __name__ == "__main__":
    main()
