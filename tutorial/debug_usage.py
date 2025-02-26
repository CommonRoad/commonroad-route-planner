import os
from pathlib import Path
from time import perf_counter

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route

# typing
from typing import List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.route_candidate_holder import PathGenerator
    from commonroad_route_planner.reference_path import ReferencePath


def main(save_imgs: bool = False, use_cr2023_challenge: bool = False):
    # ========== initialization =========== #
    if use_cr2023_challenge:
        path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
    else:
        path_scenarios = Path(__file__).parents[1] / "scenarios"

    use_list: List = [
        # "DEU_Stu-1_49_I-1",
        # "DEU_Stu-1_49_I-1",
        # "USA_US101-22_1_T-1",
        "DEU_Frankfurt-3_19_I-1",
    ]

    for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
        id_scenario = filename.split(".")[0]
        if id_scenario not in use_list:
            print(f"scenario id {id_scenario}")
            continue
        print(f"Testing scenario {filename}")
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(f"{path_scenarios / id_scenario}.xml").open()
        # retrieve the first planning problem in the problem set
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        t_start = perf_counter()
        # ========== reference_path planning =========== #
        # instantiate a reference_path planner with the scenario and the planning problem
        route_planner = RoutePlanner(scenario=scenario, planning_problem=planning_problem, extended_search=False)
        # plan routes, and save the routes in a reference_path candidate holder
        route_selector: "ReferencePathPlanner" = route_planner.plan_routes()

        # ========== retrieving routes =========== #
        # here we retrieve the first reference_path in the list, this is equivalent to: reference_path = list_routes[0]
        route: "ReferencePath" = route_selector.retrieve_shortest_route()
        print(f"[Time] Retrieving first route took {perf_counter() - t_start}")

        # option 2: retrieve all routes
        list_routes, num_routes_retrieved = route_selector.retrieve_all_routes()
        print(f"Number of routes retrieved: {num_routes_retrieved}")

        # ========== visualization =========== #
        visualize_route(
            reference_path=route,
            scenario=scenario,
            planning_problem=planning_problem,
            save_img=False,
            draw_route_lanelets=True,
            draw_reference_path=True,
        )

        print(f"checked {(idx * 100 / len(os.listdir(path_scenarios))):.2f}% of scenarios")

        print(" \n \n")


if __name__ == "__main__":
    main(save_imgs=False, use_cr2023_challenge=False)
