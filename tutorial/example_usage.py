import os
from pathlib import Path
from time import perf_counter

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import LaneChangeMethod
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import DefaultGenerationStrategy


# typing
from typing import List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.reference_path import ReferencePath
    from commonroad_route_planner.lanelet_sequence import LaneletSequence


def main(save_imgs: bool = False, use_cr2023_challenge: bool = False):
    # ========== initialization =========== #
    if use_cr2023_challenge:
        path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
    else:
        path_scenarios = Path(__file__).parents[1] / "scenarios"

    ignored_scenarios: List = [
        # "DEU_Frankfurt-3_25_I-1",
        # "DEU_Stu-1_49_I-1"
        # "DEU_Frankfurt-3_19_I-1",
        # "DEU_Frankfurt-3_23_I-1",
        # "DEU_Frankfurt-3_27_I-1",
        # "DEU_Frankfurt-3_29_I-1"
    ]

    for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
        id_scenario = filename.split(".")[0]
        if id_scenario in ignored_scenarios:
            continue

        print(f"Testing scenario {filename}")
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(f"{path_scenarios / id_scenario}.xml").open()
        # retrieve the first planning problem in the problem set
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        t_start = perf_counter()
        # ========== reference_path planning =========== #
        # instantiate a reference_path planner with the scenario and the planning problem
        route_planner = RoutePlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            extended_search=False,
        )
        # find routing sequence as sequence of lanelet ids
        routes: List[LaneletSequence] = route_planner.plan_routes()

        # generate reference paths
        ref_path_planner: ReferencePathPlanner = ReferencePathPlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            routes=routes,
            lane_change_method=LaneChangeMethod.QUINTIC_SPLINE,
            generation_strategy=DefaultGenerationStrategy
        )

        # ========== retrieving routes =========== #
        # here we retrieve the shortest reference_path that has the least amount of disjoint lane changes
        reference_path: "ReferencePath" = ref_path_planner.plan_shortest_reference_path(
            retrieve_shortest=True, consider_least_lance_changes=True
        )
        print(f"[Time] Retrieving first route took {perf_counter() - t_start}")

        # Init reference_path extendor
        route_extendor: RouteExtendor = RouteExtendor(reference_path)
        # Extend reference path at start and end
        route_extendor.extend_reference_path_at_start_and_end()

        # This is unnecessary but shows that the route_extendor modified the reference_path object
        reference_path: ReferencePath = route_extendor.get_route()

        # option 2: retrieve all routes
        list_routes, num_routes_retrieved = ref_path_planner.plan_all_reference_paths()
        print(f"Number of routes retrieved: {num_routes_retrieved}")

        # ========== visualization =========== #
        visualize_route(
            reference_path=reference_path,
            scenario=scenario,
            planning_problem=planning_problem,
            save_img=save_imgs,
            draw_route_lanelets=True,
            draw_reference_path=True,
        )

        print(f"checked {(idx*100/len(os.listdir(path_scenarios))):.2f}% of scenarios")

        print(" \n \n")


if __name__ == "__main__":
    main(save_imgs=True, use_cr2023_challenge=False)
