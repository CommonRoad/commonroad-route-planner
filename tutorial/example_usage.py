import os
from pathlib import Path
from time import perf_counter

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor


# typing
from typing import List
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_route_planner.route_selector import RouteSelector
    from commonroad_route_planner.route import Route


def main(save_imgs: bool = False, use_cr2023_challenge: bool = False):
    # ========== initialization =========== #
    if(use_cr2023_challenge):
        path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
    else:
        path_scenarios = Path(__file__).parents[1] / "scenarios"

    ignored_scenarios: List = [
        #"DEU_Frankfurt-3_25_I-1",
        #"DEU_Stu-1_49_I-1"
        #"DEU_Frankfurt-3_19_I-1",
        #"DEU_Frankfurt-3_23_I-1",
        #"DEU_Frankfurt-3_27_I-1",
        #"DEU_Frankfurt-3_29_I-1"
    ]



    for idx, filename in enumerate(sorted(os.listdir(path_scenarios))):
        id_scenario = filename.split(".")[0]
        if id_scenario in ignored_scenarios:
            continue

        print(f"Testing scenario {filename}")
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(
            f"{path_scenarios / id_scenario}.xml"
        ).open()
        # retrieve the first planning problem in the problem set
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        t_start = perf_counter()
        # ========== route planning =========== #
        # instantiate a route planner with the scenario and the planning problem
        route_planner = RoutePlanner(
            scenario=scenario,
            planning_problem=planning_problem,
            extended_search=False
        )
        # plan routes, and save the routes in a route candidate holder
        route_selector: "RouteSelector" = route_planner.plan_routes()

        # ========== retrieving routes =========== #
        # here we retrieve the shortest route that has the least amount of disjoint lane changes
        route: "Route" = route_selector.retrieve_shortest_route(
            retrieve_shortest=True,
            consider_least_lance_changes=True
        )
        print(f"[Time] Retrieving first route took {perf_counter() - t_start}")

        # Init route extendor
        route_extendor: RouteExtendor = RouteExtendor(route)
        # Extend reference path at start and end
        route_extendor.extend_reference_path_at_start_and_end()

        # This is unnecessary but shows that the route_extendor modified the route object
        route: Route = route_extendor.get_route()


        # option 2: retrieve all routes
        list_routes, num_routes_retrieved = route_selector.retrieve_all_routes()
        print(f"Number of routes retrieved: {num_routes_retrieved}")

        # ========== visualization =========== #
        visualize_route(
            route=route,
            scenario=scenario,
            planning_problem=planning_problem,
            save_img=save_imgs,
            draw_route_lanelets=True,
            draw_reference_path=True,
        )

        print(f"checked {(idx*100/len(os.listdir(path_scenarios))):.2f}% of scenarios")

        print(f" \n \n")





if __name__ == "__main__":
    main(save_imgs=True, use_cr2023_challenge=False)
