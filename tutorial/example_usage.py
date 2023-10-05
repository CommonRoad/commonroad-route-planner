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


def main(save_imgs: bool = False, use_cr2023_challenge: bool = False):
    # ========== initialization =========== #

    if(use_cr2023_challenge):
        path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
    else:
        path_scenarios = Path(__file__).parents[1] / "scenarios"

    ignored_scenarios: List = [
        #"USA_Peach-3_1_T-1",
    ]

    for idx, filename in enumerate(os.listdir(path_scenarios)):
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

        t_start = perf_counter()
        # ========== route planning =========== #
        # instantiate a route planner with the scenario and the planning problem
        route_planner = RoutePlanner(
            scenario,
            planning_problem,
            use_predecessors_to_pass_through_goal_state=False,
        )
        # plan routes, and save the routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()

        # ========== retrieving routes =========== #
        # here we retrieve the first route in the list, this is equivalent to: route = list_routes[0]
        route = candidate_holder.retrieve_first_route()
        print(f'[Time] Retrieving first route took {perf_counter() - t_start}')

        # option 2: retrieve all routes
        list_routes, num_route_candidates = candidate_holder.retrieve_all_routes()
        print(f"Number of route candidates: {num_route_candidates}")


        # ========== visualization =========== #
        visualize_route(route, id_scenario,
                        save_img=save_imgs,
                        draw_route_lanelets=True, draw_reference_path=True)


        print(f'checked {(idx*100/len(os.listdir(path_scenarios))):.2f}% of scenarios')

        print(f' \n \n')


if __name__ == "__main__":
    main(save_imgs=True, use_cr2023_challenge=True)
