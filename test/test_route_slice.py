import unittest
from pathlib import Path
from time import perf_counter

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route

# typing
from typing import List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_route_planner.utility.route_slice.route_slice import RouteSlice



class TestRouteSlice(unittest.TestCase):
    """
    Tests whether reference path intersects with goal region
    """
    def test_routeslice(self,
                        scenarios_path: str = None, use_cr2023_challenge: bool = False,
                        development: bool=False):

        scenario_names: List[str] = ["DEU_GarchingCampus2D-2"]
        range_list: List[range] = list()
        range_list.append(range(0, 4000, 200))

        if (scenarios_path is not None):
            path_scenarios = Path(scenarios_path)
        elif (use_cr2023_challenge):
            path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
        else:
            path_scenarios = Path(__file__).parents[1] / "scenarios"



        for filename, range_object in zip(scenario_names, range_list):
            self._slice_route(path_scenarios, filename, range_object,
                                      development=development)


    def _slice_route(self, path_scenarios: Path, filename: str,
                           range_object: range,
                           development: bool=False) -> None:
        """
        Test if polyline of reference path intersects with goal.
        Returns (success_scenarios, failed_scenarios)

        """

        print(f"Testing scenario {filename}")
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(
            f"{path_scenarios / filename}.xml"
        ).open()

        # Plan route with extended search
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
        route_planner = RoutePlanner(
            scenario=scenario,
            planning_problem=planning_problem,
            extended_search=True,
            use_predecessors_to_pass_through_goal_state=False,
        )

        included_lanelet_ids: List[int] = [1932]

        route_selector = route_planner.plan_routes()
        route = route_selector.retrieve_shortest_route(retrieve_shortest=True,
                                                       included_lanelet_ids=included_lanelet_ids)


        for idx in range_object:
            # FIXME: Currently there is a bug in the garching map, it is centimeter not meter....
            point = route.reference_path[idx, :]

            t_0 = perf_counter()

            route_slice: "RouteSlice" = route.get_route_slice_from_position(x=point[0], y=point[1],
                                                                            distance_ahead_in_m=30,
                                                                            distance_behind_in_m=7)

            print(f'Slicing took {perf_counter() - t_0}s')

            if(True):
                visualize_route(
                    route_slice,
                    filename.split(".")[0],
                    save_img=False,
                    draw_route_lanelets=True,
                    draw_reference_path=True,
                )


