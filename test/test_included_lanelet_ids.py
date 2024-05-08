import unittest
from pathlib import Path

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route

# typing
from typing import List, Tuple


class TestIncludedLanelets(unittest.TestCase):
    """
    Tests whether reference path intersects with goal region
    """

    def test_goalreached(self, development: bool = False):

        scenario_names: List[str] = ["DEU_GarchingCampus2D-2"]
        success_scenarios, failed_scenarios = self._analyse_scenarios(
            scenario_names, development=development, use_cr2023_challenge=False
        )

        if len(failed_scenarios) > 0:
            raise ValueError(
                "Not all scenarios passed: \n " + f"succeded easy: {len(success_scenarios)} --  {success_scenarios} \n"
                f"failed easy: {len(failed_scenarios)}  --  {failed_scenarios} \n"
            )

    def _analyse_scenarios(
        self,
        scenarios: List[str],
        use_cr2023_challenge: bool = False,
        scenarios_path: str = None,
        development: bool = False,
    ) -> Tuple[List[str], List[str]]:
        """
        Test if polyline of reference path intersects with goal.
        Returns (success_scenarios, failed_scenarios)

        """

        success_scenarios: List[str] = list()
        failed_scenarios: List[str] = list()

        if scenarios_path is not None:
            path_scenarios = Path(scenarios_path)
        elif use_cr2023_challenge:
            path_scenarios = Path(__file__).parents[1] / "tutorial/commonroad_challenge_2023"
        else:
            path_scenarios = Path(__file__).parents[1] / "scenarios"

        for filename in scenarios:
            print(f"Testing scenario {filename}")

            # read in scenario and planning problem set
            scenario, planning_problem_set = CommonRoadFileReader(f"{path_scenarios / filename}.xml").open()

            # Plan route with extended search
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
            route_planner = RoutePlanner(lanelet_network=scenario.lanelet_network, planning_problem=planning_problem, extended_search=True)

            included_lanelet_ids: List[int] = [1932]

            route_selector = route_planner.plan_routes()
            route = route_selector.retrieve_shortest_route(
                retrieve_shortest=True, included_lanelet_ids=included_lanelet_ids
            )

            if development:
                visualize_route(
                    route,
                    scenario=scenario,
                    planning_problem=planning_problem,
                    save_img=False,
                    draw_route_lanelets=True,
                    draw_reference_path=True,
                )

            if set(included_lanelet_ids).issubset(route.lanelet_ids):
                success_scenarios.append(filename)
            else:
                failed_scenarios.append(filename)

        return success_scenarios, failed_scenarios
