import unittest
from pathlib import Path

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_route_planner.lanelet_sequence import LaneletSequence

# typing
from typing import List


class TestMinimalExample(unittest.TestCase):

    scenario_root = Path(__file__).parents[1] / "scenarios"
    scenarios = [
        "DEU_Aachen-9_50_I-1",
        "DEU_Cologne-19_130_I-1",
        "DEU_GarchingCampus2D-2",
        "USA_Lanker-2_18_T-1",
    ]

    def test_minimal_example(self):
        """
        Test minimal example
        """

        for filename in self.scenarios:
            scenario_path = self.scenario_root / filename
            scenario, planning_problem_set = CommonRoadFileReader(f"{scenario_path}.xml").open()
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
            with self.subTest(msg=f"Testing scenario: {filename} with generation from scenario", filename=filename):
                self.minimal_example(
                    scenario=scenario,
                    planning_problem=planning_problem
                )

    def minimal_example(
            self,
            scenario: Scenario,
            planning_problem: PlanningProblem
    ) -> ReferencePath:
        """
        Minimal example.
        """
        route_planner = RoutePlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
        )

        routes: List[LaneletSequence] = route_planner.plan_routes()

        ref_path_planner: ReferencePathPlanner = ReferencePathPlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            routes=routes,
        )

        return ref_path_planner.plan_shortest_reference_path(
            retrieve_shortest=True, consider_least_lance_changes=True
        )
