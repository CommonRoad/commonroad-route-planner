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

from commonroad_route_planner.utility.visualization import visualize_route


class TestNotThroughInitAndGoal(unittest.TestCase):

    scenario_root = Path(__file__).parents[1] / "scenarios"
    scenarios = [
        "ZAM_Turorial-1_2_T-1_modified",
        "USA_US101-29_1_T-1",
        "USA_Peach-4_1_T-1",
        "USA_Peach-3_1_T-1"
    ]

    def test_not_through_init_and_goal(self):
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
        Minimal example for how to deactivate path through initial state or goal
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
            path_trough_initial_state=False,
            path_through_goal_state=False
        )

        reference_path: ReferencePath = ref_path_planner.plan_shortest_reference_path(
            retrieve_shortest=True, consider_least_lance_changes=True
        )

        # TODO: Do algorithmic check, e.g. via max(abs(curvature)) to see that it worked, currently only visual check

        visualize_route(
            reference_path=reference_path,
            scenario=scenario,
            planning_problem=planning_problem,
            save_img=False,
            draw_route_lanelets=True,
            draw_reference_path=True,
        )


