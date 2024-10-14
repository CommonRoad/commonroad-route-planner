import unittest
from pathlib import Path


# third party
from shapely import LineString as ShapelyLineString
from shapely import Polygon as ShapelyPolygon

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import ShapeGroup as CRShapeGroup

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.lanelet_sequence import LaneletSequence


# typing
from typing import List, Tuple


class TestGoalReached(unittest.TestCase):
    """
    Tests whether reference path intersects with goal region
    """

    def test_goalreached(self, development=False):

        scenarios_without_problem: List[str] = [
            "DEU_Aachen-9_50_I-1",
            "DEU_Cologne-19_130_I-1",
            "DEU_GarchingCampus2D-2",
            "USA_Lanker-2_18_T-1",
        ]
        success_scenarios_easy, failed_scenarios_easy = self._analyse_scenarios(
            scenarios_without_problem, use_cr2023_challenge=False
        )
        print("Tested easy scenarios: ")
        print(f"  >>  successes: {len(success_scenarios_easy)}")
        print(f"  >>  failed scenarios: {len(failed_scenarios_easy)}")

        if len(failed_scenarios_easy) > 0:
            raise ValueError(
                "Not all reference paths intersect goal regions: \n "
                + f"succeded easy: {len(success_scenarios_easy)} --  {success_scenarios_easy} \n"
                f"failed easy: {len(failed_scenarios_easy)}  --  {failed_scenarios_easy} \n"
            )

        if development:
            scenarios_with_problem: List[str] = [
                "USA_Peach-3_1_T-1",
                "USA_US101-22_1_T-1",
                "USA_US101-29_1_T-1",
                "ZAM_Turorial-1_2_T-1_modified",
            ]
            success_scenarios_hard, failed_scenarios_hard = self._analyse_scenarios(
                scenarios_with_problem, use_cr2023_challenge=False
            )
            print("Tested hard scnearios:")
            print(f"  >>  successes: {len(success_scenarios_hard)}")
            print(f"  >>  failed scenarios: {len(failed_scenarios_hard)}")

            if len(failed_scenarios_easy) > 0 or len(failed_scenarios_hard) > 0:
                raise ValueError(
                    "Not all reference paths intersect goal regions: \n "
                    + f"succeded easy: {len(success_scenarios_easy)} --  {success_scenarios_easy} \n"
                    f"failed easy: {len(failed_scenarios_easy)}  --  {failed_scenarios_easy} \n"
                    f"succeded hard: {len(success_scenarios_hard)} --  {success_scenarios_hard} \n"
                    f"failed hard: {len(failed_scenarios_hard)}  --  {failed_scenarios_hard} \n"
                )

    def _analyse_scenarios(
        self, scenarios: List[str], use_cr2023_challenge: bool = False, scenarios_path: str = None
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

            # Plan reference_path
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
            route_planner = RoutePlanner(scenario.lanelet_network, planning_problem)

            # find routing sequence as sequence of lanelet ids
            route_candidates: List[LaneletSequence] = route_planner.plan_routes()

            # generate reference paths
            ref_path_generator: ReferencePathPlanner = ReferencePathPlanner(
                lanelet_network=scenario.lanelet_network,
                planning_problem=planning_problem,
                routes=route_candidates
            )

            route = ref_path_generator.plan_shortest_reference_path(retrieve_shortest=True)

            # create shapely objects
            shapely_route_line: ShapelyLineString = ShapelyLineString(route.reference_path)

            shapely_goal_polygons: List[ShapelyPolygon] = list()
            for state in planning_problem.goal.state_list:
                goal_shape = state.position
                if hasattr(goal_shape, "shapely_object"):
                    shapely_goal_polygons.append(goal_shape.shapely_object)
                elif isinstance(goal_shape, CRShapeGroup):
                    shapely_goal_polygons.extend([shape.shapely_object for shape in goal_shape.shapes])
                else:
                    shapely_goal_polygons.append(goal_shape.shapes.shapely_object)

            # Check if the linestring of the start intersects with at least one polyon of the goal
            flag_success = False
            for shapely_polygon in shapely_goal_polygons:
                if shapely_polygon.intersects(shapely_route_line):
                    success_scenarios.append(filename)
                    flag_success = True
                    break

            if not flag_success:
                failed_scenarios.append(filename)

        return success_scenarios, failed_scenarios
