import unittest
from pathlib import Path

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.lanelet_sequence import LaneletSequence

# typing
from typing import List


class TestRouteSlice(unittest.TestCase):
    """
    Tests whether reference path intersects with goal region
    """

    def test_route_section(self):

        scenario_name: str = "ZAM_Turorial-1_2_T-1_modified"
        lanelet_id: int = 1
        adjacent_lanelets_ids: List[int] = [1, 2, 3]
        has_neighbors: bool = True

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        print(f"Testing scenario {scenario_name}")
        # read in scenario and planning problem set
        scenario, planning_problem_set = CommonRoadFileReader(f"{path_scenarios / scenario_name}.xml").open()

        # Plan reference_path with extended search
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
        route_planner = RoutePlanner(lanelet_network=scenario.lanelet_network, planning_problem=planning_problem, extended_search=True)

        included_lanelet_ids: List[int] = [1932]

        # find routing sequence as sequence of lanelet ids
        route_candidates: List[LaneletSequence] = route_planner.plan_routes()

        # generate reference paths
        ref_path_generator: ReferencePathPlanner = ReferencePathPlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            routes=route_candidates
        )

        route = ref_path_generator.plan_shortest_reference_path(
            retrieve_shortest=True, included_lanelet_ids=included_lanelet_ids
        )

        lanelet_section: LaneletSection = route.get_lanelet_section(lanelet_id=lanelet_id)

        if adjacent_lanelets_ids != sorted(lanelet_section.adjacent_lanelet_ids, reverse=False):
            raise ValueError(
                f"Expected adjacent ids: {adjacent_lanelets_ids} \n"
                f"\t but got {sorted(lanelet_section.adjacent_lanelet_ids, reverse=False)}"
            )
        else:
            print(f"Got expected route section {sorted(lanelet_section.adjacent_lanelet_ids, reverse=False)}")

        if has_neighbors != lanelet_section.has_neighbors():
            raise ValueError(
                f"Expected has_neighbors={has_neighbors} \n" f"\t but got {lanelet_section.has_neighbors()}"
            )

        else:
            print(f"Got expected expected has_neighbors={lanelet_section.has_neighbors()}")
