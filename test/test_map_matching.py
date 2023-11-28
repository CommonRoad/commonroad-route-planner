import unittest
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.utility.map_matching import MapMatcher


class TestMapMatching(unittest.TestCase):
    """
    Unit test for FloFi's map matching
    """
    def test_Garching(self):
        scenario, planning_problem = CommonRoadFileReader(
            Path(__file__).parents[1] / "scenarios/DEU_Gar-3_2_T-1.xml"
        ).open()

        mm = MapMatcher(scenario.lanelet_network)
        dyn_obst = scenario.dynamic_obstacles[0]
        lt_sequence = mm.map_matching(
            dyn_obst.prediction.trajectory.state_list,
            dyn_obst.initial_state
        )

        assert lt_sequence == [54508, 54541, 54494, 54520, 54490]

    def test_Lanker(self):
        scenario, planning_problem = CommonRoadFileReader(
            Path(__file__).parents[1] / "scenarios/USA_Lanker-2_6_T-1.xml"
        ).open()

        mm = MapMatcher(scenario.lanelet_network)
        solutions = {
            2331: [3665, 3667, 3666, 3664],
            2343: [3442, 3665, 3664, 3666],
            2392: [3442, 3665, 3664],
            2446: [3430, 3442, 3665],
            2452: [3420, 3430, 3442],
        }

        for obst_id, solution in solutions.items():
            dyn_obst = scenario.obstacle_by_id(obst_id)
            lt_sequence = mm.map_matching(
                dyn_obst.prediction.trajectory.state_list,
                dyn_obst.initial_state,
                True, 4
            )
            assert lt_sequence == solution













