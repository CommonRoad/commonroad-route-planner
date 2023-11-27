import unittest
from pathlib import Path

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base

from commonroad_route_planner.utility.map_matching import MapMatcher



class TestExample(unittest.TestCase):
    """
    Unit test for FloFi's map matching
    """
    def test_example(self):
        # ========== map matching ============ #
        scenario, planning_problem = CommonRoadFileReader(
            Path(__file__).parents[1] / "scenarios/DEU_Gar-3_2_T-1.xml"
        ).open()

        mm = MapMatcher(scenario.lanelet_network, 2)
        lt_sequence = mm.map_matching(
            scenario.dynamic_obstacles[0].prediction.trajectory.state_list
        )

        if lt_sequence != [54508, 54541, 54494, 54520, 54490]:
            raise ValueError
        else:
            print("Map matching successful.")











