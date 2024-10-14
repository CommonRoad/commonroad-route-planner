import unittest
from pathlib import Path

# commonrad
from commonroad.common.file_reader import CommonRoadFileReader

# Own Code base
import commonroad_route_planner.fast_api.fast_api as fast_api


class TestFastApi(unittest.TestCase):
    """
    Fast API test cases
    """
    scenario_root = Path(__file__).parents[1] / "scenarios"
    scenarios = [
        "DEU_Aachen-9_50_I-1",
        "DEU_Cologne-19_130_I-1",
        "DEU_GarchingCampus2D-2",
        "USA_Lanker-2_18_T-1",
    ]

    def test_from_scenario(self):
        """
        Test fast_api from scenario and pp
        """
        for filename in self.scenarios:
            scenario_path = self.scenario_root / filename
            scenario, planning_problem_set = CommonRoadFileReader(f"{scenario_path}.xml").open()
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
            with self.subTest(msg=f"Testing scenario: {filename} with generation from scenario", filename=filename):
                fast_api.generate_reference_path_from_scenario_and_planning_problem(
                    scenario=scenario,
                    planning_problem=planning_problem
                )

    def test_from_lanelet_network(self):
        for filename in self.scenarios:
            scenario_path = self.scenario_root / filename
            scenario, planning_problem_set = CommonRoadFileReader(f"{scenario_path}.xml").open()
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
            with self.subTest(msg=f"Testing scenario: {filename} with generation from scenario", filename=filename):
                fast_api.generate_reference_path_from_lanelet_network_and_planning_problem(
                    lanelet_network=scenario.lanelet_network,
                    planning_problem=planning_problem
                )


