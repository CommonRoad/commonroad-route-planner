import copy
from pathlib import Path

import unittest


import numpy as np
# commonrad
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import GoalRegion, PlanningProblem
from commonroad.scenario.state import CustomState, InitialState
from commonroad.geometry.shape import Rectangle

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.utility.visualization import visualize_route
from commonroad_route_planner.frenet_tools.route_extendor import RouteExtendor
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import LaneChangeMethod
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import DefaultGenerationStrategy
from commonroad_route_planner.utility.exceptions import PointNotOnGoalLaneletException


# typing
from typing import List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.route_candidate_holder import RouteGenerator
    from commonroad_route_planner.route import Route



class UpdatePlanningProblemTest(unittest.TestCase):

    def test_update_pp(self):

        # ========== initialization =========== #
        path_scenario = Path(__file__).parents[1] / "scenarios"  / "DEU_Stu-1_49_I-1.xml"

        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        route_planner = RoutePlanner(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem,
            extended_search=False,
        )
        route_generator: "RouteGenerator" = route_planner.plan_routes(
            lane_change_method=LaneChangeMethod.QUINTIC_SPLINE, GenerationStrategy=DefaultGenerationStrategy
        )
        route: "Route" = route_generator.retrieve_shortest_route()

        # new planning problem
        new_is: InitialState = copy.copy(planning_problem.initial_state)
        region = Rectangle(
            length=2,
            width=2,
            center=np.asarray([-66, 276]),
            orientation=0,
        )

        goal_state: CustomState = CustomState(
            position=region,
            time_step=copy.deepcopy(planning_problem.goal.state_list[0].time_step),
        )

        new_goal = GoalRegion(
            state_list=[goal_state],
            lanelets_of_goal_position={0: [20]}
        )

        new_pp: PlanningProblem = PlanningProblem(
            planning_problem_id=20000,
            initial_state=new_is,
            goal_region=new_goal
        )

        new_rg: "RouteGenerator" = route_planner.update_planning_problem_and_plan_routes(
            planning_problem=new_pp
        )

        new_route: "Route" = new_rg.retrieve_shortest_route()

        ids_old_pp: List[int] = scenario.lanelet_network.find_lanelet_by_position([route.reference_path[-5]])[0]
        ids_new_pp: List[int] = scenario.lanelet_network.find_lanelet_by_position([new_route.reference_path[-5]])[0]

        if 23 not in ids_old_pp:
            raise PointNotOnGoalLaneletException(
                planning_problem_id=planning_problem.planning_problem_id,
                goal_lanelet_id=23
            )

        if 20 not in ids_new_pp:
            raise PointNotOnGoalLaneletException(
                planning_problem_id=new_pp.planning_problem_id,
                goal_lanelet_id=20
            )
