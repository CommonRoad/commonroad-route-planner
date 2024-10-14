# commonrad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario

# Own Code base
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_route_planner.lanelet_sequence import LaneletSequence

# typing
from typing import List


def generate_reference_path_from_scenario_and_planning_problem(
    scenario: Scenario, planning_problem: PlanningProblem
) -> ReferencePath:
    """
    Generates reference path from scenario and planning problem.
    :param scenario: CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    :return: CommonRoad reference path
    """
    return generate_reference_path_from_lanelet_network_and_planning_problem(
        lanelet_network=scenario.lanelet_network, planning_problem=planning_problem
    )


def generate_reference_path_from_lanelet_network_and_planning_problem(
    lanelet_network: LaneletNetwork, planning_problem: PlanningProblem
) -> ReferencePath:
    """
    Generates reference path from scenario and planning problem.
    :param lanelet_network: CommonRoad lanelet network
    :param planning_problem: CommonRoad planning problem
    :return: CommonRoad reference path
    """
    route_planner = RoutePlanner(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
    )

    routes: List[LaneletSequence] = route_planner.plan_routes()

    ref_path_planner: ReferencePathPlanner = ReferencePathPlanner(
        lanelet_network=lanelet_network,
        planning_problem=planning_problem,
        routes=routes,
    )

    return ref_path_planner.plan_shortest_reference_path(
        retrieve_shortest=True, consider_least_lance_changes=True
    )
