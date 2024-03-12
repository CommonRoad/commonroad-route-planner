import logging

# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState


# own code base
from commonroad_route_planner.route_generator import RouteGenerator
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)


# typing
from typing import List


class RouteCandidateHolder(RouteGenerator):
    """
    For Legacy reasons.
    """

    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        initial_state: InitialState,
        goal_region: GoalRegion,
        route_candidates: List[List[int]],
        prohibited_lanelet_ids: List[int],
        logger: logging.Logger,
        lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE,
    ) -> None:

        super().__init__(
            lanelet_network=lanelet_network,
            initial_state=initial_state,
            goal_region=goal_region,
            route_candidates=route_candidates,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            logger=logger,
            lane_change_method=lane_change_method,
        )
