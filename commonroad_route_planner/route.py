from enum import Enum
from dataclasses import dataclass

import numpy as np


# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState

# own code base
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.frenet_tools.route_slice import RouteSlice
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)


# typing
from typing import List, Union


class RouteType(Enum):
    # Survival routes have no specific goal lanelet
    REGULAR = "regular"
    SURVIVAL = "survival"


@dataclass
class Route:
    """
    A route in a commonroad scenario.
    """

    lanelet_network: LaneletNetwork
    initial_state: InitialState
    goal_region: GoalRegion

    # a route is created given the list of lanelet ids from start to goal
    lanelet_ids: List[int]

    # a section is a list of lanelet ids that are adjacent to a lanelet in the route
    sections: List[LaneletSection]

    prohibited_lanelet_ids: List[int]

    lane_change_method: LaneChangeMethod

    # generate reference path from the list of lanelet ids leading to goal
    reference_path: np.ndarray
    num_lane_change_actions: int

    interpoint_distances: np.ndarray
    average_interpoint_distance: float
    path_length_per_point: np.ndarray
    length_reference_path: float
    path_orientation: np.ndarray
    path_curvature: np.ndarray

    def get_route_slice_from_position(
        self,
        x: float,
        y: float,
        distance_ahead_in_m: float = 30,
        distance_behind_in_m: float = 7,
    ) -> RouteSlice:
        """
        Takes an x and y coordinate, finds, the closest point on the reference path and returns slice of the reference
        path around that point with the distance ahead and behind.

        :param x: x-position
        :param y: y-position
        :param distance_ahead_in_m: how long the path should continue in front of position
        :param distance_behind_in_m: how long the path should continue after position

        :return: route slice
        """
        return RouteSlice(
            self,
            x,
            y,
            distance_ahead_in_m=distance_ahead_in_m,
            distance_behind_in_m=distance_behind_in_m,
        )

    def get_lanelet_section(self, lanelet_id: int) -> Union[LaneletSection, None]:
        """
        Takes lanelet id and retrieves lanelet section

        :param lanelet_id: lanelet id for which the lanelet section should be returned

        :return: lanelet section or none
        """
        if lanelet_id not in self.lanelet_ids:
            raise ValueError("Lanelet id not part of route")

        return LaneletSection.get_section_by_lanelet_id(lanelet_id)
