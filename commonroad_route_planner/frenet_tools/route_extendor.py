__author__ = "Tobias Mascetta"
__copyright__ = ""
__credits__ = [""]
__version__ = "2022.3"
__maintainer__ = "Tobias Mascetta, Gerald Wuersching"
__email__ = "tobias.mascetta@tum.de"
__status__ = "Release"

import copy
import math
import warnings

import numpy as np


# commonroad
from commonroad.scenario.lanelet import Lanelet

# Own code base
from commonroad_route_planner.route import Route
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import (
    DefaultGenerationStrategy,
)

# typing
from typing import List, Union

#  _logger = logging.getLogger(__name__)


class RouteExtendor:
    """
    Extends a given route.
    """

    def __init__(
        self,
        route: Route,
        extrapolation_length: float = 5.0,
        RouteGenerationMethod: Union[
            DefaultGenerationStrategy
        ] = DefaultGenerationStrategy,
    ) -> None:
        self.route: Route = route
        # additional_lenght_in_meters
        self.extrapolation_length: float = extrapolation_length

        self.RouteGenerationMethod: Union[DefaultGenerationStrategy] = (
            RouteGenerationMethod
        )

    def _perform_no_successor_extension(self) -> None:
        """
        Performs extension for no successor at end
        """

        reference_path = copy.copy(self.route.reference_path)

        # get distance between first two points to know what the pseudo-uniform sampling would be
        point_0: np.ndarray = reference_path[-2, :]
        point_1: np.ndarray = reference_path[-1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(self.extrapolation_length / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray(
                [point_1[0] + idx * delta_x, point_1[1] + idx * delta_y]
            )
            reference_path: np.ndarray = np.vstack((reference_path, new_point))

        self.route: Route = self.RouteGenerationMethod.update_route(
            route=self.route, reference_path=reference_path
        )

    def _perform_successor_extension(self, successor_ids: List[int]) -> None:
        """
        Use successor road of end lanelet for extension
        """
        if len(successor_ids) > 1:
            # Maybe better Algorithm
            # ---------
            # 1. Extrapolate last two points
            # 2. Check on which lanelets most of the extrapolated points go
            # 2.B Edge Case T-Junction -> No points --> Choose random
            # 2.B Tie-Break: Chose longer, chose random lanenelet
            # 3. Chose that lanelet
            if len(successor_ids) > 1:
                warnings.warn(
                    "Current lane has more than one successor, choosing first"
                )

        successor_id = successor_ids[0]
        successor_lanelet: Lanelet = self.route.lanelet_network.find_lanelet_by_id(
            successor_id
        )
        centerline: np.ndarray = pops.sample_polyline(
            successor_lanelet.center_vertices,
            step=self.route.average_interpoint_distance,
        )
        reference_path: np.ndarray = np.concatenate(
            (self.route.reference_path, centerline), axis=0
        )

        # Resample polyline for better distance
        self.route: Route = self.RouteGenerationMethod.update_route(
            route=self.route, reference_path=reference_path
        )

    def _perform_predecessor_extension(self, predecessor_ids: List[int]):
        """
        Use successor road of end lanelet for extension
        """
        if len(predecessor_ids) > 1:
            warnings.warn("Current lane has more than one predecessor, choosing first")
        predecessor_lanelet: Lanelet = self.route.lanelet_network.find_lanelet_by_id(
            predecessor_ids[0]
        )
        centerline: np.ndarray = pops.sample_polyline(
            predecessor_lanelet.center_vertices,
            step=self.route.average_interpoint_distance,
        )
        reference_path: np.ndarray = np.concatenate(
            (centerline, self.route.reference_path), axis=0
        )

        # Resample polyline for better distance
        self.route: Route = self.RouteGenerationMethod.update_route(
            route=self.route, reference_path=reference_path
        )

    def _perform_no_predecessor_extension(self) -> None:
        """
        performs extension for no predecessor at start
        """
        reference_path = copy.copy(self.route.reference_path)
        # get distance between first two points to know what the pseudo-uniform sampling would be
        point_0: np.ndarray = reference_path[0, :]
        point_1: np.ndarray = reference_path[1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(self.extrapolation_length / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray(
                [point_0[0] - idx * delta_x, point_0[1] - idx * delta_y]
            )
            reference_path: np.ndarray = np.vstack((new_point, reference_path))

        self.route: Route = self.RouteGenerationMethod.update_route(
            route=self.route, reference_path=reference_path
        )

    def extend_reference_path_at_end(self) -> None:
        """
        Adds additional points at the end along a line between first two points of reference path.
        Returns new reference path and success indicator

        Reasoning
        ----------
        Otherwise, an occuring edge-case would be that
        the rear axel of the vehicle is after the last point of the ref path, which makes
        Frenet-Localization problematic.

        If the closest point to the final position is the last point of the reference path and their
        distance is below a certain threshold (currently infinity), additional points are being placed
        """

        # check if there is a successor lane.
        last_lanelet: Lanelet = self.route.lanelet_network.find_lanelet_by_id(
            self.route.lanelet_ids[-1]
        )
        successor_ids: List[int] = last_lanelet.successor
        if len(successor_ids) == 0:
            self._perform_no_successor_extension()
        elif len(successor_ids) == 1:
            self._perform_successor_extension(successor_ids)

        else:
            self._perform_no_successor_extension()

    def extend_reference_path_at_start(self) -> None:
        """
        Adds additional points at the beginning of the route
        """

        # check if there is a successor lane.
        first_lanelet: Lanelet = self.route.lanelet_network.find_lanelet_by_id(
            self.route.lanelet_ids[0]
        )
        predecessor_ids: List[int] = first_lanelet.predecessor
        if len(predecessor_ids) == 0:
            self._perform_no_predecessor_extension()

        elif len(predecessor_ids) == 1:
            self._perform_predecessor_extension(predecessor_ids)

        else:
            self._perform_no_predecessor_extension()

    def extend_reference_path_at_start_and_end(self) -> None:
        """
        Extends both at start and end
        """
        self.extend_reference_path_at_end()

        self.extend_reference_path_at_start()

    def get_route(self) -> Route:
        """
        So students know that this is useful....
        """
        return self.route
