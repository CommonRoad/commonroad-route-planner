import copy
from logging import Logger
import math

import numpy as np


# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

# own code base
from commonroad_route_planner.utility.route_util import chaikins_corner_cutting
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    MethodInterface,
    LaneChangeMethod,
)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad.scenario.scenario import Lanelet

# from commonroad_route_planner.utility.visualization import plot_clcs_line_with_projection_domain

from typing import List


class LaneChangeHandler:
    """
    Handles one specific lane change
    """

    def __init__(
        self,
        lanelet_start: Lanelet,
        lanelet_end: Lanelet,
        lanelet_section: LaneletSection,
        lanelet_network: LaneletNetwork,
        route_lanelet_ids: List[int],
        clcs_extension: float = 50.0,
        logger: Logger = None,
    ) -> None:
        """
        :param lanelet_start: start lanelet of lane change in lane section
        :param lanelet_end: end lanelet of lane change
        :param lanelet_network: lanelet network of scenario
        :param route_lanelet_ids: lanelet ids of entire route
        :param clcs_extension: how long the curvilinear coordinate system should be extended in
            longitudinal direction for lane changes
        :param logger: logger
        """

        self._logger: Logger = logger if (logger is not None) else Logger(__name__)

        self._lanelet_start: Lanelet = lanelet_start
        self._lanelet_end: Lanelet = lanelet_end

        self._lanelet_section: LaneletSection = lanelet_section
        self._lanelet_network: LaneletNetwork = lanelet_network
        self._route_lanelet_ids: List[int] = route_lanelet_ids

        self._clcs: CurvilinearCoordinateSystem = None
        self._clcs_extension: float = clcs_extension
        self._init_clcs()

        self._method_interface: MethodInterface = MethodInterface(logger=self._logger)

    @property
    def lanelet_start(self) -> Lanelet:
        """
        :return: start lanelet of lane change
        """
        return self._lanelet_start

    @property
    def lanelet_end(self) -> Lanelet:
        """
        :return: end lanelet of lane change
        """
        return self._lanelet_end

    @property
    def lanelet_section(self) -> LaneletSection:
        """
        :return: lanelet section of parallel lanelets with same direction
        """
        return self._lanelet_section

    @property
    def clcs_extension(self) -> float:
        """
        :return: extension of curvilinear coordinate system along longitudinal axis
        """
        return self._clcs_extension

    @property
    def clcs(self) -> CurvilinearCoordinateSystem:
        """
        :return: curvilinear coordinate system of lane change
        """
        return self._clcs

    @property
    def method_interface(self) -> MethodInterface:
        """
        :return: method interface for lane change
        """
        return self._method_interface

    def compute_lane_change(
        self,
        sample_step_size: float = 1.0,
        initial_state: InitialState = None,
        goal_region: GoalRegion = None,
        method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE,
    ) -> np.ndarray:
        """
        Computes simple lane change

        :param sample_step_size: step size for resampling
        :param initial_state: cr initials state. If given, checks whether lane change has to go through it
        :param goal_region: cr goal region. If given, checks whether lane change has to go through it
        :param method: lane change method

        :return: lane change portion of reference path
        """

        # Algorithm
        # Constructs curvilinear frame around center line of first lanelet of lane change.
        # uses selected lane change method to construct lane change path
        # if goal and/or start are within lane change, use parts of end/start lane outside them

        start_point: np.ndarray = self._define_start_point_of_lane_change_in_cvl(
            initial_state=initial_state
        )

        end_point: np.ndarray = self._define_end_point_of_lance_change_in_cvl(
            goal_region=goal_region
        )

        ref_path_curv: np.ndarray = self._method_interface.compute_lane_change_ref_path(
            start_point=start_point, end_point=end_point, method=method
        )

        reference_path: np.ndarray = (
            self._clcs.convert_list_of_points_to_cartesian_coords(ref_path_curv, 4)
        )

        reference_path: np.ndarray = self._add_start_portion_of_lanelet(
            reference_path=reference_path, start_point=start_point
        )

        reference_path: np.ndarray = self._add_end_portion_of_lanelet(
            reference_path=reference_path, end_point=end_point
        )

        reference_path = pops.resample_polyline(reference_path, step=sample_step_size)

        return reference_path

    def _define_end_point_of_lance_change_in_cvl(
        self, goal_region: GoalRegion
    ) -> np.ndarray:
        """
        Defines start point of lane change given the initial state.

        :param goal_region: cr goal region

        :return: (2,) np array as end point of lane change in curvilinear coords
        """

        if goal_region is not None:
            if hasattr(goal_region.state_list[0].position, "center"):
                goal_mid_position: np.ndarray = goal_region.state_list[
                    0
                ].position.center
            else:
                # For uncertain position route planner takes first polygon
                goal_mid_position: np.ndarray = (
                    goal_region.state_list[0].position.shapes[0].center
                )

            goal_lanelet_ids: List[int] = (
                self._lanelet_network.find_lanelet_by_position([goal_mid_position])[0]
            )

            if set(goal_lanelet_ids) & set(self._lanelet_section.adjacent_lanelet_ids):
                end_point: np.ndarray = self._clcs.convert_to_curvilinear_coords(
                    goal_mid_position[0], goal_mid_position[1]
                )
            else:
                end_point: np.ndarray = self._clcs.convert_to_curvilinear_coords(
                    self._lanelet_end.center_vertices[-1, 0],
                    self._lanelet_end.center_vertices[-1, 1],
                )

        else:
            end_point: np.ndarray = self._clcs.convert_to_curvilinear_coords(
                self._lanelet_end.center_vertices[-1, 0],
                self._lanelet_end.center_vertices[-1, 1],
            )

        return end_point

    def _define_start_point_of_lane_change_in_cvl(
        self, initial_state: InitialState
    ) -> np.ndarray:
        """
        Defines start point of lane change given the initial state.

        :param initial_state: cr initial state

        :return: (2,) np array as start point of lane change in curvilinear coords
        """

        if initial_state is not None:
            initial_state_ids: List[int] = (
                self._lanelet_network.find_lanelet_by_position(
                    [initial_state.position]
                )[0]
            )
            if set(initial_state_ids) & set(self._lanelet_section.adjacent_lanelet_ids):
                start_point = self._clcs.convert_to_curvilinear_coords(
                    initial_state.position[0],
                    initial_state.position[1],
                )
            else:
                start_point: np.ndarray = self._clcs.convert_to_curvilinear_coords(
                    self._lanelet_start.center_vertices[0, 0],
                    self._lanelet_start.center_vertices[0, 1],
                )

        else:
            start_point: np.ndarray = self._clcs.convert_to_curvilinear_coords(
                self._lanelet_start.center_vertices[0, 0],
                self._lanelet_start.center_vertices[0, 1],
            )

        return start_point

    def _add_start_portion_of_lanelet(
        self, reference_path: np.ndarray, start_point: np.ndarray
    ) -> np.ndarray:
        """
        Adds start portion to lanelet, if reference path start after lanelet

        :param reference_path: current reference path as (n,2) np array in curvilinear frame
        :param start point: start point of lane change

        :return: modified reference path as (n,2) np array
        """

        # start lanelet
        start_lanelet_curv: np.ndarray = np.asarray(
            self._clcs.convert_list_of_points_to_curvilinear_coords(
                self._lanelet_start.center_vertices, 4
            )
        )

        start_lanelet_curv: np.ndarray = start_lanelet_curv[
            start_lanelet_curv[:, 0] < start_point[0], :
        ]

        if start_lanelet_curv.shape[0] > 0:
            start_lanelet_cart: np.ndarray = np.asarray(
                self._clcs.convert_list_of_points_to_cartesian_coords(
                    start_lanelet_curv, 4
                )
            )

            reference_path: np.ndarray = np.concatenate(
                (start_lanelet_cart, reference_path), axis=0
            )

        return reference_path

    def _add_end_portion_of_lanelet(
        self, reference_path: np.ndarray, end_point: np.ndarray
    ) -> np.ndarray:
        """
        Adds end portion to lanelet, if reference path ends before lanelet

        :param reference_path: current reference path as (n,2) np array in curvilinear frame
        :param end_point: end point of lane change

        :return: modified reference path as (n,2) np array
        """

        end_lanelet_curv: np.ndarray = np.asarray(
            self._clcs.convert_list_of_points_to_curvilinear_coords(
                self._lanelet_end.center_vertices, 4
            )
        )

        end_lanelet_curv: np.ndarray = end_lanelet_curv[
            end_lanelet_curv[:, 0] > end_point[0], :
        ]

        if end_lanelet_curv.shape[0] > 0:
            end_lanelet_cart: np.ndarray = np.asarray(
                self._clcs.convert_list_of_points_to_cartesian_coords(
                    end_lanelet_curv, 4
                )
            )
            reference_path: np.ndarray = np.concatenate(
                (reference_path, end_lanelet_cart), axis=0
            )

        return reference_path

    def _init_clcs(self) -> None:
        """
        initis curvilinear coordinate system
        """

        # construct _clcs abscissa around center of first lanelet of lane change
        clcs_line: np.ndarray = copy.copy(self._lanelet_start.center_vertices)

        # get distance between first two points and extrapolate start
        point_0: np.ndarray = clcs_line[0, :]
        point_1: np.ndarray = clcs_line[1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(self._clcs_extension / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray(
                [point_0[0] - idx * delta_x, point_0[1] - idx * delta_y]
            )
            clcs_line: np.ndarray = np.vstack((new_point, clcs_line))

        # get distance between last two points and extrapolate end
        point_0: np.ndarray = clcs_line[-2, :]
        point_1: np.ndarray = clcs_line[-1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(self._clcs_extension / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray(
                [point_1[0] + idx * delta_x, point_1[1] + idx * delta_y]
            )
            clcs_line: np.ndarray = np.vstack((clcs_line, new_point))

        # smooth and resample
        clcs_line = pops.remove_duplicate_points(clcs_line)
        clcs_line = chaikins_corner_cutting(clcs_line, num_refinements=8)
        clcs_line = pops.resample_polyline(clcs_line, step=1)

        self._clcs = CurvilinearCoordinateSystem(clcs_line, 1000, 0.1)

        # plot_clcs_line_with_projection_domain(clcs_line, self._clcs)
