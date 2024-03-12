from enum import Enum
from logging import Logger

import numpy as np


# own code base
from commonroad_route_planner.lane_changing.lane_change_methods.polynomial_change import (
    generate_cubic_spline_ref_path,
    generate_quintic_spline_ref_path,
)


class LaneChangeMethod(Enum):
    CUBIC_SPLINE = 1
    QUINTIC_SPLINE = 2


class MethodInterface:
    """
    Interface for the different lane changing methods.
    """

    def __init__(self, logger: Logger = None) -> None:
        """
        :param logger: Logger
        """

        self._logger = logger if (logger is not None) else Logger(__name__)

    def compute_lane_change_ref_path(
        self,
        start_point: np.ndarray,
        end_point: np.ndarray,
        method=LaneChangeMethod.QUINTIC_SPLINE,
        step_size=0.1,
    ) -> np.ndarray:
        """
        Computes cubic lane change

        :param start_point: start point of cubic interpolation
        :param end_point: end point of cubic interpolation
        :param method: method of lance change
        :param step_size: size between points of reference path

        :return: (n,2) path
        """

        if method == LaneChangeMethod.CUBIC_SPLINE:
            reference_path: np.ndarray = self._compute_cubic_spline_lane_change(
                start_point=start_point, end_point=end_point, step_size=step_size
            )
        elif method == LaneChangeMethod.QUINTIC_SPLINE:
            reference_path: np.ndarray = self._compute_quintic_spline_lane_change(
                start_point=start_point, end_point=end_point, step_size=step_size
            )

        else:
            self._logger.error(f"Method {method} not implemented")
            raise NotImplementedError(f"Method {method} not implemented")

        return reference_path

    def _compute_cubic_spline_lane_change(
        self, start_point: np.ndarray, end_point: np.ndarray, step_size=0.1
    ) -> np.ndarray:
        """
        Computes cubic lane change

        :param start_point: start point of cubic interpolation
        :param end_point: end point of cubic interpolation
        :param step_size: size between points of reference path

        :return: (n,2) path
        """

        ref_path: np.ndarray = generate_cubic_spline_ref_path(
            start_point=start_point, end_point=end_point, step_size=step_size
        )
        self._logger.info("Computed cubic lane change")

        return ref_path

    def _compute_quintic_spline_lane_change(
        self, start_point: np.ndarray, end_point: np.ndarray, step_size=0.1
    ) -> np.ndarray:
        """
        Computes cubic lane change

        :param start_point: start point of cubic interpolation
        :param end_point: end point of cubic interpolation
        :param step_size: size between points of reference path

        :return: (n,2) path
        """

        ref_path: np.ndarray = generate_quintic_spline_ref_path(
            start_point=start_point, end_point=end_point, step_size=step_size
        )
        self._logger.info("Computed quintic lane change")

        return ref_path
