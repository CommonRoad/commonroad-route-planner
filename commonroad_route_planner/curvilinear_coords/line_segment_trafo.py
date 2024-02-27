import math

import numpy as np


# own code base
from commonroad_route_planner.curvilinear_coords.vector import Vector2D


from typing import List, Dict, Tuple, Union

class LineSegmentTrafo:

    def __init__(self,
                 start_point_cart: np.ndarray,
                 end_point_cart: np.ndarray,
                 ) -> None:
        """
        Line Segment computed with TraFo Matrix

        :param start_point_cart: cartesian start point in line segment
        :param end_point_cart: cartesian end point in line segment
        """

        self._start_point_cart: np.ndarray = start_point_cart
        self._end_point_cart: np.ndarray = end_point_cart


        self._vector2d: Vector2D = Vector2D(
            p_start=self._start_point_cart,
            p_end=end_point_cart
        )
        self._angle: float = self._vector2d.angle_to_x


        self._transformation_matrix: np.matrix = None
        self._invers_trafo_matrix: np.matrix = None
        self._init_trafo_matrix()


    @property
    def start_point_cart(self) -> np.ndarray:
        """
        :return: (2,) cartesian start point as np array
        """
        return self._start_point_cart

    @property
    def end_point_cart(self) -> np.ndarray:
        """
        :return: (2,) cartesian end point as np arras
        """
        return self._end_point_cart

    @property
    def vector2d(self) -> Vector2D:
        """
        :return: Vector2D of line segment
        """
        return self._vector2d

    @property
    def angle(self) -> float:
        """
        :return: angle relative to x axis
        """
        return self._angle



    def convert_point_cartesian_curvilinear_segment(self,
                                                    x: float,
                                                    y: float,
                                                    ) -> np.ndarray:
        """
        converts point cartesian. Note that p_lon value is only relative to the line's beginning and
        not the total value

        :param x: x cartesian
        :param y: y cartesian
        """
        # transform point
        calc: np.ndarray = np.asarray(self._invers_trafo_matrix @ np.array([x, y, 1]))
        transformed_point: np.ndarray = np.asarray([calc[0][0], calc[0][1]])

        return transformed_point


    def convert_cartesian_orientation_to_curvilinear_segment(self,
                                                             theta_rad: float,
                                                             precision: int = 5) -> float:
        """
        Convert orientation to
        """
        # theta
        theta_rad_segment: float = round(theta_rad - self._vector2d.angle_to_x, precision)
        return theta_rad_segment



    def _init_trafo_matrix(self) -> None:
        """
        Calculate linear equation params given 2 points using y=ax+b
        """

        # rotation matrix -> np.ndarray is generated from column vectors, so each sub-list is a row
        self._transformation_matrix: np.matrix = np.matrix(
            np.array(
                [
                    [np.cos(self._angle), -np.sin(self._angle), self._start_point_cart[0]],
                    [np.sin(self._angle), np.cos(self._angle), self._start_point_cart[0]],
                    [0, 0, 1]
                 ]
            )
        )

        self._invers_trafo_matrix: np.matrix = np.linalg.inv(self._transformation_matrix)







if __name__ == "__main__":
    start_point = np.asarray([0, 0])
    end_point = np.asarray([2, 2])
    line_segment = LineSegmentTrafo(start_point, end_point)
    converted_point: np.ndarray = line_segment.convert_point_cartesian_curvilinear_segment(
        x=1,
        y=3
    )

    converted_orientation: float = line_segment.convert_cartesian_orientation_to_curvilinear_segment(theta_rad=np.pi/2    )

    print(converted_point, converted_orientation)



