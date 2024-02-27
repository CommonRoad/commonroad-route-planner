import math

import numpy as np

from shapely import LineString, Point, Polygon


from typing import List, Dict, Tuple, Union

class LineSegmentLin:

    def __init__(self,
                 start_point_cart: np.ndarray,
                 end_point_cart: np.ndarray,
                 ) -> None:


        self.start_point_cart: np.ndarray = start_point_cart
        self.end_point_cart: np.ndarray = end_point_cart


        self.slope: float = None
        self.offset: float = None
        self._init_linear_eq_params()





    def convert_point_cartesian_curvilinear_segment(self,
                                                    x: float,
                                                    y: float,
                                                    theta_rad: float=0.0,
                                                    precision: int = 5,
                                                    ) -> Tuple[np.ndarray, float]:

        # project point on line
        projected_point: np.ndarray = self.project_point_on_line(x, y)

        # get length on segment abscissa
        s_segment: float = math.sqrt((projected_point[0] - self.start_point_cart[0])**2
                                     + (projected_point[1] - self.start_point_cart[1])**2)

        # get orthogonal value
        n_segment: float = math.sqrt((x - projected_point[0])**2
                                     + (y - projected_point[1])**2)

        # get orientation
        slope_point: float = math.tan(theta_rad)
        theta_rad_segment: float = round(math.atan2(self.slope - slope_point, 1 + self.slope * slope_point), precision)

        return (np.asarray([s_segment, n_segment]), theta_rad_segment)





    def project_point_on_line(self,
                              x:float,
                              y:float
                              ) -> np.ndarray:
        """
        Projects point on line

        :param x: cartesian x value
        :param y: cartesian y  value

        :return: x,y tuple of projected point
        """

        # calc equation of orthogonal
        orth_slope: float = self._calc_orth_slope()
        orth_offset: float = y - orth_slope * x


        x_proj: float = (orth_offset - self.offset) / (self.slope - orth_slope)
        y_proj: float = orth_slope * x_proj + orth_offset

        return np.asarray([x_proj, y_proj])



    def _calc_orth_slope(self) -> float:
        """
        Calcs slope of orthogonal
        """
        # TODO: special case slope = 0
        return -1 / self.slope


    def _init_linear_eq_params(self) -> None:
        """
        Calculate linear equation params given 2 points using y=ax+b
        """

        # TODO handle special cases

        # slope
        self.slope: Union[float, np.ndarray] = ((self.end_point_cart[1] - self.start_point_cart[1]) /
                                                (self.end_point_cart[0] - self.start_point_cart[0]))

        # offset
        self.offset = self.start_point_cart[1] - (self.slope) * self.start_point_cart[0]







if __name__ == "__main__":
    start_point = np.asarray([0, 0])
    end_point = np.asarray([2, 2])
    line_segment = LineSegmentLin(start_point, end_point)
    point, theta = line_segment.convert_point_cartesian_curvilinear_segment(
        x=1,
        y=1.1,
        theta_rad=math.pi/4
    )

    print(point, theta)

    x = 3


