import numpy as np

# own code base
from commonroad_route_planner.curvilinear_coords.line_segment_trafo import LineSegmentTrafo

# typing
from typing import List, Dict, Tuple, Set


class CurvilinearCoordinateSystem:
    """
    Class that generates a curvilinear coordinate frame along a polyline.
    Useful here so that there is no dependency of the drivability checker.
    """

    def __init__(self, reference_path: np.ndarray) -> None:
        self.reference_path: np.ndarray = reference_path


        self.line_segments: List[LineSegmentTrafo] = None




    def _init_line_segments(self) -> None:


        for idx in range(self.reference_path.shape[0] - 1):
            line_segment: LineSegmentTrafo = LineSegmentTrafo(
                start_point_cart=self.reference_path[idx],
                end_point_cart=self.reference_path[idx + 1]
            )
            self.line_segments.append(line_segment)





