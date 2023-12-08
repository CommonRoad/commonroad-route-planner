import numpy as np

# third party
from scipy.spatial.kdtree import KDTree

# own code base
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops



# typing
from typing import List
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_route_planner.route import Route
    from commonroad.scenario.scenario import Scenario
    from commonroad.scenario.lanelet import LaneletNetwork
    from commonroad.planning.planning_problem import PlanningProblem



class RouteSlice:
    """
    Slice of a route given a point
    """

    # TODO: maybe inherite from route???



    def __init__(self, route: "Route",
                x: float, y: float,
                distance_ahead_in_m: float = 30,
                distance_behind_in_m: float = 7):

        # original route
        self.original_route: "Route" = route
        self.scenario: "Scenario" = self.original_route.scenario
        self.lanelet_network: "LaneletNetwork" = self.original_route.lanelet_network
        self.planning_problem: "PlanningProblem" = self.original_route.planning_problem
        self.lanelet_ids: List[int] = self.original_route.lanelet_ids

        # query point
        self.x: float = x
        self.y: float = y

        # distance along reference path ahead and behind point
        self.distance_ahead_in_m: float = distance_ahead_in_m
        self.distance_behind_in_m: float = distance_behind_in_m

        # sliced reference path
        self.reference_path: np.ndarray = None
        self.point_idx_query: int = None
        self.point_idx_ahead: int = None
        self.point_idx_behind: int = None

        self._init_route_slice_from_position()


        # save additional information about sliced reference path
        self.interpoint_distances: np.ndarray = pops.compute_interpoint_distances_from_polyline(self.reference_path)
        self.length_reference_path: float = pops.compute_length_of_polyline(self.reference_path)
        self.path_orientation: np.ndarray = pops.compute_orientation_from_polyline(self.reference_path)
        self.path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(self.reference_path)



    def _init_route_slice_from_position(self) -> None:
        """
        Finds, the closest point on the reference path and returns slice of the reference
        path around that point with the distance ahead and behind.
        """
        point: np.ndarray = np.asarray([self.x, self.y], float)
        _, point_idx = KDTree(self.original_route.reference_path).query(point)

        running_distance: float = 0
        self.point_idx_ahead: int = point_idx
        for idx in range(point_idx + 1, self.original_route.reference_path.shape[0] - 1):
            running_distance += abs(self.original_route.interpoint_distances[idx])
            self.point_idx_ahead = idx
            if (running_distance >= self.distance_ahead_in_m):
                break

        running_distance = 0
        self.point_idx_behind = point_idx
        for idx in reversed(range(0, point_idx - 1)):
            running_distance += abs(self.original_route.interpoint_distances[idx])
            self.point_idx_behind = idx
            if (running_distance >= self.distance_behind_in_m):
                break

        self.reference_path = self.original_route.reference_path[self.point_idx_behind:self.point_idx_ahead, :]

        if(self.reference_path is None or len(self.reference_path) == 0):
            raise ValueError(f'Could not slice reference path={self.reference_path}')

