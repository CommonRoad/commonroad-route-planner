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


class RouteSlice:
    """
    Slice of a route given a point
    """

    def __init__(
        self,
        route: "Route",
        x: float,
        y: float,
        distance_ahead_in_m: float = 30.0,
        distance_behind_in_m: float = 7.0,
    ) -> None:
        """
        :param route: route object the slice is from
        :param x: x value of point slice is made from
        :param x: y value of point slice is made from
        :param distance_ahead_in_m: how many meters ahead of current vehicle position slice should end
        :param distance_behind_in_m: how many meters behind the vehicle the slice should end
        """

        # original route
        self._original_route: "Route" = route

        # query point
        self._x: float = x
        self._y: float = y

        # distance along reference path ahead and behind point
        self._distance_ahead_in_m: float = distance_ahead_in_m
        self._distance_behind_in_m: float = distance_behind_in_m

        # sliced reference path
        self._reference_path: np.ndarray = None
        self._point_idx_query: int = None
        self._point_idx_ahead: int = None
        self._point_idx_behind: int = None

        self._init_route_slice_from_position()

        # lanelet ids
        self._lanelet_ids: List[int] = list()
        # self._init_lanelet_ids()

        # save additional information about sliced reference path
        self._interpoint_distances: np.ndarray = (
            pops.compute_interpoint_distances_from_polyline(self._reference_path)
        )
        self._average_interpoint_distance: float = np.mean(
            self._interpoint_distances, axis=0
        )
        self._path_length_per_point: np.ndarray = pops.compute_path_length_per_point(
            self._reference_path
        )
        self._length_reference_path: float = pops.compute_length_of_polyline(
            self._reference_path
        )
        self._path_orientation: np.ndarray = pops.compute_orientation_from_polyline(
            self._reference_path
        )
        self._path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(
            self._reference_path
        )

    @property
    def original_route(self) -> "Route":
        """
        :return: original route the slice is based of
        """
        return self._original_route

    @property
    def vehicle_point(self) -> List[float]:
        """
        :return: List[x,y] of original point
        """
        return [self._x, self._y]

    @property
    def lanelet_ids(self) -> List[int]:
        """
        :return: list of lanelet ids
        """
        return self._lanelet_ids

    @property
    def distance_ahead_in_m(self) -> float:
        """
        :return: distance ahead from original vehicle point
        """
        return self._distance_ahead_in_m

    @property
    def distance_behind_in_m(self) -> float:
        """
        :return: distance behind from original vehicle point
        """
        return self._distance_behind_in_m

    @property
    def reference_path(self) -> np.ndarray:
        """
        :return (n,2) np ndarray of points of ref path
        """
        return self._reference_path

    @property
    def interpoint_distances(self) -> np.ndarray:
        """
        :return: (n,1) distance between points
        """
        return self._interpoint_distances

    @property
    def average_interpoint_distance(self) -> float:
        """
        :return: average interpoint distance of route
        """
        return self._average_interpoint_distance

    @property
    def length_reference_path(self) -> float:
        """
        :return: total length of reference path
        """
        return self._length_reference_path

    @property
    def path_length_per_point(self) -> np.ndarray:
        """
        :return: (n,1) np ndarray of path length for each point
        """
        return self._path_length_per_point

    @property
    def path_orientation(self) -> np.ndarray:
        """
        :return: (n,1) per point orientation values in rad
        """
        return self._path_orientation

    @property
    def path_curvature(self) -> np.ndarray:
        """
        :return: (n,1) per point curvature of reference path
        """
        return self._path_curvature

    def _init_route_slice_from_position(self) -> None:
        """
        Finds, the closest point on the reference path and returns slice of the reference
        path around that point with the distance ahead and behind.
        """
        point: np.ndarray = np.asarray([self._x, self._y], float)
        _, point_idx = KDTree(self._original_route.reference_path).query(point)

        running_distance: float = 0
        self._point_idx_ahead: int = point_idx
        for idx in range(
            point_idx + 1, self._original_route.reference_path.shape[0] - 1
        ):
            running_distance += abs(self._original_route.interpoint_distances[idx])
            self._point_idx_ahead = idx
            if running_distance >= self._distance_ahead_in_m:
                break

        running_distance = 0
        self._point_idx_behind = point_idx
        for idx in reversed(range(0, point_idx - 1)):
            running_distance += abs(self._original_route.interpoint_distances[idx])
            self._point_idx_behind = idx
            if running_distance >= self._distance_behind_in_m:
                break

        self._reference_path = self._original_route.reference_path[
            self._point_idx_behind : self._point_idx_ahead, :
        ]

        if self._reference_path is None or len(self._reference_path) == 0:
            raise ValueError(f"Could not slice reference path={self._reference_path}")

    def _init_lanelet_ids(self) -> None:
        """
        Checks the value of first and last point of original route and adds all lanelet ids between them
        """

        # Find index of first and last point of slice ref path and corresponding lanelet ids
        behind_point: np.ndarray = self._reference_path[0, :]
        infront_point: np.ndarray = self._reference_path[-1, :]
        behind_lanelet_id: int = (
            self._original_route.lanelet_network.find_lanelet_by_position(
                [behind_point]
            )[0][0]
        )
        infront_lanelet_id: int = (
            self._original_route.lanelet_network.find_lanelet_by_position(
                [infront_point]
            )[0][0]
        )

        # As the lanelet ids are ordered in the route class we know that every id between behind and front also
        # has to be part of the route slice
        behind_idx: int = self.original_route.lanelet_ids.index(behind_lanelet_id)
        infront_idx: int = self.original_route.lanelet_ids.index(infront_lanelet_id)

        self._lanelet_ids.append(behind_idx)
        self._lanelet_ids.append(infront_idx)

        for idx in range(behind_idx, infront_idx):
            self._lanelet_ids.append(self.original_route.lanelet_ids[idx])

        # remove duplicates
        self._lanelet_ids: List[int] = list(set(self._lanelet_ids))
