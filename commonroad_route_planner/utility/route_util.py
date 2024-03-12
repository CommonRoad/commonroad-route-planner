import warnings
import numpy as np


# commonroad
from commonroad.geometry.shape import Rectangle, Shape
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork

# Typing
from typing import List


def relative_orientation(angle_1: float, angle_2: float) -> float:
    """
    Computes the angle between two angles.

    :param angle_1: first angle in rad
    :param angle_2: second angle in rad

    :return: orientation in rad
    """
    phi = (angle_2 - angle_1) % (2 * np.pi)
    if phi > np.pi:
        phi -= 2 * np.pi

    return phi


def chaikins_corner_cutting(
    polyline: np.ndarray, num_refinements: int = 4
) -> np.ndarray:
    """Chaikin's corner cutting algorithm.

    Chaikin's corner cutting algorithm smooths a polyline by replacing each original point with two new points.
    The new points are at 1/4 and 3/4 along the way of an edge.

    :param polyline: polyline with 2D points
    :param num_refinements: how many times to apply the chaikins corner cutting algorithm. setting to 6 is smooth enough
                            for most cases
    :return: smoothed polyline
    """
    for _ in range(num_refinements):
        L = polyline.repeat(2, axis=0)
        R = np.empty_like(L)
        R[0] = L[0]
        R[2::2] = L[1:-1:2]
        R[1:-1:2] = L[2::2]
        R[-1] = L[-1]
        polyline = L * 0.75 + R * 0.25

    return polyline


def lanelet_orientation_at_position(lanelet: Lanelet, position: np.ndarray) -> float:
    """Approximates the lanelet orientation with the two closest point to the given state

    :param lanelet: Lanelet on which the orientation at the given state should be calculated
    :param position: Position where the lanelet's orientation should be calculated
    :return: An orientation in interval [-pi,pi]
    """
    center_vertices = lanelet.center_vertices

    position_diff = []
    for idx in range(len(center_vertices) - 1):
        vertex1 = center_vertices[idx]
        position_diff.append(np.linalg.norm(position - vertex1))

    closest_vertex_index = position_diff.index(min(position_diff))

    vertex1 = center_vertices[closest_vertex_index, :]
    vertex2 = center_vertices[closest_vertex_index + 1, :]
    direction_vector = vertex2 - vertex1
    return np.arctan2(direction_vector[1], direction_vector[0])


def sort_lanelet_ids_by_orientation(
    list_ids_lanelets: List[int],
    orientation: float,
    position: np.ndarray,
    lanelet_network: LaneletNetwork,
) -> List[int]:
    """Returns the lanelets sorted by relative orientation to the given position and orientation."""

    if len(list_ids_lanelets) <= 1:
        return list_ids_lanelets
    else:
        lanelet_id_list = np.array(list_ids_lanelets)

        def get_lanelet_relative_orientation(lanelet_id):
            lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
            lanelet_orientation = lanelet_orientation_at_position(lanelet, position)
            return np.abs(relative_orientation(lanelet_orientation, orientation))

        orientation_differences = np.array(
            list(map(get_lanelet_relative_orientation, lanelet_id_list))
        )
        sorted_indices = np.argsort(orientation_differences)
        return list(lanelet_id_list[sorted_indices])


def sort_lanelet_ids_by_goal(
    lanelet_network: LaneletNetwork, goal_region: GoalRegion
) -> List[int]:
    """Sorts lanelet ids by goal region

    :return: lanelet id, if the obstacle is out of lanelet boundary (no lanelet is found, therefore return the
    lanelet id of last time step)
    """
    if (
        hasattr(goal_region, "lanelets_of_goal_position")
        and goal_region.lanelets_of_goal_position is not None
    ):
        goal_lanelet_id_batch_list = list(
            goal_region.lanelets_of_goal_position.values()
        )
        goal_lanelet_id_list = [
            item for sublist in goal_lanelet_id_batch_list for item in sublist
        ]
        goal_lanelet_id_set = set(goal_lanelet_id_list)
        goal_lanelets = [
            lanelet_network.find_lanelet_by_id(goal_lanelet_id)
            for goal_lanelet_id in goal_lanelet_id_list
        ]
        goal_lanelets_with_successor = np.array(
            [
                (
                    1.0
                    if len(
                        set(goal_lanelet.successor).intersection(goal_lanelet_id_set)
                    )
                    > 0
                    else 0.0
                )
                for goal_lanelet in goal_lanelets
            ]
        )

        return [
            x
            for _, x in sorted(zip(goal_lanelets_with_successor, goal_lanelet_id_list))
        ]

    if goal_region.state_list is not None and len(goal_region.state_list) != 0:
        if len(goal_region.state_list) > 1:
            raise ValueError("More than one goal state is not supported yet!")
        goal_state = goal_region.state_list[0]

        if hasattr(goal_state, "orientation"):
            goal_orientation: float = (
                goal_state.orientation.start + goal_state.orientation.end
            ) / 2

        else:
            goal_orientation = 0.0
            warnings.warn(
                "The goal state has no <orientation> attribute! It is set to 0.0"
            )

        if hasattr(goal_state, "position"):
            goal_shape: Shape = goal_state.position

        else:
            goal_shape: Shape = Rectangle(length=0.01, width=0.01)

        # the goal shape has always a shapley object -> because it is a rectangle
        # noinspection PyUnresolvedReferences
        return sort_lanelet_ids_by_orientation(
            lanelet_network.find_lanelet_by_shape(goal_shape),
            goal_orientation,
            goal_shape.shapely_object.centroid.coords,
            lanelet_network,
        )

    raise NotImplementedError("Whole lanelet as goal must be implemented here!")
