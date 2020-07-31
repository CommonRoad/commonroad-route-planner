import logging
import os
import sys
import warnings
from datetime import datetime
from enum import Enum
from typing import List, Union, Generator, Set, Tuple

import networkx as nx
import numpy as np
from commonroad.geometry.shape import Shape
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State

from commonroad_route_planner.priority_queue import PriorityQueue

try:
    import pycrccosy
    from commonroad_ccosy.geometry.util import resample_polyline
    import commonroad.geometry.shape as cr_shape
    from shapely.ops import cascaded_union
    from shapely.geometry import LineString, Point, Polygon
except ModuleNotFoundError as exp:
    warnings.warn(f"You won't be able to use the ccosy for the Navigator, the calculations won't be precise. {exp}")


class _LaneletNode:
    def __init__(self, laneletID: int, lanelet: Lanelet, cost: float, current_length: int):
        # it must be id because of the implementation of the priority queue
        self.id = laneletID
        self.lanelet = lanelet
        self.parent_node = None
        self.cost = cost
        self.count = current_length

    def __lt__(self, other):
        return self.cost < other.cost


def relative_orientation(from_angle1_in_rad, to_angle2_in_rad):
    phi = (to_angle2_in_rad - from_angle1_in_rad) % (2 * np.pi)
    if phi > np.pi:
        phi -= (2 * np.pi)

    return phi


def lanelet_orientation_at_position(lanelet: Lanelet, position: np.ndarray):
    """
    Approximates the lanelet orientation with the two closest point to the given state
    # TODO optimize more for speed

    :param lanelet: Lanelet on which the orientation at the given state should be calculated
    :param state: State where the lanelet's orientation should be calculated
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


def sorted_lanelet_ids(lanelet_ids: List[int], orientation: float, position: np.ndarray, scenario: Scenario)\
        -> List[int]:
    """
    return the lanelets sorted by relative orientation to the position and orientation given
    """

    if len(lanelet_ids) <= 1:
        return lanelet_ids
    else:
        lanelet_id_list = np.array(lanelet_ids)

        def get_lanelet_relative_orientation(lanelet_id):
            lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            lanelet_orientation = lanelet_orientation_at_position(lanelet, position)
            return np.abs(relative_orientation(lanelet_orientation, orientation))

        orientation_differences = np.array(list(map(get_lanelet_relative_orientation, lanelet_id_list)))
        sorted_indices = np.argsort(orientation_differences)
        return list(lanelet_id_list[sorted_indices])


def sorted_lanelet_ids_by_goal(scenario: Scenario, goal: GoalRegion) -> List[int]:
    """
    Get the lanelet id of the goal
    :param goal:
    :param scenario: commonroad scenario
    :return: lanelet id, if the obstacle is out of lanelet boundary (no lanelet is found, therefore return the
    lanelet id of last time step)
    """
    if hasattr(goal, 'lanelets_of_goal_position') and goal.lanelets_of_goal_position is not None:
        goal_lanelet_id_batch_list = list(goal.lanelets_of_goal_position.values())
        goal_lanelet_id_list = [item for sublist in goal_lanelet_id_batch_list for item in sublist]
        goal_lanelet_id_set = set(goal_lanelet_id_list)
        goal_lanelets = [scenario.lanelet_network.find_lanelet_by_id(goal_lanelet_id) for goal_lanelet_id in
                         goal_lanelet_id_list]
        goal_lanelets_with_successor = np.array(
            [1.0 if len(set(goal_lanelet.successor).intersection(goal_lanelet_id_set)) > 0 else 0.0 for goal_lanelet
             in goal_lanelets])
        return [x for _, x in sorted(zip(goal_lanelets_with_successor, goal_lanelet_id_list))]
    if goal.state_list is not None and len(goal.state_list) != 0:
        if len(goal.state_list) > 1:
            raise ValueError("More than one goal state is not supported yet!")
        goal_state = goal.state_list[0]
        goal_orientation: float = (goal_state.orientation.start + goal_state.orientation.end) / 2
        goal_shape: Shape = goal_state.position
        return sorted_lanelet_ids(
            scenario.lanelet_network.find_lanelet_by_shape(goal_shape),
            goal_orientation,
            np.array(goal_shape.shapely_object.centroid),
            scenario
        )

    raise NotImplementedError("Whole lanelet as goal must be implemented here!")


class RouteType(Enum):
    UNIQUE = "unique"
    SURVIVAL = "survival"


class Route:

    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem, route: List[int],
                 route_type: RouteType, allowed_lanelet_ids: Set[int] = None):
        self.scenario = scenario
        self.lanelet_network = scenario.lanelet_network
        self.planning_problem = planning_problem
        self.route = route
        self.type = route_type

        self._sectionized_environment = None

        # ==================== #
        #        Extra         #
        # ==================== #
        if allowed_lanelet_ids is None:
            self.allowed_lanelet_ids = {lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets}
        else:
            self.allowed_lanelet_ids = allowed_lanelet_ids

        self.lanelet_ids_in_the_opposite_direction = set()

    def get_navigator(self):
        return Navigator(route=self)

    def _get_adjacent_lanelets_list(self, lanelet_id: int, is_opposite_direction_allowed=False) -> list:
        """
        Recursively gets adj_left and adj_right lanelets of current lanelet id
        :param lanelet_id: current lanelet id
        :param is_opposite_direction_allowed: specifies if it should give back only the lanelets in the driving
            direction or it should give back the first neighbouring lanelet in the opposite direction
        :return: list of adjacent lanelet ids: all lanelets which are going in the same direction and one-one from the
                 left and right side which are going in the opposite direction, empty lists if there are none
        """
        adjacent_list = list()
        base_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)

        # left direction
        current_lanelet = base_lanelet
        temp_id = current_lanelet.adj_left
        while temp_id is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if temp_id in self.allowed_lanelet_ids:
                if current_lanelet.adj_left_same_direction:
                    if temp_id in self.allowed_lanelet_ids:
                        # append the left adjacent lanelet if it exists
                        adjacent_list.append(temp_id)

                        # Update current_lanelet
                        current_lanelet = self.lanelet_network.find_lanelet_by_id(temp_id)
                        temp_id = current_lanelet.adj_left

                # this lanelet was already such which goes in the opposite direction -> exit the loop
                else:
                    self.lanelet_ids_in_the_opposite_direction.add(temp_id)
                    if is_opposite_direction_allowed:
                        adjacent_list.append(temp_id)
                    break
            else:
                # it is not allowed to drive in that lane, so just breaking
                break

        # right direction
        current_lanelet = base_lanelet
        temp_id = current_lanelet.adj_right
        while temp_id is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if temp_id in self.allowed_lanelet_ids:
                if current_lanelet.adj_right_same_direction:
                    # append the right adjacent lanelet if it exists
                    adjacent_list.append(temp_id)

                    # Update current_lanelet
                    current_lanelet = self.lanelet_network.find_lanelet_by_id(temp_id)
                    temp_id = current_lanelet.adj_right
                # this lanelet was already such which goes in the opposite direction -> exit the loop
                else:
                    self.lanelet_ids_in_the_opposite_direction.add(temp_id)
                    if is_opposite_direction_allowed:
                        adjacent_list.append(temp_id)
                    break
            else:
                # it is not allowed to drive in that lane, so just breaking
                break

        return adjacent_list

    def _get_sectionized_environment_from_route(self, route: List[int], is_opposite_direction_allowed: bool = False)\
            -> Union[None, List[List[int]]]:
        """
        Creates sectionized environment from a given route.
        :param route: The route as a list of lanelet ids
        :param is_opposite_direction_allowed: Indicates whether it is required to contain one opposite lanelet in the
                                              environment
        :return: Returns a sectional environment in a form of list of list of lanelet ids. This environment contains
                 the environment of the planned route in every time steps.
        """
        if route is None:
            return None

        sections = list()
        for lanelet_id_in_route in route:
            lanelet_ids_in_section = self._get_adjacent_lanelets_list(lanelet_id_in_route,
                                                                      is_opposite_direction_allowed)
            lanelet_ids_in_section.append(lanelet_id_in_route)
            lanelet_ids_in_section.sort()

            if len(sections) == 0:
                sections.append(lanelet_ids_in_section)
            elif sections[-1] != lanelet_ids_in_section:
                sections.append(lanelet_ids_in_section)

        return sections

    def get_sectionized_environment(self, is_opposite_direction_allowed: bool = False):
        if self._sectionized_environment is None:
            self._sectionized_environment = \
               self._get_sectionized_environment_from_route(self.route,
                                                            is_opposite_direction_allowed=is_opposite_direction_allowed)

        return self._sectionized_environment


class RouteCandidates:
    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem, route_candidates: List[List[int]],
                 route_type: RouteType, allowed_lanelet_ids: Set[int] = None):
        self.scenario = scenario
        self.lanelet_network = self.scenario.lanelet_network
        self.planning_problem = planning_problem
        self.route_candidates = route_candidates
        self.route_type = route_type

        # ==================== #
        #        Extra         #
        # ==================== #
        if allowed_lanelet_ids is None:
            self.allowed_lanelet_ids = {lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets}
        else:
            self.allowed_lanelet_ids = allowed_lanelet_ids

    def get_first_route(self) -> Route:
        route = self.route_candidates[0]
        return Route(self.scenario, self.planning_problem, route, self.route_type, self.allowed_lanelet_ids)

    def get_most_likely_route_by_orientation(self) -> Union[Route, None]:
        # handling the survival scenarios and where only one path found
        if len(self.route_candidates) == 1:
            return Route(self.scenario, self.planning_problem, self.route_candidates[0], self.route_type,
                         self.allowed_lanelet_ids)

        current_state = self.planning_problem.initial_state
        sorted_initial_lanelet_ids = sorted_lanelet_ids(
            self.scenario.lanelet_network.find_lanelet_by_position([current_state.position])[0],
            current_state.orientation,
            current_state.position,
            self.scenario
        )
        sorted_goal_lanelet_ids = sorted_lanelet_ids_by_goal(self.scenario, self.planning_problem.goal)

        candidates_goal_lanelet_ids = np.array([route_candidate[-1] for route_candidate in self.route_candidates])

        for goal_lanelet_id in sorted_goal_lanelet_ids:
            if goal_lanelet_id in candidates_goal_lanelet_ids:
                # candidates_initial_lanelet_ids = [route_candidate[0] for route_candidate in self.route_candidates if
                #                                   route_candidate[-1] == goal_lanelet_id else None]
                candidates_initial_lanelet_ids = np.array(
                    [route_candidate[0] if route_candidate[-1] == goal_lanelet_id else None for route_candidate in
                     self.route_candidates])
                for initial_lanelet_id in sorted_initial_lanelet_ids:
                    if initial_lanelet_id in candidates_initial_lanelet_ids:
                        route = self.route_candidates[
                            np.where(candidates_initial_lanelet_ids == initial_lanelet_id)[0][0]]
                        return Route(self.scenario, self.planning_problem, route, self.route_type,
                                     self.allowed_lanelet_ids)
        return None

    def __repr__(self):
        return f"{len(self.route_candidates)} routeCandidates of scenario {self.scenario.benchmark_id}, " \
               f"planning problem {self.planning_problem.planning_problem_id}"

    def __str__(self):
        return self.__repr__()


class Navigator:
    class Backend(Enum):
        PYCRCCOSY = 'pycrccosy'
        APPROXIMATE = 'approx'

    def __init__(self, route: Route):
        if 'pycrccosy' in sys.modules:
            self.backend = self.Backend.PYCRCCOSY
        else:
            self.backend = self.Backend.APPROXIMATE

        self.scenario = route.scenario
        self.lanelet_network = self.scenario.lanelet_network
        self.planning_problem = route.planning_problem
        self.route = route
        self.sectionized_environment = self.route.get_sectionized_environment()
        self.sectionized_environment_set = set([item for sublist in self.sectionized_environment for item in sublist])
        self.merged_route_lanelets = None
        self.ccosy_list = self._get_route_cosy()
        self.num_of_lane_changes = len(self.ccosy_list)
        self.merged_section_length_list = np.array([self._get_length(curvi_cosy) for curvi_cosy in self.ccosy_list])

        # ==================== #
        #         Goal         #
        # ==================== #
        self.goal_curvi_face_coords = None
        self.goal_curvi_minimal_coord = None
        self._initialize_goal()

    def _initialize_goal(self):
        if self.route.type == RouteType.UNIQUE:
            goal_face_coords = self._get_goal_face_points(self._get_goal_polygon(self.planning_problem.goal))
            self.goal_curvi_face_coords = np.array([(self._get_safe_curvilinear_coords(self.ccosy_list[-1], g))[0]
                                                    for g in goal_face_coords])

            self.goal_min_curvi_coords = np.min(self.goal_curvi_face_coords, axis=0)
            self.goal_max_curvi_coord = np.max(self.goal_curvi_face_coords, axis=0)

    def _get_route_cosy(self) -> Union[pycrccosy.TrapezoidCoordinateSystem, List[Lanelet]]:
        # Merge reference route
        self.merged_route_lanelets = []

        # Append predecessor of the initial to ensure that the goal state is not out of the projection domain
        initial_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.route[0])
        predecessors_lanelet = initial_lanelet.predecessor
        if predecessors_lanelet is not None and len(predecessors_lanelet) != 0:
            predecessor_lanelet = self.lanelet_network.find_lanelet_by_id(predecessors_lanelet[0])
            current_merged_lanelet = predecessor_lanelet
        else:
            current_merged_lanelet = None

        for current_lanelet_id, next_lanelet_id in zip(self.route.route[:-1], self.route.route[1:]):
            lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            # If the lanelet is the end of a section, then change section
            if next_lanelet_id not in lanelet.successor:
                if current_merged_lanelet is not None:
                    self.merged_route_lanelets.append(current_merged_lanelet)
                    current_merged_lanelet = None
            else:
                if current_merged_lanelet is None:
                    current_merged_lanelet = lanelet
                else:
                    current_merged_lanelet = Lanelet.merge_lanelets(current_merged_lanelet, lanelet)

        goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.route[-1])
        if current_merged_lanelet is not None:
            current_merged_lanelet = Lanelet.merge_lanelets(current_merged_lanelet, goal_lanelet)
        else:
            current_merged_lanelet = goal_lanelet

        # Append successor of the goal to ensure that the goal state is not out of the projection domain
        goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.route[-1])
        successors_of_goal = goal_lanelet.successor
        if successors_of_goal is not None and len(successors_of_goal) != 0:
            successor_lanelet = self.lanelet_network.find_lanelet_by_id(successors_of_goal[0])
            current_merged_lanelet = Lanelet.merge_lanelets(current_merged_lanelet, successor_lanelet)

        self.merged_route_lanelets.append(current_merged_lanelet)

        if self.backend == self.Backend.APPROXIMATE:
            # If ccosy is not installed using temporary method for approximate calculations
            return self.merged_route_lanelets
        else:
            return [self._create_coordinate_system_from_polyline(merged_lanelet.center_vertices)
                    for merged_lanelet in self.merged_route_lanelets]

    @staticmethod
    def _create_coordinate_system_from_polyline(polyline):
        if len(polyline) <= 4:
            last_point = polyline[-1]
            length = np.linalg.norm(polyline[-1] - polyline[0])
            polyline = resample_polyline(polyline, length / 4.0)
            # make sure that the original vertices are all contained
            if not np.all(np.isclose(polyline[-1], last_point)):
                polyline = np.append(polyline, [last_point], axis=0)
        return pycrccosy.TrapezoidCoordinateSystem(polyline)

    def _get_length(self, ccosy):
        if self.backend == self.Backend.PYCRCCOSY:
            return ccosy.get_length()
        else:
            lanelet = ccosy
            return lanelet.distance[-1]

    def _get_safe_curvilinear_coords(self, ccosy, position: np.ndarray):
        try:
            rel_pos_to_domain = 0
            return self._get_curvilinear_coords(ccosy, position), rel_pos_to_domain
        except ValueError:
            return self._project_out_of_domain(ccosy, position)

    def _project_out_of_domain(self, ccosy, position: np.ndarray):
        if self.backend == self.Backend.PYCRCCOSY:

            segment_list = ccosy.get_segment_list()
            bounding_segments = [segment_list[0], segment_list[-1]]

            rel_positions = position - np.array([segment.pt_1 for segment in bounding_segments])
            distances = np.linalg.norm(rel_positions, axis=1)

            if distances[0] < distances[1]:
                rel_pos_to_domain = -1
                nearest_idx = 0
                long_dist = 0
            else:
                rel_pos_to_domain = 1
                nearest_idx = 1
                long_dist = ccosy.get_length()

            nearest_segment = bounding_segments[nearest_idx]
            rel_position = rel_positions[nearest_idx]

            long_dist = long_dist + np.dot(nearest_segment.tangent, rel_position)
            lat_dist = np.dot(nearest_segment.normal, rel_position)

        else:
            lanelet = ccosy

            bounding_center_vertices = np.array([lanelet.center_vertices[0], lanelet.center_vertices[-1]])
            rel_positions = position - bounding_center_vertices
            distances = np.linalg.norm(rel_positions, axis=1)

            if distances[0] < distances[1]:
                rel_pos_to_domain = -1
                long_dist = 0
                nearest_vertex = bounding_center_vertices[0, :]
                normal_vector = lanelet.left_vertices[0] - nearest_vertex
                tangent_vector = lanelet.center_vertices[1] - nearest_vertex
                rel_position = rel_positions[0]
            else:
                rel_pos_to_domain = 1
                long_dist = lanelet.distance[-1]
                nearest_vertex = bounding_center_vertices[1, :]
                normal_vector = lanelet.left_vertices[-1] - nearest_vertex
                tangent_vector = lanelet.center_vertices[-2] - nearest_vertex
                rel_position = rel_positions[1]

            long_dist = long_dist + np.dot(normal_vector, rel_position)
            lat_dist = np.dot(tangent_vector, rel_position)

        return np.array([long_dist, lat_dist]), rel_pos_to_domain

    def _get_curvilinear_coords(self, ccosy, position: np.ndarray):
        if self.backend == self.Backend.PYCRCCOSY:
            return ccosy.convert_to_curvilinear_coords(position[0], position[1])
        else:
            lanelet = ccosy
            position_diffs = np.linalg.norm(position - lanelet.center_vertices, axis=1)
            closest_vertex_index = np.argmin(position_diffs)

            if closest_vertex_index < len(lanelet.center_vertices) - 1:
                direction_vector_pos_1 = lanelet.center_vertices[closest_vertex_index]
                direction_vector_pos_2 = lanelet.center_vertices[closest_vertex_index + 1]
            else:
                direction_vector_pos_1 = lanelet.center_vertices[closest_vertex_index - 1]
                direction_vector_pos_2 = lanelet.center_vertices[closest_vertex_index]

            direction_vector = direction_vector_pos_2 - direction_vector_pos_1
            relative_pos_to_point_1 = position - direction_vector_pos_1
            relative_pos_to_point_2 = position - direction_vector_pos_2

            if closest_vertex_index == len(lanelet.center_vertices) - 1 and np.dot(direction_vector,
                                                                                   relative_pos_to_point_2) > 0:
                raise ValueError("Position is out of projection domain")

            relative_lanelet_side = np.sign(np.cross(direction_vector, relative_pos_to_point_1))

            long_distance = lanelet.distance[closest_vertex_index]
            lat_distance = relative_lanelet_side * position_diffs[closest_vertex_index]
            return long_distance, lat_distance

    def _get_curvilinear_coords_over_lanelet(self, lanelet: Lanelet, position):
        if self.backend == self.Backend.PYCRCCOSY:
            current_ccosy = self._create_coordinate_system_from_polyline(lanelet.center_vertices)
            return self._get_curvilinear_coords(current_ccosy, position)
        else:
            return self._get_curvilinear_coords(lanelet, position)

    @staticmethod
    def _get_goal_face_points(goal_shape: Polygon):
        """
        Extracts the middle points of each face of the goal region
        NOTE in pathological examples, this can still result in points outside the coordinate system
        however for the points on both ends of the lanelet, they coincide with the center vertices,
        which is what the curvilinear coordinate system is based on
        NOTE if the goal areas edges are not all intersecting with any lanelet in the ccosy, this operation will fail
        :param goal_shape: shape of the goal area
        :return: tuples of x,y coordinates of the middle points of each face of the goal region
        """
        assert isinstance(goal_shape, Polygon), f"Only single Polygon is supported, but {type(goal_shape)} was given," \
                                                f" Use a planning problem with contiguous goal region"

        goal_coords = [np.array(x) for x in zip(*goal_shape.exterior.coords.xy)]

        # round the same precision as is done within the commonroad xml files
        goal_coords = [np.round((a + b) / 2, 6) for a, b in zip(goal_coords, goal_coords[1:])]
        return goal_coords

    def _get_goal_polygon(self, goal: GoalRegion) -> Polygon:
        """
        Get the goal position as Polygon
        :param goal: the goal given as a GoalRegion
        :return: Polygon of the goal position
        """

        def get_polygon_list_from_shapegroup(shapegroup: cr_shape.ShapeGroup) -> List[Polygon]:
            """
            Converts cr_shape.ShapeGroup to list of Polygons
            :param shapegroup: the ShapeGroup to be converted
            :return: The list of the polygons
            """

            polygon_list = []
            for shape in shapegroup.shapes:
                if isinstance(shape, cr_shape.ShapeGroup):
                    polygon_list.append(get_polygon_list_from_shapegroup(shape))
                elif isinstance(shape, (cr_shape.Rectangle, cr_shape.Polygon)):
                    polygon_list.append(shape.shapely_object)
                else:
                    raise ValueError(f"Shape can't be converted to Shapely Polygon: {shape}")
            return polygon_list

        def merge_polygons(polygons_to_merge):
            return cascaded_union([
                geom if geom.is_valid else geom.buffer(0) for geom in polygons_to_merge
            ])

        polygons = [Polygon()]
        for goal_state in goal.state_list:
            if hasattr(goal_state, 'position'):
                if isinstance(goal_state.position, cr_shape.ShapeGroup):
                    polygons.extend(get_polygon_list_from_shapegroup(goal_state.position))
                elif isinstance(goal_state.position, (cr_shape.Rectangle, cr_shape.Polygon)):
                    polygons.append(goal_state.position.shapely_object)
                else:
                    raise NotImplementedError(
                        f"Goal position not supported yet, "
                        f"only ShapeGroup, Rectangle or Polygon shapes can be used, "
                        f"the given shape was: {type(goal_state.position)}")

        merged_polygon = merge_polygons(polygons)
        return merged_polygon

    def get_state_curvi_coords(self, ego_vehicle_state_position: np.ndarray):
        for cosy_idx, curvi_cosy in enumerate(self.ccosy_list):

            ego_curvi_coords, rel_pos_to_domain = self._get_safe_curvilinear_coords(curvi_cosy,
                                                                                    ego_vehicle_state_position)

            is_last_section = (cosy_idx == self.num_of_lane_changes - 1)
            if rel_pos_to_domain == 1 and not is_last_section:
                continue

            return ego_curvi_coords, cosy_idx

        raise ValueError("Unable to project the ego vehicle on the global cosy")

    def get_long_lat_distance_to_goal(self, ego_vehicle_state_position: np.ndarray) -> Tuple[float, float]:
        """
        Get the longitudinal and latitudinal distance from the ego vehicle to the goal.
        :param ego_vehicle_state_position: position of the ego vehicle
        :return: longitudinal distance, latitudinal distance
        """
        # If the route is survival, then return zero
        if self.route.type == RouteType.SURVIVAL:
            return 0.0, 0.0

        ego_curvi_coords, cosy_idx = self.get_state_curvi_coords(ego_vehicle_state_position)

        is_last_section = (cosy_idx == self.num_of_lane_changes - 1)

        if is_last_section:
            relative_distances = self.goal_curvi_face_coords - ego_curvi_coords
            min_distance = np.min(relative_distances, axis=0)
            max_distance = np.max(relative_distances, axis=0)

            (min_distance_long, min_distance_lat) = np.maximum(np.minimum(0.0, max_distance), min_distance)
        else:
            min_distance_long = self.merged_section_length_list[cosy_idx] - ego_curvi_coords[0]
            current_section_idx = cosy_idx + 1
            while current_section_idx != self.num_of_lane_changes - 1:
                min_distance_long += self.merged_section_length_list[current_section_idx]
                current_section_idx += 1

            relative_lat_distances = self.goal_curvi_face_coords[:, 1] - ego_curvi_coords[1]
            min_distance = np.min(relative_lat_distances, axis=0)
            max_distance = np.max(relative_lat_distances, axis=0)

            min_distance_long += self.goal_min_curvi_coords[0]
            min_distance_lat = np.maximum(np.minimum(0.0, max_distance), min_distance)

        return min_distance_long, min_distance_lat

    def get_long_lat_distance_to_goal_center(self, ego_vehicle_state_position: np.ndarray) -> Tuple[float, float]:
        """
        Get the longitudinal and latitudinal distance from the ego vehicle to the goal.
        :param ego_vehicle_state_position: position of the ego vehicle
        :return: longitudinal distance, latitudinal distance
        """
        # If the route is survival, then return zero
        if self.route.type == RouteType.SURVIVAL:
            return 0.0, 0.0

        for cosy_idx, curvi_cosy in enumerate(self.ccosy_list):

            ego_curvi_coords, rel_pos_to_domain = self._get_safe_curvilinear_coords(curvi_cosy,
                                                                                    ego_vehicle_state_position)

            is_last_section = (cosy_idx == self.num_of_lane_changes - 1)
            if rel_pos_to_domain == 1 and not is_last_section:
                continue

            if is_last_section:
                relative_distances = self.goal_curvi_face_coords - ego_curvi_coords
                min_distance = np.min(relative_distances, axis=0)
                max_distance = np.max(relative_distances, axis=0)

                (min_distance_long, min_distance_lat) = np.maximum(np.minimum(0.0, max_distance), min_distance)
            else:
                min_distance_long = self.merged_section_length_list[cosy_idx] - ego_curvi_coords[0]
                current_section_idx = cosy_idx + 1
                while current_section_idx != self.num_of_lane_changes - 1:
                    min_distance_long += self.merged_section_length_list[current_section_idx]
                    current_section_idx += 1

                relative_lat_distances = self.goal_curvi_face_coords[:, 1] - ego_curvi_coords[1]
                min_distance = np.min(relative_lat_distances, axis=0)
                max_distance = np.max(relative_lat_distances, axis=0)

                min_distance_long += self.goal_min_curvi_coords[0]
                min_distance_lat = np.maximum(np.minimum(0.0, max_distance), min_distance)

            return min_distance_long, min_distance_lat

        raise ValueError("Unable to project the ego vehicle on the global cosy")

    def get_lane_change_distance(self, state: State, active_lanelets: List[int] = None) -> float:
        # If the route is survival, then return zero
        if self.route.type == RouteType.SURVIVAL:
            return 0.0
        if active_lanelets is None:
            active_lanelets = self.scenario.lanelet_network.find_lanelet_by_position([state.position])[0]

        current_lanelet_ids_on_route = [
            current_lanelet_id
            for current_lanelet_id in active_lanelets
            if current_lanelet_id in self.sectionized_environment_set
        ]
        # The state is not on the route, instant lane change is required
        if len(current_lanelet_ids_on_route) == 0:
            return 0.0

        sorted_current_lanelet_ids_on_route = sorted_lanelet_ids(
            current_lanelet_ids_on_route,
            state.orientation,
            state.position,
            self.scenario
        )

        # The most likely current lanelet id by considering the orientation of the state
        current_lanelet_id = sorted_current_lanelet_ids_on_route[0]

        distance_until_lane_change = 0.0
        route_successors = {current_lanelet_id}
        while len(route_successors) != 0:
            # Add the length of the current lane
            current_lanelet_id = route_successors.pop()
            current_lanelet = self.lanelet_network.find_lanelet_by_id(current_lanelet_id)
            try:
                if distance_until_lane_change == 0.0:
                    # Calculate the remaining distance in this lanelet
                    current_distance = self._get_curvilinear_coords_over_lanelet(current_lanelet, state.position)
                    current_distance_long = current_distance[0]
                    distance_until_lane_change = current_lanelet.distance[-1] - current_distance_long
                else:
                    distance_until_lane_change += current_lanelet.distance[-1]

            except ValueError:
                pass

            successors_set = set(current_lanelet.successor)
            route_successors = successors_set.intersection(self.sectionized_environment_set)

        return distance_until_lane_change


# ================================================= #
#                   Route Planner                   #
# ================================================= #


class RoutePlanner:
    class Backend(Enum):
        NETWORKX = "networkx"
        NETWORKX_REVERSED = "networkx_reversed"
        PRIORITY_QUEUE = "priority_queue"

    """
    Class implements route planning for the CommonRoad scenarios
    This is a higher level planner to plan only on the lanelet structure.
    It is like plan a route with google maps.
    It gives back the best route (list of lanelet IDs in the right order)
    from each start position to all goal positions.
    If there are no goal position then it is going forward and if it can not go forward then goes right.
    The best route is the route with the lowest cost according to the heuristic function.
    """

    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem,
                 lanelet_type_blacklist=None,
                 allow_diagonal=False, backend=None, log_to_console=True):
        """
        Initializes a RoutePlanner object
        :param scenario: Scenario which should be used for the route planning
        :param planning_problem: PlanningProblem for which the route should be planned
        :param lanelet_type_blacklist: Set of lanelet types which should be avoided during route planning
        :type lanelet_type_blacklist: Set[LaneletType]
        :param allow_diagonal: Indicates whether diagonal movements are allowed - experimental
        :param backend: The backend which should be used, supported choices: networkx, priority_queue
        :param log_to_console: Indicates whether the outputs should be logged to the console
        """

        # ============================== #
        #       Binding variables        #
        # ============================== #
        if lanelet_type_blacklist is None:
            lanelet_type_blacklist = set()

        if backend is None:
            backend = RoutePlanner.Backend.NETWORKX

        self.scenario = scenario
        self.scenario_id = scenario.benchmark_id
        self.lanelet_network = scenario.lanelet_network
        self.planning_problem = planning_problem
        self.lanelet_type_blacklist = lanelet_type_blacklist
        # self.priority_queue = PriorityQueue()
        self.allow_diagonal = allow_diagonal
        self.backend = backend
        # ============================== #
        #        Create Logger           #
        # ============================== #
        self.logger = logging.getLogger("Route Planner [{}]".format(self.scenario_id))
        self._init_logger(log_to_file=False, log_to_console=log_to_console)

        # ================================================= #
        #               Find allowed lanelets               #
        # ================================================= #
        self.allowed_lanelet_ids = {allowed_lanelet.lanelet_id for allowed_lanelet in
                                    self._filter_lanelets_by_type(self.lanelet_network.lanelets,
                                                                  self.lanelet_type_blacklist)}

        # ================================================= #
        #                Check initial state                #
        # ================================================= #
        self.startLanelet_ids = None
        self._check_initial_state()

        # ================================================= #
        #                Check goal position                #
        # ================================================= #
        self.goal_lanelet_ids = None
        self._check_goal_state()

        # ================================================= #
        #        Create graph network from Lanelets         #
        # ================================================= #
        # if there is no goal lanelet ids than it is a survival scenario and we do not need to make
        # a graph from the lanelet network
        if self.goal_lanelet_ids is None:
            self.route_type = RouteType.SURVIVAL
            self.logger.info("SURVIVAL Scenario: There is no goal position or lanelet given")
        else:
            self.route_type = RouteType.UNIQUE
            if self.backend == RoutePlanner.Backend.NETWORKX:
                if self.allow_diagonal:
                    self.logger.warning("diagonal search not tested")
                    self.digraph = self._create_graph_from_lanelet_network_lane_change()
                else:
                    self.digraph = self._create_graph_from_lanelet_network()
            elif self.backend == RoutePlanner.Backend.NETWORKX_REVERSED:
                if self.allow_diagonal:
                    self.logger.warning("diagonal search not tested")
                    self.digraph = self._create_reversed_graph_from_lanelet_network_lane_change()
                else:
                    self.digraph = self._create_reversed_graph_from_lanelet_network()
            elif self.backend == RoutePlanner.Backend.PRIORITY_QUEUE:
                if self.allow_diagonal:
                    self.logger.critical("diagonal search with custom backend is not implemented")
                self.frontier = PriorityQueue()
                self.explored = set()
            else:
                raise ValueError(f"The backend {self.backend} is not recognized as supported backend "
                                 f"for the RoutePlanner")

    # =============== end of constructor ============== #

    @staticmethod
    def _filter_lanelets_by_type(lanelets_to_filter: List[Lanelet], lanelet_type_blacklist: Set[LaneletType]) \
            -> Generator[Lanelet, None, None]:
        """
        Generator filters the lanelets by the defined blacklist

        :param lanelets_to_filter: The list of the lanelets which should be filtered
        :return: List of filtered lanelets
        """
        for lanelet_to_filter in lanelets_to_filter:
            if len(lanelet_to_filter.lanelet_type.intersection(lanelet_type_blacklist)) == 0:
                yield lanelet_to_filter

    def _filter_allowed_lanelet_ids(self, lanelet_ids_to_filter: List[int]) \
            -> Generator[Lanelet, None, None]:
        """
        Generator filters the lanelet ids by the defined blacklist

        :param lanelet_ids_to_filter: The list of the lanelet ids which should be filtered
        :return: List of filtered lanelets
        """
        for lanelet_id_to_filter in lanelet_ids_to_filter:
            if lanelet_id_to_filter in self.allowed_lanelet_ids:
                yield lanelet_id_to_filter

    def _init_logger(self, log_to_console=True, log_to_file=True, add_timestamp_to_log_file=True):
        # path relative to the running script
        log_file_dir = "solutions/logs/scenario_logs"
        log_file_name = "route_planner_result_with_priority_queue_backend"
        # release_logger(self.logger)
        self.logger.setLevel(logging.INFO)
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_formatter = logging.Formatter('%(asctime)s\t\t%(name)s\t%(levelname)s\t%(message)s')
        file_formatter = logging.Formatter('%(asctime)s\t%(name)s\t%(levelname)s\t%(message)s')

        if log_to_console:
            # create console handler
            console_handler = logging.StreamHandler()
            # set the level of logging to console
            console_handler.setLevel(logging.DEBUG)
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)

        if log_to_file:
            date_time_string = ''
            if add_timestamp_to_log_file:
                now = datetime.now()  # current date and time
                date_time_string = now.strftime("_%Y_%m_%d_%H-%M-%S")

            # if directory not exists create it
            os.makedirs(log_file_dir, exist_ok=True)

            log_file_path = os.path.join(log_file_dir,
                                         "{}_{}{}.log".format(self.scenario_id, log_file_name, date_time_string))
            file_handler = logging.FileHandler(log_file_path)
            # set the level of logging to file
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(file_formatter)
            self.logger.addHandler(file_handler)

        self.logger.debug("Using backend: {}".format(self.backend))

    def _check_initial_state(self):
        if hasattr(self.planning_problem.initial_state, 'position'):
            start_position = self.planning_problem.initial_state.position
            # noinspection PyTypeChecker
            all_start_lanelet_ids = self.lanelet_network.find_lanelet_by_position([start_position])[0]
            self.startLanelet_ids = list(self._filter_allowed_lanelet_ids(all_start_lanelet_ids))
            if len(self.startLanelet_ids) > 1:
                self.logger.info("More start lanelet ids - some of it can results in an unsuccessful search")
        else:
            self.logger.critical("There is no start position given")
            raise self._NoSourceLaneletId("There is no start position given")

    def _check_goal_state(self):
        self.goal_lanelet_ids = list()

        if hasattr(self.planning_problem.goal, 'lanelets_of_goal_position'):
            if self.planning_problem.goal.lanelets_of_goal_position is None:
                self.logger.debug("No goal lanelet is given")
            else:
                self.logger.debug("Goal lanelet is given")
                # the goals are in the dict, one goal can consist of multiple lanelets
                # now we just iterating over the goals and adding every ID which we find to
                # the goal_lanelet_ids list
                all_goal_lanelet_ids = list()
                for all_goal_lanelet_id in list(self.planning_problem.goal.lanelets_of_goal_position.values()):
                    all_goal_lanelet_ids.extend(all_goal_lanelet_id)
                self.goal_lanelet_ids.extend(list(self._filter_allowed_lanelet_ids(all_goal_lanelet_ids)))

        if (len(self.goal_lanelet_ids) == 0) and hasattr(self.planning_problem.goal, 'state_list'):
            for idx, state in enumerate(self.planning_problem.goal.state_list):
                if hasattr(state, 'position'):
                    goal_position = state.position
                    # noinspection PyTypeChecker
                    all_goal_state_lanelet_ids = self.lanelet_network.find_lanelet_by_shape(goal_position)
                    goal_state_lanelet_ids = list(self._filter_allowed_lanelet_ids(all_goal_state_lanelet_ids))

                    if len(goal_state_lanelet_ids) != 0:
                        self.goal_lanelet_ids.extend(goal_state_lanelet_ids)
                        self.logger.debug("Goal lanelet IDs estimated from goal shape in state [{}]".format(idx))
                    else:
                        self.logger.debug(
                            "No Goal lanelet IDs could be determined from the goal shape in state [{}]".format(idx))

        # Removing duplicates and reset to none if no lanelet IDs found
        if len(self.goal_lanelet_ids) != 0:
            # remove duplicates and sort in ascending order
            self.goal_lanelet_ids = sorted(list(dict.fromkeys(self.goal_lanelet_ids)))
        else:
            self.goal_lanelet_ids = None

    def _find_survival_route(self, start_lanelet_id: int) -> List:
        """
        Finds a route along the lanelet network with a similar approach like in driving exams.
        Priority:
            1, forward
            2, right

        Notes:
            - it only considers lanes with same driving direction
            - the priority of right should be changed to left in left-rule road networks like in the UK
        :param start_lanelet_id: the initial lanelet where we start from
        :return: route that consists of a list of lanelet IDs
        """
        route = list()

        lanelet = self.lanelet_network.find_lanelet_by_id(start_lanelet_id)

        # TODO: a minimal distance check could be added in a recursive greedy best first search algorithm
        # basically we are always going forward and if we can not go forward the go right
        # this approach is similar to the one in driving exams
        # it goes until the end of the lanelet network or when it is hits itself (like dying in the Snake game)
        while lanelet.lanelet_id not in route:

            route.append(lanelet.lanelet_id)

            if lanelet.successor:
                # naively concatenate successors
                # TODO:
                #  use some rules? eg.:
                #   check which of the successors goes better in the direction of the previous lane and choose that one
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.successor[0])
            # add edge if right lanelet
            elif lanelet.adj_right and lanelet.adj_right_same_direction:
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
            # add edge if left lanelet
            elif lanelet.adj_left and lanelet.adj_left_same_direction:
                break
                # lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
            # break out of the loop
            else:
                break

        return route

    def _create_graph_from_lanelet_network(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network.

        :return: created graph from lanelet network
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id not in self.allowed_lanelet_ids:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelet
            for successor in lanelet.successor:
                if successor not in self.allowed_lanelet_ids:
                    continue
                edges.append((lanelet.lanelet_id, successor, {'weight': lanelet.distance[-1]}))

            # add edge if left lanelet
            adj_left = lanelet.adj_left
            if adj_left and lanelet.adj_left_same_direction and adj_left in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, adj_left, {'weight': 1.0}))

            # add edge if right lanelet
            adj_right = lanelet.adj_right
            if adj_right and lanelet.adj_right_same_direction and adj_right in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, adj_right, {'weight': 1.0}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_reversed_graph_from_lanelet_network(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network.

        :return: created graph from lanelet network
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id not in self.allowed_lanelet_ids:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelet
            for predecessor in lanelet.predecessor:
                if predecessor not in self.allowed_lanelet_ids:
                    continue
                edges.append((lanelet.lanelet_id, predecessor, {'weight': lanelet.distance[-1]}))

            # add edge if left lanelet
            adj_left = lanelet.adj_left
            if adj_left and lanelet.adj_left_same_direction and adj_left in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, adj_left, {'weight': np.inf}))

            # add edge if right lanelet
            adj_right = lanelet.adj_right
            if adj_right and lanelet.adj_right_same_direction and adj_right in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, adj_right, {'weight': np.inf}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_graph_from_lanelet_network_lane_change(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network allowing diagonal lane changes
        TODO: test implementation
        :return: created graph from lanelet network with diagonal lane changes
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()

        # check for each lanelet for diagonal successors
        for lanelet in self.lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)

            # check if lanelet has successor
            # add edge if succeeding lanelet
            for successor in lanelet.successor:
                successor_lanelet = self.lanelet_network.find_lanelet_by_id(successor)
                edges.append((lanelet.lanelet_id, successor_lanelet.lanelet_id, {'weight': lanelet.distance[-1]}))

                # check for diagonal left succeeding lanelet
                if successor_lanelet.adj_left and lanelet.adj_left_same_direction:
                    edges.append((lanelet.lanelet_id, successor_lanelet.adj_right,
                                  {'weight': 0}))

                # check for diagonal right succeeding lanelet
                if successor_lanelet.adj_right and lanelet.adj_right_same_direction:
                    edges.append((lanelet.lanelet_id, successor_lanelet.adj_right,
                                  {'weight': 0}))

            # check if succeeding lanelet of right lanelet (e.g. turning lane highway)
            if lanelet.adj_right and lanelet.adj_right_same_direction:
                l_right = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)

                # check for diagonal right succeeding lanelet
                for right_successor in l_right.successor:

                    # if not already in graph add it
                    if (lanelet.lanelet_id, right_successor, {'weight': 0}) not in edges:
                        edges.append((lanelet.lanelet_id, right_successor, {'weight': 0}))

            # check if succeeding lanelet of right lanelet (e.g. turning lane highway)
            if lanelet.adj_left and lanelet.adj_left_same_direction:
                l_left = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)

                # check for diagonal left succeeding lanelet
                for left_successor in l_left.successor:

                    # if not already in graph add it
                    if (lanelet.lanelet_id, left_successor, {'weight': 0}) not in edges:
                        edges.append((lanelet.lanelet_id, left_successor, {'weight': 0}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_reversed_graph_from_lanelet_network_lane_change(self):
        raise NotImplementedError

    def _find_all_shortest_paths(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        """
         Find all shortest paths using networkx module
         :param source_lanelet_id: ID of source lanelet
         :param target_lanelet_id: ID of goal lanelet
         :return: list of simple paths with lanelet IDs
         """
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")
        if target_lanelet_id is None:
            self.logger.info("SURVIVAL SCENARIO")
            return self._find_survival_route(source_lanelet_id)
        found_paths = list()
        try:
            found_paths = list(nx.all_shortest_paths(self.digraph, source=source_lanelet_id, target=target_lanelet_id,
                                                     weight='weight', method='dijkstra'))
        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self.logger.debug("The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                                     source_lanelet_id))
        return found_paths

    def _find_shortest_reversed_path(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        """
         Find the shortest paths reversed using networkx module
         :param source_lanelet_id: ID of source lanelet
         :param target_lanelet_id: ID of goal lanelet
         :return: list of lanelet IDs
         """
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")
        if target_lanelet_id is None:
            self.logger.info("SURVIVAL SCENARIO")
            return self._find_survival_route(source_lanelet_id)
        found_paths = list()
        try:
            found_paths.append(list(nx.shortest_path(self.digraph, source=target_lanelet_id, target=source_lanelet_id,
                                                     weight='weight', method='dijkstra'))[::-1])
        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self.logger.debug("The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                                     source_lanelet_id))
        return found_paths

    @staticmethod
    def _calc_cost(current: Lanelet) -> float:
        return current.distance[-1]

    @staticmethod
    def _calc_heuristic(current: Lanelet, target: Lanelet) -> float:
        diff = target.center_vertices[0] - current.center_vertices[-1]
        return np.sqrt(np.dot(diff, diff))

    def _add_child(self, parent_node: _LaneletNode, next_lanelet_id: int, target: Lanelet, extra_cost: float = 0.0):
        next_lanelet = self.lanelet_network.find_lanelet_by_id(next_lanelet_id)

        frontier_lanelet_ids = self.frontier.get_item_ids()
        cost = extra_cost + parent_node.cost + self._calc_cost(next_lanelet) + self._calc_heuristic(next_lanelet,
                                                                                                    target)

        node = _LaneletNode(next_lanelet_id, next_lanelet, cost, parent_node.count + 1)
        node.parent_node = parent_node

        # TODO: check speed
        if (next_lanelet_id not in self.explored) and (next_lanelet_id not in frontier_lanelet_ids):
            self.frontier.put(next_lanelet_id, node, cost)
        elif next_lanelet_id in frontier_lanelet_ids:
            self.frontier.update_item_if_exists(next_lanelet_id, node, cost)

    def _find_astar_path(self, source_lanelet_id, target_lanelet_id: int) -> List:

        if source_lanelet_id not in self.allowed_lanelet_ids:
            raise self._NoPathFound("It is not allowed to drive on the source lanelet, check the type of the lanelet")

        if target_lanelet_id not in self.allowed_lanelet_ids:
            raise self._NoPathFound("It is not allowed to drive on the target lanelet, check the type of the lanelet")

        self.frontier = PriorityQueue()
        self.explored = set()

        target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)

        lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet_id)
        node = _LaneletNode(source_lanelet_id, lanelet,
                            self._calc_cost(lanelet) + self._calc_heuristic(lanelet, target_lanelet), 1)

        self.frontier.put(node.id, node, node.cost)
        while not self.frontier.is_empty():

            node: _LaneletNode = self.frontier.pop()

            # maybe the frontier is not empty but only contains invalid elements
            if node is None:
                continue

            if node.id == target_lanelet_id:
                break

            self.explored.add(node.id)

            # ================= #
            #    expand node    #
            # ================= #
            lanelet = node.lanelet
            # add successors
            for successor_id in lanelet.successor:
                if successor_id not in self.allowed_lanelet_ids:
                    continue
                self._add_child(node, successor_id, target_lanelet, node.cost)

            # if we are changing lanelets then remove the lanelet lengths because it would be added twice
            lanelet_length = self._calc_cost(lanelet)

            # add left lanelet
            adj_left_id = lanelet.adj_left
            if lanelet.adj_left_same_direction and adj_left_id and adj_left_id in self.allowed_lanelet_ids:
                self._add_child(node, adj_left_id, target_lanelet, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    left_lanelet_successor_ids = self.lanelet_network.find_lanelet_by_id(adj_left_id).successor
                    for left_lanelet_successor_id in left_lanelet_successor_ids:
                        self._add_child(node, left_lanelet_successor_id, target_lanelet, 0.9)

            # add right lanelet
            adj_right_id = lanelet.adj_right
            if lanelet.adj_right_same_direction and adj_right_id and adj_right_id in self.allowed_lanelet_ids:
                self._add_child(node, adj_right_id, target_lanelet, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    right_lanelet_successor_ids = self.lanelet_network.find_lanelet_by_id(adj_right_id).successor
                    for right_lanelet_successor_id in right_lanelet_successor_ids:
                        self._add_child(node, right_lanelet_successor_id, target_lanelet, 0.9)
        else:
            raise self._NoPathFound(
                "The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                       source_lanelet_id))

        reverse_path = list()

        while node is not None:
            reverse_path.append(node.id)
            node = node.parent_node

        return reverse_path[::-1]

    def _find_path(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        found_paths = list()
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")

        if target_lanelet_id is None:
            self.logger.debug("SURVIVAL SCENARIO")
            found_paths.append(self._find_survival_route(source_lanelet_id))
        else:
            try:
                found_paths.append(self._find_astar_path(source_lanelet_id, target_lanelet_id))
            except self._NoPathFound:
                # it is a normal behaviour because of the overlapping lanelets in a road network
                self.logger.debug(
                    "The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                           source_lanelet_id))
        return found_paths

    def search_alg(self) -> List[List]:
        """
        Find all paths to all of the goal lanelet IDs from all the start lanelet IDs using networkx module.
        If no goal lanelet ID is given then return a survival route
        :return: empty list if no path has been found
        """
        self.logger.info("Route planning started")
        routes = list()
        for start_lanelet_id in self.startLanelet_ids:
            if self.goal_lanelet_ids:
                for goal_lanelet_id in self.goal_lanelet_ids:
                    if self.backend == RoutePlanner.Backend.NETWORKX:
                        results = self._find_all_shortest_paths(start_lanelet_id, goal_lanelet_id)
                    elif self.backend == RoutePlanner.Backend.NETWORKX_REVERSED:
                        results = self._find_shortest_reversed_path(start_lanelet_id, goal_lanelet_id)
                    elif self.backend == RoutePlanner.Backend.PRIORITY_QUEUE:
                        results = self._find_path(start_lanelet_id, goal_lanelet_id)
                    else:
                        raise ValueError(f"The backend {self.backend} is not recognized as supported backend "
                                         f"for the RoutePlanner")

                    routes.extend(results)
            else:
                routes.append(self._find_survival_route(start_lanelet_id))
        self.logger.info("Route planning finished")
        return routes

    def get_route_candidates(self) -> RouteCandidates:
        """
        Find all paths to all of the goal lanelet IDs from all the start lanelet IDs using networkx module.
        If no goal lanelet ID is given then return a survival route
        :return: empty list if no path has been found
        """
        route_candidates = self.search_alg()
        return RouteCandidates(self.scenario, self.planning_problem, route_candidates,
                               self.route_type, self.allowed_lanelet_ids)

    # ================================================= #
    #                 Custom Exceptions                 #
    # ================================================= #
    class _NoSourceLaneletId(Exception):
        def __init__(self, message):
            self.message = message

    class _NoPathFound(Exception):
        def __init__(self, message):
            self.message = message
