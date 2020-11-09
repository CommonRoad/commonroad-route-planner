import sys
import warnings
import itertools
from enum import Enum
from typing import List, Union, Tuple
from typing import Set

import numpy as np
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State
import commonroad.geometry.shape as cr_shape
from shapely.geometry import Polygon
from shapely.ops import cascaded_union

from route_planner.utils_route import resample_polyline_with_length_check, chaikins_corner_cutting, \
    compute_polyline_length, sort_lanelet_ids_by_orientation, resample_polyline, compute_curvature_from_polyline

try:
    import pycrccosy
except ModuleNotFoundError as exp:
    warnings.warn(f"""You won't be able to use the Curvilinear Coordinate System for the Navigator, 
                      the calculations won't be precise. {exp}""")


class RouteType(Enum):
    # Survival routes have no specific goal lanelet
    REGULAR = "regular"
    SURVIVAL = "survival"


class Route:
    """
    Class to represent a route in the scenario.
    """
    scenario = None
    planning_problem = None
    lanelet_network = None

    @classmethod
    def initialize(cls, scenario: Scenario, planning_problem: PlanningProblem):
        cls.scenario = scenario
        cls.planning_problem = planning_problem
        cls.lanelet_network = scenario.lanelet_network
        # initialized Navigator class attributes as well
        Navigator.initialize(scenario, planning_problem)

    def __init__(self, list_ids_lanelets: List[int], route_type: RouteType,
                 set_ids_lanelets_permissible: Set[int] = None):
        # a route is created given the list of lanelet ids from start to goal
        self.list_ids_lanelets = list_ids_lanelets
        self.type = route_type

        # a section is a list of lanelet ids that are adjacent to a lanelet in the route
        self.list_sections = list()
        self.set_ids_lanelets_in_sections = set()
        self.set_ids_lanelets_opposite_direction = set()

        if set_ids_lanelets_permissible is None:
            self.set_ids_lanelets_permissible = {lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets}
        else:
            self.set_ids_lanelets_permissible = set_ids_lanelets_permissible

        self.reference_path = self._generate_reference_path()

    @property
    def navigator(self):
        return Navigator(route=self)

    def retrieve_route_sections(self, is_opposite_direction_allowed: bool = False) -> Union[None, List[List[int]]]:
        """
        Retrieves route sections for lanelets in the route. A section is a list of lanelet ids that are adjacent to
        a given lanelet.
        """
        if not self.list_sections:
            # compute list of sections
            for id_lanelet in self.list_ids_lanelets:
                # for every lanelet in the route, get its adjacent lanelets
                list_ids_lanelets_in_section = self._get_adjacent_lanelets_ids(id_lanelet,
                                                                               is_opposite_direction_allowed)
                # add lanelet from the route too
                list_ids_lanelets_in_section.append(id_lanelet)
                list_ids_lanelets_in_section.sort()

                if len(self.list_sections) == 0:
                    self.list_sections.append(list_ids_lanelets_in_section)

                elif self.list_sections[-1] != list_ids_lanelets_in_section:
                    # only add new sections if it is not the same as the last section
                    self.list_sections.append(list_ids_lanelets_in_section)

        for section in self.list_sections:
            for id_lanelet in section:
                self.set_ids_lanelets_in_sections.add(id_lanelet)

        return self.list_sections

    def _get_adjacent_lanelets_ids(self, id_lanelet: int, is_opposite_direction_permissible=False) -> list:
        """
        Recursively gets adj_left and adj_right lanelets of the given lanelet

        :param id_lanelet: current lanelet id
        :param is_opposite_direction_permissible: specifies if it should give back only the lanelets in the driving
            direction or it should give back the first neighbouring lanelet in the opposite direction
        :return: list of adjacent lanelet ids: all lanelets which are going in the same direction and one-one from the
                 left and right side which are going in the opposite direction, empty lists if there are none
        """
        list_lanelets_adjacent = list()
        lanelet_base = self.lanelet_network.find_lanelet_by_id(id_lanelet)

        # goes in left direction
        lanelet_current = lanelet_base
        id_lanelet_temp = lanelet_current.adj_left
        while id_lanelet_temp is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if id_lanelet_temp in self.set_ids_lanelets_permissible:
                if lanelet_current.adj_left_same_direction:
                    # append the left adjacent lanelet
                    list_lanelets_adjacent.append(id_lanelet_temp)

                    # update lanelet_current
                    lanelet_current = self.lanelet_network.find_lanelet_by_id(id_lanelet_temp)
                    id_lanelet_temp = lanelet_current.adj_left

                else:
                    # if the lanelet is in opposite direction, we add them into a set
                    # if driving in opposite lanelet is allowed, they can be traversed too to form the route
                    self.set_ids_lanelets_opposite_direction.add(id_lanelet_temp)
                    if is_opposite_direction_permissible:
                        list_lanelets_adjacent.append(id_lanelet_temp)
                    break
            else:
                # it is not allowed to drive in that lane, so just break
                break

        # goes in right direction
        lanelet_current = lanelet_base
        id_lanelet_temp = lanelet_current.adj_right
        while id_lanelet_temp is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if id_lanelet_temp in self.set_ids_lanelets_permissible:
                if lanelet_current.adj_right_same_direction:
                    # append the right adjacent lanelet
                    list_lanelets_adjacent.append(id_lanelet_temp)

                    # Update lanelet_current
                    lanelet_current = self.lanelet_network.find_lanelet_by_id(id_lanelet_temp)
                    id_lanelet_temp = lanelet_current.adj_right
                else:
                    # if the lanelet is in opposite direction, we add them into a set
                    # if driving in opposite lanelet is allowed, they can be traversed too to form the route
                    self.set_ids_lanelets_opposite_direction.add(id_lanelet_temp)
                    if is_opposite_direction_permissible:
                        list_lanelets_adjacent.append(id_lanelet_temp)
                    break
            else:
                # it is not allowed to drive in that lane, so just break
                break

        return list_lanelets_adjacent

    def _generate_reference_path(self) -> np.ndarray:
        """
        Generates a reference path (polyline) out of the given route. This is done in four steps:
        1. compute lane change instructions
        2. compute the portion of each lanelet based on the instructions
        3. compute the reference path based on the portion
        4. smoothen the reference path

        :return: reference path in 2d numpy array ([[x0, y0], [x1, y1], ...])
        """
        instruction = self._compute_lane_change_instructions()
        list_portions = self._compute_lanelet_portion(instruction)
        reference_path = self._compute_reference_path(list_portions)
        reference_path_smoothed = chaikins_corner_cutting(reference_path, num_refinements=6)

        return reference_path_smoothed

    def _compute_lane_change_instructions(self) -> List[int]:
        """
        Computes lane change instruction for planned routes. The instruction is a list of 0s and 1s, with 0 indicating
        no lane change is required (driving straight forward0, and 1 indicating that a lane change (to the left or right)
        is required.
        """
        list_instructions = []
        for idx, id_lanelet in enumerate(self.list_ids_lanelets[:-1]):
            if self.list_ids_lanelets[idx + 1] in self.lanelet_network.find_lanelet_by_id(id_lanelet).successor:
                list_instructions.append(0)
            else:
                list_instructions.append(1)

        # add 0 for the last lanelet
        list_instructions.append(0)

        return list_instructions

    @staticmethod
    def _compute_lanelet_portion(list_instructions: List) -> List[Tuple[float, float]]:
        """
        Given a list of instructions, computes the portion of the center vertices of the lanelets required to
        construct the reference path. This is done by first grouping the instructions into consecutive sections
        (each with only 0s or 1s). For the group of 0s, as no lane change is required, the whole lanelet is used; for
        the group of 1s, the upper limit of the portion is computed within the group as
        (idx_lanelet in the group) / (num_lanelets).
        For example, if there are three consecutive lane changes (assuming three lanes are all parallel), the proportion
        would be [0 - 0.25], [0.25 -0.5], [0.5 - 0.75] and [0.75 - 1.0] for these four lanes.
        """

        # returns a list of consecutive instructions
        # e.g. input: [0, 0, 1, 1, 0, 1] output: [[0, 0], [1, 1], [0], [1]]
        list_instructions_consecutive = [list(v) for k, v in itertools.groupby(list_instructions)]

        list_bounds_upper = []
        list_bounds_lower = [0.]
        for instructions in list_instructions_consecutive:
            for idx, instruction in enumerate(instructions):
                if instruction == 0:
                    # goes till the end of the lanelet
                    bound_upper = 1.
                else:
                    # goes only till a specific portion
                    bound_upper = (idx + 1) / (len(instructions) + 1)

                list_bounds_upper.append(bound_upper)

        if len(list_bounds_upper) > 1:
            for idx in range(1, len(list_bounds_upper)):
                if np.isclose(list_bounds_upper[idx - 1], 1.):
                    list_bounds_lower.append(0.)
                else:
                    list_bounds_lower.append(list_bounds_upper[idx - 1])

        assert len(list_bounds_lower) == len(list_bounds_upper) == len(list_instructions), \
            f"The lengths of portions do not match."

        return [(lower, upper) for lower, upper in zip(list_bounds_lower, list_bounds_upper)]

    def _compute_reference_path(self, list_portions, num_vertices_lane_change_max=5,
                                percentage_vertices_lane_change_max=0.1):
        """
        Computes reference path given the list of portions of each lanelet.

        :param list_portions
        :param num_vertices_lane_change_max: number of vertices to perform lane change.
                                             if set to 0, it will produce a zigzagged polyline.
        :param percentage_vertices_lane_change_max: maximum percentage of vertices that should be used for lane change.
        """
        reference_path = None

        for idx, id_lanelet in enumerate(self.list_ids_lanelets):
            lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet)
            # resample the center vertices to prevent too few vertices with too large distances
            vertices_resampled = resample_polyline(lanelet.center_vertices, 2)
            num_vertices = len(vertices_resampled)
            num_vertices_lane_change = min(int(num_vertices * percentage_vertices_lane_change_max) + 1,
                                           num_vertices_lane_change_max)

            if reference_path is None:
                idx_start = int(list_portions[idx][0] * num_vertices)
                idx_end = int(list_portions[idx][1] * num_vertices) - num_vertices_lane_change
                # prevent index out of bound
                idx_end = max(idx_end, 1)
                reference_path = vertices_resampled[idx_start:idx_end, :]
            else:
                idx_start = int(list_portions[idx][0] * num_vertices) + num_vertices_lane_change
                # prevent index out of bound
                idx_start = min(idx_start, num_vertices - 1)

                idx_end = int(list_portions[idx][1] * num_vertices) - num_vertices_lane_change
                # prevent index out of bound
                idx_end = max(idx_end, 1)

                path_to_be_concatenated = vertices_resampled[idx_start:idx_end, :]

                reference_path = np.concatenate((reference_path, path_to_be_concatenated), axis=0)

        return reference_path


class Navigator:
    """
    Class to navigate the planned route
    """
    scenario = None
    planning_problem = None
    lanelet_network = None

    class Backend(Enum):
        """
        todo:
        """
        PYCRCCOSY = 'pycrccosy'
        APPROXIMATE = 'approx'

    @classmethod
    def initialize(cls, scenario: Scenario, planning_problem: PlanningProblem):
        cls.scenario = scenario
        cls.planning_problem = planning_problem
        cls.lanelet_network = scenario.lanelet_network

    def __init__(self, route: Route):
        """

        """
        if 'pycrccosy' in sys.modules:
            self.backend = self.Backend.PYCRCCOSY
        else:
            self.backend = self.Backend.APPROXIMATE

        self.route = route
        self.list_sections = route.retrieve_route_sections()
        self.set_ids_lanelets_in_sections = route.set_ids_lanelets_in_sections

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
        if self.route.type == RouteType.REGULAR:
            goal_face_coords = self._get_goal_face_points(self._get_goal_polygon(self.planning_problem.goal))
            self.goal_curvi_face_coords = np.array([(self._get_safe_curvilinear_coords(self.ccosy_list[-1], g))[0]
                                                    for g in goal_face_coords])

            self.goal_min_curvi_coords = np.min(self.goal_curvi_face_coords, axis=0)
            self.goal_max_curvi_coord = np.max(self.goal_curvi_face_coords, axis=0)

    def _get_route_cosy(self) -> Union[pycrccosy.CurvilinearCoordinateSystem, List[Lanelet]]:
        # Merge reference route
        self.merged_route_lanelets = []

        # Append predecessor of the initial to ensure that the goal state is not out of the projection domain
        # initial_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.route[0])
        # predecessors_lanelet = initial_lanelet.predecessor
        # if predecessors_lanelet is not None and len(predecessors_lanelet) != 0:
        #     predecessor_lanelet = self.lanelet_network.find_lanelet_by_id(predecessors_lanelet[0])
        #     current_merged_lanelet = predecessor_lanelet
        # else:
        #     current_merged_lanelet = None
        current_merged_lanelet = None

        for current_lanelet_id, next_lanelet_id in zip(self.route.list_ids_lanelets[:-1],
                                                       self.route.list_ids_lanelets[1:]):
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

        goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.list_ids_lanelets[-1])
        if current_merged_lanelet is not None:
            current_merged_lanelet = Lanelet.merge_lanelets(current_merged_lanelet, goal_lanelet)
        else:
            current_merged_lanelet = goal_lanelet

        # Append successor of the goal to ensure that the goal state is not out of the projection domain
        # goal_lanelet = self.lanelet_network.find_lanelet_by_id(self.route.route[-1])
        # successors_of_goal = goal_lanelet.successor
        # if successors_of_goal is not None and len(successors_of_goal) != 0:
        #     successor_lanelet = self.lanelet_network.find_lanelet_by_id(successors_of_goal[0])
        #     current_merged_lanelet = Lanelet.merge_lanelets(current_merged_lanelet, successor_lanelet)

        self.merged_route_lanelets.append(current_merged_lanelet)

        if self.backend == self.Backend.APPROXIMATE:
            # If ccosy is not installed using temporary method for approximate calculations
            return self.merged_route_lanelets
        else:
            return [self._create_coordinate_system_from_polyline(merged_lanelet.center_vertices)
                    for merged_lanelet in self.merged_route_lanelets]

    @staticmethod
    def _create_coordinate_system_from_polyline(polyline) -> pycrccosy.CurvilinearCoordinateSystem:

        polyline = resample_polyline_with_length_check(polyline)

        abs_curvature = abs(compute_curvature_from_polyline(polyline))
        max_curvature = max(abs_curvature)
        infinite_loop_counter = 0
        while max_curvature > 0.1:
            polyline = np.array(chaikins_corner_cutting(polyline))

            length = compute_polyline_length(polyline)
            if length > 10:
                polyline = resample_polyline(polyline, 1.0)
            else:
                polyline = resample_polyline(polyline, length / 10.0)

            abs_curvature = abs(compute_curvature_from_polyline(polyline))
            max_curvature = max(abs_curvature)

            infinite_loop_counter += 1

            if infinite_loop_counter > 20:
                break

        return pycrccosy.CurvilinearCoordinateSystem(polyline)

    def _get_length(self, ccosy):
        if self.backend == self.Backend.PYCRCCOSY:
            return ccosy.length()
        else:
            lanelet = ccosy
            return lanelet.distance[-1]

    def _get_safe_curvilinear_coords(self, ccosy, position: np.ndarray) -> Tuple[np.ndarray, int]:

        try:
            rel_pos_to_domain = 0
            long_lat_distance = self._get_curvilinear_coords(ccosy, position)
        except ValueError:
            long_lat_distance, rel_pos_to_domain = self._project_out_of_domain(ccosy, position)

        return np.array(long_lat_distance), rel_pos_to_domain

    def _project_out_of_domain(self, ccosy, position: np.ndarray) -> Tuple[np.ndarray, int]:
        if self.backend == self.Backend.PYCRCCOSY:
            eps = 0.0001
            curvi_coords_of_projection_domain = np.array(ccosy.curvilinear_projection_domain())

            longitudinal_min, normal_min = np.min(curvi_coords_of_projection_domain, axis=0) + eps
            longitudinal_max, normal_max = np.max(curvi_coords_of_projection_domain, axis=0) - eps
            normal_center = (normal_min + normal_max) / 2
            bounding_points = np.array(
                [ccosy.convert_to_cartesian_coords(longitudinal_min, normal_center),
                 ccosy.convert_to_cartesian_coords(longitudinal_max, normal_center)])
            rel_positions = position - np.array([bounding_point for bounding_point in bounding_points])
            distances = np.linalg.norm(rel_positions, axis=1)

            if distances[0] < distances[1]:
                # Nearer to the first bounding point
                rel_pos_to_domain = -1
                long_dist = longitudinal_min + np.dot(ccosy.tangent(longitudinal_min), rel_positions[0])
                lat_dist = normal_center + np.dot(ccosy.normal(longitudinal_min), rel_positions[0])
            else:
                # Nearer to the last bounding point
                rel_pos_to_domain = 1
                long_dist = longitudinal_max + np.dot(ccosy.tangent(longitudinal_max), rel_positions[1])
                lat_dist = normal_center + np.dot(ccosy.normal(longitudinal_max), rel_positions[1])

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

    def _get_curvilinear_coords(self, ccosy, position: np.ndarray) -> np.ndarray:
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
            return np.array([long_distance, lat_distance])

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

    def get_position_curvi_coords(self, ego_vehicle_state_position: np.ndarray):
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

        ego_curvi_coords, cosy_idx = self.get_position_curvi_coords(ego_vehicle_state_position)

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

    def get_lane_change_distance(self, state: State, active_lanelets: List[int] = None) -> float:
        # If the route is survival, then return zero
        if self.route.type == RouteType.SURVIVAL:
            return 0.0
        if active_lanelets is None:
            active_lanelets = self.scenario.lanelet_network.find_lanelet_by_position([state.position])[0]

        current_lanelet_ids_on_route = [
            current_lanelet_id
            for current_lanelet_id in active_lanelets
            if current_lanelet_id in self.set_ids_lanelets_in_sections
        ]
        # The state is not on the route, instant lane change is required
        if len(current_lanelet_ids_on_route) == 0:
            return 0.0

        sorted_current_lanelet_ids_on_route = sort_lanelet_ids_by_orientation(
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
            route_successors = successors_set.intersection(self.set_ids_lanelets_in_sections)

        return distance_until_lane_change
