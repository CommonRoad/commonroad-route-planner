import itertools
from enum import Enum

import numpy as np


# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState

# own code base
from commonroad_route_planner.utility.route import (chaikins_corner_cutting,
                                                    resample_polyline,
                                                    sort_lanelet_ids_by_goal,
                                                    sort_lanelet_ids_by_orientation)
from commonroad_route_planner.pseudo_dataclasses.lanelet_section import LaneletSection
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops



# typing
from typing import List, Set, Tuple, Union
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork, Lanelet



class RouteType(Enum):
    # Survival routes have no specific goal lanelet
    REGULAR = "regular"
    SURVIVAL = "survival"


class Route:
    """A route in a commonroad scenario."""

    scenario: Scenario = None
    planning_problem: PlanningProblem = None

    def __init__(self, lanelet_network: LaneletNetwork, lanelet_ids: List[int],
                 permissible_lanelet_ids: Set[int] = None):

        self.lanelet_network: LaneletNetwork = lanelet_network

        # a route is created given the list of lanelet ids from start to goal
        self.lanelet_ids: List[int] = lanelet_ids

        # a section is a list of lanelet ids that are adjacent to a lanelet in the route
        self.sections: List[LaneletSection] = list()
        
        self.set_ids_lanelets_in_sections = set()
        self.set_ids_lanelets_opposite_direction = set()

        if permissible_lanelet_ids is None:
            self.permissible_lanelet_ids: Set[int] = {
                lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets
            }
        else:
            self.permissible_lanelet_ids: Set[int]  = permissible_lanelet_ids

        # generate reference path from the list of lanelet ids leading to goal
        self.reference_path: np.ndarray = None
        self._generate_reference_path()

        # save additional information about the reference path
        self.interpoint_distances: np.ndarray = pops.compute_interpoint_distances_from_polyline(self.reference_path)
        self.length_reference_path: np.ndarray = pops.compute_length_of_polyline(self.reference_path)
        self.path_orientation: np.ndarray = pops.compute_orientation_from_polyline(self.reference_path)
        self.path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(self.reference_path)



    def retrieve_route_sections(self):
        """Retrieves route sections for lanelets in the route.

        A section is a list of lanelet ids that are adjacent to a given lanelet.
        """
        if(len(self.list_sections) == 0):
            # compute list of sections
            for id_lanelet in self.lanelet_ids:
                current_lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(id_lanelet)
                current_section: LaneletSection = LaneletSection(current_lanelet, self.lanelet_network, self.permissible_lanelet_ids)
                
                # TODO: check if that weird check has some meening?
                
                self.sections.append(current_section)
                
                
    

    def _generate_reference_path(self) -> None:
        """Generates a reference path (polyline) out of the given route

        This is done in four steps:
        1. compute lane change instructions
        2. compute the portion of each lanelet based on the instructions
        3. compute the reference path based on the portion
        4. smoothen the reference path

        :return: reference path in 2d numpy array ([[x0, y0], [x1, y1], ...])
        """
        instruction = self._compute_lane_change_instructions()
        list_portions = self._compute_lanelet_portion(instruction)
        reference_path: np.ndarray = self._compute_reference_path(list_portions)
        reference_path: np.ndarray = pops.remove_duplicate_points(reference_path)
        reference_path_smoothed: np.ndarray = chaikins_corner_cutting(reference_path)
        
        self.reference_path: np.ndarray = reference_path_smoothed



 



    def _compute_lane_change_instructions(self) -> List[int]:
        """Computes lane change instruction for planned routes

        The instruction is a list of 0s and 1s, with 0 indicating  no lane change is required
        (driving straight forward0, and 1 indicating that a lane change (to the left or right) is required.
        """
        list_instructions = []
        for idx, id_lanelet in enumerate(self.lanelet_ids[:-1]):
            if (
                self.lanelet_ids[idx + 1]
                in self.lanelet_network.find_lanelet_by_id(id_lanelet).successor
            ):
                list_instructions.append(0)
            else:
                list_instructions.append(1)

        # add 0 for the last lanelet
        list_instructions.append(0)

        return list_instructions

    @staticmethod
    def _compute_lanelet_portion(list_instructions: List) -> List[Tuple[float, float]]:
        """Computes the portion of the center vertices of the lanelets required to construct the reference path

        This is done by first grouping the instructions into consecutive sections (each with only 0s or 1s).
        For the group of 0s, as no lane change is required, the whole lanelet is used; for the group of 1s,
        the upper limit of the portion is computed within the group as (idx_lanelet in the group) / (num_lanelets).
        For example, if there are three consecutive lane changes (assuming three lanes are all parallel), the proportion
        would be [0 - 0.25], [0.25 -0.5], [0.5 - 0.75] and [0.75 - 1.0] for these four lanes.
        """

        # returns a list of consecutive instructions
        # e.g. input: [0, 0, 1, 1, 0, 1] output: [[0, 0], [1, 1], [0], [1]]
        list_instructions_consecutive = [
            list(v) for k, v in itertools.groupby(list_instructions)
        ]

        list_bounds_upper = []
        list_bounds_lower = [0.0]
        for instructions in list_instructions_consecutive:
            for idx, instruction in enumerate(instructions):
                if instruction == 0:
                    # goes till the end of the lanelet
                    bound_upper = 1.0
                else:
                    # goes only till a specific portion
                    bound_upper = (idx + 1) / (len(instructions) + 1)

                list_bounds_upper.append(bound_upper)

        if len(list_bounds_upper) > 1:
            for idx in range(1, len(list_bounds_upper)):
                if np.isclose(list_bounds_upper[idx - 1], 1.0):
                    list_bounds_lower.append(0.0)
                else:
                    list_bounds_lower.append(list_bounds_upper[idx - 1])

        assert (
            len(list_bounds_lower) == len(list_bounds_upper) == len(list_instructions)
        ), "The lengths of portions do not match."

        return [
            (lower, upper) for lower, upper in zip(list_bounds_lower, list_bounds_upper)
        ]
        
        

    def _compute_reference_path(
        self,
        list_portions,
        num_vertices_lane_change_max=6,
        percentage_vertices_lane_change_max=0.1,
        step_resample=1.0,
    ):
        """Computes reference path given the list of portions of each lanelet

        :param list_portions
        :param num_vertices_lane_change_max: number of vertices to perform lane change.
                                             if set to 0, it will produce a zigzagged polyline.
        :param percentage_vertices_lane_change_max: maximum percentage of vertices that should be used for lane change.
        """
        reference_path = None
        num_lanelets_in_route = len(self.lanelet_ids)
        for idx, id_lanelet in enumerate(self.lanelet_ids):
            lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet)
            # resample the center vertices to prevent too few vertices with too large distances
            vertices_resampled = resample_polyline(
                lanelet.center_vertices, step_resample
            )
            num_vertices = len(vertices_resampled)
            num_vertices_lane_change = min(
                int(num_vertices * percentage_vertices_lane_change_max) + 1,
                num_vertices_lane_change_max,
            )

            if reference_path is None:
                idx_start = int(list_portions[idx][0] * num_vertices)
                idx_end = int(list_portions[idx][1] * num_vertices)
                # prevent index out of bound
                idx_end = max(idx_end, 1)
                # reserve some vertices if it is not the last lanelet
                if idx != (num_lanelets_in_route - 1):
                    idx_end = idx_end - num_vertices_lane_change
                    # prevent index out of bound
                    idx_end = max(idx_end, 1)

                reference_path = vertices_resampled[idx_start:idx_end, :]
            else:
                idx_start = (
                    int(list_portions[idx][0] * num_vertices) + num_vertices_lane_change
                )
                # prevent index out of bound
                idx_start = min(idx_start, num_vertices - 1)

                idx_end = int(list_portions[idx][1] * num_vertices)
                # reserve some vertices if it is not the last lanelet
                if idx != (num_lanelets_in_route - 1):
                    idx_end = idx_end - num_vertices_lane_change
                    # prevent index out of bound
                    idx_end = max(idx_end, 1)

                path_to_be_concatenated = vertices_resampled[idx_start:idx_end, :]

                reference_path = np.concatenate(
                    (reference_path, path_to_be_concatenated), axis=0
                )

        reference_path = resample_polyline(reference_path, 2)
        return reference_path






    

    def orientation(self, position) -> float:
        """
        Calculates orientation of lane given a longitudinal position along lane

        :param position: longitudinal position
        :returns orientation of lane at a given position
        """
        return np.interp(position, self.interpoint_distances, self.path_orientation)


