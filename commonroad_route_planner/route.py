from enum import Enum
import numpy as np



# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario

# own code base
from commonroad_route_planner.utility.route import (chaikins_corner_cutting,
                                                    resample_polyline)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops



# typing
from typing import List, Set
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
        # FIXME: What are these sections good for???
        self.sections: List[LaneletSection] = list()

        if permissible_lanelet_ids is None:
            self.permissible_lanelet_ids: Set[int] = {
                lanelet.lanelet_id for lanelet in self.lanelet_network.lanelets
            }
        else:
            self.permissible_lanelet_ids: Set[int]  = permissible_lanelet_ids

        # generate reference path from the list of lanelet ids leading to goal
        self.reference_path: np.ndarray = None
        self.lane_change_position_handler: LaneChangePositionHandler = None
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
        self.lane_change_position_handler: LaneChangePositionHandler = LaneChangePositionHandler(self.lanelet_ids, 
                                                                                                 self.lanelet_network)
        reference_path: np.ndarray = self._compute_reference_path(self.lane_change_position_handler.lanelet_portions)
        reference_path: np.ndarray = pops.remove_duplicate_points(reference_path)
        reference_path_smoothed: np.ndarray = chaikins_corner_cutting(reference_path)
        
        self.reference_path: np.ndarray = reference_path_smoothed

        

    def _compute_reference_path(
        self,
        list_portions,
        num_vertices_lane_change_max=6,
        percentage_vertices_lane_change_max=0.1,
        step_resample=1.0,
    ):
        """Computes reference path stair function given the list of portions of each lanelet

        :param list_portions
        :param num_vertices_lane_change_max: number of vertices to perform lane change.
                                             if set to 0, it will produce a zigzagged polyline.
        :param percentage_vertices_lane_change_max: maximum percentage of vertices that should be used for lane change.
        """
        
        # TODO Refactor
        
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



    def orientation(self, longitudinal_position: float) -> float:
        """
        Calculates orientation of lane given a longitudinal position along lane

        :param position: longitudinal position
        :returns orientation of lane at a given position
        """
        return np.interp(longitudinal_position, self.interpoint_distances, self.path_orientation)


