from enum import Enum
import numpy as np

# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario

# own code base
from commonroad_route_planner.utility.route_util import (chaikins_corner_cutting)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler, LaneChangeInstruction
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.utility.route_slice.route_slice import RouteSlice



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

        self.interpoint_distances: np.ndarray = None
        self.path_length_per_point: np.ndarray = None
        self.length_reference_path: np.ndarray = None
        self.path_orientation: np.ndarray = None
        self.path_curvature: np.ndarray = None
        self.update_geometric_ref_path_properties()



    def update_geometric_ref_path_properties(self, reference_path: np.ndarray=None):
        """
        Updates the geometric properties of ref path.
        If reference path is specified, the new reference path will be updated and resamples before.
        """
        if(reference_path is not None):
            self.reference_path = reference_path

        # save additional information about the reference path
        self.interpoint_distances: np.ndarray = pops.compute_interpoint_distances_from_polyline(self.reference_path)
        self.average_interpoint_distance: float = np.mean(self.interpoint_distances, axis=0)
        self.path_length_per_point: np.ndarray = pops.compute_path_length_per_point(self.reference_path)
        self.length_reference_path: float = pops.compute_length_of_polyline(self.reference_path)
        self.path_orientation: np.ndarray = pops.compute_orientation_from_polyline(self.reference_path)
        self.path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(self.reference_path)



    def retrieve_route_sections(self):
        """Retrieves route sections for lanelets in the route.

        A section is a list of lanelet ids that are adjacent to a given lanelet.
        """
        if(len(self.sections) == 0):
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
        
        reference_path_stair: np.ndarray = self._compute_reference_path_as_stair_function()
        reference_path_star_without_duplicated: np.ndarray = pops.remove_duplicate_points(reference_path_stair)
        reference_path_smoothed: np.ndarray = chaikins_corner_cutting(reference_path_star_without_duplicated)
        
        self.reference_path: np.ndarray = reference_path_smoothed

        

    def _compute_reference_path_as_stair_function(
        self,
        num_vertices_lane_change_max: int=6,
        percentage_vertices_lane_change_max: float=0.1,
        step_resample: float=1.0,
    ):
        """Computes reference path stair function given the list of portions of each lanelet

        :param list_portions
        
        # TODO: sounds not very pracitcal??
        :param num_vertices_lane_change_max: number of vertices to perform lane change.
                                             if set to 0, it will produce a zigzagged polyline.
        :param percentage_vertices_lane_change_max: maximum percentage of vertices that should be used for lane change.
        """
        
        # TODO Refactor, since this does not consider that the reference path is actually intersecting with the goal region, if existing.
        
        reference_path: np.ndarray = None
        num_lanelets_in_route = len(self.lanelet_ids)
        
        
        for idx, id_lanelet in enumerate(self.lanelet_ids):
            
            # Sample the center vertices of the lanelet as foundation for the reference path
            lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(id_lanelet)
            centerline_vertices: np.ndarray = pops.sample_polyline(lanelet.center_vertices, step_resample)
            num_vertices: int = len(centerline_vertices)
            
            # get driving instruction object for lanelet
            instruction: LaneChangeInstruction = self.lane_change_position_handler.get_driving_instruction_for_lanelet(lanelet)
            
            
            # FIXME: Does not sound very practical
            # Number of vertices to be used in the lane change
            num_vertices_lane_change: int = min(
                int(num_vertices * percentage_vertices_lane_change_max) + 1,
                num_vertices_lane_change_max,
            )


            # First time computation at initial lanelet
            if(reference_path is None):
                idx_start = int(instruction.lanelet_portions[0] * num_vertices)
                idx_end = int(instruction.lanelet_portions[1] * num_vertices)

                # reserve some vertices if it is not the last lanelet
                if idx != (num_lanelets_in_route - 1):
                    idx_end = idx_end - num_vertices_lane_change
                    

                # Since we are rounding down, make sure that idx_end is not zero
                idx_end = max(idx_end, 1)
                reference_path: np.ndarray = centerline_vertices[idx_start:idx_end, :]
                
                
            # Concatenate new parts to old reference path  
            else:
                idx_start = (
                    int(instruction.lanelet_portions[0] * num_vertices) + num_vertices_lane_change
                )
                # prevent index out of bound, since we are alway rounding down
                idx_start = min(idx_start, num_vertices - 1)

                idx_end = int(instruction.lanelet_portions[1] * num_vertices)
                # reserve some vertices if it is not the last lanelet
                if idx != (num_lanelets_in_route - 1):
                    idx_end = idx_end - num_vertices_lane_change
                    # prevent index out of bound
                    idx_end = max(idx_end, 1)

                path_to_be_concatenated: np.ndarray = centerline_vertices[idx_start:idx_end, :]

                reference_path: np.ndarray = np.concatenate(
                    (reference_path, path_to_be_concatenated), axis=0
                )

        # Resample polyline for better distance
        reference_path: np.ndarray = pops.sample_polyline(reference_path, 2)
        
        return reference_path



    def get_route_slice_from_position(self, x: float, y:float,
                                      distance_ahead_in_m: float=30,
                                      distance_behind_in_m: float=7) -> RouteSlice:
        """
        Takes an x and y coordinate, finds, the closest point on the reference path and returns slice of the reference
        path around that point with the distance ahead and behind.
        """
        return RouteSlice(
            self,
            x, y,
            distance_ahead_in_m=distance_ahead_in_m,
            distance_behind_in_m=distance_behind_in_m
        )



    def orientation(self, longitudinal_position: float) -> float:
        """
        Calculates orientation of lane given a longitudinal position along lane

        :param position: longitudinal position
        :returns orientation of lane at a given position
        """
        return np.interp(longitudinal_position, self.interpoint_distances, self.path_orientation)


