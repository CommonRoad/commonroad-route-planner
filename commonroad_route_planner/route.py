import logging
from enum import Enum
import numpy as np

# third party


# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState

# own code base
from commonroad_route_planner.utility.route_util import (chaikins_corner_cutting)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler, LaneChangeInstruction
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.frenet_tools.route_slice import RouteSlice
from commonroad_route_planner.lane_changing.lane_change_handler import LaneChangeHandler
from commonroad_route_planner.lane_changing.change_position import LaneChangeMarker
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import LaneChangeMethod



# typing
from typing import List, Union
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork, Lanelet



class RouteType(Enum):
    # Survival routes have no specific goal lanelet
    REGULAR = "regular"
    SURVIVAL = "survival"


class Route:
    """
    A route in a commonroad scenario.
    """
    def __init__(self,
                 lanelet_network: LaneletNetwork,
                 lanelet_ids: List[int],
                 logger: logging.Logger,
                 initial_state: InitialState,
                 goal_region: GoalRegion,
                 prohibited_lanelet_ids: List[int] = None,
                 lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE
                 )->None:

        self._logger = logger

        self._lanelet_network: LaneletNetwork = lanelet_network
        self._initial_state: InitialState = initial_state
        self._goal_region: GoalRegion = goal_region

        # a route is created given the list of lanelet ids from start to goal
        self._lanelet_ids: List[int] = lanelet_ids

        # a section is a list of lanelet ids that are adjacent to a lanelet in the route
        self._sections: List[LaneletSection] = list()
        self._calc_route_sections()

        self._prohibited_lanelet_ids: List[int]  = prohibited_lanelet_ids if(lanelet_ids is not None) else list()

        self._lane_change_method: LaneChangeMethod = lane_change_method

        # generate reference path from the list of lanelet ids leading to goal
        self._reference_path: np.ndarray = None
        self._lane_change_position_handler: LaneChangePositionHandler = None
        self._num_lane_chang_action: int = 0
        self._generate_reference_path()

        self._interpoint_distances: np.ndarray = None
        self._average_interpoint_distance: float = None
        self._path_length_per_point: np.ndarray = None
        self._length_reference_path: float = None
        self._path_orientation: np.ndarray = None
        self._path_curvature: np.ndarray = None
        self.update_geometric_ref_path_properties()




    @property
    def list_ids_lanelets(self) -> List[int]:
        """
        Dummy interface for old lanelet ids

        :return: list of lanelet id in route
        """
        return self._lanelet_ids


    @property
    def lanelet_ids(self) -> List[int]:
        """
        :return: list of lanelet id in route
        """
        return self._lanelet_ids

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """
        :return: lanelet network of route
        """
        return self._lanelet_network

    @property
    def goal_region(self) -> GoalRegion:
        """
        :return: commonroad goal region
        """
        return self._goal_region

    @property
    def lane_change_method(self) -> LaneChangeMethod:
        """
        :return: lane change method
        """
        return self._lane_change_method

    @property
    def num_lane_change_action(self) -> int:
        """
        :return: number of lane change actions containing two or more lanelets
        """
        return self._num_lane_chang_action


    @property
    def reference_path(self) -> np.ndarray:
        """
        :return: (n,2) np ndarray of points of ref path
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



    def update_geometric_ref_path_properties(self,
                                             reference_path: np.ndarray=None,
                                             default_resample_step: float=2) -> None:
        """
        Updates the geometric properties of ref path.
        If reference path is specified, the new reference path will be updated and resamples before.
        """
        if(reference_path is not None):
            if(self._average_interpoint_distance is not None):
                resample_step: float = self._average_interpoint_distance
            else:
                resample_step: float = default_resample_step

            self._reference_path = pops.sample_polyline(reference_path,
                                                        step=resample_step)

        # save additional information about the reference path
        self._interpoint_distances: np.ndarray = pops.compute_interpoint_distances_from_polyline(self._reference_path)
        self._average_interpoint_distance: float = np.mean(self._interpoint_distances, axis=0)
        self._path_length_per_point: np.ndarray = pops.compute_path_length_per_point(self._reference_path)
        self._length_reference_path: float = pops.compute_length_of_polyline(self._reference_path)
        self._path_orientation: np.ndarray = pops.compute_orientation_from_polyline(self._reference_path)
        self._path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(self._reference_path)




    def get_route_slice_from_position(self,
                                      x: float,
                                      y:float,
                                      distance_ahead_in_m: float = 30,
                                      distance_behind_in_m: float = 7
                                      ) -> RouteSlice:
        """
        Takes an x and y coordinate, finds, the closest point on the reference path and returns slice of the reference
        path around that point with the distance ahead and behind.

        :param x: x-position
        :param y: y-position
        :param distance_ahead_in_m: how long the path should continue in front of position
        :param distance_behind_in_m: how long the path should continue after position

        :return: route slice
        """
        return RouteSlice(
            self,
            x, y,
            distance_ahead_in_m=distance_ahead_in_m,
            distance_behind_in_m=distance_behind_in_m
        )


    def get_lanelet_section(self, lanelet_id: int) -> Union[LaneletSection, None]:
        """
        Takes lanelet id and retrieves lanelet section

        :param lanelet_id: lanelet id for which the lanelet section should be returned

        :return: lanelet section or none
        """
        if(lanelet_id not in self._lanelet_ids):
            self._logger.error('Lanelet id not part of route')
            raise ValueError('Lanelet id not part of route')

        return LaneletSection.get_section_by_lanelet_id(lanelet_id)








    def _calc_route_sections(self):
        """Retrieves route _sections for lanelets in the route.

        A section is a list of lanelet ids that are adjacent to a given lanelet.
        """
        if(len(self._sections) == 0):
            # compute list of _sections
            for id_lanelet in self._lanelet_ids:
                current_lanelet: "Lanelet" = self._lanelet_network.find_lanelet_by_id(id_lanelet)
                current_section: LaneletSection = LaneletSection(current_lanelet, self._lanelet_network)
                
                self._sections.append(current_section)
                
                
    

    def _generate_reference_path(self) -> None:
        """Generates a reference path (polyline) out of the given route

        This is done in four steps:
        1. compute lane change instructions
        2. compute the portion of each lanelet based on the instructions
        3. compute the reference path based on the portion
        4. smoothen the reference path

        :return: reference path in 2d numpy array ([[x0, y0], [x1, y1], ...])
        """
        self._lane_change_position_handler: LaneChangePositionHandler = LaneChangePositionHandler(self._lanelet_ids,

                                                                                                  self._lanelet_network)
        reference_path_stair: np.ndarray = self._compute_reference_path_with_lane_changes()
        reference_path_star_without_duplicated: np.ndarray = pops.remove_duplicate_points(reference_path_stair)
        reference_path_smoothed: np.ndarray = chaikins_corner_cutting(reference_path_star_without_duplicated)

        
        self._reference_path: np.ndarray = reference_path_smoothed

        

    def _compute_reference_path_with_lane_changes(
        self,
        step_resample: float = 1.0,
    ) -> np.ndarray:
        """
        Computes reference path stair function given the list of portions of each lanelet

        :param step_resample: sample step size for resampling

        :return: (n,2) reference path
        """

        reference_path: np.ndarray = None
        skip_ids: List[int] = list()

        for idx, lanelet_id in enumerate(self._lanelet_ids):
            # necessary since lane change takes care of multiple ids
            if (lanelet_id in skip_ids):
                continue

            # Sample the center vertices of the lanelet as foundation for the reference path
            lanelet: "Lanelet" = self._lanelet_network.find_lanelet_by_id(lanelet_id)
            centerline_vertices: np.ndarray = pops.sample_polyline(lanelet.center_vertices, step_resample)
            lanelet_section: LaneletSection = LaneletSection.get_section_by_lanelet_id(lanelet_id)

            # get driving instruction object for lanelet
            instruction: LaneChangeInstruction = self._lane_change_position_handler.get_driving_instruction_for_lanelet(
                lanelet)

            if(instruction.instruction_markers == LaneChangeMarker.NO_CHANGE):
                # No lane change required
                reference_path: np.ndarray = np.concatenate(
                    (reference_path, centerline_vertices), axis=0
                ) if(reference_path is not None) else centerline_vertices

            else:
                # lane change required
                lanelet_end: Lanelet = self._find_last_lanelet_of_lane_change(
                    lanelet_start=lanelet,
                    lanelet_section=lanelet_section
                )
                lane_change_handler: LaneChangeHandler = LaneChangeHandler(
                    lanelet_start=lanelet,
                    lanelet_end=lanelet_end,
                    lanelet_section=lanelet_section,
                    lanelet_network=self.lanelet_network,
                    route_lanelet_ids=self.lanelet_ids
                )

                self._num_lane_chang_action += 1

                skip_ids.extend(lanelet_section.adjacent_lanelet_ids)

                lane_change_path: np.ndarray = lane_change_handler.compute_lane_change(
                    initial_state=self._initial_state,
                    goal_region=self._goal_region,
                    method=self._lane_change_method
                )

                # No lane change required
                reference_path: np.ndarray = np.concatenate(
                    (reference_path, lane_change_path), axis=0
                ) if(reference_path is not None) else lane_change_path


        # Resample polyline for better distance
        reference_path: np.ndarray = pops.sample_polyline(reference_path, step=2)

        return reference_path


    def _find_last_lanelet_of_lane_change(self,
                                          lanelet_start: "Lanelet",
                                          lanelet_section: LaneletSection
                                          ) -> "Lanelet":
        """
        Finds last lanelet of lane change
        """

        idx_start: int = self._lanelet_ids.index(lanelet_start.lanelet_id)
        lanelet_return: Lanelet = None


        # NOTE: This check assumes that self._lanelet_ids has the correct order from start to finish
        for i in range(idx_start, (len(self._lanelet_ids))):
            if(self._lanelet_ids[i] not in lanelet_section.adjacent_lanelet_ids):
                lanelet_return: Lanelet = self._lanelet_network.find_lanelet_by_id(self._lanelet_ids[i - 1])
                break


        # if route ends in lane section of lane change
        if (lanelet_return is None):
            self._logger.info(f'Encountered goal in lane change')
            lanelet_return: Lanelet = self._lanelet_network.find_lanelet_by_id(self._lanelet_ids[-1])

        return lanelet_return








