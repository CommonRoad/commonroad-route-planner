import logging

import numpy as np


# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState

# own code base
from commonroad_route_planner.utility.route_util import chaikins_corner_cutting
from commonroad_route_planner.route import Route
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import (
    LaneChangePositionHandler,
    LaneChangeInstruction,
)
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.lane_changing.lane_change_handler import LaneChangeHandler
from commonroad_route_planner.lane_changing.change_position import LaneChangeMarker
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)
from commonroad_route_planner.route_generation_strategies.base_generation_strategy import (
    BaseGenerationStrategy,
)


# typing
from typing import List, Tuple
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad.scenario.scenario import Lanelet


class DefaultGenerationStrategy(BaseGenerationStrategy):
    """
    Default strategy to generate and update a route
    """

    logger: logging.Logger = logging.Logger(__name__)

    @staticmethod
    def generate_route(
        lanelet_network: LaneletNetwork,
        lanelet_ids: List[int],
        initial_state: InitialState,
        goal_region: GoalRegion,
        prohibited_lanelet_ids: List[int] = None,
        lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE,
    ) -> Route:
        """
        Generates a route from a list of lanelet ids and a lane change method

        :param lanelet_network: lanelet network
        :param lanelet_ids: ordered lanelet ids of route from start to finish
        :param initial_state: initital state
        :param goal_region: goal region
        :param prohibited_lanelet_ids: prohibited lanelet ids of route
        :param lane_change_method: lane change method

        :return: route instance
        :rtype Route
        """

        sections: List[LaneletSection] = DefaultGenerationStrategy._calc_route_sections(
            lanelet_ids=lanelet_ids, lanelet_network=lanelet_network
        )

        reference_path, num_lane_change_actions = (
            DefaultGenerationStrategy._calc_reference_path(
                lanelet_ids=lanelet_ids,
                lanelet_network=lanelet_network,
                initial_state=initial_state,
                goal_region=goal_region,
                lane_change_method=lane_change_method,
            )
        )

        # save additional information about the reference path
        interpoint_distances: np.ndarray = (
            pops.compute_interpoint_distances_from_polyline(reference_path)
        )
        average_interpoint_distance: float = np.mean(interpoint_distances, axis=0)
        path_length_per_point: np.ndarray = pops.compute_path_length_per_point(
            reference_path
        )
        length_reference_path: float = pops.compute_length_of_polyline(reference_path)
        path_orientation: np.ndarray = pops.compute_orientation_from_polyline(
            reference_path
        )
        path_curvature: np.ndarray = pops.compute_scalar_curvature_from_polyline(
            reference_path
        )

        return Route(
            lanelet_network=lanelet_network,
            initial_state=initial_state,
            goal_region=goal_region,
            lanelet_ids=lanelet_ids,
            sections=sections,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            lane_change_method=lane_change_method,
            num_lane_change_actions=num_lane_change_actions,
            reference_path=reference_path,
            interpoint_distances=interpoint_distances,
            average_interpoint_distance=average_interpoint_distance,
            path_length_per_point=path_length_per_point,
            length_reference_path=length_reference_path,
            path_orientation=path_orientation,
            path_curvature=path_curvature,
        )

    @staticmethod
    def update_route(
        route: Route,
        reference_path: np.ndarray,
    ) -> Route:
        """
        Updates all route properties given a new reference path.

        :param route: route instace to be updated
        :param reference_path: (n,2) updated reference path from which route should be created

        :return: updated route instance
        :rtype Route
        """

        resample_step: float = route.average_interpoint_distance
        reference_path = pops.sample_polyline(reference_path, step=resample_step)
        route.interpoint_distances = pops.compute_interpoint_distances_from_polyline(
            reference_path
        )
        route.average_interpoint_distance = np.mean(route.interpoint_distances, axis=0)
        route.path_length_per_point = pops.compute_path_length_per_point(
            route.reference_path
        )
        route.length_reference_path = pops.compute_length_of_polyline(
            route.reference_path
        )
        route.path_orientation = pops.compute_orientation_from_polyline(
            route.reference_path
        )
        route.path_curvature = pops.compute_scalar_curvature_from_polyline(
            route.reference_path
        )

        return route

    @staticmethod
    def _calc_route_sections(
        lanelet_ids: List[int],
        lanelet_network: LaneletNetwork,
    ) -> List[LaneletSection]:
        """
        Calculates route _sections for lanelets in the route.
        A section is a list of lanelet ids that are adjacent to a given lanelet.

        :param lanelet_ids: ordered lanelet ids of route from start to goal
        :param lanelet_network: lanelet network of scenrio

        :return: list of lanelet sections
        :rtype List[LaneletSection]
        """

        sections: List[LaneletSection] = [
            LaneletSection(
                lanelet_network.find_lanelet_by_id(lanelet_id), lanelet_network
            )
            for lanelet_id in lanelet_ids
        ]
        return sections

    @staticmethod
    def _calc_reference_path(
        lanelet_ids: List[int],
        lanelet_network: LaneletNetwork,
        initial_state: InitialState,
        goal_region: GoalRegion,
        lane_change_method: LaneChangeMethod,
        step_resample: float = 1.0,
    ) -> Tuple[np.ndarray, int]:
        """
        Computes reference path stair function given the list of portions of each lanelet

        :param lanelet_ids: ordered lanelet ids of route from start to finish
        :param initial_state: initial state of planning problem
        :param goal_region: goal region of planning problem
        :param lane_change_method: method with which lane changes are performerd
        :param step_resample: sample step size for resampling

        :return: [(n,2) reference path, number of lane changes]
        :rtype Tuple[np.ndarray, int]
        """

        lane_change_position_handler: LaneChangePositionHandler = (
            LaneChangePositionHandler(lanelet_ids, lanelet_network)
        )
        num_lane_change_action: int = 0
        reference_path: np.ndarray = None
        skip_ids: List[int] = list()

        for idx, lanelet_id in enumerate(lanelet_ids):
            # necessary since lane change takes care of multiple ids
            if lanelet_id in skip_ids:
                continue

            # Sample the center vertices of the lanelet as foundation for the reference path
            lanelet: "Lanelet" = lanelet_network.find_lanelet_by_id(lanelet_id)
            centerline_vertices: np.ndarray = pops.sample_polyline(
                lanelet.center_vertices, step_resample
            )
            lanelet_section: LaneletSection = LaneletSection.get_section_by_lanelet_id(
                lanelet_id
            )

            # get driving instruction object for lanelet
            instruction: LaneChangeInstruction = (
                lane_change_position_handler.get_driving_instruction_for_lanelet(
                    lanelet
                )
            )

            if instruction.instruction_markers == LaneChangeMarker.NO_CHANGE:
                # No lane change required
                reference_path: np.ndarray = (
                    np.concatenate((reference_path, centerline_vertices), axis=0)
                    if (reference_path is not None)
                    else centerline_vertices
                )

            else:
                # lane change required
                lanelet_end: Lanelet = (
                    DefaultGenerationStrategy._find_last_lanelet_of_lane_change(
                        lanelet_start=lanelet,
                        lanelet_section=lanelet_section,
                        lanelet_ids=lanelet_ids,
                        lanelet_network=lanelet_network,
                    )
                )
                lane_change_handler: LaneChangeHandler = LaneChangeHandler(
                    lanelet_start=lanelet,
                    lanelet_end=lanelet_end,
                    lanelet_section=lanelet_section,
                    lanelet_network=lanelet_network,
                    route_lanelet_ids=lanelet_ids,
                )

                num_lane_change_action += 1

                skip_ids.extend(lanelet_section.adjacent_lanelet_ids)

                lane_change_path: np.ndarray = lane_change_handler.compute_lane_change(
                    initial_state=initial_state,
                    goal_region=goal_region,
                    method=lane_change_method,
                )

                # No lane change required
                reference_path: np.ndarray = (
                    np.concatenate((reference_path, lane_change_path), axis=0)
                    if (reference_path is not None)
                    else lane_change_path
                )

        # Resample polyline for better distance
        reference_path: np.ndarray = pops.sample_polyline(reference_path, step=2)

        # Smooth with chaikin
        reference_path: np.ndarray = pops.remove_duplicate_points(reference_path)
        reference_path: np.ndarray = chaikins_corner_cutting(reference_path)

        return reference_path, num_lane_change_action

    @staticmethod
    def _find_last_lanelet_of_lane_change(
        lanelet_start: "Lanelet",
        lanelet_section: LaneletSection,
        lanelet_ids: List[int],
        lanelet_network: LaneletNetwork,
    ) -> "Lanelet":
        """
        Finds last lanelet of lane change

        :param lanelet_start: lanelet entering the lane change
        :param lanelet_section: lanelet section of the lane change
        :param lanelet_ids: ordered lanelet ids of route from start to finish
        :param lanelet_network: lanelet network

        :return: Lanelet instance
        :rtype Lanelet
        """

        idx_start: int = lanelet_ids.index(lanelet_start.lanelet_id)
        lanelet_return: Lanelet = None

        # NOTE: This check assumes that self._lanelet_ids has the correct order from start to finish
        for i in range(idx_start, (len(lanelet_ids))):
            if lanelet_ids[i] not in lanelet_section.adjacent_lanelet_ids:
                lanelet_return: Lanelet = lanelet_network.find_lanelet_by_id(
                    lanelet_ids[i - 1]
                )
                break

        # if route ends in lane section of lane change
        if lanelet_return is None:
            DefaultGenerationStrategy.logger.info("Encountered goal in lane change")
            lanelet_return: Lanelet = lanelet_network.find_lanelet_by_id(
                lanelet_ids[-1]
            )

        return lanelet_return
