import numpy as np
import logging

# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.scenario.lanelet import LaneletNetwork

# own code base
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_route_planner.utility.visualization import debug_visualize
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import (
    DefaultGenerationStrategy,
)
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.lanelet_sequence import LaneletSequence

# typing
from typing import List, Set, Tuple, Union


class ReferencePathPlanner:
    """
    Generates reference paths from a given sequence of lanelets, per default the shortest reference_path.
    Offers methods to select them.
    """

    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        planning_problem: PlanningProblem,
        routes: List[LaneletSequence],
        prohibited_lanelet_ids: List[int] = None,
        lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE,
        generation_strategy: Union[
            DefaultGenerationStrategy
        ] = DefaultGenerationStrategy,
        logging_level: int = logging.WARNING,
    ) -> None:
        """
        :param lanelet_network: cr lanelet network,
        :param planning_problem: cr planning problem,
        :param routes: list of LaneletSequence objects, each representing a reference_path
        :param logger: central logger
        :param prohibited_lanelet_ids: prohibited lanelet ids
        :param lane_change_method: method to compute lane changes
        :param GenerationStrategy: strategy to generate reference_path
        """

        self._logging_level = logging_level
        self._logger = logging.Logger(name=__name__, level=logging_level)
        generation_strategy.logger = self._logger

        self._lanelet_network: LaneletNetwork = lanelet_network
        self._prohibited_lanelet_ids: List[int] = (
            prohibited_lanelet_ids if (prohibited_lanelet_ids is not None) else list()
        )

        self._planning_problem: PlanningProblem = planning_problem

        self._lane_change_method: LaneChangeMethod = lane_change_method

        # create a list of ReferencePath objects for all routes found by the reference_path planner which is not empty
        self._GenerationMethod: Union[DefaultGenerationStrategy] = generation_strategy

        self._route_candidates: List[ReferencePath] = [
            generation_strategy.generate_route(
                lanelet_network=lanelet_network,
                lanelet_ids=lanelet_sequence.lanelet_ids,
                initial_state=self._planning_problem.initial_state,
                goal_region=self._planning_problem.goal,
                prohibited_lanelet_ids=prohibited_lanelet_ids,
                lane_change_method=lane_change_method,
            )
            for lanelet_sequence in routes
            if lanelet_sequence.lanelet_ids
        ]

        self._num_route_candidates: int = len(self._route_candidates)

        if self._num_route_candidates == 0:
            self._logger.error("could not compute a single reference_path due to clcs")
            raise ValueError("could not compute a single reference_path due to clcs")

    @property
    def planning_problem(self) -> PlanningProblem:
        """
        :return: cr planning problem
        """
        return self._planning_problem

    @property
    def route_candidates(self) -> List[ReferencePath]:
        """
        :return: list of routes
        """
        return self._route_candidates

    @property
    def num_route_candidates(self) -> int:
        """
        :return: number of routes found
        """
        return self._num_route_candidates

    @property
    def lane_change_method(self) -> LaneChangeMethod:
        """
        :return: lane change method
        """
        return self._lane_change_method

    def plan_shortest_reference_path(
        self,
        retrieve_shortest: bool = True,
        consider_least_lance_changes: bool = True,
        included_lanelet_ids: List[int] = None,
    ) -> ReferencePath:
        """
        Retrieves shortest reference_path object.
        Optionally can be forced to go through specific lanelets.

        :param retrieve_shortest: if True, will only find shortest distance routes,
        :param consider_least_lance_changes: considers least amount of disjoint lane changes, if possible
        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: reference_path instance
        """

        if consider_least_lance_changes:
            return self.plan_shortetest_reference_path_with_least_lane_changes(
                included_lanelet_ids=included_lanelet_ids
            )

        else:
            return self.plan_first_reference_path(
                retrieve_shortest=retrieve_shortest,
                included_lanelet_ids=included_lanelet_ids,
            )

    def plan_first_reference_path(
        self, retrieve_shortest: bool = True, included_lanelet_ids: List[int] = None
    ) -> ReferencePath:
        """
        Retrieves the first ReferencePath object.
        If retrieve shortest, the shortest reference_path is used and orientation of the lanelet is checked.

        :param retrieve_shortest: if True, only checks shortest distance routes
        :param included_lanelet_ids: forces planner to go through lanelets if possible. Ignores shortest reference_path

        :return: reference_path object
        """

        # No routes
        if len(self._route_candidates) == 0:
            self._logger.error("Not a single reference_path candidate was found")
            raise ValueError("[CR Not a single reference_path candidate was found")

        # one reference_path
        elif len(self._route_candidates) == 1 or not retrieve_shortest:
            selected_route = self._route_candidates[0]

        # multpiple routes
        else:
            sorted_routes: List[ReferencePath] = sorted(
                self._route_candidates,
                key=lambda x: x.length_reference_path,
                reverse=False,
            )

            for route in sorted_routes:
                # check init state orientation
                if self._heuristic_check_matching_orientation_of_initial_state(
                    route.reference_path
                ):
                    if included_lanelet_ids is None:
                        selected_route = route
                        break
                    elif self._check_routes_includes_lanelets(
                        route, included_lanelet_ids
                    ):
                        # additionally check if lanelets are included
                        selected_route = route
                        break
            else:
                debug_visualize(self._route_candidates, self._lanelet_network)
                self._logger.error(
                    "could not find a well oriented reference_path. Perhaps increase distance threshold, "
                    "returning first reference_path"
                )
                selected_route = sorted_routes[0]

        return selected_route

    def plan_shortetest_reference_path_with_least_lane_changes(
        self, included_lanelet_ids: List[int] = None
    ) -> ReferencePath:
        """
        Retrieves reference_path with least lane changes. Tie break is length of reference path

        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: reference_path instance
        """

        # No routes
        if len(self._route_candidates) == 0:
            self._logger.error("Not a single reference_path candidate was found")
            raise ValueError("Not a single reference_path candidate was found")

        # one reference_path
        elif len(self._route_candidates) == 1:
            selected_route = self._route_candidates[0]

        # multpiple routes
        else:
            sorted_routes: List[ReferencePath] = sorted(
                self._route_candidates,
                key=lambda x: x.num_lane_change_actions,
                reverse=False,
            )

            minimal_lane_change_routes: List[ReferencePath] = [
                route
                for route in sorted_routes
                if route.num_lane_change_actions
                == sorted_routes[0].num_lane_change_actions
            ]

            minimal_lane_change_routes_by_length = sorted(
                minimal_lane_change_routes,
                key=lambda x: x.length_reference_path,
                reverse=False,
            )

            for route in minimal_lane_change_routes_by_length:
                # check init state orientation
                if self._heuristic_check_matching_orientation_of_initial_state(
                    route.reference_path
                ):
                    if included_lanelet_ids is None:
                        selected_route = route
                        break
                    elif self._check_routes_includes_lanelets(
                        route, included_lanelet_ids
                    ):
                        # additionally check if lanelets are included
                        selected_route = route
                        break
            else:
                debug_visualize(self._route_candidates, self._lanelet_network)
                self._logger.error(
                    "could not find a well oriented reference_path. Perhaps increase distance threshold, "
                    "returning first reference_path"
                )
                selected_route = minimal_lane_change_routes_by_length[0]

        return selected_route

    def plan_all_reference_paths(self) -> Tuple[List[ReferencePath], int]:
        """
        Returns the list of ReferencePath objects and the total number of routes

        :return: Tuple of list of reference_path candidates and total number of routes
        """
        return self._route_candidates, self._num_route_candidates

    def _heuristic_check_matching_orientation_of_initial_state(
        self, reference_path: np.ndarray, distance_threshold_in_meters: float = 1.0
    ) -> bool:
        """
        Necessary to filter out the corner case, where the initial position is on multiple lanelets (i.e. on an
        intersection) and the shortest path might choose the wrong one
        """

        # Algorithm
        # ----------
        # use KDTree to check for closest point on ref path
        distance, idx = KDTree(reference_path).query(
            self._planning_problem.initial_state.position
        )

        if distance <= distance_threshold_in_meters:
            return True
        else:
            return False

    @staticmethod
    def _check_routes_includes_lanelets(
        route: ReferencePath, lanelet_ids_to_go_through: List[int]
    ) -> bool:
        """
        Checks wheter lanelets are included.
        """

        lanelet_ids: Set[int] = set(lanelet_ids_to_go_through)
        route_ids: Set[int] = set(route.lanelet_ids)
        if lanelet_ids.issubset(route_ids):
            return True
        else:
            return False

    def __repr__(self):
        return (
            f"RouteCandidateHolder(#candidates:{len(self._route_candidates)},initial_state={self.planning_problem.initial_state},"
            f"_goal_region={self.planning_problem.goal})"
        )

    def __str__(self):
        return self.__repr__()
