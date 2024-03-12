import numpy as np
import logging

# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState

# own code base
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.visualization import debug_visualize
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)
from commonroad_route_planner.route_generation_strategies.default_generation_strategy import (
    DefaultGenerationStrategy,
)

# typing
from typing import List, Set, Tuple, Union


class RouteGenerator:
    """
    Generates routes from a given sequence of lanelets, per default the shortest route.
    Offers methods to select them.
    """

    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        initial_state: InitialState,
        goal_region: GoalRegion,
        route_candidates: List[List[int]],
        logger: logging.Logger,
        prohibited_lanelet_ids: List[int] = None,
        lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE,
        GenerationStrategy: Union[
            DefaultGenerationStrategy
        ] = DefaultGenerationStrategy,
    ) -> None:
        """
        :param lanelet_network: cr lanelet network,
        :param initial_state: cr initial state
        :param goal_region: cr goal
        :param route_candidates: list of list of lanelet ids of routes
        :param logger: central logger
        :param prohibited_lanelet_ids: prohibited lanelet ids
        :param lane_change_method: method to compute lane changes
        :param GenerationStrategy: strategy to generate route
        """

        self._logger = logger
        GenerationStrategy.logger = logger

        self._lanelet_network: LaneletNetwork = lanelet_network
        self._prohibited_lanelet_ids: List[int] = (
            prohibited_lanelet_ids if (prohibited_lanelet_ids is not None) else list()
        )

        self._initial_state: InitialState = initial_state
        self._goal_region: GoalRegion = goal_region

        self._lane_change_method: LaneChangeMethod = lane_change_method

        # create a list of Route objects for all routes found by the route planner which is not empty
        self._GenerationMethod: Union[DefaultGenerationStrategy] = GenerationStrategy

        self._route_candidates: List[Route] = [
            GenerationStrategy.generate_route(
                lanelet_network=lanelet_network,
                lanelet_ids=lanelet_ids,
                initial_state=initial_state,
                goal_region=goal_region,
                prohibited_lanelet_ids=prohibited_lanelet_ids,
                lane_change_method=lane_change_method,
            )
            for lanelet_ids in route_candidates
            if lanelet_ids
        ]

        self._num_route_candidates: int = len(self._route_candidates)

        if self._num_route_candidates == 0:
            self._logger.error("could not compute a single route due to clcs")
            raise ValueError("could not compute a single route due to clcs")

    @property
    def route_candidates(self) -> List[Route]:
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

    def retrieve_shortest_route(
        self,
        retrieve_shortest: bool = True,
        consider_least_lance_changes: bool = True,
        included_lanelet_ids: List[int] = None,
    ) -> Route:
        """
        Retrieves shortest route object.
        Optionally can be forced to go through specific lanelets.

        :param retrieve_shortest: if True, will only find shortest distance routes,
        :param consider_least_lance_changes: considers least amount of disjoint lane changes, if possible
        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: route instance
        """

        if consider_least_lance_changes:
            return self.retrieve_shortetest_route_with_least_lane_changes(
                included_lanelet_ids=included_lanelet_ids
            )

        else:
            return self.retrieve_first_route(
                retrieve_shortest=retrieve_shortest,
                included_lanelet_ids=included_lanelet_ids,
            )

    def retrieve_first_route(
        self, retrieve_shortest: bool = True, included_lanelet_ids: List[int] = None
    ) -> Route:
        """
        Retrieves the first Route object.
        If retrieve shortest, the shortest route is used and orientation of the lanelet is checked.

        :param retrieve_shortest: if True, only checks shortest distance routes
        :param included_lanelet_ids: forces planner to go through lanelets if possible. Ignores shortest route

        :return: route object
        """

        # No routes
        if len(self._route_candidates) == 0:
            self._logger.error("Not a single route candidate was found")
            raise ValueError("[CR Not a single route candidate was found")

        # one route
        elif len(self._route_candidates) == 1 or not retrieve_shortest:
            selected_route = self._route_candidates[0]

        # multpiple routes
        else:
            sorted_routes: List[Route] = sorted(
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
                    "could not find a well oriented route. Perhaps increase distance threshold, "
                    "returning first route"
                )
                selected_route = sorted_routes[0]

        return selected_route

    def retrieve_shortetest_route_with_least_lane_changes(
        self, included_lanelet_ids: List[int] = None
    ) -> Route:
        """
        Retrieves route with least lane changes. Tie break is length of reference path

        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: route instance
        """

        # No routes
        if len(self._route_candidates) == 0:
            self._logger.error("Not a single route candidate was found")
            raise ValueError("Not a single route candidate was found")

        # one route
        elif len(self._route_candidates) == 1:
            selected_route = self._route_candidates[0]

        # multpiple routes
        else:
            sorted_routes: List[Route] = sorted(
                self._route_candidates,
                key=lambda x: x.num_lane_change_actions,
                reverse=False,
            )

            minimal_lane_change_routes: List[Route] = [
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
                    "could not find a well oriented route. Perhaps increase distance threshold, "
                    "returning first route"
                )
                selected_route = minimal_lane_change_routes_by_length[0]

        return selected_route

    def retrieve_all_routes(self) -> Tuple[List[Route], int]:
        """
        Returns the list of Route objects and the total number of routes

        :return: Tuple of list of route candidates and total number of routes
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
        distance, idx = KDTree(reference_path).query(self._initial_state.position)

        if distance <= distance_threshold_in_meters:
            return True
        else:
            return False

    @staticmethod
    def _check_routes_includes_lanelets(
        route: Route, lanelet_ids_to_go_through: List[int]
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
            f"RouteCandidateHolder(#candidates:{len(self._route_candidates)},initial_state={self._initial_state},"
            f"_goal_region={self._goal_region})"
        )

    def __str__(self):
        return self.__repr__()
