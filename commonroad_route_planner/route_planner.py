__author__ = "Daniel Tar, Peter Kocsis, Edmond Irani Liu"
__copyright__ = ""
__credits__ = [""]
__version__ = "2022.3"
__maintainer__ = "Edmond Irani Liu"
__email__ = "edmond.irani@tum.de"
__status__ = "Release"

from enum import Enum
from typing import List, Generator, Set

import numpy as np
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet, LaneletType, LaneletNetwork
from commonroad.scenario.state import InitialState
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.planning.goal import GoalRegion

from commonroad_route_planner.planners.a_star import AStarRoutePlanner
from commonroad_route_planner.planners.networkx import (
    ReversedNetworkxRoutePlanner,
    NetworkxRoutePlanner,
)
from commonroad_route_planner.planners.survival import SurvivalRoutePlanner
from commonroad_route_planner.route import RouteType, Route, RouteCandidateHolder
from commonroad_route_planner.utility.route import (
    lanelet_orientation_at_position,
    relative_orientation,
)

import logging

_logger = logging.getLogger(__name__)


class RoutePlanner:
    """Main class for planning routes in CommonRoad scenarios.

    This is a high-level planner that plans on the lanelet level. It returns the best routes for each pair
    of start/goal lanelets, with each route in the form of an ordered list of lanelet IDs. Depending on the
    utilized backend, the best route may have the shortest distance (if using NETWORKX and NETWORKX_REVERSED)
    or may have the lowest cost computed per the heuristic function (if using PRIORITY_QUEUE).
    In survival scenarios (no goal lanelet), the planner advances in the order of forward, right, left when possible.
    """

    class Backend(Enum):
        """Supported backend for constructing the routes.

        NETWORKX: uses built-in functions from the networkx package, tends to change lanes later
        NETWORKX_REVERSED: uses built-in functions from the networkx package, tends to change lanes earlier
        PRIORITY_QUEUE: uses A-star search to find routes, lane changing maneuver depends on the heuristic cost
        """

        NETWORKX = "networkx"
        NETWORKX_REVERSED = "networkx_reversed"
        PRIORITY_QUEUE = "priority_queue"

        @classmethod
        def values(cls):
            return [item.value for item in cls]

    def __init__(
        self,
        scenario: Scenario = None,
        planning_problem: PlanningProblem = None,
        lanelet_network: LaneletNetwork = None,
        state_initial: InitialState = None,
        goal_region: GoalRegion = None,
        set_types_lanelets_forbidden: List[LaneletType] = None,
        allow_diagonal=False,
        backend: Backend = Backend.NETWORKX,
        reach_goal_state: bool = False,
    ):
        """Initialization of a RoutePlanner object.

        :param scenario: scenario on which the routes should be planned
        :param planning_problem: planning problem for which the routes should be planned
        :param lanelet_network: lanelet network on which the routes should be planned
        :param state_initial: initial state for which the routes should be planned
        :param goal_region: goal region for which the routes should be planned
        :param set_types_lanelets_forbidden: set of lanelet types which should be avoided during route planning
        :param allow_diagonal: indicates whether diagonal movements are allowed - experimental
        :param backend: the backend to be used
        :param reach_goal_state: indicates whether the reference path should pass through the goal state (position).
        """
        # setting backend
        if backend not in RoutePlanner.Backend.values():
            backend = RoutePlanner.Backend.NETWORKX
        self.backend = backend

        self.lanelet_network = (
            lanelet_network if lanelet_network else scenario.lanelet_network
        )
        self.state_initial = (
            state_initial if state_initial else planning_problem.initial_state
        )
        self.goal_region = goal_region if goal_region else planning_problem.goal

        if scenario:
            self.scenario_id = scenario.scenario_id
            Route.scenario = scenario
        else:
            self.scenario_id = "None"

        if planning_problem:
            Route.planning_problem = planning_problem

        if set_types_lanelets_forbidden is None:
            set_types_lanelets_forbidden = set()
        self.set_types_lanelets_forbidden = set_types_lanelets_forbidden

        self.allow_diagonal = allow_diagonal
        self.reach_goal_state = reach_goal_state

        # find permissible lanelets
        list_lanelets_filtered = self._filter_lanelets_by_type(
            self.lanelet_network.lanelets, self.set_types_lanelets_forbidden
        )
        self.set_ids_lanelets_permissible = {
            lanelet_permissible.lanelet_id
            for lanelet_permissible in list_lanelets_filtered
        }

        # examine initial and goal lanelet ids
        self.id_lanelets_start = self._retrieve_ids_lanelets_start()

        (
            self.ids_lanelets_goal,
            self.ids_lanelets_goal_original,
        ) = self._retrieve_ids_lanelets_goal()

        if self.reach_goal_state and not self.ids_lanelets_goal:
            # if the predecessors of the goal states cannot be reached, fall back to reaching the goal lanelets
            self.reach_goal_state = False
            (
                self.ids_lanelets_goal,
                self.ids_lanelets_goal_original,
            ) = self._retrieve_ids_lanelets_goal()

        if len(self.ids_lanelets_goal) == 0:
            self.planner = SurvivalRoutePlanner(
                self.lanelet_network, self.set_ids_lanelets_permissible
            )
        elif self.backend == RoutePlanner.Backend.NETWORKX:
            self.planner = NetworkxRoutePlanner(
                self.lanelet_network,
                self.set_ids_lanelets_permissible, self.ids_lanelets_start_overtake,
                self.allow_diagonal,
            )
        elif self.backend == RoutePlanner.Backend.NETWORKX_REVERSED:
            self.planner = ReversedNetworkxRoutePlanner(
                self.lanelet_network, self.set_ids_lanelets_permissible,
            self.ids_lanelets_start_overtake
            )
        elif self.backend == RoutePlanner.Backend.PRIORITY_QUEUE:
            self.planner = AStarRoutePlanner(
                self.lanelet_network, self.set_ids_lanelets_permissible
            )
        else:
            assert False

        if self.ids_lanelets_goal is None:
            self.route_type = RouteType.SURVIVAL
        else:
            self.route_type = RouteType.REGULAR

    def _retrieve_ids_lanelets_start(self):
        """Retrieves the ids of the lanelets in which the initial position is situated"""
        if hasattr(self.state_initial, "position"):
            post_start = self.state_initial.position
            # noinspection PyTypeChecker
            list_ids_lanelets_start = self.lanelet_network.find_lanelet_by_position(
                [post_start]
            )[0]

            list_ids_lanelets_start = list(
                self._filter_allowed_lanelet_ids(list_ids_lanelets_start)
            )

            # Check if any of the start positions are during an overtake:
            # if the car is not driving in the correct direction for the lanelet,
            # it will also consider routes taking an adjacent lanelet in the opposite direction
            self.ids_lanelets_start_overtake = list()
            if (
                hasattr(self.state_initial, "orientation")
                and not self.state_initial.is_uncertain_orientation
            ):
                orientation = self.state_initial.orientation

                for id_lanelet_start in list_ids_lanelets_start:
                    lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet_start)
                    lanelet_angle = lanelet_orientation_at_position(lanelet, post_start)

                    # Check if the angle difference is larger than 90 degrees
                    if (
                        abs(relative_orientation(orientation, lanelet_angle))
                        > 0.5 * np.pi
                    ):
                        if (
                            lanelet.adj_left is not None
                            and not lanelet.adj_left_same_direction
                            and lanelet.adj_left in self.set_ids_lanelets_permissible
                        ):
                            self.ids_lanelets_start_overtake.append(
                                (id_lanelet_start, lanelet.adj_left)
                            )

                        elif (
                            lanelet.adj_right is not None
                            and not lanelet.adj_right_same_direction
                            and lanelet.adj_right in self.set_ids_lanelets_permissible
                        ):
                            self.ids_lanelets_start_overtake.append(
                                (id_lanelet_start, lanelet.adj_right)
                            )

            if len(list_ids_lanelets_start) > 1:
                _logger.info(
                    "Multiple start lanelet IDs: some may fail to reach goal lanelet"
                )
        else:
            _logger.critical("No initial position in the given planning problem")
            raise self.NoSourceLaneletId()

        return list_ids_lanelets_start

    def _retrieve_ids_lanelets_goal(self):
        """Retrieves the ids of the lanelets in which the goal position is situated"""
        list_ids_lanelets_goal = list()
        list_ids_lanelets_goal_original = list()

        if hasattr(self.goal_region, "lanelets_of_goal_position"):
            if self.goal_region.lanelets_of_goal_position is None:
                _logger.debug("No goal lanelet given")
            else:
                _logger.debug("Goal lanelet given")
                # the goals are stored in a dict, one goal can consist of multiple lanelets
                # now we just iterate over the goals and add every ID which we find to
                # the goal_lanelet_ids list
                for list_ids_lanelets_pos_goal in list(
                    self.goal_region.lanelets_of_goal_position.values()
                ):
                    list_ids_lanelets_goal.extend(list_ids_lanelets_pos_goal)

                list_ids_lanelets_goal = list(
                    self._filter_allowed_lanelet_ids(list_ids_lanelets_goal)
                )

        if list_ids_lanelets_goal:
            self.reach_goal_state = False

        elif hasattr(self.goal_region, "state_list"):
            for idx, state in enumerate(self.goal_region.state_list):
                if hasattr(state, "position"):
                    if hasattr(state.position, "center"):
                        pos_goal = state.position.center

                    else:
                        pos_goal = state.position
                    [
                        list_ids_lanelets_pos_goal
                    ] = self.lanelet_network.find_lanelet_by_position([pos_goal])
                    list_ids_lanelets_pos_goal = list(
                        self._filter_allowed_lanelet_ids(list_ids_lanelets_pos_goal)
                    )

                    if self.reach_goal_state:
                        # we want to reach the goal states (not just the goal lanelets), here we instead demand
                        # reaching the predecessor lanelets of the goal states
                        list_ids_lanelets_goal_original = (
                            list_ids_lanelets_pos_goal.copy()
                        )
                        list_ids_lanelets_pos_goal.clear()

                        for id_lanelet_goal in list_ids_lanelets_goal_original:
                            lanelet_goal = self.lanelet_network.find_lanelet_by_id(
                                id_lanelet_goal
                            )
                            # make predecessor as goal
                            list_ids_lanelets_pos_goal.extend(lanelet_goal.predecessor)

                    if list_ids_lanelets_pos_goal:
                        list_ids_lanelets_goal.extend(list_ids_lanelets_pos_goal)
                        _logger.debug(
                            "Goal lanelet IDs estimated from goal shape in state [{}]".format(
                                idx
                            )
                        )
                    else:
                        _logger.debug(
                            "No Goal lanelet IDs could be determined from the goal shape in state [{}]".format(
                                idx
                            )
                        )

        # remove duplicates and reset to none if no lanelet IDs found
        if list_ids_lanelets_goal:
            # remove duplicates and sort in ascending order
            list_ids_lanelets_goal = sorted(list(dict.fromkeys(list_ids_lanelets_goal)))
        else:
            list_ids_lanelets_goal = None

        return list_ids_lanelets_goal, list_ids_lanelets_goal_original

    def plan_routes(self):
        """Plans routes for every pair of start/goal lanelets.

        If no goal lanelet ID is given then return a survival route.
        :return: list of lanelet ids from start to goal.
        """
        _logger.debug("Route planner started")
        # route is a list that holds lists of lanelet ids from start lanelet to goal lanelet
        list_routes = list()

        # iterate through start lanelet ids
        for id_lanelet_start in self.id_lanelets_start:
            # iterate through goal lanelet ids
            for id_lanelet_goal in self.ids_lanelets_goal:
                list_lists_ids_lanelets = self.planner.find_routes(
                    id_lanelet_start, id_lanelet_goal
                )

                if self.reach_goal_state:
                    # append the original goal lanelet back to the found route
                    for id_lanelet_goal_original in self.ids_lanelets_goal_original:
                        for list_ids_lanelets in list_lists_ids_lanelets:
                            list_routes.append(
                                list_ids_lanelets + [id_lanelet_goal_original]
                            )

                else:
                    list_routes.extend(list_lists_ids_lanelets)

        return RouteCandidateHolder(
            self.lanelet_network,
            self.state_initial,
            self.goal_region,
            list_routes,
            self.set_ids_lanelets_permissible,
        )

    @staticmethod
    def _filter_lanelets_by_type(
        list_lanelets_to_filter: List[Lanelet],
        set_types_lanelets_forbidden: Set[LaneletType],
    ) -> Generator[Lanelet, None, None]:
        """Filters lanelets with the set of forbidden types.

        :param list_lanelets_to_filter: The list of the lanelets which should be filtered
        :return: List of desirable lanelets
        """
        for lanelet in list_lanelets_to_filter:
            if (
                len(lanelet.lanelet_type.intersection(set_types_lanelets_forbidden))
                == 0
            ):
                yield lanelet

    def _filter_allowed_lanelet_ids(
        self, list_ids_lanelets_to_filter: List[int]
    ) -> Generator[int, None, None]:
        """Filters lanelets with the list of ids of forbidden lanelets.

        :param list_ids_lanelets_to_filter: The list of the lanelet ids which should be filtered
        :return: List of desirable lanelets
        """
        for id_lanelet in list_ids_lanelets_to_filter:
            if id_lanelet in self.set_ids_lanelets_permissible:
                yield id_lanelet
