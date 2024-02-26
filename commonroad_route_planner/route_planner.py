__author__ = "Daniel Tar, Peter Kocsis, Edmond Irani Liu, Luis Gressenbuch, Tobias Mascetta"
__copyright__ = ""
__credits__ = [""]
__version__ = "2022.3"
__maintainer__ = "Tobias Mascetta, Gerald Wuersching"
__email__ = "tobias.mascetta@tum.de"
__status__ = "Release"


#############################################################
#
#
#  FIXME: Refactor implementation
#
#
#
#
#
#
#
######################################################################

import logging
from enum import Enum
import warnings
import math

import numpy as np


# third party
from scipy.spatial.kdtree import KDTree


# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LaneletType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState



# Own code base
from commonroad_route_planner.planners.networkx import (
    NetworkxRoutePlanner,
)
from commonroad_route_planner.planners.survival import NoGoalFoundRoutePlanner
from commonroad_route_planner.route import Route, RouteType
from commonroad_route_planner.route_selector import RouteSelector
from commonroad_route_planner.utility.route_util import (lanelet_orientation_at_position, relative_orientation)
from commonroad_route_planner.utility.overtake_init_state import OvertakeInitState

# typing
from typing import Generator, List, Set, Tuple, Union

#  _logger = logging.getLogger(__name__)


class RoutePlanner:
    """Main class for planning routes in CommonRoad scenarios.

    This is a high-level _planner that plans on the lanelet level. It returns the best routes for each pair
    of start/goal lanelets, with each route in the form of an ordered list of lanelet IDs. Depending on the
    utilized backend, the best route may have the shortest distance (if using NETWORKX and NETWORKX_REVERSED)
    or may have the lowest cost computed per the heuristic function (if using PRIORITY_QUEUE).
    In survival scenarios (no goal lanelet), the _planner advances in the order of forward, right, left when possible.
    """




    def __init__(self,
                 scenario: Scenario,
                 planning_problem: PlanningProblem,
                 extended_search: bool = False
                 ) -> None:
        """
        Initialization of a RoutePlanner object.

        :param scenario: cr scenario
        :param planning_problem: cr planning problem
        :param extended_search: necessary, if not the shortest route is searched, e.g. if a specific lanelet must be included
        """

        self._lanelet_network: LaneletNetwork = scenario.lanelet_network

        self._planning_problem: PlanningProblem = planning_problem

        self._extended_search: bool = extended_search


        # examine initial and goal lanelet ids
        self._id_lanelets_start: List[int] = list()
        self._overtake_states = list()
        self._ids_lanelets_goal: List[int] = list()
        self._ids_lanelets_goal_original: List[int] = list()
        self._set_lanelet_ids_for_start_and_overtake()
        self._set_goal_lanelet_ids()


        self._planner: Union[NetworkxRoutePlanner, NoGoalFoundRoutePlanner] = None
        self._init_planner()



    def _init_planner(self) -> None:
        """
        Initialize planner
        """
        # if there are no lanelets of the goal, activate the NoGoalFound _planner
        if(len(self._ids_lanelets_goal) == 0):
            warnings.warn(f'[CR Route Planner] starting NoGoalFound Planner, since no goal information was found')
            self._planner = NoGoalFoundRoutePlanner(self._lanelet_network, self._ids_lanelets_permissible)

        # check different backend
        else:
            self._planner = NetworkxRoutePlanner(
                lanelet_network=self._lanelet_network,
                overtake_states=self._overtake_states,
                extended_search=self._extended_search
            )





    def _set_lanelet_ids_for_start_and_overtake(self) -> None:
        """
        Retrieves the ids of the lanelets in which the initial position is situated.
        Also checks if the initial state is during a lanechange
        """

        # sanity check
        if(not hasattr(self.state_initial, "position")):
            raise ValueError(f'No initial position in the given planning problem found')

        # FIXME: Why is only the lanelet at index 0 used
        # Add start lanelets
        self._id_lanelets_start = (self._get_filtered_ids(
            self._lanelet_network.find_lanelet_by_position([self.state_initial.position])[0]))


        # Check if any of the start positions are during an overtake:
        # if the car is not driving in the correct direction for the lanelet,
        # it will also consider routes taking an adjacent lanelet in the opposite direction
        # FIXME: Orientation may fail because int
        if (hasattr(self.state_initial, "orientation") and not self.state_initial.is_uncertain_orientation):
            orientation = self.state_initial.orientation

            for id_lanelet_start in self._id_lanelets_start:
                lanelet: Lanelet = self._lanelet_network.find_lanelet_by_id(id_lanelet_start)
                lanelet_angle = lanelet_orientation_at_position(lanelet, self.state_initial.position)

                # Check if the angle difference is larger than 90 degrees
                if(abs(relative_orientation(orientation, lanelet_angle)) > 0.5 * np.pi):
                    if (lanelet.adj_left is not None and not lanelet.adj_left_same_direction
                            and lanelet.adj_left in self.ids_lanelets_permissible):
                        overtake_state = OvertakeInitState(id_lanelet_start, lanelet.adj_left, self._lanelet_network)
                        self._overtake_states.append(overtake_state)


                    elif (lanelet.adj_right is not None and not lanelet.adj_right_same_direction and
                          lanelet.adj_right in self.ids_lanelets_permissible):
                        overtake_state = OvertakeInitState(id_lanelet_start, lanelet.adj_right, self._lanelet_network)
                        self._overtake_states.append(overtake_state)


        if(len(self._id_lanelets_start) > 1):
            warnings.warn("Multiple start lanelet IDs: some may fail to reach goal lanelet")

        if(len(self._id_lanelets_start) == 0):
            raise ValueError(f'No initial lanelet ids found')






    def _set_goal_lanelet_ids(self) -> None:
        """
        Sets the goal lanelet ids in the attribute.
        Takes first goal polygon for uncertain goal position
        """

        # If the goal region is directly defined by lanelets, use it
        if (hasattr(self.goal_region, "lanelets_of_goal_position")):
            if (self.goal_region.lanelets_of_goal_position is None):
                warnings.warn(f'[CR Route Planner] lanelets_of_goal_position not given')

            else:
                for list_ids_lanelets_pos_goal in self.goal_region.lanelets_of_goal_position.values():
                    self._ids_lanelets_goal.extend(self._get_filtered_ids(list_ids_lanelets_pos_goal))


        # TODO: Why is this necessary
        if(self._ids_lanelets_goal):
            self.use_predecessors_to_pass_through_goal_state = False

        # if the goal region has a state list, also use it
        if(hasattr(self.goal_region, "state_list")):
            for idx, state in enumerate(self.goal_region.state_list):

                if(not hasattr(state, "position")):
                    warnings.warn(f'[CR Route Planner] goal state of state list has no position entry, will pass')
                    continue


                # set goal position, which can either be defined by center for regions or by position
                if(hasattr(state.position, "center")):
                    goal_position: np.ndarray = state.position.center
                else:
                    # For uncertain position route _planner takes first polygon
                    warnings.warn(f'[CR Route Planner] For uncertain positions, CR route _planner uses the center of the first shape')
                    goal_position: np.ndarray = state.position.shapes[0].center

                # use predecessors to pass through goal state
                if(self.use_predecessors_to_pass_through_goal_state):
                    for lanelet_id_list in self._lanelet_network.find_lanelet_by_position([goal_position]):
                        for lanelet_id in lanelet_id_list:
                            lanelet: Lanelet = self._lanelet_network.find_lanelet_by_id(lanelet_id)
                            self._ids_lanelets_goal.extend(lanelet.predecessor)

                    # TODO: weird fallback
                    # if lanelets are empty afterwards, use normal method
                    if(len(self._ids_lanelets_goal) == 0):
                        warnings.warn(f'[CR Route Planner] want to use predecessors but could not generate goal')
                        self.use_predecessors_to_pass_through_goal_state = False
                        for lanelet_id_list in self._lanelet_network.find_lanelet_by_position([goal_position]):
                            self._ids_lanelets_goal.extend(lanelet_id_list)

                # use normal mode of computing the lanes
                else:
                    for lanelet_id_list in self._lanelet_network.find_lanelet_by_position([goal_position]):
                        self._ids_lanelets_goal.extend(lanelet_id_list)

            # remove duplicated and filter for permitted lanelets
            self._ids_lanelets_goal = self._get_filtered_ids(list(set(self._ids_lanelets_goal)))

        if(not self._ids_lanelets_goal):
            warnings.warn(f'[CR Route Planner] Could not find a single goal position or lane')





    def plan_routes(self) -> RouteSelector:
        """Plans routes for every pair of start/goal lanelets.

        If no goal lanelet ID is given then return a survival route.
        :return: list of lanelet ids from start to goal.
        """
        # route is a list that holds lists of lanelet ids from start lanelet to goal lanelet
        list_routes: List[List[int]] = list()

        # For each start lanelet, find route to each goal lanelet
        for id_lanelet_start in self._id_lanelets_start:
            # if survival route _planner
            if(len(self._ids_lanelets_goal) == 0):
                list_routes.append(self._planner.find_routes(id_lanelet_start, None))

            else:
            # if normal _planner iterate through goal lanelet ids
                for id_lanelet_goal in self._ids_lanelets_goal:
                    ids_lanelets = self._planner.find_routes(
                        id_lanelet_start, id_lanelet_goal
                    )

                    if self.use_predecessors_to_pass_through_goal_state:
                        list_routes.extend(ids_lanelets)
                        # append the original goal lanelet back to the found route
                        # FIXME: self._ids_lanelets_goal_original
                        #for id_lanelet_goal_original in self._ids_lanelets_goal_original:
                        #    for list_ids_lanelets in ids_lanelets:
                        #        list_routes.append(
                        #            list_ids_lanelets + [id_lanelet_goal_original]
                        #        )

                    else:
                        list_routes.extend(ids_lanelets)


        if(len(list_routes) == 0):
            raise ValueError(f'[CR Route Planner] _planner {self._planner} could not find a single route')

        return RouteSelector(
            self._lanelet_network,
            self.state_initial,
            self.goal_region,
            list_routes,
            self.ids_lanelets_permissible)


    @staticmethod
    def _filter_lanelets_by_type(list_lanelets_to_filter: List[Lanelet],
        set_types_lanelets_forbidden: Set[LaneletType]) -> Generator[Lanelet, None, None]:
        """Filters lanelets with the set of forbidden types.

        :param list_lanelets_to_filter: The list of the lanelets which should be filtered
        :return: List of desirable lanelets
        """
        for lanelet in list_lanelets_to_filter:
            if (len(lanelet.lanelet_type.intersection(set_types_lanelets_forbidden)) == 0):
                yield lanelet


    def _get_filtered_ids(self, ids_lanelets_to_filter: List[int]) -> List[int]:
        """Filters lanelets with the list of ids of forbidden lanelets.

        :param ids_lanelets_to_filter: The list of the lanelet ids which should be filtered
        :return: List of desirable lanelets
        """
        filtered_ids = list()
        for id_lanelet in ids_lanelets_to_filter:
            if id_lanelet in self.ids_lanelets_permissible:
                filtered_ids.append(id_lanelet)

        return filtered_ids

