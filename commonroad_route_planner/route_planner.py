__author__ = "Daniel Tar, Peter Kocsis, Edmond Irani Liu, Luis Gressenbuch, Tobias Mascetta"
__copyright__ = ""
__credits__ = [""]
__version__ = "2022.3"
__maintainer__ = "Tobias Mascetta, Gerald Wuersching"
__email__ = "tobias.mascetta@tum.de"
__status__ = "Release"


import logging
from enum import Enum
import warnings
import numpy as np


# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LaneletType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState



# Own code base
from commonroad_route_planner.planners.networkx import (
    NetworkxRoutePlanner,
    ReversedNetworkxRoutePlanner,
)
from commonroad_route_planner.planners.survival import NoGoalFoundRoutePlanner
from commonroad_route_planner.route import Route, RouteCandidateHolder, RouteType
from commonroad_route_planner.utility.route import (lanelet_orientation_at_position, relative_orientation)
from commonroad_route_planner.utility.overtake_init_state import OvertakeInitState

# typing
from typing import Generator, List, Set

#  _logger = logging.getLogger(__name__)


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

        # FIXME: This is a bad implementation

        NETWORKX = "networkx"
        #NETWORKX_REVERSED = "networkx_reversed"
        #PRIORITY_QUEUE = "priority_queue"

        @classmethod
        def values(cls):
            return [item.value for item in cls]

        @classmethod
        def keys(cls):
            return [item for item in cls]


    def __init__(self,
                 scenario: Scenario = None,
                 planning_problem: PlanningProblem = None,
                 lanelet_network: LaneletNetwork = None,
                 state_initial: InitialState = None,
                 goal_region: GoalRegion = None,
                 types_lanelets_forbidden: Set[LaneletType] = None,
                 allow_diagonal=False,
                 backend: Backend = Backend.NETWORKX,
                 # FIXME: Not working correctly
                 use_predecessors_to_pass_through_goal_state: bool = False):
        """Initialization of a RoutePlanner object.

        :param scenario: scenario on which the routes should be planned
        :param planning_problem: planning problem for which the routes should be planned
        :param lanelet_network: lanelet network on which the routes should be planned
        :param state_initial: initial state for which the routes should be planned
        :param goal_region: goal region for which the routes should be planned
        :param types_lanelets_forbidden: set of lanelet types which should be avoided during route planning
        :param allow_diagonal: indicates whether diagonal movements are allowed - experimental
        :param backend: the backend to be used
        :param use_predecessors_to_pass_through_goal_state: indicates whether the reference path should pass through the goal state (position).
        """

        # setting backend
        if(backend not in self.Backend.keys() and backend not in self.Backend.values()):
            raise NotImplementedError(f'[CR Route Planner] backend {backend} not implemented')
        self.backend = backend

        # set lanelet network
        self.lanelet_network: LaneletNetwork = lanelet_network if lanelet_network else scenario.lanelet_network


        # start and goal
        self.state_initial: InitialState = state_initial if state_initial else planning_problem.initial_state
        self.goal_region: GoalRegion = goal_region if goal_region else planning_problem.goal

        # set scenario, if it exists
        if(scenario):
            self.scenario_id = scenario.scenario_id
            Route.scenario = scenario
        else:
            self.scenario_id = "None"

        # set planning problem, if it exists
        if(planning_problem):
            Route.planning_problem = planning_problem

        if(types_lanelets_forbidden is None):
            types_lanelets_forbidden = set()
        self.set_types_lanelets_forbidden: Set = types_lanelets_forbidden

        # Bool params
        self.allow_diagonal = allow_diagonal
        self.use_predecessors_to_pass_through_goal_state = use_predecessors_to_pass_through_goal_state

        # find permissible lanelets
        list_lanelets_filtered = self._filter_lanelets_by_type(
            self.lanelet_network.lanelets, self.set_types_lanelets_forbidden)

        self.ids_lanelets_permissible: Set = {
            lanelet_permissible.lanelet_id for lanelet_permissible in list_lanelets_filtered}

        # examine initial and goal lanelet ids
        self.id_lanelets_start = list()
        self.overtake_states = list()
        self.ids_lanelets_goal = list()
        self.ids_lanelets_goal_original = list()
        self._set_lanelet_ids_for_start_and_overtake()
        self._set_goal_lanelet_ids()

        # if there are no lanelets of the goal, activate the survival route planner
        if(len(self.ids_lanelets_goal) == 0):
            warnings.warn(f'[CR Route Planner] starting Survival-Mode Planner, since no goal information was found')
            self.planner = NoGoalFoundRoutePlanner(self.lanelet_network, self.ids_lanelets_permissible)

        # check different backend
        elif self.backend == RoutePlanner.Backend.NETWORKX:
            self.planner = NetworkxRoutePlanner(
                self.lanelet_network,
                self.ids_lanelets_permissible,
                self.overtake_states,
                self.allow_diagonal)

        else:
            raise NotImplementedError(f'There are lanelets at the goal state, but the backend does not match:'
                                      f'ids_lanelet_goal {self.ids_lanelets_permissible}  --  backend {self.backend}')

        # set route type
        if(self.ids_lanelets_goal is None):
            self.route_type = RouteType.SURVIVAL
        else:
            self.route_type = RouteType.REGULAR



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
        self.id_lanelets_start = (self._get_filtered_ids(
            self.lanelet_network.find_lanelet_by_position([self.state_initial.position])[0]))


        # Check if any of the start positions are during an overtake:
        # if the car is not driving in the correct direction for the lanelet,
        # it will also consider routes taking an adjacent lanelet in the opposite direction
        # FIXME: Orientation may fail because int
        if (hasattr(self.state_initial, "orientation") and not self.state_initial.is_uncertain_orientation):
            orientation = self.state_initial.orientation

            for id_lanelet_start in self.id_lanelets_start:
                lanelet: Lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet_start)
                lanelet_angle = lanelet_orientation_at_position(lanelet, self.state_initial.position)

                # Check if the angle difference is larger than 90 degrees
                if(abs(relative_orientation(orientation, lanelet_angle)) > 0.5 * np.pi):
                    if (lanelet.adj_left is not None and not lanelet.adj_left_same_direction
                            and lanelet.adj_left in self.ids_lanelets_permissible):
                        overtake_state = OvertakeInitState(id_lanelet_start, lanelet.adj_left, self.lanelet_network)
                        self.overtake_states.append(overtake_state)


                    elif (lanelet.adj_right is not None and not lanelet.adj_right_same_direction and
                          lanelet.adj_right in self.ids_lanelets_permissible):
                        overtake_state = OvertakeInitState(id_lanelet_start, lanelet.adj_right, self.lanelet_network)
                        self.overtake_states.append(overtake_state)


        if(len(self.id_lanelets_start) > 1):
            warnings.warn("Multiple start lanelet IDs: some may fail to reach goal lanelet")

        if(len(self.id_lanelets_start) == 0):
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
                    self.ids_lanelets_goal.extend(self._get_filtered_ids(list_ids_lanelets_pos_goal))


        # TODO: Why is this necessary
        if(self.ids_lanelets_goal):
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
                    # For uncertain position route planner takes first polygon
                    warnings.warn(f'[CR Route Planner] For uncertain positions, CR route planner uses the center of the first shape')
                    goal_position: np.ndarray = state.position.shapes[0].center

                # use predecessors to pass through goal state
                if(self.use_predecessors_to_pass_through_goal_state):
                    for lanelet_id_list in self.lanelet_network.find_lanelet_by_position([goal_position]):
                        for lanelet_id in lanelet_id_list:
                            lanelet: Lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
                            self.ids_lanelets_goal.extend(lanelet.predecessor)

                    # TODO: weird fallback
                    # if lanelets are empty afterwards, use normal method
                    if(len(self.ids_lanelets_goal) == 0):
                        warnings.warn(f'[CR Route Planner] want to use predecessors but could not generate goal')
                        self.use_predecessors_to_pass_through_goal_state = False
                        for lanelet_id_list in self.lanelet_network.find_lanelet_by_position([goal_position]):
                            self.ids_lanelets_goal.extend(lanelet_id_list)

                # use normal mode of computing the lanes
                else:
                    for lanelet_id_list in self.lanelet_network.find_lanelet_by_position([goal_position]):
                        self.ids_lanelets_goal.extend(lanelet_id_list)

            # remove duplicated and filter for permitted lanelets
            self.ids_lanelets_goal = self._get_filtered_ids(list(set(self.ids_lanelets_goal)))

        if(not self.ids_lanelets_goal):
            warnings.warn(f'[CR Route Planner] Could not find a single goal position or lane')





    def plan_routes(self) -> RouteCandidateHolder:
        """Plans routes for every pair of start/goal lanelets.

        If no goal lanelet ID is given then return a survival route.
        :return: list of lanelet ids from start to goal.
        """
        # route is a list that holds lists of lanelet ids from start lanelet to goal lanelet
        list_routes: List[List[int]] = list()

        # For each start lanelet, find route to each goal lanelet
        for id_lanelet_start in self.id_lanelets_start:
            # if survival route planner
            if(len(self.ids_lanelets_goal) == 0):
                list_routes.append(self.planner.find_routes(id_lanelet_start, None))

            else:
            # if normal planner iterate through goal lanelet ids
                for id_lanelet_goal in self.ids_lanelets_goal:
                    ids_lanelets = self.planner.find_routes(
                        id_lanelet_start, id_lanelet_goal
                    )

                    if self.use_predecessors_to_pass_through_goal_state:
                        list_routes.extend(ids_lanelets)
                        # append the original goal lanelet back to the found route
                        # FIXME: self.ids_lanelets_goal_original
                        #for id_lanelet_goal_original in self.ids_lanelets_goal_original:
                        #    for list_ids_lanelets in ids_lanelets:
                        #        list_routes.append(
                        #            list_ids_lanelets + [id_lanelet_goal_original]
                        #        )

                    else:
                        list_routes.extend(ids_lanelets)


        if(len(list_routes) == 0):
            raise ValueError(f'[CR Route Planner] planner {self.planner} could not find a single route')

        return RouteCandidateHolder(
            self.lanelet_network,
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

