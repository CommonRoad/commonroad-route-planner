import logging
import os
from datetime import datetime
from typing import List, Union, Generator, Set

import networkx as nx
import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import Lanelet, LaneletType
from commonroad.scenario.scenario import Scenario

from commonroad_route_planner.priority_queue import PriorityQueue


class _LaneletNode:
    def __init__(self, laneletID: int, lanelet: Lanelet, cost: float, current_length: int):
        # it must be id because of the implementation of the priority queue
        self.id = laneletID
        self.lanelet = lanelet
        self.parent_node = None
        self.cost = cost
        self.count = current_length

    def __lt__(self, other):
        return self.cost < other.cost


# ================================================= #
#                   Route Planner                   #
# ================================================= #
class RoutePlanner:
    """
        Class implements route planning for the CommonRoad scenarios
        This is a higher level planner to plan only on the lanelet structure.
        It is like plan a route with google maps.
        It gives back the best route (list of lanelet IDs in the right order)
        from each start position to all goal positions.
        If there are no goal position then it is going forward and if it can not go forward then goes right.
        The best route is the route with the lowest cost according to the heuristic function.
        """

    def __init__(self, scenario: Scenario, planning_problem: PlanningProblem,
                 lanelet_type_blacklist=None,
                 allow_diagonal=False, backend="networkx", log_to_console=True):
        """
        Initializes a RoutePlanner object
        :param scenario: Scenario which should be used for the route planning
        :param planning_problem: PlanningProblem for which the route should be planned
        :param lanelet_type_blacklist: Set of lanelet types which should be avoided during route planning
        :type lanelet_type_blacklist: Set[LaneletType]
        :param allow_diagonal: Indicates whether diagonal movements are allowed - experimental
        :param backend: The backend which should be used, supported choices: networkx, priority_queue
        :param log_to_console: Indicates whether the outputs should be logged to the console
        """

        # ============================== #
        #       Binding variables        #
        # ============================== #
        if lanelet_type_blacklist is None:
            lanelet_type_blacklist = set()

        self.scenario_id = scenario.benchmark_id
        self.lanelet_network = scenario.lanelet_network
        self.planningProblem = planning_problem
        self.lanelet_type_blacklist = lanelet_type_blacklist
        # self.priority_queue = PriorityQueue()
        self.allow_diagonal = allow_diagonal
        self.backend = backend
        # ============================== #
        #        Create Logger           #
        # ============================== #
        self.logger = logging.getLogger("Route Planner [{}]".format(self.scenario_id))
        self._init_logger(log_to_file=False, log_to_console=log_to_console)

        # ================================================= #
        #               Find allowed lanelets               #
        # ================================================= #
        self.allowed_lanelet_ids = \
            {allowed_lanelet.lanelet_id
             for allowed_lanelet in
             self._filter_lanelets_by_type(self.lanelet_network.lanelets, self.lanelet_type_blacklist)}

        # ================================================= #
        #                Check initial state                #
        # ================================================= #
        self.startLanelet_ids = None
        self._check_initial_state()

        # ================================================= #
        #                Check goal position                #
        # ================================================= #
        self.goal_lanelet_ids = None
        self._check_goal_state()

        # ================================================= #
        #        Create graph network from Lanelets         #
        # ================================================= #
        # if there is no goal lanelet ids than it is a survival scenario and we do not need to make
        # a graph from the lanelet network
        if self.goal_lanelet_ids is None:
            self.logger.info("SURVIVAL Scenario: There is no goal position or lanelet given")
        else:
            if self.backend == "networkx":
                if self.allow_diagonal:
                    self.logger.warning("diagonal search not tested")
                    self.digraph = self._create_graph_from_lanelet_network_lane_change()
                else:
                    self.digraph = self._create_graph_from_lanelet_network()
            elif self.backend == "networkx_reversed":
                if self.allow_diagonal:
                    self.logger.warning("diagonal search not tested")
                    self.digraph = self._create_reversed_graph_from_lanelet_network_lane_change()
                else:
                    self.digraph = self._create_reversed_graph_from_lanelet_network()
            elif self.backend == "priority_queue":
                if self.allow_diagonal:
                    self.logger.critical("diagonal search with custom backend is not implemented")
                self.frontier = PriorityQueue()
                self.explored = set()
            else:
                raise ValueError(f"The backend {self.backend} is not recognized as supported backend "
                                 f"for the RoutePlanner")

        # ==================== #
        #        Extra         #
        # ==================== #
        self.lanelet_ids_in_the_opposite_direction = set()

    # =============== end of constructor ============== #

    @staticmethod
    def _filter_lanelets_by_type(lanelets_to_filter: List[Lanelet], lanelet_type_blacklist: Set[LaneletType]) \
            -> Generator[Lanelet, None, None]:
        """
        Generator filters the lanelets by the defined blacklist

        :param lanelets_to_filter: The list of the lanelets which should be filtered
        :return: List of filtered lanelets
        """
        for lanelet_to_filter in lanelets_to_filter:
            if len(lanelet_to_filter.lanelet_type.intersection(lanelet_type_blacklist)) == 0:
                yield lanelet_to_filter

    def _filter_allowed_lanelet_ids(self, lanelet_ids_to_filter: List[int]) \
            -> Generator[Lanelet, None, None]:
        """
        Generator filters the lanelet ids by the defined blacklist

        :param lanelet_ids_to_filter: The list of the lanelet ids which should be filtered
        :return: List of filtered lanelets
        """
        for lanelet_id_to_filter in lanelet_ids_to_filter:
            if lanelet_id_to_filter in self.allowed_lanelet_ids:
                yield lanelet_id_to_filter

    def _init_logger(self, log_to_console=True, log_to_file=True, add_timestamp_to_log_file=True):
        # path relative to the running script
        log_file_dir = "solutions/logs/scenario_logs"
        log_file_name = "route_planner_result_with_priority_queue_backend"
        # release_logger(self.logger)
        self.logger.setLevel(logging.INFO)
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_formatter = logging.Formatter('%(asctime)s\t\t%(name)s\t%(levelname)s\t%(message)s')
        file_formatter = logging.Formatter('%(asctime)s\t%(name)s\t%(levelname)s\t%(message)s')

        if log_to_console:
            # create console handler
            console_handler = logging.StreamHandler()
            # set the level of logging to console
            console_handler.setLevel(logging.DEBUG)
            console_handler.setFormatter(console_formatter)
            self.logger.addHandler(console_handler)

        if log_to_file:
            date_time_string = ''
            if add_timestamp_to_log_file:
                now = datetime.now()  # current date and time
                date_time_string = now.strftime("_%Y_%m_%d_%H-%M-%S")

            # if directory not exists create it
            os.makedirs(log_file_dir, exist_ok=True)

            log_file_path = os.path.join(log_file_dir,
                                         "{}_{}{}.log".format(self.scenario_id, log_file_name, date_time_string))
            file_handler = logging.FileHandler(log_file_path)
            # set the level of logging to file
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(file_formatter)
            self.logger.addHandler(file_handler)

        self.logger.debug("Using backend: {}".format(self.backend))

    def _check_initial_state(self):
        if hasattr(self.planningProblem.initial_state, 'position'):
            start_position = self.planningProblem.initial_state.position
            # noinspection PyTypeChecker
            all_startLanelet_ids = self.lanelet_network.find_lanelet_by_position([start_position])[0]
            self.startLanelet_ids = list(self._filter_allowed_lanelet_ids(all_startLanelet_ids))
            if len(self.startLanelet_ids) > 1:
                self.logger.info("More start lanelet ids - some of it can results in an unsuccessful search")
        else:
            self.logger.critical("There is no start position given")
            raise self._NoSourceLaneletId("There is no start position given")

    def _check_goal_state(self):
        self.goal_lanelet_ids = list()

        if hasattr(self.planningProblem.goal, 'lanelets_of_goal_position'):
            if self.planningProblem.goal.lanelets_of_goal_position is None:
                self.logger.debug("No goal lanelet is given")
            else:
                self.logger.debug("Goal lanelet is given")
                # the goals are in the dict, one goal can consist of multiple lanelets
                # now we just iterating over the goals and adding every ID which we find to
                # the goal_lanelet_ids list
                all_goal_lanelet_ids = list()
                for all_goal_lanelet_id in list(self.planningProblem.goal.lanelets_of_goal_position.values()):
                    all_goal_lanelet_ids.extend(all_goal_lanelet_id)
                self.goal_lanelet_ids.extend(list(self._filter_allowed_lanelet_ids(all_goal_lanelet_ids)))

        if (len(self.goal_lanelet_ids) == 0) and hasattr(self.planningProblem.goal, 'state_list'):
            for idx, state in enumerate(self.planningProblem.goal.state_list):
                if hasattr(state, 'position'):
                    goal_position = state.position
                    # noinspection PyTypeChecker
                    all_goal_state_lanelet_ids = self.lanelet_network.find_lanelet_by_shape(goal_position)
                    goal_state_lanelet_ids = list(self._filter_allowed_lanelet_ids(all_goal_state_lanelet_ids))

                    if len(goal_state_lanelet_ids) != 0:
                        self.goal_lanelet_ids.extend(goal_state_lanelet_ids)
                        self.logger.debug("Goal lanelet IDs estimated from goal shape in state [{}]".format(idx))
                    else:
                        self.logger.debug(
                            "No Goal lanelet IDs could be determined from the goal shape in state [{}]".format(idx))

        # Removing duplicates and reset to none if no lanelet IDs found
        if len(self.goal_lanelet_ids) != 0:
            # remove duplicates and sort in ascending order
            self.goal_lanelet_ids = sorted(list(dict.fromkeys(self.goal_lanelet_ids)))
        else:
            self.goal_lanelet_ids = None

    def _find_survival_route(self, start_lanelet_id: int) -> List:
        """
        Finds a route along the lanelet network with a similar approach like in driving exams.
        Priority:
            1, forward
            2, right

        Notes:
            - it only considers lanes with same driving direction
            - the priority of right should be changed to left in left-rule road networks like in the UK
        :param start_lanelet_id: the initial lanelet where we start from
        :return: route that consists of a list of lanelet IDs
        """
        route = list()

        lanelet = self.lanelet_network.find_lanelet_by_id(start_lanelet_id)

        # TODO: a minimal distance check could be added in a recursive greedy best first search algorithm
        # basically we are always going forward and if we can not go forward the go right
        # this approach is similar to the one in driving exams
        # it goes until the end of the lanelet network or when it is hits itself (like dying in the Snake game)
        while lanelet.lanelet_id not in route:

            route.append(lanelet.lanelet_id)

            if lanelet.successor:
                # naively concatenate successors
                # TODO:
                #  use some rules? eg.:
                #   check which of the successors goes better in the direction of the previous lane and choose that one
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.successor[0])
            # add edge if right lanelet
            elif lanelet.adj_right and lanelet.adj_right_same_direction:
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
            # add edge if left lanelet
            elif lanelet.adj_left and lanelet.adj_left_same_direction:
                break
                # lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
            # break out of the loop
            else:
                break

        return route

    def _create_graph_from_lanelet_network(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network.

        :return: created graph from lanelet network
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in self.lanelet_network.lanelets:
            if lanelet.lanelet_id not in self.allowed_lanelet_ids:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelet
            for successor in lanelet.successor:
                if successor not in self.allowed_lanelet_ids:
                    continue
                edges.append((lanelet.lanelet_id, successor, {'weight': lanelet.distance[-1]}))

            # add edge if left lanelet
            adj_left = lanelet.adj_left
            if adj_left and lanelet.adj_left_same_direction and adj_left in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, lanelet.adj_left, {'weight': 1.0}))

            # add edge if right lanelet
            adj_right = lanelet.adj_right
            if adj_right and lanelet.adj_right_same_direction and adj_right in self.allowed_lanelet_ids:
                edges.append((lanelet.lanelet_id, lanelet.adj_right, {'weight': 1.0}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_reversed_graph_from_lanelet_network(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network.

        :return: created graph from lanelet network
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()
        for lanelet in self.lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelet
            for predecessor in lanelet.predecessor:
                edges.append((lanelet.lanelet_id, predecessor, {'weight': lanelet.distance[-1]}))

            # add edge if left lanelet
            if lanelet.adj_left and lanelet.adj_left_same_direction:
                edges.append((lanelet.lanelet_id, lanelet.adj_left, {'weight': np.inf}))

            # add edge if right lanelet
            if lanelet.adj_right and lanelet.adj_right_same_direction:
                edges.append((lanelet.lanelet_id, lanelet.adj_right, {'weight': np.inf}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_graph_from_lanelet_network_lane_change(self) -> nx.DiGraph:
        """
        Build a graph from the lanelet network allowing diagonal lane changes
        TODO: test implementation
        :return: created graph from lanelet network with diagonal lane changes
        """

        graph = nx.DiGraph()
        nodes = list()
        edges = list()

        # check for each lanelet for diagonal successors
        for lanelet in self.lanelet_network.lanelets:
            nodes.append(lanelet.lanelet_id)

            # check if lanelet has successor
            # add edge if succeeding lanelet
            for successor in lanelet.successor:
                successor_lanelet = self.lanelet_network.find_lanelet_by_id(successor)
                edges.append((lanelet.lanelet_id, successor_lanelet.lanelet_id, {'weight': lanelet.distance[-1]}))

                # check for diagonal left succeeding lanelet
                if successor_lanelet.adj_left and lanelet.adj_left_same_direction:
                    edges.append((lanelet.lanelet_id, successor_lanelet.adj_right,
                                  {'weight': 0}))

                # check for diagonal right succeeding lanelet
                if successor_lanelet.adj_right and lanelet.adj_right_same_direction:
                    edges.append((lanelet.lanelet_id, successor_lanelet.adj_right,
                                  {'weight': 0}))

            # check if succeeding lanelet of right lanelet (e.g. turning lane highway)
            if lanelet.adj_right and lanelet.adj_right_same_direction:
                l_right = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)

                # check for diagonal right succeeding lanelet
                for right_successor in l_right.successor:

                    # if not already in graph add it
                    if (lanelet.lanelet_id, right_successor, {'weight': 0}) not in edges:
                        edges.append((lanelet.lanelet_id, right_successor, {'weight': 0}))

            # check if succeeding lanelet of right lanelet (e.g. turning lane highway)
            if lanelet.adj_left and lanelet.adj_left_same_direction:
                l_left = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)

                # check for diagonal left succeeding lanelet
                for left_successor in l_left.successor:

                    # if not already in graph add it
                    if (lanelet.lanelet_id, left_successor, {'weight': 0}) not in edges:
                        edges.append((lanelet.lanelet_id, left_successor, {'weight': 0}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_reversed_graph_from_lanelet_network_lane_change(self):
        raise NotImplementedError

    def _find_all_shortest_paths(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        """
         Find all shortest paths using networkx module
         :param source_lanelet_id: ID of source lanelet
         :param target_lanelet_id: ID of goal lanelet
         :return: list of simple paths with lanelet IDs
         """
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")
        if target_lanelet_id is None:
            self.logger.info("SURVIVAL SCENARIO")
            return self._find_survival_route(source_lanelet_id)
        found_paths = list()
        try:
            found_paths = list(nx.all_shortest_paths(self.digraph, source=source_lanelet_id, target=target_lanelet_id,
                                                     weight='weight', method='dijkstra'))
        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self.logger.debug("The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                                     source_lanelet_id))
        return found_paths

    def _find_shortest_reversed_path(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        """
         Find the shortest paths reversed using networkx module
         :param source_lanelet_id: ID of source lanelet
         :param target_lanelet_id: ID of goal lanelet
         :return: list of lanelet IDs
         """
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")
        if target_lanelet_id is None:
            self.logger.info("SURVIVAL SCENARIO")
            return self._find_survival_route(source_lanelet_id)
        found_paths = list()
        try:
            found_paths.append(list(nx.shortest_path(self.digraph, source=target_lanelet_id, target=source_lanelet_id,
                                                     weight='weight', method='dijkstra'))[::-1])
        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self.logger.debug("The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                                     source_lanelet_id))
        return found_paths

    @staticmethod
    def _calc_cost(current: Lanelet) -> float:
        return current.distance[-1]

    @staticmethod
    def _calc_heuristic(current: Lanelet, target: Lanelet) -> float:
        diff = target.center_vertices[0] - current.center_vertices[-1]
        return np.sqrt(np.dot(diff, diff))

    def _add_child(self, parent_node: _LaneletNode, next_lanelet_id: int, target: Lanelet, extra_cost: float = 0.0):
        next_lanelet = self.lanelet_network.find_lanelet_by_id(next_lanelet_id)

        frontier_lanelet_ids = self.frontier.get_item_ids()
        cost = extra_cost + parent_node.cost + self._calc_cost(next_lanelet) + self._calc_heuristic(next_lanelet,
                                                                                                    target)

        node = _LaneletNode(next_lanelet_id, next_lanelet, cost, parent_node.count + 1)
        node.parent_node = parent_node

        # TODO: check speed
        if (next_lanelet_id not in self.explored) and (next_lanelet_id not in frontier_lanelet_ids):
            self.frontier.put(next_lanelet_id, node, cost)
        elif next_lanelet_id in frontier_lanelet_ids:
            self.frontier.update_item_if_exists(next_lanelet_id, node, cost)

    def _find_astar_path(self, source_lanelet_id, target_lanelet_id: int) -> List:

        if source_lanelet_id not in self.allowed_lanelet_ids:
            raise self._NoPathFound("It is not allowed to drive on the source lanelet, check the type of the lanelet")

        if target_lanelet_id not in self.allowed_lanelet_ids:
            raise self._NoPathFound("It is not allowed to drive on the target lanelet, check the type of the lanelet")

        self.frontier = PriorityQueue()
        self.explored = set()

        target_lanelet = self.lanelet_network.find_lanelet_by_id(target_lanelet_id)

        lanelet = self.lanelet_network.find_lanelet_by_id(source_lanelet_id)
        node = _LaneletNode(source_lanelet_id, lanelet,
                            self._calc_cost(lanelet) + self._calc_heuristic(lanelet, target_lanelet), 1)

        self.frontier.put(node.id, node, node.cost)
        while not self.frontier.is_empty():

            node: _LaneletNode = self.frontier.pop()

            # maybe the frontier is not empty but only contains invalid elements
            if node is None:
                continue

            if node.id == target_lanelet_id:
                break

            self.explored.add(node.id)

            # ================= #
            #    expand node    #
            # ================= #
            lanelet = node.lanelet
            # add successors
            for successor_id in lanelet.successor:
                if successor_id not in self.allowed_lanelet_ids:
                    continue
                self._add_child(node, successor_id, target_lanelet, node.cost)

            # if we are changing lanelets then remove the lanelet lengths because it would be added twice
            lanelet_length = self._calc_cost(lanelet)

            # add left lanelet
            adj_left_id = lanelet.adj_left
            if lanelet.adj_left_same_direction and adj_left_id and adj_left_id in self.allowed_lanelet_ids:
                self._add_child(node, adj_left_id, target_lanelet, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    left_lanelet_successor_ids = self.lanelet_network.find_lanelet_by_id(adj_left_id).successor
                    for left_lanelet_successor_id in left_lanelet_successor_ids:
                        self._add_child(node, left_lanelet_successor_id, target_lanelet, 0.9)

            # add right lanelet
            adj_right_id = lanelet.adj_right
            if lanelet.adj_right_same_direction and adj_right_id and adj_right_id in self.allowed_lanelet_ids:
                self._add_child(node, adj_right_id, target_lanelet, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    right_lanelet_successor_ids = self.lanelet_network.find_lanelet_by_id(adj_right_id).successor
                    for right_lanelet_successor_id in right_lanelet_successor_ids:
                        self._add_child(node, right_lanelet_successor_id, target_lanelet, 0.9)
        else:
            raise self._NoPathFound(
                "The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                       source_lanelet_id))

        reverse_path = list()

        while node is not None:
            reverse_path.append(node.id)
            node = node.parent_node

        return reverse_path[::-1]

    def _find_path(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        found_paths = list()
        if source_lanelet_id is None:
            raise self._NoSourceLaneletId("There is no start position given")

        if target_lanelet_id is None:
            self.logger.debug("SURVIVAL SCENARIO")
            found_paths.append(self._find_survival_route(source_lanelet_id))
        else:
            try:
                found_paths.append(self._find_astar_path(source_lanelet_id, target_lanelet_id))
            except self._NoPathFound:
                # it is a normal behaviour because of the overlapping lanelets in a road network
                self.logger.debug(
                    "The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                           source_lanelet_id))
        return found_paths

    def search_alg(self) -> List[List]:
        """
        Find all paths to all of the goal lanelet IDs from all the start lanelet IDs using networkx module.
        If no goal lanelet ID is given then return a survival route
        :return: empty list if no path has been found
        """
        self.logger.info("Route planning started")
        routes = list()
        for start_lanelet_id in self.startLanelet_ids:
            if self.goal_lanelet_ids:
                for goal_lanelet_id in self.goal_lanelet_ids:
                    if self.backend == "networkx":
                        results = self._find_all_shortest_paths(start_lanelet_id, goal_lanelet_id)
                    elif self.backend == "networkx_reversed":
                        results = self._find_shortest_reversed_path(start_lanelet_id, goal_lanelet_id)
                    elif self.backend == "priority_queue":
                        results = self._find_path(start_lanelet_id, goal_lanelet_id)
                    else:
                        raise ValueError(f"The backend {self.backend} is not recognized as supported backend "
                                         f"for the RoutePlanner")

                    routes.extend(results)
            else:
                routes.append(self._find_survival_route(start_lanelet_id))
        self.logger.info("Route planning finished")
        return routes

    def _get_adjacent_lanelets_list(self, lanelet_id: int, is_opposite_direction_allowed=False) -> list:
        """
        Recursively gets adj_left and adj_right lanelets of current lanelet id
        :param lanelet_id: current lanelet id
        :param is_opposite_direction_allowed: specifies if it should give back only the lanelets in the driving
            direction or it should give back the first neighbouring lanelet in the opposite direction
        :return: list of adjacent lanelet ids: all lanelets which are going in the same direction and one-one from the
                 left and right side which are going in the opposite direction, empty lists if there are none
        """
        adjacent_list = list()
        base_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)

        # left direction
        current_lanelet = base_lanelet
        temp_id = current_lanelet.adj_left
        while temp_id is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if temp_id in self.allowed_lanelet_ids:
                if current_lanelet.adj_left_same_direction:
                    if temp_id in self.allowed_lanelet_ids:
                        # append the left adjacent lanelet if it exists
                        adjacent_list.append(temp_id)

                        # Update current_lanelet
                        current_lanelet = self.lanelet_network.find_lanelet_by_id(temp_id)
                        temp_id = current_lanelet.adj_left

                # this lanelet was already such which goes in the opposite direction -> exit the loop
                else:
                    self.lanelet_ids_in_the_opposite_direction.add(temp_id)
                    if is_opposite_direction_allowed:
                        adjacent_list.append(temp_id)
                    break
            else:
                # it is not allowed to drive in that lane, so just breaking
                break

        # right direction
        current_lanelet = base_lanelet
        temp_id = current_lanelet.adj_right
        while temp_id is not None:
            # set this lanelet as the current if it goes in the same direction and iterate further
            if temp_id in self.allowed_lanelet_ids:
                if current_lanelet.adj_right_same_direction:
                    # append the right adjacent lanelet if it exists
                    adjacent_list.append(temp_id)

                    # Update current_lanelet
                    current_lanelet = self.lanelet_network.find_lanelet_by_id(temp_id)
                    temp_id = current_lanelet.adj_right
                # this lanelet was already such which goes in the opposite direction -> exit the loop
                else:
                    self.lanelet_ids_in_the_opposite_direction.add(temp_id)
                    if is_opposite_direction_allowed:
                        adjacent_list.append(temp_id)
                    break
            else:
                # it is not allowed to drive in that lane, so just breaking
                break

        return adjacent_list

    def get_sectionized_environment_from_route(self, route: List[int], is_opposite_direction_allowed: bool = False) -> \
            Union[None, List[List[int]]]:
        """
        Creates sectionized environment from a given route.
        :param route: The route as a list of lanelet ids
        :param is_opposite_direction_allowed: Indicates whether it is required to contain one opposite lanelet in the
                                              environment
        :return: Returns a sectional environment in a form of list of list of lanelet ids. This environment contains
                 the environment of the planned route in every time steps.
        """
        if route is None:
            return None

        sections = list()
        for lanelet_id_in_route in route:
            lanelet_ids_in_section = self._get_adjacent_lanelets_list(lanelet_id_in_route,
                                                                      is_opposite_direction_allowed)
            lanelet_ids_in_section.append(lanelet_id_in_route)
            lanelet_ids_in_section.sort()

            if len(sections) == 0:
                sections.append(lanelet_ids_in_section)
            elif sections[-1] != lanelet_ids_in_section:
                sections.append(lanelet_ids_in_section)

        return sections

    # ================================================= #
    #                 Custom Exceptions                 #
    # ================================================= #
    class _NoSourceLaneletId(Exception):
        def __init__(self, message):
            self.message = message

    class _NoPathFound(Exception):
        def __init__(self, message):
            self.message = message
