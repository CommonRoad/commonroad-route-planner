import logging
import os
from datetime import datetime
from typing import List

import networkx as nx
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork


# ================================================= #
#                   Route Planner                   #
# ================================================= #
class RoutePlanner:
    """
        Class implements route planning for the CommonRoad scenarios
        This is a higher level planner to plan only on the lanelet structure.
        It is like plan a route with google maps.
        It gives back the best route (list of lanelet IDs in the right order) from each start position to all goal positions.
        If there are no goal position then it is going forward and if it can not go forward then goes right.
        The best route is the route with the lowest cost according to the heuristic function.
        """

    def __init__(self, scenario_id: int, laneletNetwork: LaneletNetwork, planningProblem: PlanningProblem,
                 allow_diagonal=False):

        # ============================== #
        #       Binding variables        #
        # ============================== #
        self.scenario_id = scenario_id
        self.lanelet_network = laneletNetwork
        self.planningProblem = planningProblem
        # self.priority_queue = PriorityQueue()

        # ============================== #
        #        Create Logger           #
        # ============================== #
        self.logger = logging.getLogger("Route Planner [{}]".format(self.scenario_id))
        self.init_logger()

        # ================================================= #
        #                Check initial state                #
        # ================================================= #
        self.startLanelet_ids = None
        self.check_initial_state()

        # ================================================= #
        #                Check goal position                #
        # ================================================= #
        self.goal_lanelet_ids = None
        self.check_goal_state()

        # ================================================= #
        #        Create graph network from Lanelets         #
        # ================================================= #
        # if there is no goal lanelet ids than it is a survival scenario and we do not need to make
        # a graph from the lanelet network
        if self.goal_lanelet_ids is None:
            self.logger.info("SURVIVAL Scenario: There is no goal position or lanelet given")
        else:
            if allow_diagonal:
                self.logger.warning("diagonal search not tested")
                self.digraph = self.create_graph_from_lanelet_network_lane_change()
            else:
                self.digraph = self.create_graph_from_lanelet_network()

    # =============== end of constructor ============== #

    def init_logger(self, log_to_console=True, log_to_file=True, add_timestamp_to_log_file=True):
        # path relative to the running script
        log_file_dir = "solutions/logs/scenario_logs"
        log_file_name = "route_planner_result"
        # release_logger(self.logger)
        self.logger.setLevel(logging.DEBUG)
        # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_formatter = logging.Formatter('\t%(name)s\t%(levelname)s\t%(message)s')
        file_formatter = logging.Formatter('%(name)s\t%(levelname)s\t%(message)s')

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
            if not os.path.exists(log_file_dir):
                os.mkdir(log_file_dir)

            log_file_path = os.path.join(log_file_dir,
                                         "{}_{}{}.log".format(self.scenario_id, log_file_name, date_time_string))
            file_handler = logging.FileHandler(log_file_path)
            # set the level of logging to file
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(file_formatter)
            self.logger.addHandler(file_handler)

    def check_initial_state(self):
        if hasattr(self.planningProblem.initial_state, 'position'):
            start_position = self.planningProblem.initial_state.position
            # TODO: convert start_position : List[str] to List[ndarray]
            self.startLanelet_ids = self.lanelet_network.find_lanelet_by_position([start_position])[0]
            if len(self.startLanelet_ids) > 1:
                self.logger.info("More start lanelet ids - some of it can results in an unsuccessful search")
        else:
            self.logger.critical("There is no start position given")
            raise NoSourceLaneletId("There is no start position given")

    def check_goal_state(self):
        if hasattr(self.planningProblem.goal, 'lanelets_of_goal_position'):
            if self.planningProblem.goal.lanelets_of_goal_position is None:
                self.logger.info("No goal lanelet is given")
            else:
                self.logger.info("Goal lanelet is given")
                # the zero element in the dict is the list of ids
                self.goal_lanelet_ids = self.planningProblem.goal.lanelets_of_goal_position[0]

        if (self.goal_lanelet_ids is None) and hasattr(self.planningProblem.goal.state_list[0], 'position'):
            if hasattr(self.planningProblem.goal.state_list[0].position, 'center'):
                self.logger.info("Goal center is found")
                goal_position = self.planningProblem.goal.state_list[0].position
                self.goal_lanelet_ids = self.lanelet_network.find_lanelet_by_position([goal_position.center])[0]
            else:
                self.logger.info("No goal center is found")

    def find_survival_route(self, start_lanelet_id: int) -> List:
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

    def create_graph_from_lanelet_network(self) -> nx.DiGraph:
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
            for successor in lanelet.successor:
                edges.append((lanelet.lanelet_id, successor, {'weight': lanelet.distance[-1]}))

            # add edge if left lanelet
            if lanelet.adj_left and lanelet.adj_left_same_direction:
                edges.append((lanelet.lanelet_id, lanelet.adj_left, {'weight': 1}))

            # add edge if right lanelet
            if lanelet.adj_right and lanelet.adj_right_same_direction:
                edges.append((lanelet.lanelet_id, lanelet.adj_right, {'weight': 1}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def create_graph_from_lanelet_network_lane_change(self) -> nx.DiGraph:
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

    def find_all_shortest_paths(self, source_lanelet_id: int, target_lanelet_id: int = None) -> List[List]:
        """
         Find all shortest paths using networkx module
         :param source_lanelet_id: ID of source lanelet
         :param target_lanelet_id: ID of goal lanelet
         :return: list of simple paths with lanelet IDs
         """
        if source_lanelet_id is None:
            raise NoSourceLaneletId("There is no start position given")
        if target_lanelet_id is None:
            self.logger.info("SURVIVAL SCENARIO")
            return self.find_survival_route(source_lanelet_id)
        found_paths = list()
        try:
            found_paths = list(nx.all_shortest_paths(self.digraph, source=source_lanelet_id, target=target_lanelet_id,
                                                     weight='weight', method='dijkstra'))
        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self.logger.info("The Target lanelet_id [{}] cannot be reached from Source [{}]".format(target_lanelet_id,
                                                                                                    source_lanelet_id))
        return found_paths

    def search_alg(self) -> List[List]:
        """
        Find all paths to all of the goal lanelet IDs from all the start lanelet IDs using networkx module.
        If no goal lanelet ID is given then return a survival route
        :return:
        """
        routes = list()
        for start_lanelet_id in self.startLanelet_ids:
            if self.goal_lanelet_ids:
                for goal_lanelet_id in self.goal_lanelet_ids:
                    results = self.find_all_shortest_paths(start_lanelet_id, goal_lanelet_id)
                    for res in results:
                        routes.append(res)
            else:
                routes.append(self.find_all_shortest_paths(start_lanelet_id))
        return routes


# ================================================= #
#                 Custom Exceptions                 #
# ================================================= #
class NoSourceLaneletId(Exception):
    def __init__(self, message):
        self.message = message
