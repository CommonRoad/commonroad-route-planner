import logging
from typing import List

import numpy as np
from commonroad.scenario.lanelet import Lanelet

from commonroad_route_planner.planners.base_route_planner import BaseRoutePlanner
from commonroad_route_planner.utility.exceptions import (
    NoPathFoundException,
    NoSourceLaneletIdException,
)
from commonroad_route_planner.utility.priority_queue import PriorityQueue

_logger = logging.getLogger(__name__)


class LaneletNode:
    """Custom node class to represent a lanelet.

    This is used to represent the lanelets as nodes when performing A-star search.
    """

    def __init__(
        self, id_lanelet: int, lanelet: Lanelet, cost: float, length_current: int
    ):
        self.id = id_lanelet
        self.lanelet = lanelet
        self.cost = cost
        self.count = length_current
        self.parent_node = None

    def __lt__(self, other):
        # define '<' operation for comparison
        return self.cost < other.cost


class AStarRoutePlanner(BaseRoutePlanner):
    def find_routes(self, id_lanelet_start, id_lanelet_goal):
        """Find the shortest paths using own implementation (A-star)

        Lane change depends on the heuristic cost.

        :param id_lanelet_start: ID of source lanelet
        :param id_lanelet_goal: ID of goal lanelet
        :return: list of lists of lanelet IDs
        """
        list_lanelets = list()

        if id_lanelet_start is None:
            raise NoSourceLaneletIdException()

        try:
            list_lanelets.append(
                self._find_route_astar(id_lanelet_start, id_lanelet_goal)
            )
        except NoPathFoundException:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            _logger.debug(
                f"The goal lanelet with ID [{id_lanelet_goal}] cannot be reached from the start lanelet with ID [{id_lanelet_start}]"
            )
        return list_lanelets

    @staticmethod
    def _calc_cost_travel(current: Lanelet) -> float:
        # use the length of the lanelet as the travel cost
        return current.distance[-1]

    @staticmethod
    def _calc_cost_heuristic(lanelet_current: Lanelet, lanelet_goal: Lanelet) -> float:
        # use the distance between the vertices of the center lines as the heuristic cost
        diff = lanelet_goal.center_vertices[0] - lanelet_current.center_vertices[-1]
        return np.sqrt(np.dot(diff, diff))

    def _add_child(
        self,
        parent_node: LaneletNode,
        next_lanelet_id: int,
        target: Lanelet,
        extra_cost: float = 0.0,
    ):
        next_lanelet = self.lanelet_network.find_lanelet_by_id(next_lanelet_id)

        frontier_lanelet_ids = self.frontier.get_item_ids()
        cost = (
            extra_cost
            + parent_node.cost
            + self._calc_cost_travel(next_lanelet)
            + self._calc_cost_heuristic(next_lanelet, target)
        )

        node = LaneletNode(next_lanelet_id, next_lanelet, cost, parent_node.count + 1)
        node.parent_node = parent_node

        if (next_lanelet_id not in self.explored) and (
            next_lanelet_id not in frontier_lanelet_ids
        ):
            self.frontier.push(next_lanelet_id, node, cost)
        elif next_lanelet_id in frontier_lanelet_ids:
            self.frontier.update_item_if_exists(next_lanelet_id, node, cost)

    def _find_route_astar(self, id_lanelet_start, id_lanelet_goal: int) -> List:

        if id_lanelet_start not in self.set_ids_lanelets_permissible:
            raise NoPathFoundException(
                "Start lanelet is not in the permissible set. Check its lanelet type."
            )

        if id_lanelet_goal not in self.set_ids_lanelets_permissible:
            raise NoPathFoundException(
                "Goal lanelet is not in the permissible set. Check its lanelet type."
            )

        self.frontier = PriorityQueue()
        self.explored = set()

        lanelet_current = self.lanelet_network.find_lanelet_by_id(id_lanelet_start)
        lanelet_goal = self.lanelet_network.find_lanelet_by_id(id_lanelet_goal)

        # in astar search, the cost of a node f is the traveled cost g + heuristic cost h
        cost_travel = self._calc_cost_travel(lanelet_current)
        cost_heuristic = self._calc_cost_heuristic(lanelet_current, lanelet_goal)
        node = LaneletNode(
            id_lanelet_start, lanelet_current, cost_travel + cost_heuristic, 1
        )
        self.frontier.push(node.id, node, node.cost)

        # execute the search
        while not self.frontier.is_empty():
            node: LaneletNode = self.frontier.pop()

            if node is None:
                # maybe the frontier is not empty but only contains invalid elements
                continue
            elif node.id == id_lanelet_goal:
                # reached goal
                break
            # add node to explored set (close list)
            self.explored.add(node.id)

            # expand node
            lanelet_current = node.lanelet
            # add successors
            for id_successor in lanelet_current.successor:
                if id_successor in self.set_ids_lanelets_permissible:
                    self._add_child(node, id_successor, lanelet_goal, node.cost)

            # if we are changing lanelets then remove the lanelet lengths because it would be added twice
            lanelet_length = self._calc_cost_travel(lanelet_current)

            # add left lanelet
            adj_left_id = lanelet_current.adj_left
            if (
                lanelet_current.adj_left_same_direction
                and adj_left_id
                and adj_left_id in self.set_ids_lanelets_permissible
            ):
                self._add_child(node, adj_left_id, lanelet_goal, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    left_lanelet_successor_ids = (
                        self.lanelet_network.find_lanelet_by_id(adj_left_id).successor
                    )
                    for left_lanelet_successor_id in left_lanelet_successor_ids:
                        self._add_child(
                            node, left_lanelet_successor_id, lanelet_goal, 0.9
                        )

            # add right lanelet
            adj_right_id = lanelet_current.adj_right
            if (
                lanelet_current.adj_right_same_direction
                and adj_right_id
                and adj_right_id in self.set_ids_lanelets_permissible
            ):
                self._add_child(node, adj_right_id, lanelet_goal, 1.0 - lanelet_length)

                if self.allow_diagonal:
                    right_lanelet_successor_ids = (
                        self.lanelet_network.find_lanelet_by_id(adj_right_id).successor
                    )
                    for right_lanelet_successor_id in right_lanelet_successor_ids:
                        self._add_child(
                            node, right_lanelet_successor_id, lanelet_goal, 0.9
                        )
        else:
            raise NoPathFoundException(
                "The Target lanelet_id [{}] cannot be reached from Source [{}]".format(
                    id_lanelet_goal, id_lanelet_start
                )
            )
        list_ids_lanelets_reversed = list()
        # add ids by looking up to parent node
        while node:
            list_ids_lanelets_reversed.append(node.id)
            node = node.parent_node

        # reverse the list to obtain the correct order from start to goal
        return list_ids_lanelets_reversed[::-1]
