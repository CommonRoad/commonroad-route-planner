import logging

# Third party
import networkx as nx

# Commonroad
from commonroad.scenario.lanelet import LaneletNetwork

# Own code base
from commonroad_route_planner.planners.base_route_planner import BaseRoutePlanner
from commonroad_route_planner.utility.exceptions import NoSourceLaneletIdException
from commonroad_route_planner.utility.overtake_init_state import OvertakeInitState


# typing
from typing import List, Dict, Tuple

_logger = logging.getLogger(__name__)


class NetworkxRoutePlanner(BaseRoutePlanner):
    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        set_ids_permissible_lanelets=None,
        overtake_states: List[OvertakeInitState]=None,
        extended_search: bool = False,
    ):
        super().__init__(lanelet_network, set_ids_permissible_lanelets)
        if overtake_states is None:
            overtake_states = set()
        self.overtake_states = overtake_states

        # These lanelets must be included if possible
        self.extended_search: bool = extended_search

        self.digraph = self._create_graph_from_lanelet_network()


    def find_routes(self, id_lanelet_start, id_lanelet_goal) -> List:
        """Find all shortest paths using networkx module

        This tends to change lane late.
        :param id_lanelet_start: ID of start lanelet
        :param id_lanelet_goal: ID of goal lanelet
        :return: list of lists of lanelet IDs
        """
        list_lanelets = list()

        if id_lanelet_start is None:
            raise NoSourceLaneletIdException

        try:
            # default that the shortest path is needed without additional lanelets included
            if(self.extended_search is False):
                list_lanelets = list(
                    nx.all_shortest_paths(
                        self.digraph,
                        source=id_lanelet_start,
                        target=id_lanelet_goal,
                        weight="weight",
                        method="dijkstra",
                    )
                )

            else:
                # special case that lanelets should be included --> increases runtime
                list_lanelets = list(
                    nx.all_simple_paths(
                        self.digraph,
                        source=id_lanelet_start,
                        target=id_lanelet_goal,
                    )
                )

        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            _logger.debug(
                f"The goal lanelet with ID [{id_lanelet_goal}] cannot be reached from the start lanelet with ID [{id_lanelet_start}]"
            )
        return list_lanelets

    def _create_longitudinal_graph(self) -> nx.DiGraph:
        graph = nx.DiGraph()
        nodes = []
        edges = []

        for lanelet in self.lanelet_network.lanelets:
            # only accept allowed lanelets
            if lanelet.lanelet_id not in self.set_ids_permissible_lanelets:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelets exist
            for id_successor in lanelet.successor:
                if id_successor not in self.set_ids_permissible_lanelets:
                    continue
                edges.append(
                    (lanelet.lanelet_id, id_successor, {"weight": lanelet.distance[-1]})
                )

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_lateral_graph(self) -> nx.DiGraph:
        graph = nx.DiGraph()
        nodes = []
        edges = []

        for lanelet in self.lanelet_network.lanelets:
            # only accept allowed lanelets
            if lanelet.lanelet_id not in self.set_ids_permissible_lanelets:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if left lanelet
            id_adj_left = lanelet.adj_left
            if (
                id_adj_left
                and lanelet.adj_left_same_direction
                and id_adj_left in self.set_ids_permissible_lanelets
            ):
                edges.append((lanelet.lanelet_id, id_adj_left, {"weight": 4.0}))

            # add edge if right lanelet
            id_adj_right = lanelet.adj_right
            if (
                id_adj_right
                and lanelet.adj_right_same_direction
                and id_adj_right in self.set_ids_permissible_lanelets
            ):
                edges.append((lanelet.lanelet_id, id_adj_right, {"weight": 4.0}))

        # Edges in case of overtake during starting state
        for overtake_state in self.overtake_states:
            edges.append((overtake_state.original_lanelet_id, overtake_state.adjecent_lanelet_id, {"weight": 1.0}))

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_graph_from_lanelet_network(self) -> nx.DiGraph:
        """Builds a graph from the lanelet network

        Edges are added from the successor relations between lanelets.

        :return: created graph from lanelet network
        """

        lon_graph = self._create_longitudinal_graph()
        lat_graph = self._create_lateral_graph()
        graph = nx.compose(lon_graph, lat_graph)

        # Edges in case of overtake during starting state
        for overtake_state in self.overtake_states:
            graph.add_edges_from([(overtake_state.original_lanelet_id, overtake_state.adjecent_lanelet_id, {"weight": 0})])

        return graph

