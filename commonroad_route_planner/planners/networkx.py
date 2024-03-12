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
from typing import List, Dict, Tuple, Union

_logger = logging.getLogger(__name__)


class NetworkxRoutePlanner(BaseRoutePlanner):
    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        logger: logging.Logger,
        prohibited_lanelet_ids: List[int] = None,
        overtake_states: List[OvertakeInitState] = None,
        extended_search: bool = False,
    ) -> None:
        """

        :param lanelet_network: cr lanelet network
        :param prohibited_lanelet_ids: ids of lanelets that must not be included
        :param overtake_states: if initial state is in an overtake situation
        :param extended_search: necessary, if not the shortest but any route should be included
        """
        super().__init__(
            lanelet_network=lanelet_network,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            logger=logger,
        )

        self._overtake_states: List[OvertakeInitState] = (
            overtake_states if (overtake_states is not None) else list()
        )

        self._extended_search: bool = extended_search

        self._digraph: nx.DiGraph = self._create_graph_from_lanelet_network()

    @property
    def extended_search(self) -> bool:
        """
        :return: whether extended search is used
        """
        return self._extended_search

    @extended_search.setter
    def extended_search(self, use_ext_search: bool) -> None:
        """
        :param use_ext_search: set to True if extended search should be used
        """
        self._extended_search = use_ext_search

    def find_routes(
        self,
        id_lanelet_start: int,
        id_lanelet_goal: Union[int, None],
    ) -> List[List[int]]:
        """Find all shortest paths using networkx module

        This tends to change lane late.
        :param id_lanelet_start: ID of start lanelet
        :param id_lanelet_goal: ID of goal lanelet
        :return: list of lists of lanelet IDs
        """
        lanelets_ids: List[int] = list()

        if id_lanelet_start is None:
            raise NoSourceLaneletIdException

        try:
            # default that the shortest path is needed without additional lanelets included
            if self._extended_search is False:
                lanelets_ids: List[List[int]] = list(
                    nx.all_shortest_paths(
                        self._digraph,
                        source=id_lanelet_start,
                        target=id_lanelet_goal,
                        weight="weight",
                        method="dijkstra",
                    )
                )

            else:
                # special case that lanelets should be included --> increases runtime
                lanelets_ids: List[List[int]] = list(
                    nx.all_simple_paths(
                        self._digraph,
                        source=id_lanelet_start,
                        target=id_lanelet_goal,
                    )
                )

                # apparently simple paths cannot deal with goal being on same lanelet as start
                if len(lanelets_ids) == 0 and id_lanelet_start == id_lanelet_goal:
                    lanelets_ids.append([id_lanelet_goal])

        except nx.exception.NetworkXNoPath:
            # it is a normal behaviour because of the overlapping lanelets in a road network
            self._logger.error(
                f"The goal lanelet with ID [{id_lanelet_goal}] cannot #"
                f"be reached from the start lanelet with ID [{id_lanelet_start}]"
            )
        return lanelets_ids

    def _create_graph_from_lanelet_network(self) -> nx.DiGraph:
        """
        Builds a graph from the lanelet network. Edges are added from the successor relations between lanelets.

        :return: created graph from lanelet network
        """

        lon_graph = self._create_longitudinal_graph()
        lat_graph = self._create_lateral_graph()
        graph = nx.compose(lon_graph, lat_graph)

        # Edges in case of overtake during starting state
        for overtake_state in self._overtake_states:
            graph.add_edges_from(
                [
                    (
                        overtake_state.original_lanelet_id,
                        overtake_state.adjecent_lanelet_id,
                        {"weight": 0},
                    )
                ]
            )

        return graph

    def _create_longitudinal_graph(self) -> nx.DiGraph:
        """
        Creates longitudinal graph

        :return: created graph from lanelet network
        """
        graph = nx.DiGraph()
        nodes: List[int] = list()
        edges: List[Tuple[int, int, Dict[str, float]]] = list()

        for lanelet in self._lanelet_network.lanelets:
            # only accept allowed lanelets
            if lanelet.lanelet_id in self._prohibited_lanelet_ids:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if succeeding lanelets exist
            for id_successor in lanelet.successor:
                if id_successor in self._prohibited_lanelet_ids:
                    continue

                edges.append(
                    (lanelet.lanelet_id, id_successor, {"weight": lanelet.distance[-1]})
                )

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph

    def _create_lateral_graph(self) -> nx.DiGraph:
        """
        Creates lateral graph

        :return: created graph from lanelet network
        """
        graph = nx.DiGraph()
        # network x requires weird tuple based input
        nodes: List[int] = list()
        edges: List[Tuple[int, int, Dict[str, float]]] = list()

        for lanelet in self._lanelet_network.lanelets:
            # only accept allowed lanelets
            if lanelet.lanelet_id in self._prohibited_lanelet_ids:
                continue

            nodes.append(lanelet.lanelet_id)

            # add edge if left lanelet
            id_adj_left: int = lanelet.adj_left
            if (
                id_adj_left
                and lanelet.adj_left_same_direction
                and id_adj_left not in self._prohibited_lanelet_ids
            ):
                weight: float = 1e6 - max(
                    lanelet.distance[-1],
                    self._lanelet_network.find_lanelet_by_id(id_adj_left).distance[-1],
                )
                edges.append((lanelet.lanelet_id, id_adj_left, {"weight": weight}))

            # add edge if right lanelet
            id_adj_right: int = lanelet.adj_right
            if (
                id_adj_right
                and lanelet.adj_right_same_direction
                and id_adj_right not in self._prohibited_lanelet_ids
            ):
                weight: float = 1e6 - max(
                    lanelet.distance[-1],
                    self._lanelet_network.find_lanelet_by_id(id_adj_right).distance[-1],
                )
                edges.append((lanelet.lanelet_id, id_adj_right, {"weight": weight}))

        # Edges in case of overtake during starting state
        for overtake_state in self._overtake_states:
            edges.append(
                (
                    overtake_state.original_lanelet_id,
                    overtake_state.adjecent_lanelet_id,
                    {"weight": 1.0},
                )
            )

        # add all nodes and edges to graph
        graph.add_nodes_from(nodes)
        graph.add_edges_from(edges)
        return graph
