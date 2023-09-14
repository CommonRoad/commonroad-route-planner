from abc import ABCMeta, abstractmethod
from typing import List

from commonroad.scenario.lanelet import LaneletNetwork


class BaseRoutePlanner(metaclass=ABCMeta):
    def __init__(
        self, lanelet_network: LaneletNetwork, set_ids_permissible_lanelets=None
    ):
        """Base class for a route planner."""

        self.lanelet_network = lanelet_network
        if set_ids_permissible_lanelets is None:
            set_ids_permissible_lanelets = [
                lanelet.lanelet_id for lanelet in lanelet_network.lanelets
            ]
        self.set_ids_permissible_lanelets = set_ids_permissible_lanelets

    @abstractmethod
    def find_routes(self, id_lanelet_start, id_lanelet_goal) -> List[List[int]]:
        pass
