from abc import ABCMeta, abstractmethod
from typing import List

from commonroad.scenario.lanelet import LaneletNetwork


class BaseRoutePlanner(metaclass=ABCMeta):
    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        prohibited_lanelet_ids: List[int] = None
    ):
        """
        Base class for a route planner.
        """
        self._lanelet_network = lanelet_network
        self._prohibited_lanelet_ids: List[int] = prohibited_lanelet_ids if(prohibited_lanelet_ids is not None) else list()

    @abstractmethod
    def find_routes(self, id_lanelet_start: int, id_lanelet_goal: int) -> List[List[int]]:
        pass
