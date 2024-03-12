from abc import ABCMeta, abstractmethod
from typing import List
from logging import Logger

from commonroad.scenario.lanelet import LaneletNetwork


class BaseRoutePlanner(metaclass=ABCMeta):
    def __init__(
        self,
        lanelet_network: LaneletNetwork,
        logger: Logger,
        prohibited_lanelet_ids: List[int] = None,
    ):
        """
        Base class for a route planner.
        """
        self._logger = logger
        self._lanelet_network = lanelet_network
        self._prohibited_lanelet_ids: List[int] = (
            prohibited_lanelet_ids if (prohibited_lanelet_ids is not None) else list()
        )

    @abstractmethod
    def find_routes(
        self, id_lanelet_start: int, id_lanelet_goal: int
    ) -> List[List[int]]:
        pass

    @property
    def lanelet_network(self) -> LaneletNetwork:
        """
        :return: cr lanelet network
        """
        return self._lanelet_network

    @property
    def prohibited_lanelet_ids(self) -> List[int]:
        """
        :return: prohibited lanelet ids
        """
        return self._prohibited_lanelet_ids
