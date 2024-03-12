from abc import ABC, abstractmethod
import numpy as np

# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState
from commonroad.scenario.lanelet import LaneletNetwork

# own code base
from commonroad_route_planner.route import Route

# typing
from typing import List


class BaseGenerationStrategy(ABC):
    """
    Abstract base strategy
    """

    @staticmethod
    @abstractmethod
    def generate_route(
        lanelet_network: LaneletNetwork,
        lanelet_ids: List[int],
        initial_state: InitialState,
        goal_region: GoalRegion,
    ) -> Route:
        """
        Instantiates route

        :param reference_path: (n,2) reference path

        :return: route object
        :rtype Route
        """
        pass

    @staticmethod
    @abstractmethod
    def update_route(route: Route, reference_path: np.ndarray) -> Route:
        """
        updates route given a reference path

        :param reference_path: (n,2) reference path

        :return: route object
        :rtype Route
        """
        pass
