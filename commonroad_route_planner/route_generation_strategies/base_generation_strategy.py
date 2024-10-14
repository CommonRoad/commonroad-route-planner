from abc import ABC, abstractmethod
import numpy as np

# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import InitialState
from commonroad.scenario.lanelet import LaneletNetwork

# own code base
from commonroad_route_planner.reference_path import ReferencePath

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
    ) -> ReferencePath:
        """
        Instantiates reference_path

        :param reference_path: (n,2) reference path

        :return: reference_path object
        :rtype ReferencePath
        """
        pass

    @staticmethod
    @abstractmethod
    def update_route(route: ReferencePath, reference_path: np.ndarray) -> ReferencePath:
        """
        updates reference_path given a reference path

        :param reference_path: (n,2) reference path

        :return: reference_path object
        :rtype ReferencePath
        """
        pass
