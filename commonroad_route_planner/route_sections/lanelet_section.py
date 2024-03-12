import warnings
from collections import defaultdict

# typing
from typing import List, Dict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad.scenario.lanelet import Lanelet, LaneletNetwork


class LaneletSection:
    """
    Pseudo dataclass that saves the adjacent lanelets and their ids to a lanelet
    """

    cnt: int = 0
    dict_lanelet_id_to_instance: Dict = defaultdict()

    @classmethod
    def get_section_by_lanelet_id(cls, lanelet_id: int):
        """
        Returns LaneletSection to corresponding lanelet
        """
        if lanelet_id in cls.dict_lanelet_id_to_instance.keys():
            return cls.dict_lanelet_id_to_instance[lanelet_id]
        else:
            warnings.warn("Could not find section for lanelet id")
            return None

    def __init__(self, lanelet: "Lanelet", lanelet_network: "LaneletNetwork") -> None:

        # current lanelet
        self.lanelet: "Lanelet" = lanelet
        self.lanelet_id: int = lanelet.lanelet_id

        # Give unique id
        self.id = LaneletSection.cnt
        LaneletSection.cnt += 1
        LaneletSection.dict_lanelet_id_to_instance[self.lanelet_id] = self

        # Lanelet Network
        self.lanelet_network: "LaneletNetwork" = lanelet_network

        # adjacent lanelets
        self.adjacent_lanelets: List["Lanelet"] = list()
        self.adjacent_lanelet_ids: List[int] = list()
        self._init_adjacent_lanelets()

    def has_neighbors(self) -> bool:
        """
        Returns true if this section consits of more than one route
        """
        return True if (len(self.adjacent_lanelets) > 1) else False

    def _init_adjacent_lanelets(self) -> None:
        """Recursively gets adj_left and adj_right lanelets of the given lanelet."""
        self.adjacent_lanelets = list()

        self._recursively_get_adjacent_left(self.lanelet)
        self._recursively_get_adjacent_right(self.lanelet)

        # Add lanelet itself
        self.adjacent_lanelets.append(self.lanelet)

        # init attributes
        self.adjacent_lanelet_ids = [
            lanelet.lanelet_id for lanelet in self.adjacent_lanelets
        ]

    def _recursively_get_adjacent_left(self, lanelet_base: "Lanelet"):

        # Check if there is a left lanelet and that it is permissible
        if lanelet_base.adj_left is not None:
            # check that it is not opposite direction
            if lanelet_base.adj_left_same_direction is True:
                new_lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(
                    lanelet_base.adj_left
                )
                if new_lanelet not in self.adjacent_lanelets:
                    self.adjacent_lanelets.append(new_lanelet)
                    self._recursively_get_adjacent_left(new_lanelet)

    def _recursively_get_adjacent_right(self, lanelet_base: "Lanelet") -> None:
        # Check if there is a left lanelet and that it is permissible
        if lanelet_base.adj_right is not None:
            # check that it is not opposite direction
            if lanelet_base.adj_right_same_direction is True:
                new_lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(
                    lanelet_base.adj_right
                )
                if new_lanelet not in self.adjacent_lanelets:
                    self.adjacent_lanelets.append(new_lanelet)
                    self._recursively_get_adjacent_right(new_lanelet)
