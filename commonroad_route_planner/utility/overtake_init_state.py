

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork, Lanelet


# typing
from typing import List, Tuple, Dict, TYPE_CHECKING





class OvertakeInitState:
    """
    Class for overtake init state.
    """
    # TODO maybe dataclass?


    def __init__(self, original_lanelet_id: int, adjecent_lanelet_id: int, lanelet_network: LaneletNetwork):
        self.original_lanelet_id: int = original_lanelet_id
        self.adjecent_lanelet_id: int = adjecent_lanelet_id

        self.lanelet_network: LaneletNetwork = lanelet_network
        self.original_lanelet: Lanelet = lanelet_network.find_lanelet_by_id(original_lanelet_id)
        self.adjecent_lanelet: Lanelet = lanelet_network.find_lanelet_by_id(adjecent_lanelet_id)






