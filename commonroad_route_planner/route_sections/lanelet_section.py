########################################################
#
# TODO: This is not needed anywhere ??
#
#
#
#
#
#
#########################################################

# typing
from typing import List, Set
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.lanelet import Lanelet, LaneletNetwork


class LaneletSection:
    """
    Pseudo dataclass that saves the adjacent lanelets and their ids to a lanelet
    """
    
    cnt: int = 0
    
    # TODO: Make actual data class
    
    def __init__(self, lanelet: "Lanelet", 
                 lanelet_network: "LaneletNetwork", 
                 permissible_lanelet_ids: List[int] = None) -> None:
        
        # Give unique id
        self.id = LaneletSection.cnt
        LaneletSection.cnt += 1       
        
        # current lanelet
        self.lanelet: "Lanelet" = lanelet
        self.lanelet_id: int = lanelet.lanelet_id
        
        # Lanelet Network
        self.lanelet_network: "LaneletNetwork" = lanelet_network
    
        # permissible lanelets
        self.permissible_lanelet_ids: List[int] = permissible_lanelet_ids
    
    
        # adjacent lanelets
        self.adjacent_lanelets: List["Lanelet"] = None
        self.adjacent_lanelet_ids: List[int] = None
        self._init_adjacent_lanelets()
        
    
    
    def _init_adjacent_lanelets(self) -> None:
        """Recursively gets adj_left and adj_right lanelets of the given lanelet."""
        adjacent_lanelets = set()
        
        self._recursively_get_adjacent_left(self.lanelet, adjacent_lanelets)
        self._recursively_get_adjacent_right(self.lanelet, adjacent_lanelets)
        
        # Add lanelet itself
        adjacent_lanelets.add(self.lanelet)
        
        # init attributes
        self.adjacent_lanelets = list(adjacent_lanelets)
        self.adjacent_lanelet_ids = [lanelet.lanelet_id for lanelet in self.adjacent_lanelets]

    
    
    
    def _recursively_get_adjacent_left(self, lanelet_base: "Lanelet", adjacent_lanelets: Set["Lanelet"]):
        
        # Check if there is a left lanelet and that it is permissible
        if(lanelet_base.adj_left is not None and lanelet_base.adj_left not in self.permissible_lanelet_ids):
            
            # check that it is not opposite direction
            if(lanelet_base.adj_left_same_direction is not None):
                new_lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(lanelet_base.adj_left_same_direction)
                adjacent_lanelets.add(new_lanelet)
                self._recursively_get_adjacent_left(new_lanelet, adjacent_lanelets)
                
            else:
                # Explicit termination for better readability
                return
        else:
            # Explicit termination for better readability
            return
            
            
    
    def _recursively_get_adjacent_right(self, lanelet_base: "Lanelet", adjacent_lanelets: Set["Lanelet"]) -> None:
        
        # Check if there is a left lanelet and that it is permissible
        if(lanelet_base.adj_right is not None and lanelet_base.adj_right not in self.permissible_lanelet_ids):
            
            # check that it is not opposite direction
            if(lanelet_base.adj_right_same_direction is not None):
                new_lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(lanelet_base.adj_right_same_direction)
                adjacent_lanelets.add(new_lanelet)
                self._recursively_get_adjacent_right(new_lanelet, adjacent_lanelets)
                
            else:
                # Explicit termination for better readability
                return
        else:
            # Explicit termination for better readability
            return
            
            
    
    
