########################################################
#
# TODO: Implement students new stuff
#
#
#
#
#
#
#
###########################################################




from collections import defaultdict

# typing

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import Lanelet
    from commonroad_route_planner.lane_changing.change_position import LaneChangeMarker
    

class LaneChangeInstruction:
    """
    Lanechange instruction, saving the respective lanechange information
    """
    cnt: int = 0
    dict_lanelet_to_instruction = defaultdict()
    
    @classmethod
    def find_instruction_by_lanelet(cls, lanelet: "Lanelet"):
        """
        Returns LaneletInstruction or None given a Lanelet
        """
        if(lanelet in cls.dict_lanelet_to_instruction.keys()):
            return cls.dict_lanelet_to_instruction[lanelet]
        
        else:
            return None
    
    
    def __init__(self,
                 lanelet: "Lanelet",
                 lane_change_marker: "LaneChangeMarker"
                 ) -> None:
               
        
        self.lanelet: "Lanelet" = lanelet
        self.lanelet_id: int = lanelet.lanelet_id
        
        # Lane portiions
        self.instruction_markers: "LaneChangeMarker" = lane_change_marker
        
        # Save in class method
        self.id = LaneChangeInstruction.cnt
        LaneChangeInstruction.cnt += 1
        LaneChangeInstruction.dict_lanelet_to_instruction[lanelet] = self