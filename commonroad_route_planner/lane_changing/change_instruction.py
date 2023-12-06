from collections import defaultdict


# typing
from typing import List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import Lanelet
    

class LaneChangeInstruction:
    """
    Lanechange instruction, saving the respective lanechange information
    """
    
    # TODO: make dataclass
    
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
    
    
    def __init__(self, lanelet: "Lanelet", instruction_markes: List[int], lanelet_portions: Tuple[float, float]) -> None:
               
        
        self.lanelet: "Lanelet" = lanelet
        self.lanelet_id: int = lanelet.lanelet_id
        
        # Lane portiions
        self.instruction_markers: List[int] = instruction_markes
        self.lanelet_portions: Tuple[float, float] = lanelet_portions
        
        # Save in class method
        self.id = LaneChangeInstruction.cnt
        LaneChangeInstruction.cnt += 1
        LaneChangeInstruction.dict_lanelet_to_instruction[lanelet] = self