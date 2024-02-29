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


import itertools
import numpy as np
from collections import defaultdict
from enum import Enum

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork


# own code base 
from commonroad_route_planner.lane_changing.change_instruction import LaneChangeInstruction

# typing
from typing import List, Tuple, Dict
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork, Lanelet


class LaneChangeMarker(Enum):
    CHANGE = 1,
    NO_CHANGE = 0



class LaneChangePositionHandler:
    """
    Handling where to do the lane change
    """
    
    def __init__(self,
                 lanelet_id_sequence: List[int],
                 lanelet_network: "LaneletNetwork"
                 ) -> None:
        
        # FIXME still bad since the instructions implicitly still assume same index as _lanelet_id_sequence without checking
        self._lanelet_id_sequence: List[int] = lanelet_id_sequence
        self._lanelet_network: "LaneletNetwork" = lanelet_network
        
        self._instruction_markers: List[LaneChangeMarker] = None
        self._compute_lane_change_instructions()
        
        self._lanelet_portions: List[Tuple[float, float]] = None
        self._compute_lanelet_portion()
        
        self.dict_lanelet_to_instructions: Dict["Lanelet", LaneChangeInstruction] = defaultdict() 
        self._generate_instruction_dict()


    def _generate_instruction_dict(self) -> None:
        """
        Generates instruction instances
        """
        
        for idx, instr in enumerate(self._instruction_markers):
            lanelet: "Lanelet" = self._lanelet_network.find_lanelet_by_id(self._lanelet_id_sequence[idx])
            instruction: LaneChangeInstruction = LaneChangeInstruction(lanelet, self._instruction_markers[idx], self._lanelet_portions[idx])
            self.dict_lanelet_to_instructions[lanelet] = instruction
            
        


    def _compute_lane_change_instructions(self) -> None:
        """Computes lane change instruction for planned routes

        The instruction is a list of 0s and 1s, with 0 indicating  no lane change is required
        (driving straight forward=0, and 1 indicating that a lane change (to the left or right) is required.
        """
        self._instruction_markers: List[LaneChangeMarker] = list()
        for idx, id_lanelet in enumerate(self._lanelet_id_sequence[:-1]):
            # Check if the next lanelet in the sequence is a direct successor. If yes, no lane change is require
            if (self._lanelet_id_sequence[idx + 1] in self._lanelet_network.find_lanelet_by_id(id_lanelet).successor):
                self._instruction_markers.append(LaneChangeMarker.NO_CHANGE)
            else:
                self._instruction_markers.append(LaneChangeMarker.CHANGE)

        # add 0 for the last lanelet
        self._instruction_markers.append(LaneChangeMarker.NO_CHANGE)



    





    def get_driving_instruction_for_lanelet(self, lanelet: "Lanelet") -> LaneChangeInstruction:
        """
        Returns change instruction for given lanelet
        """
        if(lanelet in self.dict_lanelet_to_instructions.keys()):
            return self.dict_lanelet_to_instructions[lanelet]
        else:
            raise ValueError(f'lanelet={lanelet.lanelet_id} has no instructions')


