########################################################
#
# TODO: Refactor
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

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork


# own code base 
from commonroad_route_planner.lane_changing.change_instruction import LaneChangeInstruction

# typing
from typing import List, Tuple, Dict
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork, Lanelet



class LaneChangePositionHandler:
    """
    Handling where to do the lane change
    """
    
    def __init__(self, lanelet_id_sequence: List[int], lanelet_network: "LaneletNetwork") -> None:
        
        # FIXME still bad since the instructions implicitly still assume same index as lanelet_id_sequence without checking
        self.lanelet_id_sequence: List[int] = lanelet_id_sequence
        self.lanelet_network: "LaneletNetwork" = lanelet_network
        
        self._instruction_markers: List[int] = None
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
            lanelet: "Lanelet" = self.lanelet_network.find_lanelet_by_id(self.lanelet_id_sequence[idx])
            instruction: LaneChangeInstruction = LaneChangeInstruction(lanelet, self._instruction_markers[idx], self._lanelet_portions[idx])
            self.dict_lanelet_to_instructions[lanelet] = instruction
            
        


    def _compute_lane_change_instructions(self) -> None:
        """Computes lane change instruction for planned routes

        The instruction is a list of 0s and 1s, with 0 indicating  no lane change is required
        (driving straight forward=0, and 1 indicating that a lane change (to the left or right) is required.
        """
        self._instruction_markers: List[int] = list()
        for idx, id_lanelet in enumerate(self.lanelet_id_sequence[:-1]):
            # Check if the next lanelet in the sequence is a direct successor. If yes, no lane change is require
            if (self.lanelet_id_sequence[idx + 1] in self.lanelet_network.find_lanelet_by_id(id_lanelet).successor):
                self._instruction_markers.append(0)
            else:
                self._instruction_markers.append(1)

        # add 0 for the last lanelet
        self._instruction_markers.append(0)


    

    def _compute_lanelet_portion(self) -> None:
        """Computes the portion of the center vertices of the lanelets required to construct the reference path

        This is done by first grouping the instructions into consecutive _sections (each with only 0s or 1s).
        For the group of 0s, as no lane change is required, the whole lanelet is used; for the group of 1s,
        the upper limit of the portion is computed within the group as (idx_lanelet in the group) / (num_lanelets).
        For example, if there are three consecutive lane changes (assuming three lanes are all parallel), the proportion
        would be [0 - 0.25], [0.25 -0.5], [0.5 - 0.75] and [0.75 - 1.0] for these four lanes.
        """

        # generates a list of consecutive instructions
        # e.g. input: [0, 0, 1, 1, 0, 1] output: [[0, 0], [1, 1], [0], [1]], meaning [drive, lane_change, drive, lane_change]
        consecutive_instructions: List[Tuple[float, float]] = [
            list(v) for k, v in itertools.groupby(self._instruction_markers)
        ]

        upper_bounds: List[float] = list()
        lower_bounds: List[float] = [0.0]
        
        # Construct upper bounds
        for instruction_group in consecutive_instructions:
            for idx, instruction in enumerate(instruction_group):
                if(instruction == 0):
                    # goes till the end of the lanelet
                    bound_upper = 1.0
                else:
                    # goes only till a specific portion
                    bound_upper = (idx + 1) / (len(instruction_group) + 1)

                upper_bounds.append(bound_upper)


        # Construct lower bound
        if len(upper_bounds) > 1:
            for idx in range(1, len(upper_bounds)):
                if np.isclose(upper_bounds[idx - 1], 1.0):
                    lower_bounds.append(0.0)
                else:
                    lower_bounds.append(upper_bounds[idx - 1])

        # Sanity check
        if(not (len(lower_bounds) == len(upper_bounds) == len(self._instruction_markers))):
            raise ValueError(f'Length of instructions do not add up')
        
        
        # each portion is (lower bound, upper bount)
        self._lanelet_portions = [
            (lower, upper) for lower, upper in zip(lower_bounds, upper_bounds)
        ]






    def get_driving_instruction_for_lanelet(self, lanelet: "Lanelet") -> LaneChangeInstruction:
        """
        Returns change instruction for given lanelet
        """
        if(lanelet in self.dict_lanelet_to_instructions.keys()):
            return self.dict_lanelet_to_instructions[lanelet]
        else:
            raise ValueError(f'lanelet={lanelet.lanelet_id} has no instructions')


