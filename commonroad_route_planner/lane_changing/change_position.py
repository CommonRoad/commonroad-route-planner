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

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork


# typing
from typing import List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork



class LaneChangePositionHandler:
    """
    Handling where to do the lane change
    """
    
    def __init__(self, lanelet_id_sequence: List[int], lanelet_network: "LaneletNetwork") -> None:
        self.lanelet_id_sequence: List[int] = lanelet_id_sequence
        self.lanelet_network: "LaneletNetwork" = lanelet_network
        
        self.instructions: List[int] = None
        self._compute_lane_change_instructions()
        
        self.lanelet_portions: List[Tuple[float, float]] = None
        self._compute_lanelet_portion()
        
    


    def _compute_lane_change_instructions(self):
        """Computes lane change instruction for planned routes

        The instruction is a list of 0s and 1s, with 0 indicating  no lane change is required
        (driving straight forward=0, and 1 indicating that a lane change (to the left or right) is required.
        """
        self.instructions: List[int] = list()
        for idx, id_lanelet in enumerate(self.lanelet_id_sequence[:-1]):
            # Check if the next lanelet in the sequence is a direct successor. If yes, no lane change is require
            if (self.lanelet_id_sequence[idx + 1] in self.lanelet_network.find_lanelet_by_id(id_lanelet).successor):
                self.instructions.append(0)
            else:
                self.instructions.append(1)

        # add 0 for the last lanelet
        self.instructions.append(0)


    

    def _compute_lanelet_portion(self):
        """Computes the portion of the center vertices of the lanelets required to construct the reference path

        This is done by first grouping the instructions into consecutive sections (each with only 0s or 1s).
        For the group of 0s, as no lane change is required, the whole lanelet is used; for the group of 1s,
        the upper limit of the portion is computed within the group as (idx_lanelet in the group) / (num_lanelets).
        For example, if there are three consecutive lane changes (assuming three lanes are all parallel), the proportion
        would be [0 - 0.25], [0.25 -0.5], [0.5 - 0.75] and [0.75 - 1.0] for these four lanes.
        """

        # generates a list of consecutive instructions
        # e.g. input: [0, 0, 1, 1, 0, 1] output: [[0, 0], [1, 1], [0], [1]], meaning [drive, lane_change, drive, lane_change]
        consecutive_instructions: List[Tuple[float, float]] = [
            list(v) for k, v in itertools.groupby(self.instructions)
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
        if(not (len(lower_bounds) == len(upper_bounds) == len(self.instructions))):
            raise ValueError(f'Length of instructions do not add up')
        
        self.lanelet_portions = [
            (lower, upper) for lower, upper in zip(lower_bounds, upper_bounds)
        ]









