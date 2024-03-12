from logging import Logger
from collections import defaultdict
from enum import Enum

# own code base
from commonroad_route_planner.lane_changing.change_instruction import (
    LaneChangeInstruction,
)

# typing
from typing import List, Union, Dict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad.scenario.scenario import LaneletNetwork, Lanelet


class LaneChangeMarker(Enum):
    CHANGE = (1,)
    NO_CHANGE = 0


class LaneChangePositionHandler:
    """
    Handling where to do the lane change
    """

    def __init__(
        self,
        lanelet_id_sequence: List[int],
        lanelet_network: "LaneletNetwork",
        logger: Logger = None,
    ) -> None:
        """
        :param lanelet_id_sequence: sequence of lanelet ids for route for lanelet network
        :param lanelet_network: lanelet network
        :param logger: logger
        """

        self._logger: Logger = logger if (logger is not None) else Logger(__name__)

        self._lanelet_id_sequence: List[int] = lanelet_id_sequence
        self._lanelet_network: "LaneletNetwork" = lanelet_network

        self._instruction_markers: List[LaneChangeMarker] = None
        self._compute_lane_change_instructions()

        self._dict_lanelet_to_instructions: Dict["Lanelet", LaneChangeInstruction] = (
            defaultdict()
        )
        self._generate_instruction_dict()

    @property
    def instruction_markers(self) -> List[LaneChangeMarker]:
        """
        :return: list of lane change markers
        """
        return self._instruction_markers

    @property
    def dict_lanelet_to_instructions(self) -> Dict["Lanelet", LaneChangeInstruction]:
        """
        :return dict mapping a lanelet to its instruction of either stay or change
        """
        return self._dict_lanelet_to_instructions

    def get_driving_instruction_for_lanelet(
        self, lanelet: "Lanelet"
    ) -> Union[LaneChangeInstruction, None]:
        """
        Returns LaneChangeInstruction for given lanelet

        :param lanelet: lanelet to query

        :return: LaneChangeInstruction or None if none is found for lanelet
        """
        if lanelet in self._dict_lanelet_to_instructions.keys():
            return self._dict_lanelet_to_instructions[lanelet]
        else:
            self._logger.warning(f"lanelet={lanelet.lanelet_id} has no instructions")
            return None

    def _generate_instruction_dict(self) -> None:
        """
        Generates LanceChangeInstruction instances
        """

        for idx, instr in enumerate(self._instruction_markers):
            lanelet: "Lanelet" = self._lanelet_network.find_lanelet_by_id(
                self._lanelet_id_sequence[idx]
            )
            instruction: LaneChangeInstruction = LaneChangeInstruction(
                lanelet, self._instruction_markers[idx]
            )
            self._dict_lanelet_to_instructions[lanelet] = instruction

    def _compute_lane_change_instructions(self) -> None:
        """Computes lane change instruction for planned routes

        The instruction is a list of 0s and 1s, with 0 indicating  no lane change is required
        (driving straight forward=0, and 1 indicating that a lane change (to the left or right) is required.
        """
        self._instruction_markers: List[LaneChangeMarker] = list()
        for idx, id_lanelet in enumerate(self._lanelet_id_sequence[:-1]):
            # Check if the next lanelet in the sequence is a direct successor. If yes, no lane change is require
            if (
                self._lanelet_id_sequence[idx + 1]
                in self._lanelet_network.find_lanelet_by_id(id_lanelet).successor
            ):
                self._instruction_markers.append(LaneChangeMarker.NO_CHANGE)
            else:
                self._instruction_markers.append(LaneChangeMarker.CHANGE)

        # add 0 for the last lanelet
        self._instruction_markers.append(LaneChangeMarker.NO_CHANGE)
