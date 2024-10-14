from dataclasses import dataclass


from typing import List


@dataclass(frozen=True)
class LaneletSequence:
    """
    Sequence of lanelet ids
    """

    lanelet_ids: List[int]
