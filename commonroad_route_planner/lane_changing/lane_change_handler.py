
from logging import Logger

import numpy as np

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

# own code base
from commonroad_route_planner.utility.route_util import (chaikins_corner_cutting)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler, LaneChangeInstruction
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad.scenario.scenario import Lanelet

from typing import List, Set, Union


class LaneChangeHandler:
    """
    Calcs Lane Changees
    """


    def __init__(self, lanelet_start: Lanelet,
                 lanelet_end: Lanelet,
                 lanelet_section: LaneletSection,
                 lanelet_network: LaneletNetwork,
                 sample_step_size: float = 1.0,
                 logger: Logger = None) -> None:


        self.logger: Logger = logger if (logger is not None) else Logger(__name__)

        self.lanelet_start: Lanelet = lanelet_start
        self.lanelet_end: Lanelet = lanelet_end
        self.lanelet_section: LaneletSection = lanelet_section
        self.lanelet_network: LaneletNetwork = lanelet_network

        self.sample_step_size: float = sample_step_size
        self.base_clcs_lanelet: Lanelet = None
        self.clcs: CurvilinearCoordinateSystem = None
        self._init_clcs()





    def _init_clcs(self) -> None:
        for lanelet_id in self.lanelet_start.successor:
            if(lanelet_id in self.lanelet_section.adjacent_lanelet_ids):
                self.base_clcs_lanelet = self.lanelet_network.find_lanelet_by_id(lanelet_id)
                centerline_vertices: np.ndarray = pops.sample_polyline(self.base_clcs_lanelet.center_vertices,
                                                                       self.sample_step_size)
                self.clcs = CurvilinearCoordinateSystem(centerline_vertices, 1000)

        self.logger.error(f'Could not find lanelet for lane change to create clcs')
        raise ValueError(f'Could not find lanelet for lane change to create clcs')













