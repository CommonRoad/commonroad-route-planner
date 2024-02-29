
from logging import Logger

import numpy as np

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.planning.goal import GoalRegion
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

# own code base
from commonroad_route_planner.utility.route_util import (chaikins_corner_cutting)
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler, LaneChangeInstruction
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.lane_changing.lane_change_methods.quintic_polynom import generate_cubic_spline_ref_path
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad.scenario.scenario import Lanelet

from typing import List, Set, Union


class LaneChangeHandler:
    """
    Handles one specific lane change
    """


    def __init__(self,
                 lanelet_start: Lanelet,
                 lanelet_end: Lanelet,
                 lanelet_section: LaneletSection,
                 lanelet_network: LaneletNetwork,
                 logger: Logger = None
                 ) -> None:


        self.logger: Logger = logger if (logger is not None) else Logger(__name__)

        self.lanelet_start: Lanelet = lanelet_start
        self.lanelet_end: Lanelet = lanelet_end

        self.lanelet_section: LaneletSection = lanelet_section
        self.lanelet_network: LaneletNetwork = lanelet_network


        self.clcs: CurvilinearCoordinateSystem = CurvilinearCoordinateSystem(self.lanelet_start.center_vertices)






    def compute_lane_change(self,
                            sample_step_size: float = 1.0,
                            goal_region: GoalRegion = None
                            ) -> np.ndarray:
        """
        Computes simple lane change
        """

        # Algorithm
        # 1. Take first point of base and last point of last_change and convert to clcs
        # 2. Connect via quintic

        start_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
            self.lanelet_start.center_vertices[0, 0],
            self.lanelet_start.center_vertices[0, 1]
        )

        # TODO: Implement through goal lane change
        """
        end_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
            self.lanelet_end.center_vertices[-1, 0],
            self.lanelet_end.center_vertices[-1, 1]
        )

        
        if(set(goal_region.lanelets_of_goal_position[0]) & set(self.lanelet_section.adjacent_lanelet_ids)):
            # Check if goal is within lane change
            goal_position: np.ndarray = goal_region.state_list[0].position
            end_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
                goal_position[-1, 0],
                goal_position[-1, 1]
            )
        """

        if(goal_region is not None):

            # check if goal region is in current lange change
            goal_mid_position: np.ndarray = goal_region.state_list[0].position.center
            goal_lanelet_ids: List[int] = self.lanelet_network.find_lanelet_by_position([goal_mid_position])[0]

            if(set(goal_lanelet_ids) & set(self.lanelet_section.adjacent_lanelet_ids)):
                end_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
                    goal_mid_position[0],
                    goal_mid_position[1]
                )
            else:
                end_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
                    self.lanelet_end.center_vertices[-1, 0],
                    self.lanelet_end.center_vertices[-1, 1]
                )

        else:
            end_point: np.ndarray = self.clcs.convert_to_curvilinear_coords(
                self.lanelet_end.center_vertices[-1, 0],
                self.lanelet_end.center_vertices[-1, 1]
            )

        ref_path_curv: np.ndarray = generate_cubic_spline_ref_path(
            start_point=start_point,
            end_point=end_point
        )

        ref_path_cart = self.clcs.convert_list_of_points_to_cartesian_coords(
            ref_path_curv,
            4
        )


        ref_path = pops.resample_polyline(ref_path_cart, step=sample_step_size)

        return ref_path












