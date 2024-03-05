import copy
from logging import Logger
import math

import numpy as np

# third party
from scipy.spatial import KDTree

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
from commonroad.planning.goal import GoalRegion
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem

# own code base
from commonroad_route_planner.utility.route_util import chaikins_corner_cutting
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.change_position import LaneChangePositionHandler, LaneChangeInstruction
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops
from commonroad_route_planner.lane_changing.lane_change_methods.quintic_polynom import generate_cubic_spline_ref_path
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad.scenario.scenario import Lanelet
from commonroad_route_planner.utility.visualization import plot_clcs_line_with_projection_domain

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
                 route_lanelet_ids: List[int],
                 logger: Logger = None
                 ) -> None:


        self.logger: Logger = logger if (logger is not None) else Logger(__name__)

        self.lanelet_start: Lanelet = lanelet_start
        self.lanelet_end: Lanelet = lanelet_end

        self.lanelet_section: LaneletSection = lanelet_section
        self.lanelet_network: LaneletNetwork = lanelet_network
        self.route_lanelet_ids: List[int] = route_lanelet_ids


        self.clcs: CurvilinearCoordinateSystem = None
        self._init_clcs()



    def _init_clcs(self,
                   additional_lenght_in_meters: float = 50.0,
                   ) -> None:

        # add before
        clcs_line: np.ndarray = copy.copy(self.lanelet_start.center_vertices)

        # get distance between first two points to know what the pseudo-uniform sampling would be
        point_0: np.ndarray = clcs_line[0, :]
        point_1: np.ndarray = clcs_line[1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(additional_lenght_in_meters / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray([point_0[0] - idx * delta_x, point_0[1] - idx * delta_y])
            clcs_line: np.ndarray = np.vstack((new_point, clcs_line))


        # get distance between first two points to know what the pseudo-uniform sampling would be
        point_0: np.ndarray = clcs_line[-2, :]
        point_1: np.ndarray = clcs_line[-1, :]
        distance: float = np.linalg.norm(point_1 - point_0)
        num_new_points: int = math.ceil(additional_lenght_in_meters / distance)

        delta_x: float = float(point_1[0] - point_0[0])
        delta_y: float = float(point_1[1] - point_0[1])

        for idx in range(1, num_new_points + 1):
            new_point: np.ndarray = np.asarray([point_1[0] + idx * delta_x, point_1[1] + idx * delta_y])
            clcs_line: np.ndarray = np.vstack((clcs_line, new_point))



        clcs_line = pops.remove_duplicate_points(clcs_line)
        clcs_line = chaikins_corner_cutting(clcs_line, num_refinements=8)
        clcs_line = pops.resample_polyline(clcs_line, step=1)


        self.clcs = CurvilinearCoordinateSystem(
            clcs_line,
            1000,
            0.1
        )
        plot_clcs_line_with_projection_domain(clcs_line, self.clcs)

        x = 3






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
            self.lanelet_start.center_vertices[0, 1],

        )

        # TODO: Implement better cases

        if(goal_region is not None):
            if (hasattr(goal_region.state_list[0].position, "center")):
                goal_mid_position: np.ndarray = goal_region.state_list[0].position.center
            else:
                # For uncertain position route planner takes first polygon
                goal_mid_position: np.ndarray = goal_region.state_list[0].position.shapes[0].center

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


        reference_path: np.ndarray = self.clcs.convert_list_of_points_to_cartesian_coords(
            ref_path_curv,
            4
        )



        # convert all points of goal lanelet and use the once after the last lane change entry
        end_lanelet_curv: np.ndarray = np.asarray(
                self.clcs.convert_list_of_points_to_curvilinear_coords(
                self.lanelet_end.center_vertices,
                4
                )
        )


        use_end_lanelet_curv: np.ndarray = end_lanelet_curv[end_lanelet_curv[:, 0] > end_point[0], :]

        if (use_end_lanelet_curv.shape[0] > 0):
            use_end_lanelet_cart: np.ndarray = np.asarray(
                self.clcs.convert_list_of_points_to_cartesian_coords(
                use_end_lanelet_curv,
                4
                )
            )


            reference_path: np.ndarray = np.concatenate(
                (reference_path, use_end_lanelet_cart), axis=0
            )

        reference_path = pops.resample_polyline(reference_path, step=sample_step_size)

        return reference_path




