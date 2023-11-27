import pathlib

from commonroad.common.file_reader import CommonRoadFileReader

import commonroad_route_planner
from commonroad_route_planner.utility.map_matching import MapMatcher

scenario, planning_problem = CommonRoadFileReader(
    pathlib.Path(commonroad_route_planner.__file__).parent.joinpath(
        "./../scenarios/DEU_Gar-3_2_T-1.xml"
    )
).open()

mm = MapMatcher(scenario.lanelet_network, 2)
lt_sequence = mm.map_matching(
    scenario.dynamic_obstacles[0].prediction.trajectory.state_list,
    scenario.dynamic_obstacles[0].initial_state
)

print(f"Map matching result: {lt_sequence}")
