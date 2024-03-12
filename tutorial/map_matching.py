import pathlib

from commonroad.common.file_reader import CommonRoadFileReader

import commonroad_route_planner
from commonroad_route_planner.map_matching import MapMatcher

# plotting
import matplotlib.pyplot as plt
from commonroad.visualization.mp_renderer import MPRenderer


def main():
    scenario, planning_problem = CommonRoadFileReader(
        pathlib.Path(commonroad_route_planner.__file__).parent.joinpath("./../scenarios/USA_Lanker-2_6_T-1.xml")
    ).open()
    dyn_obst = scenario.obstacle_by_id(2452)

    plt.figure(figsize=(25, 25))
    rnd = MPRenderer()
    rnd.draw_params.lanelet_network.lanelet.show_label = True
    rnd.draw_params.lanelet_network.lanelet.unique_colors = True
    rnd.draw_params.lanelet_network.lanelet.draw_border_vertices = True
    scenario.lanelet_network.draw(rnd)
    dyn_obst.draw(rnd)
    rnd.render()
    plt.show()

    mm = MapMatcher(scenario.lanelet_network)
    lt_sequence = mm.map_matching(
        dyn_obst.prediction.trajectory.state_list,
        dyn_obst.initial_state,
        allow_diagonal_transition=True,
        relax_consistency_constraint=6,
    )

    print(f"Map matching result: {lt_sequence}")


if __name__ == "__main__":
    main()
