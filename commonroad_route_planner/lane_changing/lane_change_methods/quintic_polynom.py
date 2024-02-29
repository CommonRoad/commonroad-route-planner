


import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import CubicSpline

# commonroad
from commonroad.geometry.shape import Rectangle, Shape
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
from commonroad_dc.pycrccosy import CurvilinearCoordinateSystem
import commonroad_route_planner.utility.polyline_operations.polyline_operations as pops

# Typing
from typing import List, Tuple, Dict






def generate_cubic_spline_ref_path(start_point: np.ndarray,
                                    end_point: np.ndarray,
                                    step_size: float = 0.1
                                    )-> np.ndarray:
    """
    quintic spline with derivatives 0
    """

    abscissa_values: np.ndarray = np.arange(start_point[0], end_point[0], step_size)

    cubic_spline: CubicSpline = CubicSpline(
        x=np.asarray([start_point[0], end_point[0]]),
        y=np.asarray([start_point[1], end_point[1]]),
        bc_type='clamped'
    )


    ordinate_values: np.ndarray = cubic_spline(abscissa_values)

    asdf: List[List[float]] = list()
    for i in range(abscissa_values.shape[0]):
        asdf.append([abscissa_values[i], ordinate_values[i]])

    interpolated_values: np.ndarray = np.asarray(asdf)


    return interpolated_values



if __name__ == "__main__":
    p_start = np.asarray([0, 0])
    p_end = np.asarray([10, 10])

    abscissa_values: np.ndarray = np.arange(p_start[0], p_end[0], 0.1)


    cs = generate_cubic_spline_ref_path(
        start_point=p_start,
        end_point=p_end,
    )


    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(abscissa_values, cs, label="lane_change")

    plt.show()
