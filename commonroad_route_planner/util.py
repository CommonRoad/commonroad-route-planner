from typing import List

import matplotlib as mpl
try:
    mpl.use('Qt5Agg')
    import matplotlib.pyplot as plt
except ImportError:
    mpl.use('TkAgg')
    import matplotlib.pyplot as plt

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_dispatch_cr import draw_object


def draw_initial_state(p_planning_problem: PlanningProblem, fig_num=None):
    # draw ego vehicle
    if fig_num is not None:
        plt.figure(fig_num)

    egoShape = Rectangle(length=5, width=2)
    initial_time_step = int(p_planning_problem.initial_state.time_step)
    trajectory = Trajectory(initial_time_step=initial_time_step,
                            state_list=[p_planning_problem.initial_state])
    prediction = TrajectoryPrediction(trajectory=trajectory, shape=egoShape)
    draw_object(prediction.occupancy_at_time_step(initial_time_step).shape)


def plot_found_routes(scenario: Scenario, planning_problem: PlanningProblem, found_routes: List[List]):
    plt.style.use('classic')
    inch_in_cm = 2.54
    figsize = [60, 30]

    for ctn, route in enumerate(found_routes):
        fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
        fig.clf()
        fig.gca().axis('equal')
        handles = {}  # collects handles of obstacle patches, plotted by matplotlib

        # plot the lanelet network and the planning problem
        draw_object(scenario, handles=handles)
        draw_object(planning_problem, handles=handles)
        fig.gca().autoscale()

        # draw ego vehicle - with a collision object - uses commonroad_cc.visualizer
        try:
            draw_initial_state(planning_problem)
            pass
        except AssertionError as error:
            print(error)

        for route_lanelet_id in route:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
            draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                'center_bound_color': '#3232ff',  # color of the found route
                'draw_stop_line': False,
                'stop_line_color': '#ff0000',
                'draw_line_markings': True,
                'draw_left_bound': False,
                'draw_right_bound': False,
                'draw_center_bound': True,
                'draw_border_vertices': False,
                'draw_start_and_direction': True,
                'show_label': False,
                'draw_linewidth': 2,
                'fill_lanelet': False,
                'facecolor': '#c7c7c7',
                'zorder': 30  # put it higher in the plot, to make it visible
            }})

            # TODO: the goal region now is covering the lanelet arrows, solution plot a simple blue line on it
            plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "b", zorder=30, scalex=False,
                     scaley=False)

        plt.show()
