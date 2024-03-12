import os

import matplotlib.pyplot as plt
import numpy as np

# commonrodad
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams


# typing
from typing import Union, List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.route import Route, RouteSlice


def visualize_route(
    route: Union["Route", "RouteSlice"],
    scenario: Scenario,
    planning_problem: PlanningProblem,
    save_img: bool = True,
    save_path: str = os.path.join(os.getcwd(), "img"),
    draw_route_lanelets: bool = True,
    draw_reference_path: bool = False,
    size_x: float = 10.0,
    size_factor_curvature: float = 5.0,
    curvature_threshold: float = 0.5,
) -> None:
    """
    Visualizes the given route.

    :param route: route or route slice
    :param planning_problem: cr planning problem
    :param save_img: if true, will not display but save imgs instead
    :param save_path: where to save the images to
    :param draw_route_lanelets: draws lanelts of route in different color
    :param draw_reference_path: draws reference path
    :param size_x: size of matplotlib figure
    :param size_factor_curvature: points with high curvature get drawn bigger
    :param curvature_threshold: threshold above with curvature is considered higher
    """

    # obtain plot limits for a better visualization.
    plot_limits = obtain_plot_limits_from_reference_path(route)

    # set the figure size and ratio
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

    # instantiate a renderer for plotting
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))
    renderer.draw_params.dynamic_obstacle.draw_icon = True

    scenario.draw(renderer)

    # draw the initial state of the planning problem
    draw_state(renderer, planning_problem.initial_state)

    # draw lanelets of the route
    if draw_route_lanelets:

        list_lanelets = []
        for id_lanelet in route.lanelet_ids:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
            list_lanelets.append(lanelet)
        lanelet_network = LaneletNetwork.create_from_lanelet_list(list_lanelets)

        renderer.draw_params.lanelet_network.lanelet.unique_colors = (
            False  # colorizes center_vertices and labels of each lanelet differently
        )
        renderer.draw_params.dynamic_obstacle.draw_icon = True
        renderer.draw_params.lanelet_network.lanelet.draw_stop_line = False
        renderer.draw_params.lanelet_network.lanelet.stop_line_color = "#ffffff"
        renderer.draw_params.lanelet_network.lanelet.draw_line_markings = True
        renderer.draw_params.lanelet_network.lanelet.draw_left_bound = False
        renderer.draw_params.lanelet_network.lanelet.draw_right_bound = False
        renderer.draw_params.lanelet_network.lanelet.draw_center_bound = True
        renderer.draw_params.lanelet_network.lanelet.draw_border_vertices = False
        renderer.draw_params.lanelet_network.lanelet.draw_start_and_direction = True
        renderer.draw_params.lanelet_network.lanelet.show_label = False
        renderer.draw_params.lanelet_network.lanelet.draw_linewidth = 1
        renderer.draw_params.lanelet_network.lanelet.fill_lanelet = True
        renderer.draw_params.lanelet_network.lanelet.facecolor = (
            "#469d89"  # color for filling
        )
        renderer.draw_params.lanelet_network.lanelet.zorder = (
            30  # put it higher in the plot, to make it visible
        )
        renderer.draw_params.lanelet_network.lanelet.center_bound_color = (
            "#3232ff"  # color of the found route with arrow
        )

        lanelet_network.draw(renderer)

    # draw reference path with dots
    if draw_reference_path:
        renderer.draw_params.shape.facecolor = "#ff477e"
        for idx, position in enumerate(route.reference_path):
            if abs(route.path_curvature[idx]) > curvature_threshold:
                occ_pos = Circle(radius=0.3 * size_factor_curvature, center=position)
            else:
                occ_pos = Circle(radius=0.3, center=position)
            occ_pos.draw(renderer)

    planning_problem.draw(renderer)

    # render and show plot
    renderer.render()

    plt.margins(0, 0)
    plt.title(str(scenario.scenario_id))

    if save_img:
        save_name: str = os.path.join(save_path, str(scenario.scenario_id))
        plt.savefig(save_name, format="png")
    else:
        plt.show()


def debug_visualize(
    route_list: List["Route"],
    lanelet_network: LaneletNetwork,
    size_x: float = 10.0,
) -> None:
    """
    Visualizes the given route.
    """

    # obtain plot limits for a better visualization.
    plot_limits = obtain_plot_limits_from_reference_path(route_list[0])

    # set the figure size and ratio
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

    # instantiate a renderer for plotting
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))

    lanelet_network.draw(renderer)

    # draw reference path with dots
    for route in route_list:
        for position in route.reference_path:
            occ_pos = Circle(radius=0.3, center=position)
            renderer.draw_params.shape.facecolor = "#ff477e"
            occ_pos.draw(renderer)

    # render and show plot
    renderer.render()

    plt.margins(0, 0)

    plt.show()


def plot_clcs_line_with_projection_domain(clcs_line: np.ndarray, clcs):
    """
    Plots scenario including projection domain of the curvilinear coordinate system used by reach_interface
    :param config: Configuration object of the ReachInterface
    """
    rnd = MPRenderer(figsize=(20, 10))
    draw_param = MPDrawParams()
    draw_param.time_begin = 0

    rnd.render()

    for idx in range(clcs_line.shape[0]):
        occ_pos = Circle(radius=0.3, center=clcs_line[idx, :])
        occ_pos.draw(rnd, draw_param)

    proj_domain_border = np.asarray(clcs.projection_domain())
    rnd.ax.plot(
        proj_domain_border[:, 0], proj_domain_border[:, 1], zorder=100, color="orange"
    )
    plt.show()


def draw_state(renderer: MPRenderer, state: InitialState, color="#ee6c4d") -> None:
    """
    Draws CommonRoad state

    :param renderer: cr renderer
    :param state: initial state
    :param color: color of points
    """
    occ_state = Rectangle(4.0, 2.0, state.position, state.orientation)
    renderer.draw_params.shape.facecolor = color
    occ_state.draw(renderer)


def obtain_plot_limits_from_routes(
    route: Union["Route", "RouteSlice"], border: float = 15
) -> List[int]:
    """
    Obtrains plot limits from lanelets of routes

    :param route: route object

    :return: list [xmin, xmax, ymin, xmax] of plot limits
    """
    x_min_values = list()
    x_max_values = list()
    y_min_values = list()
    y_max_values = list()
    for route_lanelet_id in route.list_ids_lanelets:
        lanelet = route.scenario._lanelet_network.find_lanelet_by_id(route_lanelet_id)
        x_min_values.append(lanelet.center_vertices[:, 0].min())
        x_max_values.append(lanelet.center_vertices[:, 0].max())
        y_min_values.append(lanelet.center_vertices[:, 1].min())
        y_max_values.append(lanelet.center_vertices[:, 1].max())

    plot_limits = [
        min(x_min_values) - border,
        max(x_max_values) + border,
        min(y_min_values) - border,
        max(y_max_values) + border,
    ]
    return plot_limits


def obtain_plot_limits_from_reference_path(
    route: Union["Route", "RouteSlice"], border: float = 10.0
) -> List[int]:
    """
    Obtrains plot limits from reference path

    :param route: route object

    :return: list [xmin, xmax, ymin, xmax] of plot limits
    """
    x_min = min(route.reference_path[:, 0])
    x_max = max(route.reference_path[:, 0])
    y_min = min(route.reference_path[:, 1])
    y_max = max(route.reference_path[:, 1])

    plot_limits = [x_min - border, x_max + border, y_min - border, y_max + border]
    return plot_limits
