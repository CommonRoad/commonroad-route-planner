import os

import matplotlib.pyplot as plt
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_route_planner.route import Route


def visualize_route(route: Route, scenario_name: str,
                    save_img: bool = True,
                    save_path: str = os.path.join(os.getcwd(), 'img'),
                    draw_route_lanelets=False, draw_reference_path=False,
                    size_x=10):
    """Visualizes the given route.

    :param route: route object to be visualized
    :param draw_route_lanelets: flag to indicate if the lanelets should be visualized
    :param draw_reference_path: flag to indicate if the reference path should be visualized
    :param size_x: size of the x-axis of the figure
    """
    assert route.scenario, "scenario attribute is not set of Route object"

    # obtain plot limits for a better visualization.
    # we can obtain them through the lanelets or the reference path
    plot_limits = obtain_plot_limits_from_reference_path(route)
    # plot_limits = obtain_plot_limits_from_routes(route)

    # set the figure size and ratio
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

    # instantiate a renderer for plotting
    renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))

    # draw scenario and planning problem
    route.scenario.draw(renderer)
    if route.planning_problem:
        route.planning_problem.draw(renderer)
    # draw the initial state of the planning problem
    draw_state(renderer, route.planning_problem.initial_state)

    # draw lanelets of the route
    if draw_route_lanelets:

        list_lanelets = []
        for id_lanelet in route.list_ids_lanelets:
            lanelet = route.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
            list_lanelets.append(lanelet)
        lanelet_network = LaneletNetwork.create_from_lanelet_list(list_lanelets)

        renderer.draw_params.lanelet_network.lanelet.unique_colors = (
            False  # colorizes center_vertices and labels of each lanelet differently
        )
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
        for position in route.reference_path:
            occ_pos = Circle(radius=0.3, center=position)
            renderer.draw_params.shape.facecolor = "#ff477e"
            occ_pos.draw(renderer)

    # render and show plot
    renderer.render()

    plt.margins(0, 0)
    plt.title(str(scenario_name))

    if(save_img):
        save_name: str = os.path.join(save_path, scenario_name)
        plt.savefig(save_name, format='png')
    else:
        plt.show()


def draw_state(renderer: MPRenderer, state: InitialState, color="#ee6c4d"):
    occ_state = Rectangle(4.0, 2.0, state.position, state.orientation)
    renderer.draw_params.shape.facecolor = color
    occ_state.draw(renderer)


def obtain_plot_limits_from_routes(route, border=15):
    x_min_values = list()
    x_max_values = list()
    y_min_values = list()
    y_max_values = list()
    for route_lanelet_id in route.list_ids_lanelets:
        lanelet = route.scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
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


def obtain_plot_limits_from_reference_path(route, border=10):
    x_min = min(route.reference_path[:, 0])
    x_max = max(route.reference_path[:, 0])
    y_min = min(route.reference_path[:, 1])
    y_max = max(route.reference_path[:, 1])

    plot_limits = [x_min - border, x_max + border, y_min - border, y_max + border]
    return plot_limits
