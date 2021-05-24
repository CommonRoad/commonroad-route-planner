from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.visualization.mp_renderer import MPRenderer

from route_planner.route import Route


def visualize_route(route: Route, draw_route_lanelets=False, draw_reference_path=False, size_x=10):
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
    route.planning_problem.draw(renderer)
    rectangle = Rectangle(4.0, 2.0,
                          route.planning_problem.initial_state.position,
                          route.planning_problem.initial_state.orientation)
    rectangle.draw(renderer)

    # draw lanelets of the route
    if draw_route_lanelets:
        dict_param = {'lanelet': {
            'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
            'draw_stop_line': False,
            'stop_line_color': '#ffffff',
            'draw_line_markings': True,
            'draw_left_bound': False,
            'draw_right_bound': False,
            'draw_center_bound': True,
            'draw_border_vertices': False,
            'draw_start_and_direction': True,
            'show_label': False,
            'draw_linewidth': 1,
            'fill_lanelet': True,
            'facecolor': '#00b8cc',  # color for filling
            'zorder': 30,  # put it higher in the plot, to make it visible
            'center_bound_color': '#3232ff',  # color of the found route with arrow
        }}

        list_lanelets = []
        for id_lanelet in route.list_ids_lanelets:
            lanelet = route.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
            list_lanelets.append(lanelet)
        lanelet_network = LaneletNetwork.create_from_lanelet_list(list_lanelets)

        lanelet_network.draw(renderer, draw_params=dict_param)

    if draw_reference_path:
        renderer.ax.plot(route.reference_path[:, 0], route.reference_path[:, 1], '-m', linewidth=3.5, zorder=30)

    renderer.render()


def draw_state(renderer: MPRenderer, state: State, color='red'):
    ego_shape = Rectangle(length=5, width=2)

    if hasattr(state, 'time_step'):
        initial_time_step = int(state.time_step)

    else:
        state.time_step = 0
        initial_time_step = 0

    trajectory = Trajectory(initial_time_step=initial_time_step, state_list=[state])
    prediction = TrajectoryPrediction(trajectory=trajectory, shape=ego_shape)
    shape = prediction.occupancy_at_time_step(initial_time_step).shape
    if isinstance(shape, Rectangle):
        shape.draw(renderer, draw_params={'shape': {'facecolor': f'{color}'}})


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

    plot_limits = [min(x_min_values) - border, max(x_max_values) + border,
                   min(y_min_values) - border, max(y_max_values) + border]
    return plot_limits


def obtain_plot_limits_from_reference_path(route, border=10):
    x_min = min(route.reference_path[:, 0])
    x_max = max(route.reference_path[:, 0])
    y_min = min(route.reference_path[:, 1])
    y_max = max(route.reference_path[:, 1])

    plot_limits = [x_min - border, x_max + border, y_min - border, y_max + border]
    return plot_limits
