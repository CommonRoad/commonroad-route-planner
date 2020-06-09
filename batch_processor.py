import errno
import os
import time

import matplotlib as mpl
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.visualization.plot_helper import set_non_blocking, redraw_obstacles

mpl.use('Qt5Agg')  # sets the backend for matplotlib
# mpl.use('TkAgg')  # sets the backend for matplotlib
import matplotlib.pyplot as plt
from HelperFunctions import get_existing_scenarios, load_config_file, get_existing_scenarios_name, \
    load_scenarios, initialize_logger, draw_initial_state, get_last_time_step_in_scenario, \
    execute_search

# load config file
configs = load_config_file('batch_processor_config.yaml')
# create a logger
logger = initialize_logger("Batch processor", configs)
logger.debug("Config file loaded and logger created")

scenarios_root_folder = configs['input_path']
# if directory not exists create it
if not os.path.exists(scenarios_root_folder):
    logger.error("Scenarios root path <{}> is not exist".format(scenarios_root_folder))
    raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), scenarios_root_folder)

specified_scenario_ids = set(configs['scenarios_to_run'])
skip_scenario_ids = configs['scenarios_to_skip']
existing_scenarios = get_existing_scenarios(scenarios_root_folder)
existing_scenario_ids = set(get_existing_scenarios_name(scenarios_root_folder))

existing_scenario_ids.difference_update(skip_scenario_ids)
if len(skip_scenario_ids) != 0:
    logger.info("Skipping the following scenarios: ")
    for idx in skip_scenario_ids:
        logger.info("\t{:<20}".format(idx))

if len(specified_scenario_ids) != 0:
    logger.info("Only run on specified scenarios:")
    not_found_ids = specified_scenario_ids.difference(existing_scenario_ids)
    if len(not_found_ids) != 0:
        for idx in not_found_ids:
            logger.warning("\t{} is NOT FOUND or SKIPPED and hence won't be processed!".format(idx))
    scenario_ids = specified_scenario_ids.intersection(existing_scenario_ids)
    for idx in scenario_ids:
        logger.info("\t{}".format(idx))
else:
    logger.info("Run on all scenarios")
    scenario_ids = existing_scenario_ids  # if we still need to use existing scenario ids the consider COPY instead

scenario_ids = sorted(list(scenario_ids))
num_of_scenarios_to_solve = len(scenario_ids)

# ========================== multiprocessing example for scenario loading ========================== #
# counter = 1
# problems = list()
# time1 = time.perf_counter()
#
# start 8 worker processes
# with Pool(processes=8) as pool:
#     logger.info("Loading scenarios: ")
#     multiple_results = [pool.apply_async(load_scenario, (scenarios_root_folder, scenario_id)) for scenario_id in
#                         scenario_ids]
#     for res in multiple_results:
#         problems.append(res.get())
#         logger.debug("{:3}/{} [{:<20}] loaded".format(counter, num_of_scenarios_to_solve, problems[counter-1][0].benchmark_id))
#         counter += 1
#
# elapsed_time = time.perf_counter() - time1
# logger.info("Loading all scenarios took {:10.4} s.".format(elapsed_time))

scenarios_path_found = list()
scenarios_path_not_found = list()
scenarios_exception = list()

plot_and_save_scenarios = True
animate_scenario = False
# Initialize Plotting
set_non_blocking()  # ensures interactive plotting is activated
plt.style.use('classic')
inch_in_cm = 2.54
figsize = [60, 30]

counter = 1
for scenario, planning_problem_set in load_scenarios(scenarios_root_folder, scenario_ids):
    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    # with the heuristic A* we do not want to solve cooperative scenarios so in all cases we will have
    # only one planning problem
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    scenario_id = scenario.benchmark_id
    base_msg = "{:3}/{}\t[{:<20}]\t ".format(counter, num_of_scenarios_to_solve, scenario_id)

    routes = None
    time1 = time.perf_counter()
    try:
        routes = execute_search(scenario, planning_problem)
    except IndexError as error:
        search_time = time.perf_counter() - time1
        scenarios_exception.append(scenario_id)
        msg = base_msg + "\tsearch took\t{:10.4f}\tms\texception:".format(search_time * 1000)
        logger.error(msg)
        logger.exception(error)
    except Exception as error:
        search_time = time.perf_counter() - time1
        scenarios_exception.append(scenario_id)
        msg = base_msg + "\tsearch took\t{:10.4f}\tms\texception:".format(search_time * 1000)
        logger.error(msg)
        logger.exception(error)
    else:
        search_time = time.perf_counter() - time1
        msg = "\tsearch took\t{:10.4f}\tms\t".format(search_time * 1000)

        # test = dim(routes)
        if len(routes) > 1:
            logger.info(base_msg + msg + "MORE path FOUND")
            msg = base_msg
        else:
            msg = base_msg + msg

        for ctn, route in enumerate(routes):
            if len(route) == 0:
                logger.debug(msg + "path NOT FOUND")
                scenarios_path_not_found.append(scenario_id)
                continue

            goal_lanelet_id = route[-1]
            logger.debug(msg + "path FOUND to goal lanelet: [{}]".format(goal_lanelet_id))
            scenarios_path_found.append(scenario_id)
            if plot_and_save_scenarios:

                # Create a Figure
                fig_num = 1
                fig = plt.figure(fig_num, figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
                fig.clf()
                fig.gca().axis('equal')
                handles = {}  # collects handles of obstacle patches, plotted by matplotlib

                # plot the lanelet network and the planning problem
                draw_object(scenario, handles=handles)
                draw_object(planning_problem, handles=handles)
                fig.gca().autoscale()

                # draw ego vehicle - with a collision object - uses commonroad_cc.visualizer
                draw_initial_state(planning_problem, fig_num)

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
                    # plot the center line vertices of the found route, scaling have to be switched off not to mess up
                    # the original scaling
                    plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "ro", zorder=31,
                             scalex=False,
                             scaley=False)
                    # TODO: the goal region now is covering the lanelet arrows, solution plot a simple blue line on it
                    #   ! should be fixed !
                    plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "b", zorder=30, scalex=False,
                             scaley=False)

                # saving solved solutions
                # output_folder
                output_folder = configs["output_path"]
                # if directory not exists create it
                if not os.path.exists(output_folder):
                    os.mkdir(output_folder)
                output_file = os.path.join(output_folder, "fig_{}_goal_ID_{}.png".format(scenario_id, goal_lanelet_id))
                fig.canvas.draw()
                fig.savefig(output_file)
                # logger.debug("{} 's solution saved out".format(scenario_id))

                # only for fun :)
                if animate_scenario:
                    # visualize the scenario with the dynamic obstacles
                    # plot_limits, and draw_params can be used to speed up the plotting - only draw necessary information
                    fig.canvas.draw()  # have to call this line before calling redraw_obstacles function call
                    last_time_step = get_last_time_step_in_scenario(scenario)
                    for time_step in range(last_time_step + 1):
                        # here the time_end can be changed to last_time_step to also visualize the occupancies of the dynamic vehicles
                        # but then the plotting will be slower
                        redraw_obstacles(scenario, handles=handles, figure_handle=fig, plot_limits=None,
                                         draw_params={'time_begin': time_step, 'time_end': time_step})
                        logger.debug("Timestep: {:3}".format(time_step))

    counter += 1

logger.debug("Successfully SOLVED scenarios {:3}/{}:".format(len(scenarios_path_found), num_of_scenarios_to_solve))
for scen in scenarios_path_found:
    logger.debug("\t{}".format(scen))
logger.warning(
    "Scenarios with path NOT FOUND {:3}/{}:".format(len(scenarios_path_not_found), num_of_scenarios_to_solve))
for scen in scenarios_path_not_found:
    logger.warning("\t{}".format(scen))
logger.critical("Scenarios with EXCEPTION {:3}/{}:".format(len(scenarios_exception), num_of_scenarios_to_solve))
for scen in scenarios_exception:
    logger.critical("\t{}".format(scen))

# switch of interactive mode not to disappear the figures at the end of the program
mpl.rc_context(rc={'interactive': False})
# plt.figure(1)
plt.show()
