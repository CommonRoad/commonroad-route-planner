import errno
import os
import time

import matplotlib as mpl
from commonroad.visualization.draw_dispatch_cr import draw_object

from commonroad_route_planner.util import plot_found_routes, draw_initial_state

mpl.use('Qt5Agg')  # sets the backend for matplotlib
# mpl.use('TkAgg')  # sets the backend for matplotlib
import matplotlib.pyplot as plt
from HelperFunctions import get_existing_scenarios, load_config_file, get_existing_scenario_ids, \
    load_scenarios, initialize_logger, execute_search

# load config file
configs = load_config_file('batch_processor/batch_processor_config.yaml')
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
existing_scenario_ids = set(get_existing_scenario_ids(scenarios_root_folder))

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

scenarios_with_different_ids = list()
scenarios_with_initial_state_error = list()

plot_and_save_scenarios = True
animate_scenario = False
# Initialize Plotting
plt.style.use('classic')
inch_in_cm = 2.54
figsize = [60, 30]
existing_scenario_ids_paths_dict = get_existing_scenarios(scenarios_root_folder)

counter = 1
for idx, (scenario, planning_problem_set) in enumerate(load_scenarios(scenarios_root_folder, scenario_ids)):
    # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    # with the heuristic A* we do not want to solve cooperative scenarios so in all cases we will have
    # only one planning problem
    planning_problem_idx = 0
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

    # scenario_id = scenario.benchmark_id
    scenario_id = scenario_ids[idx]
    if scenario_id != scenario.benchmark_id:
        scenarios_with_different_ids.append(scenario_id)
        logger.warning("Benchmark ID and scenario name differs in scenario: [{:<20}]".format(scenario_id))

    base_msg = "{:3}/{}\t[{:<20}]\t ".format(counter, num_of_scenarios_to_solve, scenario_id)

    routes = None
    time1 = time.perf_counter()
    try:
        routes = execute_search(scenario, planning_problem, backend="priority_queue")
    except IndexError as error:
        search_time = time.perf_counter() - time1
        scenarios_exception.append(scenario_id)
        msg = base_msg + "search took\t{:10.4f}\tms\texception:".format(search_time * 1000)
        logger.error(msg)
        logger.exception(error)
    except Exception as error:
        search_time = time.perf_counter() - time1
        scenarios_exception.append(scenario_id)
        msg = base_msg + "search took\t{:10.4f}\tms\texception:".format(search_time * 1000)
        logger.error(msg)
        logger.exception(error)
    else:
        search_time = time.perf_counter() - time1
        msg = "search took\t{:10.4f}\tms\t".format(search_time * 1000)

        # test = dim(routes)
        if len(routes) > 1:
            logger.info(base_msg + msg + "MORE path FOUND")
            msg = base_msg
        else:
            msg = base_msg + msg

        for ctn, route in enumerate(routes):
            if len(route) == 0:
                logger.warning(msg + "path NOT FOUND")
                scenarios_path_not_found.append(scenario_id)
                continue

            goal_lanelet_id = route[-1]
            logger.debug(msg + "\tpath FOUND to goal lanelet: [{}]".format(goal_lanelet_id))
            scenarios_path_found.append(scenario_id)
            if plot_and_save_scenarios:
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

                # saving solved solutions
                # output_folder
                output_folder = configs["output_path"]

                relative_path = existing_scenario_ids_paths_dict[scenario_id]
                list_from_relative_path = os.path.normpath(relative_path).split(os.path.sep)

                remove_first = len(os.path.normpath(scenarios_root_folder).split(os.path.sep))
                rel_path_to_scenario_from_root = os.path.join(*list_from_relative_path[remove_first:])

                output_folder = os.path.join(output_folder, rel_path_to_scenario_from_root)
                # if directory not exists create it
                os.makedirs(output_folder, exist_ok=True)

                output_file = os.path.join(output_folder, "fig_{}_goal_ID_{}.png".format(scenario_id, goal_lanelet_id))
                plt.savefig(output_file)
                # logger.debug("{} 's solution saved out".format(scenario_id))

    counter += 1

logger.info("=" * 30 + " RESULTS " + "=" * 30)

logger.debug("Successfully SOLVED scenarios {:3}/{}:".format(len(scenarios_path_found), num_of_scenarios_to_solve))
for scen in scenarios_path_found:
    logger.debug("\t{}".format(scen))
logger.warning(
    "Scenarios with path NOT FOUND {:3}/{}:".format(len(scenarios_path_not_found), num_of_scenarios_to_solve))
for scen in scenarios_path_not_found:
    logger.warning("\t{}".format(scen))
logger.critical(
    "Scenarios with EXCEPTION in path finding {:3}/{}:".format(len(scenarios_exception), num_of_scenarios_to_solve))
for scen in scenarios_exception:
    logger.critical("\t{}".format(scen))

logger.info("=" * 30 + " OTHER WARNINGS " + "=" * 30)
logger.warning("Scenarios with different name and benchmark ID {:3}/{}:".format(len(scenarios_with_different_ids),
                                                                                num_of_scenarios_to_solve))
for scen in scenarios_with_different_ids:
    logger.warning("\t{}".format(scen))

logger.warning("Scenarios with initial state assertion {:3}/{}:".format(len(scenarios_with_initial_state_error),
                                                                        num_of_scenarios_to_solve))
for scen in scenarios_with_initial_state_error:
    logger.warning("\t{}".format(scen))

plt.show()
