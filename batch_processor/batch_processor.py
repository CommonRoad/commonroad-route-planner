import errno
import os
import sys
import time

commonroad_route_planner_root = "./"
sys.path.append(os.path.join(commonroad_route_planner_root))

import numpy as np
import matplotlib as mpl

try:
    mpl.use('Qt5Agg')
    import matplotlib.pyplot as plt
except ImportError:
    mpl.use('TkAgg')
    import matplotlib.pyplot as plt

from commonroad.scenario.trajectory import State
from commonroad_route_planner.RoutePlanner import RoutePlanner
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_route_planner.utils_visualization import draw_state
import HelperFunctions as hf


class BatchProcessor:
    def __init__(self, name_config):
        self.list_result_scenarios = list()
        # load config file
        self.configs = hf.load_config_file(os.path.join(os.path.dirname(__file__), name_config))
        # create a logger
        self.logger = hf.initialize_logger("Batch processor", self.configs)
        self.logger.debug("Config file loaded and logger created")

        # if directory not exists create it
        self.scenarios_root_folder = self.configs['input_path']
        if not os.path.exists(self.scenarios_root_folder):
            self.logger.error("Scenarios root path <{}> is not exist".format(self.scenarios_root_folder))
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.scenarios_root_folder)

        # get scenarios to process
        specified_scenario_ids = set(self.configs['scenarios_to_run'])
        skip_scenario_ids = self.configs['scenarios_to_skip']
        existing_scenario_ids = set(hf.get_existing_scenario_ids(self.scenarios_root_folder))

        existing_scenario_ids.difference_update(skip_scenario_ids)
        if len(skip_scenario_ids) != 0:
            self.logger.info("Skipping the following scenarios: ")
            for idx in skip_scenario_ids:
                self.logger.info("\t{:<20}".format(idx))

        if len(specified_scenario_ids) != 0:
            self.logger.info("Only run on specified scenarios:")
            not_found_ids = specified_scenario_ids.difference(existing_scenario_ids)
            if len(not_found_ids) != 0:
                for idx in not_found_ids:
                    self.logger.warning("\t{} is NOT FOUND or SKIPPED and hence won't be processed!".format(idx))
            scenario_ids = specified_scenario_ids.intersection(existing_scenario_ids)
            for idx in scenario_ids:
                self.logger.info("\t{}".format(idx))
        else:
            self.logger.info("Run on all scenarios")
            scenario_ids = existing_scenario_ids  # if we still need to use existing scenario ids the consider COPY instead

        self.scenario_ids = sorted(list(scenario_ids))
        self.num_of_scenarios_to_solve = len(self.scenario_ids)

    def setup_and_run(self):
        scenarios_path_found = list()
        scenarios_path_not_found = list()
        scenarios_exception = list()

        scenarios_with_different_ids = list()
        scenarios_with_initial_state_error = list()

        # plotting
        plot_and_save_scenarios = True
        plt.style.use('classic')
        inch_in_cm = 2.54
        figsize = [60, 30]

        counter = 1
        for idx, (scenario, planning_problem_set) in enumerate(
                hf.load_scenarios(self.scenarios_root_folder, self.scenario_ids)):
            # retrieve planning problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
            # with the heuristic A* we do not want to solve cooperative scenarios so in all cases we will have
            # only one planning problem
            planning_problem_idx = 0
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx]

            # scenario_id = scenario.benchmark_id
            scenario_id = self.scenario_ids[idx]
            if scenario_id != scenario.benchmark_id:
                scenarios_with_different_ids.append(scenario_id)
                self.logger.warning("Benchmark ID and scenario name differs in scenario: [{:<20}]".format(scenario_id))

            base_msg = "{:3}/{}\t[{:<20}]\t ".format(counter, self.num_of_scenarios_to_solve, scenario_id)

            time1 = time.perf_counter()
            try:
                # create route planner
                route_planner = RoutePlanner(scenario, planning_problem, backend=RoutePlanner.Backend.NETWORKX_REVERSED)
                # route candidate holder holds all feasible route candidates
                route_planner.plan_routes()

                # retrieve the best route by orientation metric
                route = route_planner.retrieve_best_route_by_orientation()
                # plot_found_routes(scenario, planning_problem, [route_obj.route])

                # Navigator
                navigator = route.navigator

                # Test States
                states = [State(position=np.array([12, 26]), orientation=np.pi),
                          State(position=np.array([20, 19]), orientation=np.pi),
                          State(position=np.array([-7.6, 57]), orientation=np.pi / 2),
                          planning_problem.initial_state]

            except IndexError as error:
                search_time = time.perf_counter() - time1
                scenarios_exception.append(scenario_id)
                msg = base_msg + "search took\t{:10.4f}\tms\texception:".format(search_time * 1000)
                self.logger.error(msg)
                self.logger.exception(error)
            except Exception as error:
                search_time = time.perf_counter() - time1
                scenarios_exception.append(scenario_id)
                msg = base_msg + "search took\t{:10.4f}\tms\texception:".format(search_time * 1000)
                self.logger.error(msg)
                self.logger.exception(error)
            else:
                search_time = time.perf_counter() - time1
                msg = base_msg + "search took\t{:10.4f}\tms\t".format(search_time * 1000)

                if len(route.list_ids_lanelets) == 0:
                    self.logger.warning(msg + "path NOT FOUND")
                    scenarios_path_not_found.append(scenario_id)
                    continue

                goal_lanelet_id = route.list_ids_lanelets[-1]
                self.logger.debug(msg + "\tpath FOUND to goal lanelet: [{}]".format(goal_lanelet_id))
                scenarios_path_found.append(scenario_id)
                if plot_and_save_scenarios:
                    fig = plt.figure(num=0, figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
                    fig.clf()
                    fig.suptitle(scenario.benchmark_id, fontsize=20)
                    fig.gca().axis('equal')
                    handles = {}  # collects handles of obstacle patches, plotted by matplotlib
                    plot_limits = hf.get_plot_limits(route.list_ids_lanelets, scenario, border=15)

                    # plot the lanelet network and the planning problem
                    draw_object(scenario, handles=handles, plot_limits=plot_limits,
                                draw_params={'lanelet': {'show_label': False}})
                    draw_object(planning_problem, handles=handles, plot_limits=plot_limits)
                    fig.gca().autoscale()

                    # draw ego vehicle - with a collision object - uses commonroad_cc.visualizer
                    try:
                        draw_state(planning_problem.initial_state)
                    except AssertionError as error:
                        print(error)

                    for route_lanelet_id in route.list_ids_lanelets:
                        lanelet = scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
                        draw_object(lanelet, handles=handles, plot_limits=plot_limits, draw_params={'lanelet': {
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
                        plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "b", zorder=30,
                                 scalex=False,
                                 scaley=False)

                    # saving solved solutions
                    output_folder = self.configs["output_path"]
                    os.makedirs(output_folder, exist_ok=True)

                    output_file = os.path.join(output_folder,
                                               "fig_{}_goal_ID_{}.png".format(scenario_id, goal_lanelet_id))
                    plt.savefig(output_file)

            counter += 1

        # plt.show()
        self.list_result_scenarios.append(scenarios_path_found)
        self.list_result_scenarios.append(scenarios_path_not_found)
        self.list_result_scenarios.append(scenarios_exception)
        self.list_result_scenarios.append(scenarios_with_different_ids)
        self.list_result_scenarios.append(scenarios_with_initial_state_error)

    def logger_output(self):
        scenarios_path_found = self.list_result_scenarios[0]
        scenarios_path_not_found = self.list_result_scenarios[1]
        scenarios_exception = self.list_result_scenarios[2]
        scenarios_with_different_ids = self.list_result_scenarios[3]
        scenarios_with_initial_state_error = self.list_result_scenarios[4]

        self.logger.info("=" * 30 + " RESULTS " + "=" * 30)

        self.logger.debug(
            "Successfully SOLVED scenarios {:3}/{}:".format(len(scenarios_path_found), self.num_of_scenarios_to_solve))
        for scenario in scenarios_path_found:
            self.logger.debug("\t{}".format(scenario))
        self.logger.warning(
            "Scenarios with path NOT FOUND {:3}/{}:".format(len(scenarios_path_not_found),
                                                            self.num_of_scenarios_to_solve))
        for scenario in scenarios_path_not_found:
            self.logger.warning("\t{}".format(scenario))
        self.logger.critical(
            "Scenarios with EXCEPTION in path finding {:3}/{}:".format(len(scenarios_exception),
                                                                       self.num_of_scenarios_to_solve))
        for scenario in scenarios_exception:
            self.logger.critical("\t{}".format(scenario))

        self.logger.info("=" * 30 + " OTHER WARNINGS " + "=" * 30)
        self.logger.warning(
            "Scenarios with different name and benchmark ID {:3}/{}:".format(len(scenarios_with_different_ids),
                                                                             self.num_of_scenarios_to_solve))
        for scenario in scenarios_with_different_ids:
            self.logger.warning("\t{}".format(scenario))

        self.logger.warning(
            "Scenarios with initial state assertion {:3}/{}:".format(len(scenarios_with_initial_state_error),
                                                                     self.num_of_scenarios_to_solve))
        for scenario in scenarios_with_initial_state_error:
            self.logger.warning("\t{}".format(scenario))


if __name__ == "__main__":
    processor = BatchProcessor('batch_processor_config.yaml')
    processor.setup_and_run()
    processor.logger_output()
