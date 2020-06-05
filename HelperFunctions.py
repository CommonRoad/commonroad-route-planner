import fnmatch
import logging
import os
from datetime import datetime
from typing import Tuple

import matplotlib as mpl
import yaml
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
# from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad_cc.collision_detection.pycrcc_collision_dispatch import create_collision_object
from commonroad_cc.visualization.draw_dispatch import draw_object as draw_cc_object

from route_planner.route_planner import RoutePlanner

mpl.use('Qt5Agg')
import matplotlib.pyplot as plt


def load_config_file(filename) -> dict:
    # open config file
    with open(filename, 'r') as stream:
        try:
            configs = yaml.load(stream, Loader=yaml.BaseLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return configs


def release_logger(logger):
    """
    Releases the logger
    :param logger: the logger to be released
    """
    handlers = logger.handlers[:]
    for handler in handlers:
        handler.close()
        logger.removeHandler(handler)


def initialize_logger(logger_name, config_file) -> logging.Logger:
    logger = logging.getLogger(logger_name)
    release_logger(logger)
    logger.setLevel(logging.DEBUG)
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    formatter = logging.Formatter('%(name)s\t%(levelname)s\t%(message)s')

    # create console handler
    console_handler = logging.StreamHandler()
    # set the level of logging to console
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    if config_file['log_to_file'] == 'True':
        log_file_dir = config_file['log_file_dir']
        date_time_string = ''
        if config_file['add_timestamp_to_log_file'] == 'True':
            now = datetime.now()  # current date and time
            date_time_string = now.strftime("_%Y_%m_%d_%H-%M-%S")

        # if directory not exists create it
        if not os.path.exists(log_file_dir):
            os.mkdir(log_file_dir)

        log_file_path = os.path.join(log_file_dir, config_file['log_file_name'] + date_time_string + ".log")
        file_handler = logging.FileHandler(log_file_path)
        # set the level of logging to file
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def get_existing_scenarios(root_dir):
    for path, directories, files in os.walk(root_dir):
        for scenario in fnmatch.filter(files, "*.xml"):
            scenario_path = os.path.join(path, scenario)
            scenario_name = scenario[:-4]  # chop the '.xml' extension
            yield {"name": scenario_name, "path": scenario_path}


def get_existing_scenarios_name(root_dir):
    existing_scenarios = get_existing_scenarios(root_dir)
    scenarios_name = list()
    for s in existing_scenarios:
        scenarios_name.append(s["name"])

    return scenarios_name


def load_scenario(root_dir, scenario_id) -> Tuple[Scenario, PlanningProblemSet]:
    scenario_path = os.path.join(root_dir, scenario_id + '.xml')
    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    return scenario, planning_problem_set


def load_scenarios(root_dir, scenario_ids):
    for scenario_id in scenario_ids:
        yield load_scenario(root_dir, scenario_id)


def execute_search(scenario, planning_problem):
    route_planner = RoutePlanner(scenario.benchmark_id, scenario.lanelet_network, planning_problem)

    return route_planner.search_alg()


def draw_initial_state(planning_problem: PlanningProblem, fig_num=None):
    # draw ego vehicle
    if fig_num is not None:
        plt.figure(fig_num)

    egoShape = Rectangle(length=5, width=2)
    trajectory = Trajectory(initial_time_step=int(planning_problem.initial_state.time_step),
                            state_list=[planning_problem.initial_state])
    prediction = TrajectoryPrediction(trajectory=trajectory, shape=egoShape)
    collision_object = create_collision_object(prediction)
    draw_cc_object(collision_object)


def get_last_time_step_in_scenario(scenario: Scenario):
    last_time_step = 0
    for dy_ob in scenario.dynamic_obstacles:
        time_step = len(dy_ob.prediction.occupancy_set)
        if time_step > last_time_step:
            last_time_step = time_step

    return last_time_step


def dim(a):
    if not type(a) == list:
        return []
    return [len(a)] + dim(a[0])
