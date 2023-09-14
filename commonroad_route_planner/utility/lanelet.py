import logging

_logger = logging.getLogger(__name__)

def _retrieve_ids_lanelets_goal(goal_region, lanelet_network, allowed_lanelet_ids, reach_goal_state: bool):
    def _filter_allowed_lanelet_ids(lanelet_ids):
        return filter(lambda x: x in allowed_lanelet_ids, lanelet_ids)

    """Retrieves the ids of the lanelets in which the goal position is situated"""
    list_ids_lanelets_goal = list()
    list_ids_lanelets_goal_original = list()

    if hasattr(goal_region, "lanelets_of_goal_position"):
        if goal_region.lanelets_of_goal_position is None:
            _logger.debug("No goal lanelet given")
        else:
            _logger.debug("Goal lanelet given")
            # the goals are stored in a dict, one goal can consist of multiple lanelets
            # now we just iterate over the goals and add every ID which we find to
            # the goal_lanelet_ids list
            for list_ids_lanelets_pos_goal in list(
                    goal_region.lanelets_of_goal_position.values()):
                list_ids_lanelets_goal.extend(list_ids_lanelets_pos_goal)

            list_ids_lanelets_goal = list(
                    _filter_allowed_lanelet_ids(list_ids_lanelets_goal))

        if list_ids_lanelets_goal:
            reach_goal_state = False

    elif hasattr(goal_region, "state_list"):
        for idx, state in enumerate(goal_region.state_list):
            if hasattr(state, "position"):
                if hasattr(state.position, "center"):
                    pos_goal = state.position.center

                else:
                    pos_goal = state.position
                [
                    list_ids_lanelets_pos_goal] = lanelet_network.find_lanelet_by_position(
                        [pos_goal])
                list_ids_lanelets_pos_goal = list(
                        _filter_allowed_lanelet_ids(
                            list_ids_lanelets_pos_goal))

                if reach_goal_state:
                    # we want to reach the goal states (not just the goal lanelets), here we instead demand
                    # reaching the predecessor lanelets of the goal states
                    list_ids_lanelets_goal_original = (
                        list_ids_lanelets_pos_goal.copy())
                    list_ids_lanelets_pos_goal.clear()

                    for id_lanelet_goal in list_ids_lanelets_goal_original:
                        lanelet_goal = lanelet_network.find_lanelet_by_id(
                                id_lanelet_goal)
                        # make predecessor as goal
                        list_ids_lanelets_pos_goal.extend(
                            lanelet_goal.predecessor)

                if list_ids_lanelets_pos_goal:
                    list_ids_lanelets_goal.extend(list_ids_lanelets_pos_goal)
                    _logger.debug(
                            "Goal lanelet IDs estimated from goal shape in state [{}]".format(
                                    idx))
                else:
                    _logger.debug(
                            "No Goal lanelet IDs could be determined from the goal shape in state [{}]".format(
                                    idx))

    # remove duplicates and reset to none if no lanelet IDs found
    if list_ids_lanelets_goal:
        # remove duplicates and sort in ascending order
        list_ids_lanelets_goal = sorted(
            list(dict.fromkeys(list_ids_lanelets_goal)))
    else:
        list_ids_lanelets_goal = None

    return list_ids_lanelets_goal, list_ids_lanelets_goal_original