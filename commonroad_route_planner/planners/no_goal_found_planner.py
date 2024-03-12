import logging

from commonroad_route_planner.planners.base_route_planner import BaseRoutePlanner


# typing
from typing import List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad.scenario.lanelet import LaneletNetwork


class NoGoalFoundRoutePlanner(BaseRoutePlanner):
    def __init__(
        self,
        lanelet_network: "LaneletNetwork",
        prohibited_lanelet_ids: List[int],
        logger: logging.Logger = None,
        threshold_network_exploring: int = 20,
    ) -> None:
        """
        Planner if no goal is set.

        :param lanelet_network: cr lanelet network
        :param prohibited_lanelet_ids: ids of prohibited lanelets
        :param threshold_network_exploring: depth threshold for exploration
        """

        super(NoGoalFoundRoutePlanner, self).__init__(
            lanelet_network=lanelet_network,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            logger=logger,
        )

        self._threshold_network_exploring: int = threshold_network_exploring

    def find_routes(self, id_lanelet_start: int, id_lanelet_goal: int) -> List[int]:
        """
        Finds a route along the lanelet network for survival scenarios.

        The planner advances in the order of forward, right, left whenever possible.
        Notes:
            - it only considers lanes with same driving direction
            - the priorities of right and left should be swapped for left-hand traffic countries, e.g. UK
            - it goes until the end of the lanelet network or when it hits itself (like dying in the Snake game)

        :param id_lanelet_start: the initial lanelet where we start from
        :param id_lanelet_goal: goal lanelet it
        :return: route that consists of a list of lanelet IDs
        """
        route = list()
        id_lanelet_current = id_lanelet_start
        lanelet = self._lanelet_network.find_lanelet_by_id(id_lanelet_current)

        loop_cnt: int = 0
        while (
            id_lanelet_current not in route
            and loop_cnt < self._threshold_network_exploring
        ):
            route.append(lanelet.lanelet_id)

            found_new_lanelet = False
            if lanelet.successor:
                # naively select the first successors
                lanelet = self._lanelet_network.find_lanelet_by_id(lanelet.successor[0])
                found_new_lanelet = True

            if (
                not found_new_lanelet
                and lanelet.adj_right
                and lanelet.adj_right_same_direction
            ):
                # try to go right
                lanelet_adj_right = self._lanelet_network.find_lanelet_by_id(
                    lanelet.adj_right
                )
                if len(lanelet_adj_right.successor) > 0:
                    # right lanelet has successor
                    lanelet = self._lanelet_network.find_lanelet_by_id(
                        lanelet.adj_right
                    )
                    found_new_lanelet = True

            if (
                not found_new_lanelet
                and lanelet.adj_left
                and lanelet.adj_left_same_direction
            ):
                # try to go left
                lanelet_adj_left = self._lanelet_network.find_lanelet_by_id(
                    lanelet.adj_left
                )
                if len(lanelet_adj_left.successor) > 0:
                    # left lanelet has successor
                    lanelet = self._lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                    found_new_lanelet = True

            if not found_new_lanelet:
                # no possible route to advance
                break
            else:
                # set lanelet
                id_lanelet_current = lanelet.lanelet_id

            loop_cnt += 1

        if len(route) == 0:
            self._logger.error(
                "[CR Route Planner] No goal found Route planner could not find a Route"
            )
            raise ValueError(
                "[CR Route Planner] No goal found Route planner could not find a Route"
            )

        return route
