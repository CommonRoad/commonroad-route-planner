from commonroad_route_planner.planners.base_route_planner import BaseRoutePlanner


class SurvivalRoutePlanner(BaseRoutePlanner):
    def find_routes(self, id_lanelet_start, id_lanelet_goal):
        """Finds a route along the lanelet network for survival scenarios.

        The planner advances in the order of forward, right, left whenever possible.
        Notes:
            - it only considers lanes with same driving direction
            - the priorities of right and left should be swapped for left-hand traffic countries, e.g. UK
            - it goes until the end of the lanelet network or when it is hits itself (like dying in the Snake game)

        :param id_lanelet_start: the initial lanelet where we start from
        :return: route that consists of a list of lanelet IDs
        """
        route = list()
        id_lanelet_current = id_lanelet_start
        lanelet = self.lanelet_network.find_lanelet_by_id(id_lanelet_current)

        while id_lanelet_current not in route:
            route.append(lanelet.lanelet_id)

            found_new_lanelet = False
            if lanelet.successor:
                # naively select the first successors
                lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.successor[0])
                found_new_lanelet = True

            if (
                not found_new_lanelet
                and lanelet.adj_right
                and lanelet.adj_right_same_direction
            ):
                # try to go right
                lanelet_adj_right = self.lanelet_network.find_lanelet_by_id(
                    lanelet.adj_right
                )
                if len(lanelet_adj_right.successor):
                    # right lanelet has successor
                    lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_right)
                    found_new_lanelet = True

            if (
                not found_new_lanelet
                and lanelet.adj_left
                and lanelet.adj_left_same_direction
            ):
                # try to go left
                lanelet_adj_left = self.lanelet_network.find_lanelet_by_id(
                    lanelet.adj_left
                )
                if len(lanelet_adj_left.successor):
                    # left lanelet has successor
                    lanelet = self.lanelet_network.find_lanelet_by_id(lanelet.adj_left)
                    found_new_lanelet = True

            if not found_new_lanelet:
                # no possible route to advance
                break
            else:
                # set lanelet
                id_lanelet_current = lanelet.lanelet_id

        return route
