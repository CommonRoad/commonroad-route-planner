####################################################################################
#
# This class will be used instead of the route candidate holder from now on.
#
#
#
#
#
#
###################################################################################
import logging

# commonroad
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState




# own code base
from commonroad_route_planner.route import Route
from commonroad_route_planner.route_candidate_holder import RouteCandidateHolder
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import LaneChangeMethod
from commonroad_route_planner.utility.visualization import debug_visualize



# typing
from typing import List, Set



class RouteSelector(RouteCandidateHolder):
    """
    Selects a route from the route planner, per default the shortest route.
    """
    
    def __init__(self,
                 lanelet_network: LaneletNetwork,
                 initial_state: InitialState,
                 goal_region: GoalRegion,
                 route_candidates: List[List[int]],
                 prohibited_lanelet_ids: List[int],
                 logger: logging.Logger,
                 lane_change_method: LaneChangeMethod = LaneChangeMethod.QUINTIC_SPLINE
                 ) -> None:
        
        super().__init__(
            lanelet_network=lanelet_network,
            initial_state=initial_state,
            goal_region=goal_region,
            route_candidates=route_candidates,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            logger=logger,
            lane_change_method=lane_change_method
            )
        
        
    
    def retrieve_shortest_route(self,
                                retrieve_shortest: bool = True,
                                consider_least_lance_changes: bool = True,
                                included_lanelet_ids: List[int] = None
                                ) -> Route:
        """
        Retrieves shortest route object.
        Optionally can be forced to go through specific lanelets.

        :param retrieve_shortest: if True, will only find shortest distance routes,
        :param consider_least_lance_changes: considers least amount of disjoint lane changes, if possible
        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: route instance
        """

        if(consider_least_lance_changes):
            return self.retrieve_shortetest_route_with_least_lane_changes(
                included_lanelet_ids=included_lanelet_ids
            )

        else:
            return self.retrieve_first_route(retrieve_shortest=retrieve_shortest,
                                             included_lanelet_ids=included_lanelet_ids
                                             )


