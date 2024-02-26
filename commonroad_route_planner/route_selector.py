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
                 logger: logging.Logger
                 ) -> None:
        
        super().__init__(
            lanelet_network=lanelet_network,
            initial_state=initial_state,
            goal_region=goal_region,
            route_candidates=route_candidates,
            prohibited_lanelet_ids=prohibited_lanelet_ids,
            logger=logger
            )
        
        
    
    def retrieve_shortest_route(self,
                                retrieve_shortest: bool = True,
                                included_lanelet_ids: List[int] = None
                                ) -> Route:
        """
        Retrieves shortest route object.
        Optionally can be forced to go through specific lanelets.

        :param retrieve_shortest: if True, will only find shortest distance routes,
        :param included_lanelet_ids: forces planner to go throug lanelets, if possible. Will ignore retrieve_shortest

        :return: route instance
        """
        return self.retrieve_first_route(retrieve_shortest=retrieve_shortest,
                                         included_lanelet_ids=included_lanelet_ids
                                         )
