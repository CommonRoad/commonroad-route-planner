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
    
    def __init__(self, lanelet_network: LaneletNetwork,
                 state_initial: InitialState, goal_region: GoalRegion,
                 route_candidates: List[List[int]], ids_lanelets_permissible: Set):
        
        super().__init__(lanelet_network,
                        state_initial, goal_region,
                        route_candidates, ids_lanelets_permissible)
        
        
    
    def retrieve_shortest_route(self,
                            retrieve_shortest: bool = True,
                            included_lanelet_ids: List[int] = None) -> Route:
        """
        Retrieves shortest route object.
        Optionally can be forced to go through specific lanelets (currently WIP)
        """
        return self.retrieve_first_route(retrieve_shortest=retrieve_shortest, included_lanelet_ids=included_lanelet_ids)
